#ifndef LILYGO_WATCH_HAS_S76_S78G
#define LILYGO_WATCH_2019_WITH_TOUCH
#define LILYGO_WATCH_HAS_S76_S78G
#define LILYGO_WATCH_LVGL                   //To use LVGL, you need to enable the macro LVGL
#endif

#include "tracking.h"

#include <Arduino.h>


typedef struct {
    // uint8_t device_id;
    // uint8_t battery;
    float lat;
    float lng;
    uint8_t valid;
} LoraPayload_t;

RfConfig_t rfConfig = {
  frequency_tx: 868100000,
  frequency_rx: 867100000,
  rf_power: 13,   // ~25 mW, max allowed in this band
  spreading_factor: 7,
  bandwidth: 125
};

Tracking* Tracking::_instance = nullptr;

Tracking* Tracking::instance() {
    if (_instance==nullptr)
        _instance = new Tracking();
    return _instance;
}

Tracking::Tracking() : 
    sip(), enabled(false), gpsFix(false), heading(0), isSetup(false)
{
    myPosition = GpsPosition{ 0.0, 0.0, false};
    remotePosition = GpsPosition{ 0.0, 0.0, false};
}

bool Tracking::setup() {
    ttgo = TTGOClass::getWatch();
    
    ttgo->enableLDO4();
    ttgo->enableLDO3();
    delay(100);

    sipSerial = new HardwareSerial(1);
    sipSerial->begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
    isSetup= sip.begin(*sipSerial);
    return isSetup;
}

// C-function callback wrapper
void rfDataReceived(const char *data, size_t len, int rssi, int snr) {
    Tracking::instance()->loraDataReceived(data, len, rssi, snr);
    
}
bool Tracking::initRf(RfConfig_t &rfConfig) {
    int ret;
    ret=sip.setRfFreq(rfConfig.frequency_tx);
    if (ret!=S7XG_OK)
        return false;

    ret=sip.setPower(rfConfig.rf_power);
    if (ret!=S7XG_OK)
        return false;

    ret=sip.setRfSpreadingFactor(rfConfig.spreading_factor);
    if (ret!=S7XG_OK)
        return false;

    ret=sip.setRfBandWitdth(rfConfig.bandwidth);
    if (ret!=S7XG_OK)
        return false;

    ret=sip.setReceiveContinuous(true);
    if (ret!=S7XG_OK)
        return false;
    
    sip.setRFCallback(rfDataReceived);
    return true;
}

void Tracking::enable() {
    if (!isSetup) 
        setup();
        
    if (enabled)
        return;
    

    int ret=sip.GPSStart(S7XG_GPS_START_HOT, S7XG_GPS_MODE_MANUAL, S7XG_SATELLITE_GPS_GLONASS, 1000);
    if (ret==S7XG_OK) {
        enabled = initRf(rfConfig);
    }
}
void Tracking::disable() {
    enabled= sip.GPSStop()!=S7XG_OK;
}
bool Tracking::isEnabled() {
    return enabled;
}
bool Tracking::hasGpsFix() {
    return gpsFix;
}
int Tracking::getRssi() {
    return rssi;
}
float Tracking::getHeading() {
    return heading;
}
bool Tracking::getRemotePosition(struct GpsPosition &data ) {
    if (remotePosition.valid) {
        data.lat=remotePosition.lat;
        data.lng=remotePosition.lng;
        data.valid=true;
        return true;
    }
    return false;
}
bool Tracking::getMyPosition( GpsPosition &data) {
    if (myPosition.valid) {
        data.lat=myPosition.lat;
        data.lng=myPosition.lng;
        data.valid=true;
        return true;
    }
    return false;
}

bool Tracking::processGps() {
    static unsigned long lastUpdate=0L;
    int ret;
    GPSDataStruct gpsData;
    
    if (!enabled)
        return false;
    if (millis() - lastUpdate >= 1000) {
    
        ret = sip.getData(gpsData, S7XG_GPS_DATA_DD);
        if (ret!=S7XG_OK) {
            gpsFix=false;
            return false;
        }
        gpsFix=true;
        if (gpsData.isValid) {
            if (myPosition.valid) {
                // switching x-y arguments to get 0: North, 90: West, 180: South
                float newHeading=(float)atan2(gpsData.dd.lng-myPosition.lng, gpsData.dd.lat - myPosition.lat) * RAD_TO_DEG;
                // exponential filter
                heading = newHeading*0.35f + heading*0.65f;
            }
            myPosition.lat=gpsData.dd.lat;
            myPosition.lng=gpsData.dd.lng;
        }
        myPosition.valid=gpsData.isValid;
        lastUpdate=millis();
        return true;
    }
    return false;
}

void Tracking::run() {
    if (enabled) {
        if (processGps()) {
            sendPostion();
        }
        sip.service();
    }
}

void Tracking::loraDataReceived(const char *data, size_t len, int rssi, int snr) {
    if (len==sizeof(LoraPayload_t)) {
        LoraPayload_t* payload = (LoraPayload_t*)data;
        remotePosition.lat = (double)payload->lat;
        remotePosition.lng = (double)payload->lng;
        remotePosition.valid = payload->valid != 0;
    }
    this->rssi=rssi;
}

void Tracking::sendPostion() {
    const char hex[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
    if (!enabled)
        return;
    
    int i = 0;
    LoraPayload_t payload={ 
        .lat=(float)myPosition.lat,
        .lng=(float)myPosition.lng,
        .valid=myPosition.valid ? 1 : 0
    }; 
    char* dataPtr = (char*)&payload;
    char *ascii = (char *)calloc(sizeof(LoraPayload_t) * 3 + 1, sizeof(char));
    if (ascii == NULL) {
        return;
    }
    while ( i < sizeof(LoraPayload_t) ) {
        int c = dataPtr[i] & 0x000000ff;
        ascii[i * 2] = hex[c / 16] ;
        ascii[i * 2 + 1] = hex[c % 16] ;
        ++i;
    }
    int ret = sip.RfSend(ascii);
    free(ascii);
}

