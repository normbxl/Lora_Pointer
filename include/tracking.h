#pragma once

#include <acsip.h>
#include <LilyGoWatch.h>

typedef struct GpsPosition {
    double lat;
    double lng;
    bool valid;
} GpsPosition_t;

typedef struct RfConfigStruct {
  uint32_t frequency_tx;
  uint32_t frequency_rx;
  uint8_t rf_power;         // 2-20
  uint8_t spreading_factor; // 7-12
  uint8_t bandwidth;        // 125, 250, 500
} RfConfig_t;

class Tracking {
  public:
    static Tracking* instance();
    bool initRf(RfConfig_t &rfConfig);
    void enable();
    void disable();
    bool isEnabled();
    bool setup();
    bool hasGpsFix();
    int getRssi();
    float getHeading();
    bool getRemotePosition( GpsPosition &data );
    bool getMyPosition( GpsPosition &data);
    void run();
    void loraDataReceived(const char *data, size_t len, int rssi, int snr);

  private:
    Acsip sip;
    bool enabled;
    bool gpsFix;
    float heading;
    bool isSetup;
    Tracking();
    static Tracking* _instance;
    TTGOClass *ttgo;
    HardwareSerial *sipSerial;
    GpsPosition_t myPosition;
    GpsPosition_t remotePosition;
    int rssi;

    bool processGps();
    void sendPostion();
};