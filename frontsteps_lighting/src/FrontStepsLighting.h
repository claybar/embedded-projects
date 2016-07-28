#include <Arduino.h>

// Logic of motion sensors
#define MOTION_SENSOR HIGH
#define MOTION_YES true
#define MOTION_NO false
#define ON true
#define OFF false

// Error detection
#define WDT                    WDTO_8S

enum partsOfDay_t {
  daytime, // main portion of day (lots of light)
  evening, // transition into night (dim light)
  night,   // main portion of night (no light)
  morning  // transition into day (dim light)
};

enum lightsLevel_t {
  off,
  ambient,
  bright
};

int serial_putchar(char c, FILE* f);
byte readI2CRegister(byte i2c_address, byte reg);
int percent2LEDInt(int p);

void updateLights();

boolean mqttConnect();
void mqttSubscribe(const char* name);
void mqttPublish(const char* name, const char* payload);
void mqtt_callback(char* topic, byte* payload, unsigned int length);
