#include <Arduino.h>
#include <Timezone.h>

// Logic of motion sensors
#define MOTION_SENSOR   (HIGH)
#define MOTION_YES      (true)
#define MOTION_NO       (false)
#define ON              (true)
#define OFF             (false)

// Error detection
#define WDT            (WDTO_8S)

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

TimeChangeRule nzdt = NZDT_RULE;
TimeChangeRule nzst = NZST_RULE;

int serial_putchar(char c, FILE* f);
byte readI2CRegister(byte i2c_address, byte reg);
int percent2LEDInt(int p);
void timeToTmpBuf(uint16_t minsAfterMidnight);
void printTime(byte h, byte m);
void printDateTime(time_t t);

time_t getUtcFromNtp();
void sendNTPpacket(IPAddress &address);

void timeOfDayAlarm();
void sunriseSunsetAlarm();
void updateLights();

boolean mqttConnect();
void mqttSubscribe(const char* name);
void mqttPublish(const char* name, const char* payload);
void mqttPublish(const char* name, const char* payload, bool retained);
void mqttCallback(char* topic, byte* payload, unsigned int length);

void publishConfigAndSettings();
void publishSunriseSunset();
void publishPortionOfDay(const char* portion);
//void publishAllRetained();
