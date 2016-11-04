#include <Arduino.h>

#define LEDPIN            9

#define VOLTAGESENSORPIN  0
#define VOLTAGESENSORCAL  300.6
#define VOLTAGESENSORPHASESHIFT 1.7

#define CURRENTSENSOR1PIN 1
#define CURRENTSENSOR1RATIO (100.0 / 0.05)
#define CURRENTSENSOR1BURDEN 33.0

#define CURRENTSENSOR2PIN 2
#define CURRENTSENSOR2RATIO (100.0 / 0.05)
#define CURRENTSENSOR2BURDEN 33.0

#define CURRENTSENSOR3PIN 3
#define CURRENTSENSOR3RATIO (100.0 / 0.05)
#define CURRENTSENSOR3BURDEN 33.0

#define CURRENTSENSOR4PIN 6
#define CURRENTSENSOR4RATIO (100.0 / 0.05)
#define CURRENTSENSOR4BURDEN 33.0

#define MEASUREMENTCROSSINGS 20


byte readI2CRegister(byte i2c_address, byte reg);
boolean ethernetConnect();
boolean mqttConnect();
void mqttSetupSubscriptions();
void mqttSubscribe(const char* name);
void mqttPublish(const char* name, const char* payload);
void mqttPublish(const char* name, const char* payload, bool retained);
void mqttPublishRaw(const char* name, const char* payload, bool retained);
void mqttCallback(char* topic, byte* payload, unsigned int length);
