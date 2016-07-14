#include <Arduino.h>

// Function prototpyes
void mqtt_callback(char* topic, byte* payload, unsigned int length);
int serial_putchar(char c, FILE* f);
void insideLights(bool state);
void outsideLights(bool state);
boolean ethernetConnect();
void mqttSetupSubscriptions();
boolean mqttConnect();
void mqttSubscribe(const char* name);
void mqttPublish(const char* name, const char* payload);
void mqttPublishRelay(const char* name, const char* payload);
byte readI2CRegister(byte i2c_address, byte reg);
void serialMQTTRelay();
void serialUnrecognized(const char *command);
void statusUpdateTimer();
byte errorMonitorMQTT();
