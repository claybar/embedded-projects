#include <Arduino.h>

// Error detection
#define WDT                     WDTO_8S
#define ERROR_NONE              0x00
#define ERROR_MQTT_DISCONNECTED 0x01
#define ERROR_MQTT_TXFAIL       0x02
byte errorMonitorMQTT();

// Hardware interaction
byte readI2CRegister(byte i2c_address, byte reg);
int serial_putchar(char c, FILE* f);
void insideLights(bool state);
void outsideLights(bool state);

// Timers
void statusUpdateTimer();

// MQTT to serial relaying
void serialMQTTRelay();
void serialUnrecognized(const char *command);

// MQTT
boolean ethernetConnect();
boolean mqttConnect();
void mqttSetupSubscriptions();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void mqttSubscribe(const char* name);
void mqttPublish(const char* name, const char* payload);
void mqttPublishRaw(const char* name, const char* payload);
