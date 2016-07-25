#include <Arduino.h>

// Hardware setup
#define ACTIVITYLEDPIN 13
#define INSIDELIGHTSPIN 6
#define OUTSIDELIGHTSPIN 9
#define MOTIONSENSORAPIN 8
#define MOTIONSENSORBPIN 3
#define DOORSENSORPIN 7

// Analog pins
#define VOLTAGE5PIN 2
#define VOLTAGE9PIN 1
#define VOLTAGE12PIN 0
#define VOLTAGE24PIN 3
#define LIGHTSENSORPIN 7

#define MAC_I2C_ADDR 0x50
#define MAC_REG_BASE 0xFA

// Logic of motion and door sensors
#define MOTION LOW
#define DOOROPEN HIGH
#define ON true
#define OFF false

// Error detection
#define WDT                    WDTO_8S
#define MQTTERROR_NONE         0x00
#define MQTTERROR_DISCONNECTED 0x01
#define MQTTERROR_TXFAIL       0x02
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
void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttSubscribe(const char* name);
void mqttPublish(const char* name, const char* payload);
void mqttPublishRaw(const char* name, const char* payload);

int percent2Int(int p);
