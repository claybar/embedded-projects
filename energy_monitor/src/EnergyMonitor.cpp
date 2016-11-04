/*
  EmonTx CT123 Voltage Serial Only example

  Part of the openenergymonitor.org project
  Licence: GNU GPL V3

  Author: Trystan Lea
*/
#include <Arduino.h>

#include <EEPROMex.h>
#include <SPI.h>
#include <Wire.h>
#include <Ethernet.h>
#include <EmonLib.h>
#include <PubSubClient.h>

#include <Settings.h>
#include <Secrets.h>
#include <EtherTen.h>

#include "EnergyMonitor.h"

// MQTT setup
const int mqttWillQos = 0;
const int mqttWillRetain = 1;
int mqttFailCount = 0;
int mqttDisconnectedCount = 0;
const int mqttFailCountLimit = 5;
const int mqttDisconnectedCountLimit = 5;


// Hardware and protocol handlers
EthernetClient ethernet;
PubSubClient mqtt(ethernet);

// Create  instances for each CT channel
EnergyMonitor ct1,ct2,ct3, ct4;

// Storage, will be set by onboard i2c device and DHCP
byte mac[] = { 0, 0, 0, 0, 0, 0 };
IPAddress mqttIP(MQTT_SERVER_IP);

// Settings storage
commonSettings0_t commonSettings;


void setup()
{
  Serial.begin(115200);

  Serial.println(F("Energy Monitor MQTT Homie-Lite"));

  // Initialise the SPI and i2c bus.
  SPI.begin();
  Wire.begin();

  // Read MAC address from on-board IC
  for (int i = 0; i < 6; i++)
  {
    mac[i] = readI2CRegister(MAC_I2C_ADDR, MAC_REG_BASE + i);
  }
  //printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.println(F("ETH: connect"));
  ethernetConnect();

  Serial.println(F("MQTT: connect"));
  mqtt.setServer(mqttIP, 1883);
  mqtt.setCallback(mqttCallback);
  mqttConnect();
  //mqttSetupSubscriptions();


  // Calibration factor = CT ratio / burden resistance = (100A / 0.05A) / 33 Ohms = 60.606
  ct1.current(CURRENTSENSOR1PIN, CURRENTSENSOR1RATIO / CURRENTSENSOR1BURDEN );
  ct2.current(CURRENTSENSOR2PIN, CURRENTSENSOR2RATIO / CURRENTSENSOR2BURDEN );
  ct3.current(CURRENTSENSOR3PIN, CURRENTSENSOR3RATIO / CURRENTSENSOR3BURDEN );
  ct4.current(CURRENTSENSOR4PIN, CURRENTSENSOR4RATIO / CURRENTSENSOR4BURDEN );

  // (ADC input, calibration, phase_shift)
  ct1.voltage(VOLTAGESENSORPIN, VOLTAGESENSORCAL, VOLTAGESENSORPHASESHIFT);
  ct2.voltage(VOLTAGESENSORPIN, VOLTAGESENSORCAL, VOLTAGESENSORPHASESHIFT);
  ct3.voltage(VOLTAGESENSORPIN, VOLTAGESENSORCAL, VOLTAGESENSORPHASESHIFT);
  ct4.voltage(VOLTAGESENSORPIN, VOLTAGESENSORCAL, VOLTAGESENSORPHASESHIFT);

  // Setup indicator LED
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
}

void loop()
{
  // Calculate all. No.of crossings, time-out
  ct1.calcVI(MEASUREMENTCROSSINGS, 2000);
  ct2.calcVI(MEASUREMENTCROSSINGS, 2000);
  ct3.calcVI(MEASUREMENTCROSSINGS, 2000);
  ct4.calcVI(MEASUREMENTCROSSINGS, 2000);

  // Print power
  Serial.print(ct1.realPower);
  Serial.print(" ");
  Serial.print(ct2.realPower);
  Serial.print(" ");
  Serial.print(ct3.realPower);
  Serial.print(" ");
  Serial.print(ct4.realPower);
  Serial.print(" ");
  Serial.print(ct1.Vrms);
  Serial.println();

  // Available properties: ct1.realPower, ct1.apparentPower, ct1.powerFactor, ct1.Irms and ct1.Vrms

  delay(5000);
}

/*-------- I2C ----------*/
byte readI2CRegister(byte i2c_address, byte reg)
{
  unsigned char v;
  Wire.beginTransmission(i2c_address);
  Wire.write(reg);  // Register to read
  Wire.endTransmission();

  Wire.requestFrom(i2c_address, (uint8_t)1); // Read a byte
  while(!Wire.available()) { }  // DANGER - INFINITE LOOP POSSIBLE
  v = Wire.read();
  return v;
}

/*-------- Ethernet interaction ----------*/
boolean ethernetConnect()
{
  byte status = Ethernet.begin(mac);
  if(status == 1)
  {
    Serial.print(F("IP: "));
    Serial.println(Ethernet.localIP());
  }
  else
  {
    Serial.println(F("ETH: No conn"));
  }

  return status == 1 ? true : false;
}

/*-------- MQTT broker interaction ----------*/
boolean mqttConnect()
{
  //boolean success = mqtt.connect(commonSettings.deviceName, MQTT_USERNAME, MQTT_PASSWORD, commonSettings.mqttWillTopic, mqttWillQos, mqttWillRetain, commonSettings.mqttWillMessage);
  boolean success = false;
  if (success)
  {
    Serial.println(F("MQTT: Conn good"));
    // publish retained LWT so anything listening knows we are alive
    const byte data[] = {"connected"};
    mqtt.publish(commonSettings.mqttWillTopic, data, 1, mqttWillRetain);
  }
  else
  {
    Serial.println(F("MQTT: Conn failed"));
  }
  return success;
}

/*-------- MQTT setup ----------*/
void mqttSetupSubscriptions()
{
  // These get bikeshed/ prepended inside subscribe
  mqttSubscribe("request");
  mqttSubscribe("+/set");
}

void mqttSubscribe(const char* name)
{
  // build the MQTT topic: mqttTopicBase/name
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%s", commonSettings.mqttTopicBase, name);

  Serial.print(F("MQTT: sub: "));
  Serial.println(topic);

  // publish to the MQTT broker
  mqtt.subscribe(topic);
}

// Prepends the MQTT topic: mqttTopicBase/
void mqttPublish(const char* name, const char* payload)
{
  mqttPublish(name, payload, false);
}

// Prepends the MQTT topic: mqttTopicBase/
void mqttPublish(const char* name, const char* payload, bool retained)
{
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%s", commonSettings.mqttTopicBase, name);
  mqttPublishRaw(topic, payload, retained);
}

// Lower-level publisher; just sends it on out
void mqttPublishRaw(const char* name, const char* payload, bool retained)
{
  Serial.print(F("MQTT: "));
  Serial.print(name);
  Serial.print(F(" "));
  Serial.print(payload);
  if (retained)
  {
    Serial.print(F(" (retained)"));
  }
  Serial.println("");

  // publish to the MQTT broker
  boolean success = mqtt.publish(name, payload, retained);
  if(!success)
  {
    Serial.print(F("MQTT: pub failed, state: "));
    Serial.println(mqtt.state());
    mqttFailCount++;
  }
  else
  {
    mqttFailCount = 0;
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  bool updateRetained = false;

  // Allocate the correct amount of memory for the payload copy
  char* p = (char*)malloc(length + 1);

  // Copy the payload to the new buffer
  for (uint8_t i = 0; i < length; i++)
  {
    p[i] = (char)payload[i];
  }
  p[length] = '\0';

  Serial.print(F("MQTT: rx: "));
  Serial.print(topic);
  Serial.print(F(" => "));
  Serial.print(p);
  Serial.println();

  // Strip off the "bikeshed/" bit
  char* topicStrip = topic + strlen(commonSettings.mqttTopicBase) + 1;

  if (strcmp(topicStrip , "request") == 0)
  {
    Serial.println(F("MQTT: request received"));
    if (strcmp(p, "status") == 0)
    {
      Serial.println(F("MQTT: status tx"));
    }
    updateRetained = true;
  }
  else if (strcmp(topicStrip, "retain/set") == 0)
  {
    Serial.println(F("MQTT: Retaining settings in EEPROM"));
    EEPROM.updateBlock(0, commonSettings);
    //EEPROM.updateBlock(512, specificSettings);
    updateRetained = true;
  }
  else
  {
    Serial.println(F("MQTT: ??"));
  }

  free(p);

  if (updateRetained)
  {
    //publishAllRetained();
  }
}
