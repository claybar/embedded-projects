#ifndef HomeAutomationHelper_h
#define HomeAutomationHelper_h

#include "Arduino.h"

boolean mqttConnect() 
{
  boolean success = mqtt.connect(mqttClientId, MQTT_USERNAME, MQTT_PASSWORD, mqttWillTopic, mqttWillQos, mqttWillRetain, mqttWillMessage); 
  if (success) {
    Serial.println ("Successfully connected to MQTT broker ");
    // publish retained LWT so anything listening knows we are alive
    byte data[] = { "connected" };
    mqtt.publish(mqttWillTopic, data, 1, mqttWillRetain);
  } else {
    Serial.println ("Failed to connect to MQTT broker");
  }
  return success;
}

void mqttSubscribe(char* name)
{
  // build the MQTT topic
  // mqttTopicBase/MAC/name
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%02X:%02X:%02X:%02X:%02X:%02X/%s", mqttTopicBase, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],name);

  Serial.print("mqttTopicBase: ");
  Serial.println(mqttTopicBase);

  Serial.print("Subscribing to: ");
  Serial.print(topic);
  Serial.println();

  // publish to the MQTT broker 
  mqtt.subscribe(topic);
}

void mqttPublish(char* name, char* payload)
{
  // build the MQTT topic
  // mqttTopicBase/MAC/name
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%02X:%02X:%02X:%02X:%02X:%02X/%s", mqttTopicBase, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],name);

  Serial.print(topic);
  Serial.print(" ");
  Serial.println(payload);

  // publish to the MQTT broker 
  mqtt.publish(topic, payload);
}

byte readI2CRegister(byte i2c_address, byte reg)
{
  unsigned char v;
  Wire.beginTransmission(i2c_address);
  Wire.write(reg);  // Register to read
  Wire.endTransmission();

  Wire.requestFrom(i2c_address, (uint8_t)1); // Read a byte
  while(!Wire.available())
  {
    // Wait
  }
  v = Wire.read();
  return v;
} 

#endif
