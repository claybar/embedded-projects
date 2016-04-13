/*
Overall operation implemeted as a FSM.

States:
- Daytime (LEDs always off)
- Dusk (LEDs on dimly)
- Dawn (LEDs on dimly)
- Night (LEDs off)
- Motion (LEDs on bright)
- Alarm (LEDs flashing)

Transitions clock:
Daytime -> Dusk
Dusk -> Night
Night -> Dawn
Dawn -> Daytime

Within states [Dusk, Night, Dawn] motion
*/

#include <avr/wdt.h>
#include <Arduino.h>         // may not be needed?
#include <SPI.h>
#include <Wire.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#include <LEDFader.h>
#include <Curve.h>

#include "Secrets.h"

#define LIGHTINGPIN 9
#define ACTIVITYLEDPIN 13
#define MOTIONSENSORAPIN 7
#define MOTIONSENSORBPIN 6

#define MAC_I2C_ADDR 0x50

// Storage, will be set by onboard i2c device and DHCP
byte mac[] = { 0, 0, 0, 0, 0, 0 };
byte ip[] = { 0, 0, 0, 0 };

// Defaults, some can be set later via MQTT
int lightingLevelOff = 0; // percentage
int lightingLevelAmbient = 10; // percentage
int lightingLevelBright = 50;  // percentage
int lightingChangeTime = 5000;  // milliseconds

int motionAState = 0;
int motionBState = 0;
int previousMotionAState = 0;
int previousMotionBState = 0;

// Counters for tracking failed transmissions.  Used to reset processor after repeated failures.
int txFailCount;
int txFailCountTotal;

// Hardware and protocol handlers
LEDFader lightingLed = LEDFader(LIGHTINGPIN);

EthernetClient ethernet;

// MQTT setup
char mqttClientId[] = "lighting";

// publish to "lighting/MAC/<variable>".
char mqttTopicBase[] = "frontsteps";
char mqttTopicLwt[] = "clients/frontsteps";
int  mqttLwtQos = 0;
int  mqttLwtRetain = 1;

PubSubClient mqtt(MQTT_SERVER_IP, 1883, mqtt_callback, ethernet);

void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
  // Allocate the correct amount of memory for the payload copy
  char* p = (char*)malloc(length + 1);
  // Copy the payload to the new buffer
  //memcpy(p,payload,length);
  
  Serial.print("MQTT: message received: ");
  Serial.print(topic);
  Serial.print(" => ");
  for (int i = 0; i < length; i++)
  {
    p[i] = (char)payload[i];
    //Serial.print((char)payload[i]);
  }
  p[length] = '\0';
  Serial.print(p);
  Serial.println();

  if (topic == "frontsteps/00:04:A3:D3:30:78/request")
  {
    Serial.println("MQTT: request received");
    if (p == "status")
    {
      Serial.println("MQTT: Sending status");
    }
    // Send a bunch of MQTT messages containing status
    /*
    IPAddress ip = WiFi.localIP();
    client.publish(mqttTopicBase + "ip", String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]));
    client.publish(mqttTopicBase + "freeheap", String(ESP.getFreeHeap()));
    client.publish(mqttTopicBase + "txfailcount", String(txFailCountTotal));
    client.publish(mqttTopicBase + "sensorcount", String(sensorCount));
    client.publish(mqttTopicBase + "sensorperiod", String(SAMPLE_PERIOD));
    client.publish(mqttTopicBase + "txperiod", String(MQTT_TX_PERIOD));
    */
  }
  else if (topic == "setdimmedlevel")
  {

  }
  else
  {
    Serial.println("  Unrecognised message");
  }

  free(p);
}


void setup()
{
  // Pin IO setup
  pinMode(MOTIONSENSORAPIN, INPUT);
  pinMode(MOTIONSENSORBPIN, INPUT);
  pinMode(ACTIVITYLEDPIN, OUTPUT);
  pinMode(LIGHTINGPIN, OUTPUT);

  // ensure the watchdog is disabled for now
  wdt_disable();
  
  // More linear illumination for human eyes
  lightingLed.set_curve(&Curve::exponential);

  Serial.begin(115200);
  Serial.println();
  Serial.println("Sketch ID: lighting_mqtt.ino");
  
  // initialise the SPI bus.  
  SPI.begin();

  // Join i2c bus as master
  Wire.begin();

  Serial.print ("MAC address: ");
  mac[0] = readI2CRegister(MAC_I2C_ADDR, 0xFA);
  mac[1] = readI2CRegister(MAC_I2C_ADDR, 0xFB);
  mac[2] = readI2CRegister(MAC_I2C_ADDR, 0xFC);
  mac[3] = readI2CRegister(MAC_I2C_ADDR, 0xFD);
  mac[4] = readI2CRegister(MAC_I2C_ADDR, 0xFE);
  mac[5] = readI2CRegister(MAC_I2C_ADDR, 0xFF);
  char tmpBuf[17];
  sprintf(tmpBuf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(tmpBuf);
    
  Serial.print ("IP address: ");
  // get an IP address
  while (Ethernet.begin(mac) != 1)
  {
    delay(5000);
    Serial.print (".");
  }
  Serial.println(Ethernet.localIP());

  Serial.println ("Connecting to MQTT broker...");
  // connect to our MQTT broker
  while (!mqttConnect())
  {
    delay(5000);
  }

  // Subscribe to topics
  mqttSubscribe("request");

}

void loop()
{
  motionAState = digitalRead(MOTIONSENSORAPIN);
  motionBState = digitalRead(MOTIONSENSORBPIN);

  // While motion is present, keep reseting the countdown timer.
  // Fire messages only on positive edges

  if (motionAState == HIGH)
  {
    // Reset countdown timer

    // Fire messages on positive edges
    if (motionAState != previousMotionAState)
    { 
      Serial.println("Motion detected, sensor A");

      mqttPublish("motion1", "detected");
    }
  }
  
  if (motionBState == HIGH)
  {
    // Reset countdown timer

    if (motionBState != previousMotionBState)
    {
      Serial.println("Motion detected, sensor B");

      mqttPublish("motion2", "detected");
    }
  }
  
  previousMotionAState = motionAState;
  previousMotionBState = motionBState;

  // Give all the worker tasks a bit of time
  lightingLed.update();
  mqtt.loop();
}


void lightingOff()
{
  lightingLed.fade(lightingLevelOff, lightingChangeTime);
}

void lightingAmbient()
{
  lightingLed.fade(lightingLevelAmbient, lightingChangeTime);
}

void lightingBright()
{
  lightingLed.fade(lightingLevelBright, lightingChangeTime);
}

boolean mqttConnect() 
{
  boolean success = mqtt.connect(mqttClientId, MQTT_USERNAME, MQTT_PASSWORD, mqttTopicLwt, mqttLwtQos, mqttLwtRetain, "0"); 
  if (success) {
    Serial.println ("Successfully connected to MQTT broker ");
    // publish retained LWT so anything listening knows we are alive
    byte data[] = { "1" };
    mqtt.publish(mqttTopicLwt, data, 1, mqttLwtRetain);
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
  snprintf(topic, 64, "%s/%02X:%02X:%02X:%02X:%02X:%02X/%s", mqttTopicBase, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],name);

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
  snprintf(topic, 64, "%s/%02X:%02X:%02X:%02X:%02X:%02X/%s", mqttTopicBase, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],name);

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


