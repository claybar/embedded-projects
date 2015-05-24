/*
TMonESP_MQTT  Temperature via MQTT with ESP8266
https://github.com/claybar/home-automation/

Samples the temperature of up to 4 DS18B20 sensors and reports using MQTT over WiFi

Buildpack required:
  - Arduino >1.6.4 http://www.arduino.cc/en/Main/Software
  - ESP build environment https://github.com/esp8266/Arduino
      Install via Board Manager by following the instructions in main README.md 
      using http://arduino.esp8266.com/package_esp8266com_index.json

Libraries required:
  - DallasTemperature https://github.com/milesburton/Arduino-Temperature-Control-Library/archive/master.zip
  - PubSubClient (MQTT) https://github.com/Imroy/pubsubclient/archive/master.zip

Hardware required:
  - ESP8266 module
  - DS18B20 temperature sensors connected to GPIO2 with pullup to VCC
  - Usual set of IO pull up/downs for programming/running ESP module

Network config required:
  - Edit Secrets.h with network credentials and IP assignments.
  Network and MQTT status is monitored indirectly by counting the number of sequential MQTT transmission 
  failures.  If this count exceeds a threshold (TX_FAIL_LIMIT) the processor is rebooted. 

MQTT messages
  MQTT messages are set to the topic "/ESP8266_DEVICEID/temperature/instant/ONEWIREID.c" with a string 
  payload of the temperature where:
    - DEVICEID is the unique (hopefully) number of the ESP module
    - ONEWIREID is the unique one-wire serial number assigned by Maxim

Temperature sampling and transmission periods  (NOT YET FULLY IMPLEMENTED)
  The sampling and transmission periods can be set independently.  At each transmission the average of all 
  temperature samples taken within the transmission is calculated and sent.  To disable averaging over the
  time period, set SAMPLE_PERIOD = MQTT_TX_PERIOD
  
TODO:
  - Implement temperature averaging using an accumulator.  Need to make sure fresh sample is sent when
    sample and tx period are equal.  If the tx fires just before the sample ticker there will be a delay
    equal to the ticker period.
  - Subscribe to messages from MQTT to configure things.  Also implement a config sending trigger.
*/

#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 12

#define TX_FAIL_LIMIT 2

#define SAMPLE_PERIOD 5.0
#define MQTT_TX_PERIOD 20.0


#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ticker.h>
#include <MQTT.h>
#include <PubSubClient.h>

#include "Secrets.h"

IPAddress MQTTServer(MQTT_SERVER_IP);

// OneWire temperature sensors
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// arrays to hold up to 4 device addresses
DeviceAddress sensorAddresses[4];
int sensorCount;

// State machine start triggers and flags
Ticker Sample;
bool TriggerSample;
Ticker Tx;
bool TriggerTx;

// Counter for failed transmissions.  Used to reset processor after repeated failures.
int txFailCount;

PubSubClient client(MQTTServer);

String DeviceName = "ESP8266_" + String(ESP.getChipId());

void callback(const MQTT::Publish& pub) {
  // handle message arrived
}

// Helper functions for state machine triggers
void SetSampleTrigger() { TriggerSample = true; }
void ResetSampleTrigger() { TriggerSample = false; }
void SetTxTrigger() { TriggerTx = true; }
void ResetTxTrigger() { TriggerTx = false; }

void wifiConnect()
{
  Serial.print("Connecting to AP");
  WiFi.begin(AP_SSID, AP_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Diagnostic info:");
  WiFi.printDiag(Serial);
}

void setup(void)
{
  // start serial port
  Serial.begin(115200);
  Serial.println("TMonESP_MQTT - Temperature via MQTT over WiFi from an ESP8266 module");
  Serial.println("https://github.com/claybar/home-automation/");
  
  // MQTT setup
  client.set_callback(callback);
  // Set up subscriptions
  //client.subscribe("testingIn");
  
  wifiConnect();

  Serial.print("Device Name: ");
  Serial.println(DeviceName);
  
  // Start up the dallas temperature library
  sensors.begin();

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensorCount = sensors.getDeviceCount();
  Serial.print("Found ");
  Serial.print(sensorCount, DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  for (int i = 0; i < sensorCount; i++)
  {
    if (!sensors.getAddress(sensorAddresses[i], i))
    {
      Serial.print("Unable to find address for Device ");
      Serial.println(i);
    }
    else
    {
      // show the addresses we found on the bus
      Serial.print("Device ");
      Serial.print(i);
      Serial.print(" Address: ");
      printAddress(sensorAddresses[i]);
      Serial.println();

      // set the resolution to 9 bit
      sensors.setResolution(sensorAddresses[i], TEMPERATURE_PRECISION);

      Serial.print("Device ");
      Serial.print(i);
      Serial.print(" Resolution: ");
      Serial.print(sensors.getResolution(sensorAddresses[i]), DEC); 
      Serial.println();
    }
  }

  // Configure the main triggers
  Sample.attach(SAMPLE_PERIOD, SetSampleTrigger);
  Tx.attach(MQTT_TX_PERIOD, SetTxTrigger);
}

String addressAsString(DeviceAddress deviceAddress)
{
  String s;
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16)
    {
      s = s + String("0");
    }
    s = s + String(deviceAddress[i], HEX);
  }
  
  return s;
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

String temperatureAsString(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  return String(tempC);
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();    
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}

void loop(void)
{ 
  if (TriggerSample)
  {
    ResetSampleTrigger();
    Serial.println("Trigger: Sensor sampling");
    
    // call sensors.requestTemperatures() to issue a global temperature 
    // request to all devices on the bus
    Serial.print("Requesting temperatures...");
    sensors.requestTemperatures();
    delay(1000);  // Wait for 1000ms to ensure conversion is complete
    Serial.println("DONE");

    // print the device information
    for (int i = 0; i < sensorCount; i++)
    {
      printData(sensorAddresses[i]);
    }
  }
  
  if (TriggerTx)
  {
    ResetTxTrigger();
    Serial.println("Trigger: Transmit");
    
    if (!client.connected())
    {
      Serial.print("MQTT connecting...");
      client.connect(DeviceName);
      if (client.connected())
      {
        Serial.println("SUCCESS");
      }
      else
      {
        Serial.println("FAILED");
      }
    }
    
    // Send messages via MQTT
    if (client.connected())
    {
      for (int i = 0; i < sensorCount; i++)
      {
        String mqttTopic = "/" + DeviceName + "/temperature/instant/" + addressAsString(sensorAddresses[i]) + ".c";
        String mqttPayload = temperatureAsString(sensorAddresses[i]);
        Serial.print("MQTT: ");
        Serial.print(mqttTopic);
        Serial.print(" ");
        Serial.println(mqttPayload);
        client.publish(mqttTopic, mqttPayload);
        // Reset the failed tx counter
        txFailCount = 0;
      }
    }
    else
    {
      txFailCount++;
      Serial.print("MQTT connect failed. Count = ");
      Serial.println(txFailCount);
    }
  }
  
  if (txFailCount > TX_FAIL_LIMIT)
  {
    // Something is preventing data transmission.  Reset processor to try again
    Serial.println("TX fail limit reached, resetting processor.");
    ESP.reset();
  }
  
  client.loop();
}
