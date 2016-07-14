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

#include <Arduino.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <Wire.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <SerialCommand.h>
#include <EEPROMex.h>

#include <elapsedMillis.h>
#include <Settings.h>
#include <Secrets.h>

#include "BikeShedController.h"

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
#define MOTION HIGH
#define DOOROPEN HIGH
#define ON true
#define OFF false

// MQTT setup
const int mqttWillQos = 0;
const int mqttWillRetain = 1;
int mqttFailCount = 0;
int mqttDisconnectedCount = 0;
const int mqttFailCountLimit = 5;
const int mqttDisconnectedCountLimit = 5;

// Settings storage
commonSettings0_t commonSettings;
bikeshedControllerSettings0_t specificSettings;

// Used for short-term string building
char tmpBuf[48];

// Used to redirect stdout to the serial port
FILE serial_stdout;

// Storage, will be set by onboard i2c device and DHCP
byte mac[] = { 0, 0, 0, 0, 0, 0 };
IPAddress mqttIP(MQTT_SERVER_IP);

// State control variables
bool motionAState = false;
bool motionBState = false;
bool doorState = false;
bool previousMotionAState = false;
bool previousMotionBState = false;
bool previousDoorState = false;
bool recentActivity = false;    // Used to track how long since motion has been detected
uint8_t sunlightLevel;

elapsedMillis motionTimer;
unsigned long timerPrevious;

// Hardware and protocol handlers
EthernetClient ethernet;
PubSubClient mqtt(ethernet);
SerialCommand serialCmd;

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  // Allocate the correct amount of memory for the payload copy
  char* p = (char*)malloc(length + 1);

  // Copy the payload to the new buffer
  for (uint8_t i = 0; i < length; i++)
  {
    p[i] = (char)payload[i];
  }
  p[length] = '\0';

  Serial.print(F("MQTT: rx "));
  Serial.print(topic);
  Serial.print(F(" => "));
  Serial.print(p);
  Serial.println();

  if (strcmp(topic, "bikeshed/request") == 0)
  {
    Serial.println(F("MQTT: request received"));
    if (strcmp(p, "status") == 0)
    {
      Serial.println(F("MQTT: status tx"));
    }

    snprintf(tmpBuf, sizeof(tmpBuf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    mqttPublish("mac", tmpBuf);

    IPAddress ip = Ethernet.localIP();
    snprintf(tmpBuf, sizeof(tmpBuf), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    mqttPublish("ip", tmpBuf);

    snprintf(tmpBuf, sizeof(tmpBuf), "%d", commonSettings.version);
    mqttPublish("commonsettingsversion", tmpBuf);
    snprintf(tmpBuf, sizeof(tmpBuf), "%d", specificSettings.version);
    mqttPublish("specificsettingsversion", tmpBuf);
    snprintf(tmpBuf, sizeof(tmpBuf), "%lu", specificSettings.outsideLightAfterMotionTime / 1000);
    mqttPublish("floodoffdelay", tmpBuf);
    snprintf(tmpBuf, sizeof(tmpBuf), "%d", specificSettings.sunlightThreshold);
    mqttPublish("sunlightthreshold", tmpBuf);
    snprintf(tmpBuf, sizeof(tmpBuf), "%d", specificSettings.insideLightsBrightness);
    mqttPublish("insidelightsbrightness", tmpBuf);
    snprintf(tmpBuf, sizeof(tmpBuf), "%d", specificSettings.outsideLightsBrightness);
    mqttPublish("outsidelightsbrightness", tmpBuf);
  }
  else if (strcmp(topic, "bikeshed/set/floodoffdelay") == 0)
  {
    Serial.print(F("MQTT: setfloodoffdelay = "));
    Serial.print(p);
    Serial.println(F("s"));
    specificSettings.outsideLightAfterMotionTime = atol(p) * 1000;
  }
  else if (strcmp(topic, "bikeshed/set/sunlightthreshold") == 0)
  {
    int v = atoi(p);
    if (v >= 0 && v <= 1023)
    {
      Serial.print(F("MQTT: sunlightthreshold = "));
      Serial.println(p);
      specificSettings.sunlightThreshold = v;
    }
    else
    {
      Serial.print(F("MQTT: error, sunlightthreshold = "));
      Serial.println(p);
    }
  }
  else if (strcmp(topic, "bikeshed/set/retain") == 0)
  {
    Serial.print(F("MQTT: Retaining settings in EEPROM"));
    EEPROM.updateBlock(0, commonSettings);
    EEPROM.updateBlock(512, specificSettings);
  }
  else if (strcmp(topic, "bikeshed/set/insidelightsbrightness") == 0)
  {
    int v = atoi(p);
    if (v >= 0 && v <= 255)
    {
      Serial.print(F("MQTT: insidelightsbrightness = "));
      Serial.println(p);
      specificSettings.insideLightsBrightness = v;
      // Apply to the light output immediately if the door is open
      if (doorState == DOOROPEN)
      {
        analogWrite(INSIDELIGHTSPIN, v);
      }
    }
    else
    {
      Serial.print(F("MQTT: error, insidelightsbrightness = "));
      Serial.println(p);
    }
  }
  else if (strcmp(topic, "bikeshed/set/outsidelightsbrightness") == 0)
  {
    int v = atoi(p);
    if (v >= 0 && v <= 255)
    {
      Serial.print(F("MQTT: outsidelightsbrightness = "));
      Serial.println(p);
      specificSettings.outsideLightsBrightness = v;
      // Apply to the light output immediately if there is current activity
      if (recentActivity)
      {
        analogWrite(OUTSIDELIGHTSPIN, v);
      }
    }
    else
    {
      Serial.print(F("MQTT: error, outsidelightsbrightness = "));
      Serial.println(p);
    }
  }
  else
  {
    Serial.println(F("MQTT: ??"));
  }

  free(p);
}

// Function that printf and related will use to print to the serial port
int serial_putchar(char c, FILE* f) {
    if (c == '\n') serial_putchar('\r', f);
    return Serial.write(c) == 1? 0 : 1;
}

void setup()
{
  // Pin IO setup
  pinMode(MOTIONSENSORAPIN, INPUT);  // 12k pulldown resistor
  pinMode(MOTIONSENSORBPIN, INPUT);  // 12k pulldown resistor
  pinMode(DOORSENSORPIN, INPUT_PULLUP);  // Also 12k pullup resistor
  pinMode(ACTIVITYLEDPIN, OUTPUT);
  pinMode(INSIDELIGHTSPIN, OUTPUT);
  pinMode(OUTSIDELIGHTSPIN, OUTPUT);

  // ADC reference connected to 2.048V
  analogReference(EXTERNAL);

  // ensure the watchdog is disabled for now
  wdt_disable();

  Serial.begin(115200);
  // Redirect stdout to the serial port helper
  fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &serial_stdout;

  // initialise busses: SPI, i2c, 1-wire temperature.
  SPI.begin();
  Wire.begin();

  // Read settings from EEPROM
  uint8_t settingsVer = EEPROM.readByte(0);
  Serial.print(F("EEP: Read common settings ver: "));
  Serial.println(settingsVer);
  if (settingsVer == 0)
  {
    EEPROM.readBlock(0, commonSettings);
  }
  else
  {
    Serial.println(F("EEP: Default common settings"));
    strcpy(commonSettings.deviceName, "bikeshed");
    strcpy(commonSettings.mqttTopicBase, "bikeshed");
    strcpy(commonSettings.mqttWillTopic, "clients/bikeshed");
    strcpy(commonSettings.mqttWillMessage, "unexpected exit");
  }

  settingsVer = EEPROM.readByte(512);
  Serial.print(F("EEP: Read specific settings ver: "));
  Serial.println(settingsVer);
  if (settingsVer == 0)
  {
    EEPROM.readBlock(512, specificSettings);
  }
  else
  {
    Serial.println(F("EEP: Default specific settings"));
    specificSettings.outsideLightAfterMotionTime = 5000; // 5 * 60 * 1000;  // milliseconds
    specificSettings.sunlightThreshold = 511;  // Lights on if reading under this
    specificSettings.insideLightsBrightness = 127;
    specificSettings.outsideLightsBrightness = 127;
  }

  Serial.print(F("Device Name: "));
  Serial.println(commonSettings.deviceName);

  Serial.print(F("MAC: "));
  for (int i = 0 ; i < 6; i++)
  {
    mac[i] = readI2CRegister(MAC_I2C_ADDR, MAC_REG_BASE + i);
  }
  printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.println(F("ETH: connect"));
  ethernetConnect();

  Serial.println(F("MQTT: connect"));
  mqtt.setServer(mqttIP, 1883);
  mqtt.setCallback(mqtt_callback);
  mqttConnect();
  mqttSetupSubscriptions();

  // Setup callbacks for SerialCommand commands
  Serial.println(F("SER: setup"));
  serialCmd.addCommand("MQTT", serialMQTTRelay);  // Relay for MQTT message using 2 parameters
  serialCmd.setDefaultHandler(serialUnrecognized);

  Serial.println(F("ALM: Setup"));
  //Alarm.timerRepeat( 5 * 60, fiveMinsTimer);
  Alarm.timerRepeat( 6, statusUpdateTimer);  // Status sent 10x per minute

  // State initialisation
  recentActivity = false;
  motionTimer = 0;
  timerPrevious = 0;
}

void loop()
{
  motionAState = digitalRead(MOTIONSENSORAPIN);
  motionBState = digitalRead(MOTIONSENSORBPIN);
  doorState = digitalRead(DOORSENSORPIN);

  // Inside lights are simply controlled by the door sensor
  if (doorState != previousDoorState)
  {
    previousDoorState = doorState;

    if (doorState == DOOROPEN)
    {
      Serial.println(F("Door open"));
      mqttPublish("door", "open");
      insideLights(ON);
    }
    else
    {
      Serial.println(F("Door closed"));
      mqttPublish("door", "closed");
      insideLights(OFF);
    }
  }

  // While motion present or door is open, keep resetting the countdown timer.
  // Fire messages only on positive edges
  if (motionAState == MOTION || motionBState == MOTION || doorState == DOOROPEN)
  {
    // Reset timer and log presence of motion
    motionTimer = 0;
    timerPrevious = 0;

    // Fire messages on positive edges
    if (motionAState != previousMotionAState)
    {
      Serial.println(F("Motion detected, sensor A"));
      recentActivity = true;
      mqttPublish("motion", "detected-ch1");
      outsideLights(ON);
    }
    if (motionBState != previousMotionBState)
    {
      Serial.println(F("Motion detected, sensor B"));
      recentActivity = true;
      mqttPublish("motion", "detected-ch2");
      outsideLights(ON);
    }
  }
  else  // Sensors all inactive, start the countdown
  {
    // Test if there has been recent motion and it has been gone for a while
    if (recentActivity)
    {
      if (motionTimer > specificSettings.outsideLightAfterMotionTime)
      {
        recentActivity = false;

        Serial.println(F("Motion timer expired"));
        mqttPublish("motion", "gone");
        outsideLights(OFF);
      }
      else
      {
        if (motionTimer > timerPrevious + 1000)
        {
          Serial.print(motionTimer / 1000);
          Serial.print(" of ");
          Serial.println(specificSettings.outsideLightAfterMotionTime / 1000);

          timerPrevious += 1000;
        }
      }
    }
  }
  previousMotionAState = motionAState;
  previousMotionBState = motionBState;

  // Give all the worker tasks a bit of time
  Ethernet.maintain();
  serialCmd.readSerial();
  mqtt.loop();
  Alarm.delay(1);
}

/*-------- Hardware Abstraction ----------*/
// Lights only turn on if light level dark enough
void insideLights(bool state)
{
  bool testedState = OFF;
  // Only test the light level when turning on the lights
  if (state == ON && sunlightLevel < specificSettings.sunlightThreshold)
  {
    testedState = ON;
  }
  mqttPublish("insidelights", testedState == ON ? "on" : "off");
  //digitalWrite(INSIDELIGHTSPIN, testedState);
  if (testedState)
  {
    analogWrite(INSIDELIGHTSPIN, specificSettings.insideLightsBrightness);
  }
  else
  {
    analogWrite(INSIDELIGHTSPIN, 0);
  }
}

void outsideLights(bool state)
{
  bool testedState = OFF;
  // Only test the light level when turning on the lights
  if (state == ON && sunlightLevel < specificSettings.sunlightThreshold)
  {
    testedState = ON;
  }
  mqttPublish("outsidelights", testedState == ON ? "on" : "off");
  //digitalWrite(OUTSIDELIGHTSPIN, testedState);
  if (testedState)
  {
    analogWrite(OUTSIDELIGHTSPIN, specificSettings.outsideLightsBrightness);
  }
  else
  {
    analogWrite(OUTSIDELIGHTSPIN, 0);
  }
}

/*-------- MQTT broker interaction ----------*/
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
void mqttSetupSubscriptions()
{
  mqttSubscribe("request");
  mqttSubscribe("set/#");
}

boolean mqttConnect()
{
  boolean success = mqtt.connect(commonSettings.deviceName, MQTT_USERNAME, MQTT_PASSWORD, commonSettings.mqttWillTopic, mqttWillQos, mqttWillRetain, commonSettings.mqttWillMessage);
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
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/status/%s", commonSettings.mqttTopicBase, name);

  mqttPublishRelay(topic, payload);
}

void mqttPublishRelay(const char* name, const char* payload)
{
  Serial.print(F("MQTT: "));
  Serial.print(name);
  Serial.print(F(" "));
  Serial.println(payload);

  // publish to the MQTT broker
  boolean success = mqtt.publish(name, payload);
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

/*-------- Serial Monitor ----------*/
void serialMQTTRelay()
{
  //int aNumber;
  char *topic;
  char *payload;

  Serial.println(F("SER: rx"));
  topic = serialCmd.next();
  Serial.print(F("  Topic: "));
  if (topic != NULL)
  {
    Serial.println(topic);
  }
  else
  {
    Serial.println(F("  None"));
  }

  payload = serialCmd.next();
  Serial.print(F("  Payload: "));
  if (payload != NULL)
  {
    Serial.println(payload);
  }
  else
  {
    Serial.println(F("  None"));
  }

  if (topic != NULL && payload != NULL)
  {
    // Send a MQTT message
    mqttPublishRelay(topic, payload);
  }
}

// This gets set as the default handler, and gets called when no other command matches.
void serialUnrecognized(const char *command) {
  Serial.println(F("SER: ??"));
}


/*-------- Timers ----------*/
void statusUpdateTimer()
{
  Serial.println(F("TICK"));

  // Measure and store light level
  sunlightLevel = analogRead(LIGHTSENSORPIN);
  snprintf(tmpBuf, sizeof(tmpBuf), "%d", sunlightLevel);
  mqttPublish("sunlight", tmpBuf);

  // Report voltages
  // Voltage reference connected to external 2.048V reference
  // Each rail via a divider of 1: 4,8, or 16
  int rail5VmV = analogRead(VOLTAGE5PIN) * 2 * 4;
  snprintf(tmpBuf, sizeof(tmpBuf), "%d.%d", rail5VmV / 1000, rail5VmV % 1000);
  mqttPublish("5V", tmpBuf);
  int rail9VmV = analogRead(VOLTAGE9PIN) * 2 * 8;
  snprintf(tmpBuf, sizeof(tmpBuf), "%d.%d", rail9VmV / 1000, rail9VmV % 1000);
  mqttPublish("9V", tmpBuf);
  int rail12VmV = analogRead(VOLTAGE12PIN) * 2 * 8;
  snprintf(tmpBuf, sizeof(tmpBuf), "%d.%d", rail12VmV / 1000, rail12VmV % 1000);
  mqttPublish("12V", tmpBuf);
  int rail24VmV = analogRead(VOLTAGE24PIN) * 2 * 16;
  snprintf(tmpBuf, sizeof(tmpBuf), "%d.%d", rail24VmV / 1000, rail24VmV % 1000);
  mqttPublish("24V", tmpBuf);

  // Something is wrong with MQTT
  if (!ethernet.connected())
  {
    // Restart ethernet
    Serial.println(F("ETH: Restart"));
    ethernetConnect();
  }

  if (errorMonitorMQTT() > 0)
  {
    // Restart MQTT
    Serial.println(F("MQTT: restart"));
    mqttConnect();
    mqttSetupSubscriptions();
  }
}

/*-------- Error detection ----------*/
#define ERROR_NONE              0x00
#define ERROR_MQTT_DISCONNECTED 0x01
#define ERROR_MQTT_TXFAIL       0x02

byte errorMonitorMQTT()
{
  byte errorCode = ERROR_NONE;

  if(!mqtt.connected())
  {
    mqttDisconnectedCount++;
    Serial.print(F("MQTT: discon, state: "));
    Serial.print(mqtt.state());
    Serial.print(", count: ");
    Serial.println(mqttDisconnectedCount);
  }
  else
  {
    mqttDisconnectedCount = 0;
  }

  if (mqttDisconnectedCount > mqttDisconnectedCountLimit)
  {
    Serial.println(F("MQTT: discon"));
    errorCode += ERROR_MQTT_DISCONNECTED;
  }

  if (mqttFailCount > mqttFailCountLimit)
  {
    Serial.println(F("MQTT: pub fail"));
    errorCode += ERROR_MQTT_TXFAIL;
  }

  return errorCode;
}