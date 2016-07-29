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
#include <EtherTen.h>
#include <SPI.h>
#include <Wire.h>
#include <Ethernet.h>
#include <PubSubClient.h>
//#include <LEDFader.h>
#include <Curve.h>
#include <elapsedMillis.h>
#include <EEPROMex.h>
#include <Timezone.h>

#include <Settings.h>
#include <Secrets.h>

#include "FrontStepsLighting.h"
#include "Curve.h"

// Hardware setup
#define LIGHTINGPIN 9
#define ACTIVITYLEDPIN 13
#define MOTIONSENSORAPIN 7
#define MOTIONSENSORBPIN 6

// MQTT setup
//#define mqttClientId "frontsteps"
//#define mqttTopicBase "frontsteps"
//#define mqttWillTopic "clients/frontsteps"
//#define mqttWillMessage "unexpected exit"
const int mqttWillQos = 0;
const int mqttWillRetain = 1;
int mqttFailCount = 0;
int mqttDisconnectedCount = 0;
const int mqttFailCountLimit = 5;
const int mqttDisconnectedCountLimit = 5;

// Settings storage
commonSettings0_t commonSettings;
frontstepsControllerSettings0_t specificSettings;

// Used for short-term string building
char tmpBuf[48];
// Used to redirect stdout to the serial port
FILE serial_stdout;

// Storage, will be set by onboard i2c device and DHCP
byte mac[] = { 0, 0, 0, 0, 0, 0 };
IPAddress mqttIP(MQTT_SERVER_IP);

int motionAState = 0;
int motionBState = 0;
int previousMotionAState = 0;
int previousMotionBState = 0;
bool recentActivity = false;    // Used to track how long since motion has been detected
partsOfDay_t portionOfDay = daytime;    // Used to track how long since motion has been detected
unsigned long timerPrevious;

// Counters for tracking failed transmissions.  Used to reset processor after repeated failures.
int txFailCount;
int txFailCountTotal;

// Used to track how long since motion has been detected
elapsedMillis motionTimer;

Timezone nzTZ(nzdt, nzst);

// Hardware and protocol handlers
EthernetClient ethernet;
EthernetUDP udp;
PubSubClient mqtt(ethernet);
Curve ledCurve;

const int NTP_PACKET_SIZE = 48;
IPAddress ntpIP(NTP_SERVER_IP);

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  // Allocate the correct amount of memory for the payload copy
  char* p = (char*)malloc(length + 1);

  // Copy the payload to the new buffer
  for (unsigned int i = 0; i < length; i++)
  {
    p[i] = (char)payload[i];
  }
  p[length] = '\0';

  Serial.print(F("MQTT: message received: "));
  Serial.print(topic);
  Serial.print(F(" => "));
  Serial.print(p);
  Serial.println();

  if (strcmp(topic, "frontsteps/request") == 0)
  {
    Serial.println(F("MQTT: request received"));
    if (strcmp(p, "status") == 0)
    {
      Serial.println(F("MQTT: Sending status"));
    }

    snprintf(tmpBuf, sizeof(tmpBuf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    mqttPublish("mac", tmpBuf);

    IPAddress ip = Ethernet.localIP();
    snprintf(tmpBuf, sizeof(tmpBuf), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    mqttPublish("ip", tmpBuf);
  }
  else if (strcmp(topic, "setdimmedlevel") == 0)
  {

  }
  else
  {
    Serial.println(F(" -> Unrecognised message"));
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
  pinMode(MOTIONSENSORAPIN, INPUT);
  pinMode(MOTIONSENSORBPIN, INPUT);
  pinMode(ACTIVITYLEDPIN, OUTPUT);
  pinMode(LIGHTINGPIN, OUTPUT);

  // ensure the watchdog is disabled for now
  wdt_disable();

  Serial.begin(115200);
  // Redirect stdout to the serial port helper
  fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &serial_stdout;

  // initialise the SPI and i2c bus.
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
    strcpy(commonSettings.deviceName, "frontsteps");
    strcpy(commonSettings.mqttTopicBase, "frontsteps");
    strcpy(commonSettings.mqttWillTopic, "clients/frontsteps");
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
    specificSettings.lightingAfterMotionTime = 5000; // 5 * 60 * 1000;  // milliseconds
    specificSettings.lightingLevelOff = 0;
    specificSettings.lightingLevelAmbient = 20;
    specificSettings.lightingLevelBright = 80;
    //specificSettings.lightingChangeTime = 500;  // milliseconds
  }

  Serial.print(F("Device Name: "));
  Serial.println(commonSettings.deviceName);

  Serial.print(F("MAC: "));
  for (int i = 0 ; i < 6; i++)
  {
    mac[i] = readI2CRegister(MAC_I2C_ADDR, MAC_REG_BASE + i);
  }
  printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.print(F("IP: "));
  while (Ethernet.begin(mac) != 1)
  {
    delay(5000);
    Serial.print(F("."));
  }
  Serial.println(Ethernet.localIP());

  // Set up callback function of time libary to use NTP
  Serial.println(F("NTP: time callback"));
  udp.begin(8888);  // Local port to listen for UDP packets
  setSyncProvider(getUtcFromNtp);
  time_t utc = now();
  Serial.print(F("  utc: "));
  Serial.print(hour(utc));
  Serial.print(F(":"));
  Serial.print(minute(utc));
  Serial.print(F(":"));
  Serial.println(second(utc));
  Serial.print(F(" "));
  Serial.print(dayShortStr(weekday(utc)));
  Serial.print(F(" "));
  Serial.print(day(utc));
  Serial.print(F(" "));
  Serial.print(monthShortStr(month(utc)));
  Serial.print(F(" "));
  Serial.print(year(utc));

  time_t local = nzTZ.toLocal(utc);
  Serial.print(F("  local: "));
  Serial.print(hour(local));
  Serial.print(F(":"));
  Serial.print(minute(local));
  Serial.print(F(":"));
  Serial.print(second(local));
  Serial.print(F(" "));
  Serial.print(dayShortStr(weekday(local)));
  Serial.print(F(" "));
  Serial.print(day(local));
  Serial.print(F(" "));
  Serial.print(monthShortStr(month(local)));
  Serial.print(F(" "));
  Serial.print(year(local));


  Serial.println(F("Connecting to MQTT broker..."));
  mqtt.setServer(mqttIP, 1883);
  mqtt.setCallback(mqtt_callback);
  while (!mqttConnect())
  {
    delay(5000);
  }
  mqttSubscribe("request");

  // enable the watchdog timer - 8s timeout
  Serial.print(F("WDT: "));
  Serial.print(WDTO_8S);
  Serial.println(F(" s"));
  wdt_enable(WDT);
  wdt_reset();

  // State initialisation
  recentActivity = false;
  portionOfDay = night;
  motionTimer = 0;

  updateLights();
}

void loop()
{
  wdt_reset();

  motionAState = digitalRead(MOTIONSENSORAPIN);
  motionBState = digitalRead(MOTIONSENSORBPIN);
  if (MOTION_SENSOR == LOW)
  {
    motionAState = !motionAState;
    motionBState = !motionBState;
  }

  // While motion present, keep resetting the countdown timer.
  // Fire detected messages only on positive edges
  if (motionAState == HIGH || motionBState == HIGH)
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
      updateLights();
    }
    if (motionBState != previousMotionBState)
    {
      Serial.println(F("Motion detected, sensor B"));
      recentActivity = true;
      mqttPublish("motion", "detected-ch2");
      updateLights();
    }
  }
  else  // Sensors all inactive, start the countdown
  {
    // Test if there has been recent motion and it has been gone for a while
    if (recentActivity)
    {
      if (motionTimer > specificSettings.lightingAfterMotionTime)
      {
        recentActivity = false;
        Serial.println(F("Motion timer expired"));
        mqttPublish("motion", "expired");
        updateLights();
      }
      else
      {
        if (motionTimer > timerPrevious + 1000)
        {
          if (motionTimer / 1000 == 1)
          {
            mqttPublish("motion", "gone");
          }
          Serial.print(motionTimer / 1000);
          Serial.print(" of ");
          Serial.println(specificSettings.lightingAfterMotionTime / 1000);
          timerPrevious += 1000;
        }
      }
    }
  }
  previousMotionAState = motionAState;
  previousMotionBState = motionBState;

  // Give all the worker tasks a bit of time
  Ethernet.maintain();
  mqtt.loop();
}

void updateLights()
{
  /*
  There are many combinations of portion-of-day and motion to consider here:
  | portion-of-day   | motion    | lights  |
  +------------------+-----------+---------+
  | daytime          | dont care | off     |
  | evening, morning | no        | ambient |
  | evening, morning | yes       | bright  |
  | night            | no        | off     |
  | night            | yes       | bright  |
  */

  lightsLevel_t lights = off;
  Serial.print(F("Portion of day: "));
  if (portionOfDay == daytime)
    Serial.println(F("DAYTIME"));
  if (portionOfDay == evening)
    Serial.println(F("EVENING"));
  if (portionOfDay == night)
    Serial.println(F("NIGHT"));
  if (portionOfDay == morning)
    Serial.println(F("MORNING"));
  Serial.print(F("Motion: "));
  Serial.println(recentActivity ? "YES" : "NO");

  if (portionOfDay == daytime)
  {
    lights = off;
  }
  else if (portionOfDay == evening || portionOfDay == morning)
  {
    if (recentActivity == MOTION_NO)
    {
      lights = ambient;
    }
    else
    {
      lights = bright;
    }
  }
  else if (portionOfDay == night)
  {
    if (recentActivity == MOTION_NO)
    {
      lights = off;
    }
    else
    {
      lights = bright;
    }
  }

  if (lights == bright)
  {
    Serial.println(F("Lights bright"));
    analogWrite(LIGHTINGPIN, percent2LEDInt(specificSettings.lightingLevelBright));
    mqttPublish("lights", "bright");
  }
  else if (lights == ambient)
  {
    Serial.println(F("Lights ambient"));
    analogWrite(LIGHTINGPIN, percent2LEDInt(specificSettings.lightingLevelAmbient));
    mqttPublish("lights", "ambient");
  }
  else // lights == off
  {
    Serial.println(F("Lights off"));
    analogWrite(LIGHTINGPIN, percent2LEDInt(specificSettings.lightingLevelOff));
    mqttPublish("lights", "off");
  }
}

boolean mqttConnect()
{
  boolean success = mqtt.connect(commonSettings.deviceName, MQTT_USERNAME, MQTT_PASSWORD, commonSettings.mqttWillTopic, mqttWillQos, mqttWillRetain, commonSettings.mqttWillMessage);
  if (success)
  {
    Serial.println(F("Successfully connected to MQTT broker "));
    // publish retained LWT so anything listening knows we are alive
    byte data[] = { "connected" };
    mqtt.publish(commonSettings.mqttWillTopic, data, 1, mqttWillRetain);
  }
  else
  {
    Serial.println(F("Failed to connect to MQTT broker"));
  }
  return success;
}

void mqttSubscribe(const char* name)
{
  // build the MQTT topic: mqttTopicBase/name
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%s", commonSettings.mqttTopicBase, name);

  Serial.print(F("Subscribing to: "));
  Serial.println(topic);

  // publish to the MQTT broker
  mqtt.subscribe(topic);
}

void mqttPublish(const char* name, const char* payload)
{
  // build the MQTT topic: mqttTopicBase/name
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/status/%s", commonSettings.mqttTopicBase, name);

  Serial.print(topic);
  Serial.print(F(" "));
  Serial.println(payload);


  // publish to the MQTT broker
  boolean success = mqtt.publish(topic, payload);
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

byte readI2CRegister(byte i2c_address, byte reg)
{
  unsigned char v;
  Wire.beginTransmission(i2c_address);
  Wire.write(reg);  // Register to read
  Wire.endTransmission();

  Wire.requestFrom(i2c_address, (uint8_t)1); // Read a byte
  while(!Wire.available()) { }
  v = Wire.read();
  return v;
}

// NO ERROR CHECKING - MUST BE 0-100
int percent2LEDInt(int p)
{
  return ledCurve.exponential(p);
}

/*-------- NTP ----------*/
time_t getUtcFromNtp()
{
  while (udp.parsePacket() > 0) ; // discard any previously received packets
  //Serial.println(F("Transmit NTP Request"));
  sendNTPpacket(ntpIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      //Serial.println(F("Receive NTP Response"));
      udp.read((byte*)tmpBuf, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)tmpBuf[40] << 24;
      secsSince1900 |= (unsigned long)tmpBuf[41] << 16;
      secsSince1900 |= (unsigned long)tmpBuf[42] << 8;
      secsSince1900 |= (unsigned long)tmpBuf[43];
      return secsSince1900 - 2208988800UL;
    }
  }
  Serial.println(F("NTP: no resp"));
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(tmpBuf, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  tmpBuf[0] = 0b11100011;   // LI, Version, Mode
  tmpBuf[1] = 0;     // Stratum, or type of clock
  tmpBuf[2] = 6;     // Polling Interval
  tmpBuf[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  tmpBuf[12]  = 49;
  tmpBuf[13]  = 0x4E;
  tmpBuf[14]  = 49;
  tmpBuf[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(tmpBuf, NTP_PACKET_SIZE);
  udp.endPacket();
}
