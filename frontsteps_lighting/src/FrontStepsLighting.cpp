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
#include <TimeLib.h>
#include <TimeAlarms.h>
//#include <TimeLord.h>
#include <Curve.h>
#include <elapsedMillis.h>
#include <EEPROMex.h>
#include <Timezone.h>
#include <Sunrise.h>

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
//FILE serial_stdout;

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
Sunrise sunrise(LATITUDE, LONGITUDE, 12);
int sunriseAfterMidnight, sunsetAfterMidnight;

// Hardware and protocol handlers
EthernetClient ethernet;
EthernetUDP udp;
PubSubClient mqtt(ethernet);
//Curve ledCurve;

const int NTP_PACKET_SIZE = 48;
IPAddress ntpIP(NTP_SERVER_IP);

// Function that printf and related will use to print to the serial port
/*
int serial_putchar(char c, FILE* f) {
    if (c == '\n') serial_putchar('\r', f);
    return Serial.write(c) == 1? 0 : 1;
}
*/

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
  //fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  //stdout = &serial_stdout;

  // initialise the SPI and i2c bus.
  SPI.begin();
  Wire.begin();

  uint8_t settingsVer = EEPROM.readByte(ADDR_COM_SETTINGS_OFFSET);
  //Serial.print(F("EEP: Common ver: "));
  //Serial.println(settingsVer);
  if (settingsVer == 0)
  {
    EEPROM.readBlock(ADDR_COM_SETTINGS_OFFSET, commonSettings);
  }
  else
  {
    strcpy(commonSettings.deviceName, "frontsteps");
    strcpy(commonSettings.mqttTopicBase, "frontsteps");
    strcpy(commonSettings.mqttWillTopic, "clients/frontsteps");
    strcpy(commonSettings.mqttWillMessage, "unexpected exit");
    EEPROM.updateBlock(ADDR_COM_SETTINGS_OFFSET, commonSettings);
  }

  settingsVer = EEPROM.readByte(ADDR_FS_SETTINGS_OFFSET);
  //Serial.print(F("EEP: Specific ver: "));
  //Serial.println(settingsVer);
  if (settingsVer == 0)
  {
    EEPROM.readBlock(ADDR_FS_SETTINGS_OFFSET, specificSettings);
  }
  else
  {
    // No settings in EEPROM, so make some numbers up and write
    //Serial.println(F("EEP: Default specific settings"));
    specificSettings.lightingAfterMotionTime = 5000; // 5 * 60 * 1000;  // milliseconds
    specificSettings.lightingLevelAmbient = 20;
    specificSettings.lightingLevelBright = 80;
    specificSettings.morningStart = 7 * 60; // minutes after midnight
    specificSettings.morningAfterSunrise = 10;
    specificSettings.eveningBeforeSunset = 10;
    specificSettings.eveningEnd = 21 * 60; // minutes after midnight
    EEPROM.updateBlock(ADDR_FS_SETTINGS_OFFSET, specificSettings);
  }

  //Serial.print(F("Name: "));
  //Serial.println(commonSettings.deviceName);

  //Serial.print(F("MAC: "));
  for (int i = 0 ; i < 6; i++)
  {
    mac[i] = readI2CRegister(MAC_I2C_ADDR, MAC_REG_BASE + i);
  }
  //printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  //Serial.print(F("IP: "));
  while (Ethernet.begin(mac) != 1)
  {
    delay(5000);
    //Serial.print(F("."));
  }
  //Serial.println(Ethernet.localIP());

  // Set up callback function of time libary to use NTP
  //Serial.println(F("NTP: Setup"));
  udp.begin(8888);  // Local port to listen for UDP packets
  setSyncProvider(getUtcFromNtp);
  delay(5000);

  /*
  time_t utc = now();
  Serial.print(F(" UTC: "));
  printDateTime(utc);
  Serial.println("");
  time_t local = nzTZ.toLocal(utc);
  Serial.print(F(" LOC: "));
  printDateTime(local);
  if (nzTZ.utcIsDST(utc))
  {
    Serial.print(F(" DST"));
  }
  else
  {
    Serial.print(F(" ST"));
  }
  Serial.println("");
  */

  // Calculate sunrise/sunset for today.
  sunriseSunsetAlarm();

  //Serial.println(F("MQTT: Setup"));
  mqtt.setServer(mqttIP, 1883);
  mqtt.setCallback(mqttCallback);
  while (!mqttConnect())
  {
    delay(5000);
  }
  mqttSubscribe("request");

  //Serial.println(F("ALM: Setup"));
  // Status sent every per minute
  Alarm.timerRepeat(60, timeOfDayAlarm);
  // Sunrise & sunset calculated every 6 hours
  Alarm.timerRepeat(60 * 60 * 6, sunriseSunsetAlarm);
  // Sunrise & sunset calculated at about 3am
  // Alarms work in Utc, so add 12 hours for timezone.
  //Alarm.alarmRepeat(3 + 12, 10, 0, sunriseSunsetAlarm);

  // enable the watchdog timer - 8s timeout
  //Serial.print(F("WDT: "));
  //Serial.print(WDTO_8S);
  //Serial.println(F(" s"));
  wdt_enable(WDT);
  wdt_reset();

  // State initialisation
  recentActivity = false;
  portionOfDay = night;
  motionTimer = 0;

  timeOfDayAlarm();
  updateLights();

  // Fire the set of retained messages
  publishConfigAndSettings();
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
      //Serial.println(F("Motion:A"));
      recentActivity = true;
      mqttPublish("motion", "detected-ch1");
      updateLights();
    }
    if (motionBState != previousMotionBState)
    {
      //Serial.println(F("Motion:B"));
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
        //Serial.println(F("Motion:Exp"));
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
          //Serial.print(motionTimer / 1000);
          //Serial.print("of");
          //Serial.println(specificSettings.lightingAfterMotionTime / 1000);
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
  Alarm.delay(1);
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
  /*
  Serial.print(F("TOD: "));
  if (portionOfDay == daytime)
    Serial.println(F("DAYTIME"));
  if (portionOfDay == evening)
    Serial.println(F("EVENING"));
  if (portionOfDay == night)
    Serial.println(F("NIGHT"));
  if (portionOfDay == morning)
    Serial.println(F("MORNING"));
  Serial.print(F("MOTION: "));
  Serial.println(recentActivity ? "YES" : "NO");
  */

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

  //Serial.print(F("Lights: "));
  if (lights == bright)
  {
    //Serial.println(F("MAX"));
    analogWrite(LIGHTINGPIN, percent2LEDInt(specificSettings.lightingLevelBright));
    mqttPublish("lights", "full");
  }
  else if (lights == ambient)
  {
    //Serial.println(F("DIM"));
    analogWrite(LIGHTINGPIN, percent2LEDInt(specificSettings.lightingLevelAmbient));
    mqttPublish("lights", "amb");
  }
  else // lights == off
  {
    //Serial.println(F("OFF"));
    //analogWrite(LIGHTINGPIN, percent2LEDInt(specificSettings.lightingLevelOff));
    analogWrite(LIGHTINGPIN, 0);
    mqttPublish("lights", "off");
  }
}

boolean mqttConnect()
{
  boolean success = mqtt.connect(commonSettings.deviceName, MQTT_USERNAME, MQTT_PASSWORD, commonSettings.mqttWillTopic, mqttWillQos, mqttWillRetain, commonSettings.mqttWillMessage);
  //Serial.print(F("MQTT: "));
  /*
  if (success)
  {
    //Serial.println(F("good"));
    // publish retained LWT so anything listening knows we are alive
    byte data[] = { "connected" };
    mqtt.publish(commonSettings.mqttWillTopic, data, 1, mqttWillRetain);
  }
  else
  {
    //Serial.println(F("MQTT:Err"));
  }
  */
  return success;
}

void mqttSubscribe(const char* name)
{
  // build the MQTT topic: mqttTopicBase/name
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%s", commonSettings.mqttTopicBase, name);

  //Serial.print(F("SUB: "));
  //Serial.println(topic);

  mqtt.subscribe(topic);
}

void mqttPublish(const char* name, const char* payload)
{
  mqttPublish(name, payload, false);
}

// Prepends the MQTT topic: mqttTopicBase/
void mqttPublish(const char* name, const char* payload, bool retained)
{
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%s", commonSettings.mqttTopicBase, name);

  Serial.print(F("MQTT: "));
  Serial.print(topic);
  Serial.print(F(" "));
  Serial.print(payload);
  if (retained)
  {
    Serial.print(F(" (retained)"));
  }
  Serial.println("");

  // publish to the MQTT broker
  boolean success = mqtt.publish(topic, payload, retained);
  if(!success)
  {
    //Serial.print(F("MQTT: pub failed, state: "));
    //Serial.println(mqtt.state());
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
  for (unsigned int i = 0; i < length; i++)
  {
    p[i] = (char)payload[i];
  }
  p[length] = '\0';

  /*
  Serial.print(F("MQTT: rx: "));
  Serial.print(topic);
  Serial.print(F(" => "));
  Serial.print(p);
  Serial.println();
  */

  // Strip off the "frontsteps/" bit
  char* topicStrip = topic + strlen(commonSettings.mqttTopicBase) + 1;

  if (strcmp(topicStrip, "request") == 0)
  {
    updateRetained = true;
  }
  else if (strcmp(topicStrip, "retain/set") == 0)
  {
    //Serial.println(F("MQTT: Retain EEPROM"));
    EEPROM.updateBlock(ADDR_COM_SETTINGS_OFFSET, commonSettings);
    EEPROM.updateBlock(ADDR_FS_SETTINGS_OFFSET, specificSettings);
    updateRetained = true;
  }
  else if (strcmp(topicStrip, "lightsoffdelay/set") == 0)
  {
    specificSettings.lightingAfterMotionTime = atol(p) * 1000;
    updateRetained = true;
  }
  else if (strcmp(topicStrip, "lightsbright/set") == 0)
  {
    int v = atoi(p);
    if (v >= 0 && v <= 100)
    {
      specificSettings.lightingLevelBright = v;
    }
    updateRetained = true;
  }
  else if (strcmp(topicStrip, "lightsambient/set") == 0)
  {
    int v = atoi(p);
    if (v >= 0 && v <= 100)
    {
      specificSettings.lightingLevelAmbient = v;
    }
    updateRetained = true;
  }
  /*
  else if (strcmp(topicStrip, "lightsoff/set") == 0)
  {
    int v = atoi(p);
    if (v >= 0 && v <= 100)
    {
      specificSettings.lightingLevelOff = v;
    }
    updateRetained = true;
  }
  */
  else if (strcmp(topicStrip, "morningstart/set") == 0)
  {
    specificSettings.morningStart = atoi(p);
    updateRetained = true;
  }
  else if (strcmp(topicStrip, "morningaftersunrise/set") == 0)
  {
    specificSettings.morningAfterSunrise = atoi(p);
    updateRetained = true;
  }
  else if (strcmp(topicStrip, "eveningbeforesunset/set") == 0)
  {
    specificSettings.eveningBeforeSunset = atoi(p);
    updateRetained = true;
  }
  else if (strcmp(topicStrip, "eveningend/set") == 0)
  {
    specificSettings.eveningEnd = atoi(p);
    updateRetained = true;
  }
  else
  {
    //Serial.println(F(" -> ??"));
  }

  free(p);

  if (updateRetained)
  {
    publishConfigAndSettings();
  }
}

void publishConfigAndSettings()
{
  /*
  snprintf(tmpBuf, sizeof(tmpBuf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  mqttPublish("$mac", tmpBuf, true);

  IPAddress ip = Ethernet.localIP();
  snprintf(tmpBuf, sizeof(tmpBuf), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  mqttPublish("$localip", tmpBuf, true);
  */
  snprintf(tmpBuf, sizeof(tmpBuf), "%d", commonSettings.version);
  mqttPublish("$commonsettingsversion", tmpBuf, true);

  snprintf(tmpBuf, sizeof(tmpBuf), "%d", specificSettings.version);
  mqttPublish("$specificsettingsversion", tmpBuf, true);

  snprintf(tmpBuf, sizeof(tmpBuf), "%lu", specificSettings.lightingAfterMotionTime / 1000);
  mqttPublish("$lightsoffdelay", tmpBuf, true);

  snprintf(tmpBuf, sizeof(tmpBuf), "%d", specificSettings.lightingLevelBright);
  mqttPublish("$lightsbright", tmpBuf, true);

  snprintf(tmpBuf, sizeof(tmpBuf), "%d", specificSettings.lightingLevelAmbient);
  mqttPublish("$lightsambient", tmpBuf, true);

  //snprintf(tmpBuf, sizeof(tmpBuf), "%d", specificSettings.lightingLevelOff);
  //mqttPublish("$lightsoff", tmpBuf, true);

  snprintf(tmpBuf, sizeof(tmpBuf), "%d", specificSettings.morningStart);
  mqttPublish("$morningstart", tmpBuf, true);

  snprintf(tmpBuf, sizeof(tmpBuf), "%d", specificSettings.morningAfterSunrise);
  mqttPublish("$aftersunrise", tmpBuf, true);

  snprintf(tmpBuf, sizeof(tmpBuf), "%d", specificSettings.eveningBeforeSunset);
  mqttPublish("$beforesunset", tmpBuf, true);

  snprintf(tmpBuf, sizeof(tmpBuf), "%d", specificSettings.eveningEnd);
  mqttPublish("$eveningend", tmpBuf, true);
}

void publishSunriseSunset()
{
  snprintf(tmpBuf, sizeof(tmpBuf), "%02d:%02d", sunriseAfterMidnight / 60, sunriseAfterMidnight % 60);
  mqttPublish("$sunrise", tmpBuf, true);

  snprintf(tmpBuf, sizeof(tmpBuf), "%02d:%02d", sunsetAfterMidnight / 60, sunsetAfterMidnight % 60);
  mqttPublish("$sunset", tmpBuf, true);
}

byte readI2CRegister(byte i2c_address, byte reg)
{
  Wire.beginTransmission(i2c_address);
  Wire.write(reg);  // Register to read
  Wire.endTransmission();

  Wire.requestFrom(i2c_address, (uint8_t)1); // Read a byte
  while(!Wire.available()) { } // DANGER
  unsigned char v = Wire.read();
  return v;
}

// NO ERROR CHECKING - MUST BE 0-100
int percent2LEDInt(int p)
{
  //return ledCurve.exponential(p);
  return(p * 2);
}

/*-------- NTP ----------*/
time_t getUtcFromNtp()
{
  // discard any previously received packets
  while (udp.parsePacket() > 0) ;
  //Serial.println(F("Transmit NTP Request"));
  sendNTPpacket(ntpIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500)
  {
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE)
    {
      //Serial.println(F("Receive NTP Response"));
      udp.read((byte*)tmpBuf, NTP_PACKET_SIZE);
      unsigned long highWord = word(tmpBuf[40], tmpBuf[41]);
      unsigned long lowWord = word(tmpBuf[42], tmpBuf[43]);
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      //Serial.print(F("Secs since 1900: "));
      //Serial.println(secsSince1900);
      //Serial.print(F("Secs since 1970: "));
      //Serial.println(secsSince1900 - 2208988800UL);
      return secsSince1900 - 2208988800UL;
    }
  }
  //Serial.println(F("NTP: bad"));
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(tmpBuf, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
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

void timeOfDayAlarm()
{
  // Serial.println(F("ALM: ToD"));
  // Determine the time of day
  time_t utc = now();
  //Serial.print(F("UTC: "));
  //printTime(hour(utc), minute(utc));
  time_t local = nzTZ.toLocal(utc);
  //Serial.print(F("LOC: "));
  //printTime(hour(local), minute(local));

  // Test against rules to determine portion of day
  uint16_t minAfterMidnight = hour(local) * 60 + minute(local);
  //Serial.print(F(" Portion: "));
  // Morning
  if (minAfterMidnight > specificSettings.morningStart &&
      minAfterMidnight < (sunriseAfterMidnight + specificSettings.morningAfterSunrise))
  {
    portionOfDay = morning;
    //Serial.println(F("M"));
    mqttPublish("$portionofday", "morning", true);
  }
  // Day
  else if (minAfterMidnight > (sunriseAfterMidnight + specificSettings.morningAfterSunrise) &&
      minAfterMidnight < (sunsetAfterMidnight - specificSettings.eveningBeforeSunset))
  {
    portionOfDay = daytime;
    //Serial.println(F("D"));
    mqttPublish("$portionofday", "day", true);
  }
  // Evening
  else if (minAfterMidnight > (sunsetAfterMidnight - specificSettings.eveningBeforeSunset) &&
      minAfterMidnight < specificSettings.eveningEnd)
  {
    portionOfDay = evening;
    //Serial.println(F("E"));
    mqttPublish("$portionofday", "evening", true);
  }
  // Night
  else
  // (minAfterMidnight < (sunrise - morningBeforeSunrise) ||
  //    minAfterMidnight > (sunset + eveningAfterSunset))
  {
    portionOfDay = night;
    //Serial.println(F("N"));
    mqttPublish("$portionofday", "night", true);
  }
}

// Calculates todays sunrise and sunset and stores globally.
void sunriseSunsetAlarm()
{
  Serial.println(F("ALM:SunCalc"));

  time_t utc = now();

  // Rise() and Set() return minutes after UTC midnight, so need to add offset if in DST
  int offset = 0;
  if (nzTZ.utcIsDST(utc))
  {
    offset = 60;
  }

  time_t local = nzTZ.toLocal(utc);
  unsigned char m = month(local);
  unsigned char d = day(local);

  // Should test if sunrise/set are valid, but since I dont live in the (ant)arctic not much point
  sunriseAfterMidnight = sunrise.Rise(m, d) + offset;
  //Serial.print(F("Sunrise: "));
  //printTime(sunrise.Hour(), sunrise.Minute());

  sunsetAfterMidnight = sunrise.Set(m, d) + offset;
  //Serial.print(F("Sunset: "));
  //printTime(sunrise.Hour(), sunrise.Minute());

  publishSunriseSunset();
}

void printTime(byte hour, byte minute)
{
  Serial.print(hour, DEC);
  Serial.print(F(":"));
  if(minute < 10)
  {
    Serial.print(F("0"));
  }
  Serial.println(minute, DEC);
}

void printDateTime(time_t t)
{
  Serial.print(hour(t));
  Serial.print(F(":"));
  byte m = minute(t);
  if(m < 10) Serial.print(F("0"));
  Serial.print(m);
  Serial.print(F(":"));
  byte s = second(t);
  if(s < 10) Serial.print(F("0"));
  Serial.print(s);
  Serial.print(F(" "));
  Serial.print(dayShortStr(weekday(t)));
  Serial.print(F(" "));
  Serial.print(day(t));
  Serial.print(F(" "));
  Serial.print(monthShortStr(month(t)));
  Serial.print(F(" "));
  Serial.print(year(t));
}
