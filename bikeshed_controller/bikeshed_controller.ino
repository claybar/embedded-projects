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
#include <SPI.h>
#include <Wire.h>
#include <Ethernet.h>
#include <Adafruit_AM2315.h>
#include <PubSubClient.h>
#include <elapsedMillis.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <SerialCommand.h>

#include <Settings.h>
#include <Secrets.h>

// Hardware setup
#define ACTIVITYLEDPIN 13
#define INSIDELIGHTSPIN 9
#define OUTSIDELIGHTSPIN 6
#define MOTIONSENSORAPIN 8
#define MOTIONSENSORBPIN 3
#define DOORSENSORPIN 7

#define MAC_I2C_ADDR 0x50
#define MAC_REG_BASE 0xFA

// Logic of motion and door sensors
#define MOTION LOW
#define DOOROPEN HIGH
#define ON true
#define OFF false

// MQTT setup
#define mqttClientId "bikeshed"
#define mqttTopicBase "bikeshed"
#define mqttWillTopic "clients/bikeshed"
#define mqttWillMessage "unexpected exit"
const int mqttWillQos = 0;
const int mqttWillRetain = 1;
int mqttFailCount = 0;
int mqttDisconnectedCount = 0;
const int mqttFailCountLimit = 5;
const int mqttDisconnectedCountLimit = 5;

// Defaults, some can be set later via MQTT
unsigned long outsideLightAfterMotionTime = 5000; // 5 * 60 * 1000;  // milliseconds

// Used for short-term string building
// Also used for NTP reply
char tmpBuf[48];

// NTP
const int NTP_PACKET_SIZE = 48;

// Used to redirect stdout to the serial port
FILE serial_stdout;

// Storage, will be set by onboard i2c device and DHCP
byte mac[] = { 0, 0, 0, 0, 0, 0 };
IPAddress mqttIP(MQTT_SERVER_IP);
IPAddress ntpIP(NTP_SERVER_IP);

// State control variables
bool motionAState = false;
bool motionBState = false;
bool doorState = false;
bool previousMotionAState = false;
bool previousMotionBState = false;
bool previousDoorState = false;
bool recentActivity = false;    // Used to track how long since motion has been detected

elapsedMillis motionTimer;
unsigned long timerPrevious;

elapsedMillis heartbeatTimer;
unsigned long heartbeatTimerPrevious;

// Hardware and protocol handlers
EthernetClient ethernet;
EthernetUDP udp;
Adafruit_AM2315 am2315;
PubSubClient mqtt(ethernet);
SerialCommand serialCmd;
const unsigned int udpPort = 8888;  // local port to listen for UDP packets

void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
  // Allocate the correct amount of memory for the payload copy
  char* p = (char*)malloc(length + 1);
  
  // Copy the payload to the new buffer
  for (int i = 0; i < length; i++)
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
  pinMode(MOTIONSENSORAPIN, INPUT_PULLUP);
  pinMode(MOTIONSENSORBPIN, INPUT_PULLUP);
  pinMode(DOORSENSORPIN, INPUT_PULLUP);
  pinMode(ACTIVITYLEDPIN, OUTPUT);
  pinMode(INSIDELIGHTSPIN, OUTPUT);
  pinMode(OUTSIDELIGHTSPIN, OUTPUT);

  // ensure the watchdog is disabled for now
  wdt_disable();
  
  Serial.begin(115200);
  // Redirect stdout to the serial port helper
  fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &serial_stdout;

  Serial.println(F("\nSketch ID: bikeshed_controller.ino\n"));
  
  // initialise busses: SPI, i2c, 1-wire temperature.  
  SPI.begin();
  Wire.begin();
  if (!am2315.begin()) {
     Serial.println(F("HW: AM2315 sensor not found"));
  }

  Serial.print(F("MAC: "));
  for (int i = 0 ; i < 6; i++)
  {
    mac[i] = readI2CRegister(MAC_I2C_ADDR, MAC_REG_BASE + i);
  }
  printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  Serial.println(F("ETH: connect"));
  ethernetConnect();

  Serial.println(F("NTP: time"));
  udp.begin(udpPort);
  setSyncProvider(getNtpTime);

  Serial.println(F("MQTT: connect"));
  mqtt.setServer(mqttIP, 1883);
  mqtt.setCallback(mqtt_callback);
  mqttConnect();
  mqttSetupSubscriptions();
  
  // Setup callbacks for SerialCommand commands
  Serial.println(F("SER: setup"));
  //sCmd.addCommand("ON",    LED_on);          // Turns LED on
  //sCmd.addCommand("OFF",   LED_off);         // Turns LED off
  //sCmd.addCommand("HELLO", sayHello);        // Echos the string argument back
  serialCmd.addCommand("MQTT", serialMQTT);  // Relays MQTT message using 2 parameters
  serialCmd.setDefaultHandler(serialUnrecognized);      // Handler for command that isn't matched  (says "What?")

  Serial.println(F("Setting alarms and timers..."));
  Alarm.timerRepeat(60 * 60, hourlyTimer);
  Alarm.timerRepeat( 5 * 60, fiveMinsTimer);

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
  
  // While motion present or door is open, keep reseting the countdown timer.
  // Fire messages only on positive edges
  if (motionAState == MOTION || motionBState == MOTION || doorState == DOOROPEN)
  {
    // Reset timer and log presence of motion
    motionTimer = 0;
    timerPrevious = 0;
    recentActivity = true;

    // Fire messages on positive edges
    if (motionAState != previousMotionAState)
    { 
      Serial.println(F("Motion detected, sensor A"));
      mqttPublish("motion", "detected-ch1");
    }
    if (motionBState != previousMotionBState)
    { 
      Serial.println(F("Motion detected, sensor B"));
      mqttPublish("motion", "detected-ch2");
    }
  }
  else  // Sensors all inactive, start the countdown
  {
    // Test if there has been recent motion and it has been gone for a while
    if (recentActivity)
    {
      if (motionTimer > outsideLightAfterMotionTime)
      {
        recentActivity = false;

        Serial.println(F("Motion timer expired"));
        mqttPublish("motion", "gone");
      }
      else
      {
        if (motionTimer > timerPrevious + 1000)
        {
          Serial.print(motionTimer / 1000);
          Serial.print(" of ");
          Serial.println(outsideLightAfterMotionTime / 1000);

          timerPrevious += 1000;
        }
      }  
    }
  }
  previousMotionAState = motionAState;
  previousMotionBState = motionBState;

  // Add in time-of-day to this logic
  if (recentActivity)
  {
    outsideLights(ON);
  }
  else
  {
    outsideLights(OFF);
  }

  // heartbeat
  if (heartbeatTimer > heartbeatTimerPrevious + 1000)
  {
    Serial.println(F("TICK"));
    heartbeatTimerPrevious += 1000;

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

  // Give all the worker tasks a bit of time
  Ethernet.maintain();
  serialCmd.readSerial();
  mqtt.loop();
  Alarm.delay(1);
}

/*-------- Hardware Abstraction ----------*/
void insideLights(bool state)
{
  digitalWrite(INSIDELIGHTSPIN, state);
}

void outsideLights(bool state)
{
  digitalWrite(OUTSIDELIGHTSPIN, state);
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
}

boolean mqttConnect() 
{
  boolean success = mqtt.connect(mqttClientId, MQTT_USERNAME, MQTT_PASSWORD, mqttWillTopic, mqttWillQos, mqttWillRetain, mqttWillMessage); 
  if (success)
  {
    Serial.println(F("MQTT: Conn good"));
    // publish retained LWT so anything listening knows we are alive
    const byte data[] = { "connected" };
    mqtt.publish(mqttWillTopic, data, 1, mqttWillRetain);
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
  snprintf(topic, sizeof(topic), "%s/%s", mqttTopicBase, name);

  Serial.print(F("MQTT: sub: "));
  Serial.println(topic);
  
  // publish to the MQTT broker 
  mqtt.subscribe(topic);
}

void mqttPublish(const char* name, const char* payload)
{
  // build the MQTT topic: mqttTopicBase/name
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/status/%s", mqttTopicBase, name);

  Serial.print(F("MQTT: "));
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
void serialMQTT()
{
  //int aNumber;
  char *topic;
  char *payload;

  Serial.println(F("SERRX: received"));
  topic = serialCmd.next();
  if (topic != NULL)
  {
    Serial.print(F("  Topic: "));
    Serial.println(topic);
  }
  else
  {
    Serial.println(F("  No topic"));
  }

  payload = serialCmd.next();
  if (payload != NULL)
  {
    Serial.print(F("  Payload: "));
    Serial.println(payload);
  }
  else
  {
    Serial.println(F("  No payload"));
  }

  if (topic != NULL && payload != NULL)
  {
    // Send a MQTT message
    mqttPublish(topic, payload);
  }
}

// This gets set as the default handler, and gets called when no other command matches.
void serialUnrecognized(const char *command) {
  Serial.println(F("SERRX: unrecognized"));
}

/*-------- NTP ----------*/
time_t getNtpTime()
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
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
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

/*-------- Timers ----------*/
void hourlyTimer()
{
  Serial.println(F("TMR: 1h"));
}

void fiveMinsTimer()
{
  Serial.println(F("TMR: 5min"));

  Serial.print(F("  Hum: "));
  Serial.println(am2315.readHumidity());
  Serial.print(F("  Temp: "));
  Serial.println(am2315.readTemperature());
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
