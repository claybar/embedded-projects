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
#include <EtherTen.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <LEDFader.h>
#include <Curve.h>
#include <elapsedMillis.h>

#include <Settings.h>
#include <Secrets.h>

// Hardware setup
#define LIGHTINGPIN 9
#define ACTIVITYLEDPIN 13
#define MOTIONSENSORAPIN 7
#define MOTIONSENSORBPIN 6

// MQTT setup
#define mqttClientId "frontsteps"
#define mqttTopicBase "frontsteps"
#define mqttWillTopic "clients/frontsteps"
#define mqttWillMessage "unexpected exit"
const int mqttWillQos = 0;
const int mqttWillRetain = 1;

// Used for short-term string building
char tmpBuf[33];
// Used to redirect stdout to the serial port
FILE serial_stdout;

// Storage, will be set by onboard i2c device and DHCP
byte mac[] = { 0, 0, 0, 0, 0, 0 };
IPAddress mqttIP(MQTT_SERVER_IP);

// Defaults, some can be set later via MQTT
int lightingLevelOff = 0; // percentage
int lightingLevelAmbient = 10; // percentage
int lightingLevelBright = 50;  // percentage
int lightingChangeTime = 5000;  // milliseconds
unsigned long lightingAfterMotionTime = 5 * 60 * 1000;  // milliseconds

int motionAState = 0;
int motionBState = 0;
int previousMotionAState = 0;
int previousMotionBState = 0;

// Counters for tracking failed transmissions.  Used to reset processor after repeated failures.
int txFailCount;
int txFailCountTotal;

// Used to track how long since motion has been detected
bool recentMotion;
elapsedMillis motionTimer;

// Hardware and protocol handlers
LEDFader lightingLed = LEDFader(LIGHTINGPIN);

EthernetClient ethernet;

PubSubClient mqtt(ethernet);

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
  
  // More linear illumination for human eyes
  lightingLed.set_curve(&Curve::exponential);

  Serial.begin(115200);
  // Redirect stdout to the serial port helper
  fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &serial_stdout;

  Serial.println(F("\nSketch ID: frontsteps_lighting.ino\n"));
  
  // initialise the SPI bus.  
  SPI.begin();

  // Join i2c bus as master
  Wire.begin();

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
  

  Serial.println(F("Connecting to MQTT broker..."));
  mqtt.setServer(mqttIP, 1883);
  mqtt.setCallback(mqtt_callback);
  while (!mqttConnect())
  {
    delay(5000);
  }
  mqttSubscribe("request");

  recentMotion = false;
  motionTimer = 0;
}

void loop()
{
  motionAState = digitalRead(MOTIONSENSORAPIN);
  motionBState = digitalRead(MOTIONSENSORBPIN);

  // While motion is present, keep reseting the countdown timer.
  // Fire messages only on positive edges
  if (motionAState == HIGH || motionBState == HIGH)
  {
    // Reset timer and log presence of motion
    motionTimer = 0;
    recentMotion = true; 

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
  else  // Sensors low
  {
    // Test if there has been recent motion and the it has been gone for a while
    if (recentMotion && (motionTimer > lightingAfterMotionTime))
    {
      recentMotion = false;

      Serial.println(F("Motion timer expired"));
      mqttPublish("motion", "gone");
    }
  }
  previousMotionAState = motionAState;
  previousMotionBState = motionBState;

  
  // Add in time-of-day to this logic
  if (recentMotion)
  {
    lightingBright();
  }
  else
  {
    lightingOff();
  }

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
  boolean success = mqtt.connect(mqttClientId, MQTT_USERNAME, MQTT_PASSWORD, mqttWillTopic, mqttWillQos, mqttWillRetain, mqttWillMessage); 
  if (success) {
    Serial.println(F("Successfully connected to MQTT broker "));
    // publish retained LWT so anything listening knows we are alive
    byte data[] = { "connected" };
    mqtt.publish(mqttWillTopic, data, 1, mqttWillRetain);
  } else {
    Serial.println(F("Failed to connect to MQTT broker"));
  }
  return success;
}

void mqttSubscribe(char* name)
{
  // build the MQTT topic: mqttTopicBase/name
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%s", mqttTopicBase, name);

  Serial.print(F("Subscribing to: "));
  Serial.println(topic);
  
  // publish to the MQTT broker 
  mqtt.subscribe(topic);
}

void mqttPublish(char* name, char* payload)
{
  // build the MQTT topic: mqttTopicBase/name
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/status/%s", mqttTopicBase, name);

  Serial.print(topic);
  Serial.print(F(" "));
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

