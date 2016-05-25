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

#define LIGHTINGPIN 9
#define ACTIVITYLEDPIN 13
#define MOTIONSENSORAPIN 7
#define MOTIONSENSORBPIN 6

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

// MQTT setup
const char mqttClientId[] = "frontsteps";

// publish to "lighting/MAC/<variable>".
const char mqttTopicBase[] = "frontsteps";
const char mqttWillTopic[] = "clients/frontsteps";
const char mqttWillMessage[] = "unexpected exit";
int  mqttWillQos = 0;
int  mqttWillRetain = 1;

// Used to track how long since motion has been detected
bool recentMotion;
elapsedMillis motionTimer;

// Hardware and protocol handlers
LEDFader lightingLed = LEDFader(LIGHTINGPIN);

EthernetClient ethernet;

//PubSubClient mqtt(MQTT_SERVER_IP, 1883, mqtt_callback, ethernet);
PubSubClient mqtt(ethernet);

// This needs to be here so objects have been defined
#include <HomeAutomationHelpers.h>

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
  
  Serial.print("MQTT: message received: ");
  Serial.print(topic);
  Serial.print(" => ");
  Serial.print(p);
  Serial.println();

  if (strcmp(topic, "frontsteps/00:04:A3:D3:30:78/request") == 0)
  {
    Serial.println("MQTT: request received");
    if (strcmp(p, "status") == 0)
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
  else if (strcmp(topic, "setdimmedlevel") == 0)
  {

  }
  else
  {
    Serial.println("  -> Unrecognised message");
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

  //printf(F("\nSketch ID: frontsteps_lighting.ino\n"));
  Serial.println(F("\nSketch ID: frontsteps_lighting.ino\n"));
  //Serial.println("");
  
  // initialise the SPI bus.  
  SPI.begin();

  // Join i2c bus as master
  Wire.begin();

  Serial.print(F("MAC address: "));
  //Serial.print ("");
  for (int i = 0 ; i < 6; i++)
  {
    mac[i] = readI2CRegister(MAC_I2C_ADDR, MAC_REG_BASE + i);
  }
  printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  //sprintf(tmpBuf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  //Serial.println(tmpBuf);

    
  Serial.print(F("IP address: "));
  while (Ethernet.begin(mac) != 1)
  {
    delay(5000);
    Serial.print (".");
  }
  Serial.println(Ethernet.localIP());
  

  Serial.println(F("Connecting to MQTT broker..."));
  mqtt.setServer(mqttIP, 1883);
  mqtt.setCallback(mqtt_callback);
  while (!mqttConnect())
  {
    delay(5000);
  }
  // Subscribe to topics
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
