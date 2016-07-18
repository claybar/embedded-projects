#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <SerialCommand.h>
#include <mqttEspRelay.h>
#include <PubSubClient.h>

#include <Secrets.h>

const char* DeviceName = "ESP8266_" + ESP.getChipId();

WiFiClient espClient;

IPAddress mqttIP(MQTT_SERVER_IP);
PubSubClient mqttClient(espClient);
//SerialCommand serialCmd;

long lastMsg = 0;
long now;
char msg[50];
int value = 0;

void setup(void)
{
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.print("Hello from ");
  Serial.println(DeviceName);

  Serial.println(F("SER: setup"));
  //serialCmd.addCommand("MQTT", serialMQTTRelay);  // Relay for MQTT message using 2 parameters
  //serialCmd.setDefaultHandler(serialUnrecognized);

  Serial.println(F("WIFI: setup"));
  setup_wifi();

  mqttClient.setServer(mqttIP, 1883);
  mqttClient.setCallback(mqttCallback);
}

void loop(void)
{
  //serialCmd.readSerial();

  if (!mqttClient.connected())
  {
    mqttReconnect();
  }

  now = millis();
  if (now - lastMsg > 2000)
  {
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());

    lastMsg = now;

    Serial.print("Now: ");
    Serial.println(now);

    Serial.print("Heap: ");
    Serial.println(ESP.getFreeHeap());

    ++value;
    snprintf (msg, 75, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    mqttClient.publish("outTopic", msg);
  }

  // Allow time for background tasks and feed the dog
  mqttClient.loop();
  delay(1);
  yield();
}

void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(AP_SSID);

  WiFi.begin(AP_SSID, AP_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/*-------- Serial Monitor ----------*/

void serialMQTTRelay()
{
}
/*
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
    Serial.print("Topic: ");
    Serial.print(topic);
    Serial.print("    Payload: ");
    Serial.println(payload);
    //mqttPublishRelay(topic, payload);
  }
}
*/

// This gets set as the default handler, and gets called when no other command matches.
void serialUnrecognized(const char *command)
{
  //Serial.println(F("SER: ??"));
}

void mqttReconnect()
{
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (mqttClient.connect(DeviceName))
    {
      Serial.println("connected");
      mqttClient.publish("outTopic", "connected");
      mqttClient.subscribe("inTopic");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
