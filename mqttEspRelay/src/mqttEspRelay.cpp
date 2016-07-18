#include <Arduino.h>
#include <SerialCommand.h>
#include <mqttEspRelay.h>

String DeviceName = "ESP8266_" + String(ESP.getChipId());

SerialCommand serialCmd;

void setup(void)
{
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.print("Hello from ");
  Serial.println(DeviceName);

  Serial.println(F("SER: setup"));
  serialCmd.addCommand("MQTT", serialMQTTRelay);  // Relay for MQTT message using 2 parameters
  serialCmd.setDefaultHandler(serialUnrecognized);
}

void loop(void)
{
  serialCmd.readSerial();
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
    Serial.print("Topic: ");
    Serial.print(topic);
    Serial.print("    Payload: ");
    Serial.println(payload);
    //mqttPublishRelay(topic, payload);
  }
}

// This gets set as the default handler, and gets called when no other command matches.
void serialUnrecognized(const char *command)
{
  Serial.println(F("SER: ??"));
}
