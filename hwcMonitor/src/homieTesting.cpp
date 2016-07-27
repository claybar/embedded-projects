#include <Wire.h>
#include <Homie.h>
#include <Adafruit_ADS1015.h>

#define FW_NAME "hwmonitor"
#define FW_VERSION "0.0.1"

#define ADCCH_TOP 0
#define ADCCH_MID 1
#define ADCCH_BOT 2
#define ADCCH_REF 3

Adafruit_ADS1015 ads;

HomieNode tempTopNode("temperatureTop", "temperature");  // ID, type
HomieNode tempMidNode("temperatureMid", "temperature");  // ID, type
HomieNode tempBotNode("temperatureBot", "temperature");  // ID, type

void homieLoopHandler()
{
  // Read common reference to get mid of VCC rail
  uint16_t refAdc = ads.readADC_SingleEnded(ADCCH_REF);

  uint16_t topAdc = ads.readADC_SingleEnded(ADCCH_TOP);
  uint16_t midAdc = ads.readADC_SingleEnded(ADCCH_MID);
  uint16_t botAdc = ads.readADC_SingleEnded(ADCCH_BOT);

  /*
  if (sensorValue != lastSensorValue) {
    if (Homie.setNodeProperty(doorNode, "state", sensorValue ? "OPEN" : "CLOSED", true)) {
      lastSensorValue = sensorValue;
    }
  }
  */
}

void onHomieEvent(HomieEvent event) {
  Serial.print("EVENT: ");
  Serial.println(event);

  switch(event) {
    case HOMIE_CONFIGURATION_MODE:
      break;
    case HOMIE_NORMAL_MODE:
       // Do whatever you want when normal mode is started
      break;
    case HOMIE_OTA_MODE:
       // Do whatever you want when OTA mode is started
      break;
    case HOMIE_ABOUT_TO_RESET:
      // Do whatever you want when the device is about to reset
      break;
    case HOMIE_WIFI_CONNECTED:
       // Do whatever you want when Wi-Fi is connected in normal mode
      break;
    case HOMIE_WIFI_DISCONNECTED:
      // Do whatever you want when Wi-Fi is disconnected in normal mode
      break;
    case HOMIE_MQTT_CONNECTED:
      // Do whatever you want when MQTT is connected in normal mode
      break;
    case HOMIE_MQTT_DISCONNECTED:
      // Do whatever you want when MQTT is disconnected in normal mode
      break;
  }
}

void setup() {
  /*
  ESP.getResetReason() returns String containing the last reset resaon in human readable format.

  ESP.getFreeHeap() returns the free heap size.

  ESP.getChipId() returns the ESP8266 chip ID as a 32-bit integer.

  Several APIs may be used to get flash chip info:

  ESP.getFlashChipId() returns the flash chip ID as a 32-bit integer.

  ESP.getFlashChipSize() returns the flash chip size, in bytes, as seen by the SDK (may be less than actual size).

  ESP.getFlashChipRealSize() returns the real chip size, in bytes, based on the flash chip ID.

  ESP.getFlashChipSpeed(void) returns the flash chip frequency, in Hz.

  ESP.getCycleCount() returns the cpu instruction cycle count since start as an unsigned 32-bit. This is useful for accurate timing of very short actions like bit banging.
  */
  Serial.print("Flash chip size: ");
  Serial.println(ESP.getFlashChipSize());

  Wire.begin(0, 2);  // SDA=GPIO0 SCL=GPIO2

  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV
  ads.begin();

  Homie.setFirmware(FW_NAME, FW_VERSION);

  Homie.onEvent(onHomieEvent);

  // nothing to set
  //tempTopNode.subscribe("on", temp1Handler);

  Homie.registerNode(tempTopNode);
  Homie.registerNode(tempMidNode);
  Homie.registerNode(tempBotNode);

  //Homie.setSetupFunction(homieSetupHandler);
  Homie.setLoopFunction(homieLoopHandler);

  Homie.setup();
}

void loop() {
  Homie.loop();
}
