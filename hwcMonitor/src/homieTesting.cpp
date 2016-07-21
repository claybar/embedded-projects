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

void setup() {
  Wire.begin(0, 2);  // SDA=GPIO0 SCL=GPIO2

  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV
  ads.begin();

  Homie.setFirmware(FW_NAME, FW_VERSION);

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
