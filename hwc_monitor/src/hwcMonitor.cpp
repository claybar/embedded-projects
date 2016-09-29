#include <Wire.h>
#include <Math.h>
#include <Homie.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_HTU21DF.h>

#include "thermistor.h"

#define FW_NAME             "hwmonitor"
#define FW_VERSION          "0.0.2"

#define ADCCH_TOP           0
#define ADCCH_MIDDLE        1
#define ADCCH_BOTTOM        2
#define ADCCH_COLLECTOR     3

#define ADS_GAIN            (GAIN_ONE)
#define ADS_VOLTS_PER_BIT   0.002  // 1x gain   1 bit = 2mV
#define THERM_SUPPLY        5.0
#define THERM_RDIV_1        10000.0

Adafruit_ADS1015 ads;
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

HomieNode cylinderNode("hotwaterCylinder", "hwcylinder");
HomieNode collectorNode("solarCollector", "solarcollector");
HomieNode cupboardNode("cupboard", "temperature,humidity");

const int TRANSMIT_INTERVAL = 10;
unsigned long lastDataSent = 0;

void homieSetupHandler()
{
  collectorNode.advertise("raw");
  //collectorNode.advertise("temperature");

  cylinderNode.advertise("rawTop");
  cylinderNode.advertise("rawMiddle");
  cylinderNode.advertise("rawBottom");
  cylinderNode.advertise("temperatureTop");
  cylinderNode.advertise("temperatureMiddle");
  cylinderNode.advertise("temperatureBottom");

  cupboardNode.advertise("temperature");
  cupboardNode.advertise("humidity");
}

void homieLoopHandler()
{
  if (millis() - lastDataSent >= TRANSMIT_INTERVAL * 1000UL)
  {
    uint16_t collAdc = ads.readADC_SingleEnded(ADCCH_COLLECTOR);

    uint16_t topAdc = ads.readADC_SingleEnded(ADCCH_TOP);
    uint16_t midAdc = ads.readADC_SingleEnded(ADCCH_MIDDLE);
    uint16_t botAdc = ads.readADC_SingleEnded(ADCCH_BOTTOM);

    double topTemp = thermistor10k(topAdc * ADS_VOLTS_PER_BIT, THERM_SUPPLY, THERM_RDIV_1);
    double midTemp = thermistor10k(midAdc * ADS_VOLTS_PER_BIT, THERM_SUPPLY, THERM_RDIV_1);
    double botTemp = thermistor10k(botAdc * ADS_VOLTS_PER_BIT, THERM_SUPPLY, THERM_RDIV_1);

    /*
    Serial.println("midAdc " + String(midAdc));
    Serial.println("midVolts " + String(midAdc * ADS_VOLTS_PER_BIT));
    Serial.println("midTemp " + String(midTemp));
    */

    Homie.setNodeProperty(collectorNode, "raw").send(String(collAdc));
    //Homie.setNodeProperty(collectorNode, "temperature").send(String(collAdc));

    Homie.setNodeProperty(cylinderNode, "rawTop").send(String(topAdc));
    Homie.setNodeProperty(cylinderNode, "rawMiddle").send(String(midAdc));
    Homie.setNodeProperty(cylinderNode, "rawBottom").send(String(botAdc));
    Homie.setNodeProperty(cylinderNode, "temperatureTop").send(String(topTemp));
    Homie.setNodeProperty(cylinderNode, "temperatureMiddle").send(String(midTemp));
    Homie.setNodeProperty(cylinderNode, "temperatureBottom").send(String(botTemp));

    Homie.setNodeProperty(cupboardNode, "temperature").send(String(htu.readTemperature()));
    Homie.setNodeProperty(cupboardNode, "humidity").send(String(htu.readHumidity()));

    lastDataSent += TRANSMIT_INTERVAL * 1000UL;
  }
}

void onHomieEvent(HomieEvent event)
{
  switch(event)
  {
    case HomieEvent::STANDALONE_MODE:
      // Do whatever you want when standalone mode is started
      break;
    case HomieEvent::CONFIGURATION_MODE:
      // Do whatever you want when configuration mode is started
      break;
    case HomieEvent::NORMAL_MODE:
      // Do whatever you want when normal mode is started
      break;
    case HomieEvent::OTA_STARTED:
      // Do whatever you want when OTA is started
      break;
    case HomieEvent::OTA_FAILED:
      // Do whatever you want when OTA is failed
      break;
    case HomieEvent::OTA_SUCCESSFUL:
      // Do whatever you want when OTA is successful
      break;
    case HomieEvent::ABOUT_TO_RESET:
      // Do whatever you want when the device is about to reset
      break;
    case HomieEvent::WIFI_CONNECTED:
      // Do whatever you want when Wi-Fi is connected in normal mode
      break;
    case HomieEvent::WIFI_DISCONNECTED:
      // Do whatever you want when Wi-Fi is disconnected in normal mode
      break;
    case HomieEvent::MQTT_CONNECTED:
      // Do whatever you want when MQTT is connected in normal mode
      break;
    case HomieEvent::MQTT_DISCONNECTED:
      // Do whatever you want when MQTT is disconnected in normal mode
      break;
  }
}

void setup()
{
  /*
  ESP.getResetReason() returns String containing the last reset resaon in human readable format.
  ESP.getFreeHeap() returns the free heap size.
  ESP.getChipId() returns the ESP8266 chip ID as a 32-bit integer.
  ESP.getCycleCount() returns the cpu instruction cycle count since start as an unsigned 32-bit. This is useful for accurate timing of very short actions like bit banging.

  Several APIs may be used to get flash chip info:
  ESP.getFlashChipId() returns the flash chip ID as a 32-bit integer.
  ESP.getFlashChipSize() returns the flash chip size, in bytes, as seen by the SDK (may be less than actual size).
  ESP.getFlashChipRealSize() returns the real chip size, in bytes, based on the flash chip ID.
  ESP.getFlashChipSpeed(void) returns the flash chip frequency, in Hz.
  */

  Wire.begin(D2, D1);  // SDA=D2=GPIO4 SCL=D1=GPIO5

  Serial.begin(115200);
  Serial.println();
  Serial.println();


  ads.setGain(ADS_GAIN);
  ads.begin();

  if (!htu.begin())
  {
    Serial.println(F("Couldn't find HTU21D sensor!"));
  }

  Homie_setFirmware(FW_NAME, FW_VERSION);

  Homie.setSetupFunction(homieSetupHandler);
  Homie.setLoopFunction(homieLoopHandler);
  Homie.onEvent(onHomieEvent);

  /*
  Serial.println("Flash chip stats:");
  Serial.print("  id: ");
  Serial.println(ESP.getFlashChipId()); // returns the flash chip ID as a 32-bit integer.
  Serial.print("  size: ");
  Serial.println(ESP.getFlashChipSize());
  Serial.print("  realsize: ");
  Serial.println(ESP.getFlashChipRealSize());
  Serial.print("  speed: ");
  Serial.println(ESP.getFlashChipSpeed());
  */

  Homie.setup();
}

void loop()
{
  Homie.loop();
}
