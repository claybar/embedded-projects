#include <Wire.h>
#include <Math.h>
#include <Homie.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_HTU21DF.h>

#include "thermistor.h"

#define FW_NAME             "hwmonitor"
#define FW_VERSION          "0.0.2"

#define ADCCH_TOP           0 // Blue
#define ADCCH_MIDDLE        1 // Orange
#define ADCCH_BOTTOM        2 // Green
#define ADCCH_COLLECTOR     3 // Brown

#define ADS_GAIN            (GAIN_TWOTHIRDS)
#define ADS_VOLTS_PER_BIT   0.003  // 2/3x gain   1 bit = 3mV
//#define ADS_VOLTS_PER_BIT   0.002  // 1x gain   1 bit = 2mV
#define THERM_SUPPLY        5.0
#define THERM_RDIV_1        10000.0

Adafruit_ADS1015 ads;
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

HomieNode cylinderNode("hotwaterCylinder", "hwcylinder");
HomieNode collectorNode("solarCollector", "solarcollector");
HomieNode cupboardNode("cupboard", "temperature,humidity");

HomieSetting<long> transmitIntervalSetting("measurementInterval", "The measurement interval in seconds");

const int DEFAULT_TRANSMIT_INTERVAL = 60;
unsigned long lastDataSent = 0;

bool tempHumSensor = false;

void homieSetupHandler()
{
  transmitIntervalSetting.setDefaultValue(DEFAULT_TRANSMIT_INTERVAL);

  collectorNode.advertise("raw");
  //collectorNode.advertise("temperature");

  cylinderNode.advertise("rawTop");
  cylinderNode.advertise("rawMiddle");
  cylinderNode.advertise("rawBottom");
  cylinderNode.advertise("temperatureTop");
  cylinderNode.advertise("temperatureMiddle");
  cylinderNode.advertise("temperatureBottom");

  if (tempHumSensor)
  {
    cupboardNode.advertise("temperature");
    cupboardNode.advertise("humidity");
  }
}

void homieLoopHandler()
{
  if (millis() - lastDataSent >= transmitIntervalSetting.get() * 1000UL || lastDataSent == 0)
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

    collectorNode.setProperty("raw").send(String(collAdc));
    //collectorNode.setProperty("temperature").send(String(collAdc));

    cylinderNode.setProperty("rawTop").send(String(topAdc));
    cylinderNode.setProperty("rawMiddle").send(String(midAdc));
    cylinderNode.setProperty("rawBottom").send(String(botAdc));
    cylinderNode.setProperty("temperatureTop").send(String(topTemp));
    cylinderNode.setProperty("temperatureMiddle").send(String(midTemp));
    cylinderNode.setProperty("temperatureBottom").send(String(botTemp));

    if (tempHumSensor)
    {
      cupboardNode.setProperty("temperature").send(String(htu.readTemperature()));
      cupboardNode.setProperty("humidity").send(String(htu.readHumidity()));
    }

    lastDataSent += transmitIntervalSetting.get() * 1000UL;
  }
}

void onHomieEvent(const HomieEvent& event)
{
  switch(event.type)
  {
    case HomieEventType::STANDALONE_MODE:
      break;
    case HomieEventType::CONFIGURATION_MODE:
      break;
    case HomieEventType::NORMAL_MODE:
      break;
    case HomieEventType::OTA_STARTED:
      break;
    case HomieEventType::OTA_FAILED:
      break;
    case HomieEventType::OTA_SUCCESSFUL:
      break;
    case HomieEventType::ABOUT_TO_RESET:
      break;
    case HomieEventType::WIFI_CONNECTED:
      break;
    case HomieEventType::WIFI_DISCONNECTED:
      break;
    case HomieEventType::MQTT_CONNECTED:
      break;
    case HomieEventType::MQTT_DISCONNECTED:
      break;
    case HomieEventType::MQTT_PACKET_ACKNOWLEDGED:
      break;
    case HomieEventType::READY_TO_SLEEP:
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
    tempHumSensor = false;
  }
  else
  {
    tempHumSensor = true;
  }

  Homie_setFirmware(FW_NAME, FW_VERSION);

  Homie.disableResetTrigger();
  Homie.setLedPin(D3, LOW);
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
