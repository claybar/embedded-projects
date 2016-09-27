#include <Wire.h>
#include <Homie.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_HTU21DF.h>

#define FW_NAME "hwmonitor"
#define FW_VERSION "0.0.1"

#define ADCCH_TOP 0
#define ADCCH_MIDDLE 1
#define ADCCH_BOTTOM 2
#define ADCCH_COLLECTOR 3

Adafruit_ADS1015 ads;
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

HomieNode cylinderNode("hotwaterCylinder", "hwcylinder");
HomieNode collectorNode("solarCollector", "solarcollector");
HomieNode cupboardNode("cupboard", "temperature,humidity");  // ID, type

//HomieNode cylinderNode("cylinder", "temperature");  // ID, type
//HomieNode collectorNode("collector", "temperature");  // ID, type

const int TRANSMIT_INTERVAL = 10;
unsigned long lastDataSent = 0;
//float temperature = 1.0;

void homieLoopHandler()
{
  if (millis() - lastDataSent >= TRANSMIT_INTERVAL * 1000UL)
  {
    uint16_t collAdc = ads.readADC_SingleEnded(ADCCH_COLLECTOR);

    uint16_t topAdc = ads.readADC_SingleEnded(ADCCH_TOP);
    uint16_t midAdc = ads.readADC_SingleEnded(ADCCH_MIDDLE);
    uint16_t botAdc = ads.readADC_SingleEnded(ADCCH_BOTTOM);

    Homie.setNodeProperty(collectorNode, "raw", String(collAdc), true);

    Homie.setNodeProperty(collectorNode, "temperature", String(collAdc), true);


    Homie.setNodeProperty(cylinderNode, "rawTop", String(topAdc), true);
    Homie.setNodeProperty(cylinderNode, "rawMiddle", String(midAdc), true);
    Homie.setNodeProperty(cylinderNode, "rawBottom", String(botAdc), true);

    Homie.setNodeProperty(cylinderNode, "temperatureTop", String(topAdc), true);
    Homie.setNodeProperty(cylinderNode, "temperatureMiddle", String(midAdc), true);
    Homie.setNodeProperty(cylinderNode, "temperatureBottom", String(botAdc), true);

    Homie.setNodeProperty(cupboardNode, "temperature", String(htu.readTemperature()), true);
    Homie.setNodeProperty(cupboardNode, "humidity", String(htu.readHumidity()), true);

    lastDataSent += TRANSMIT_INTERVAL * 1000UL;
  }
}

void onHomieEvent(HomieEvent event) {
  Serial.print("EVENT: ");
  Serial.println(event);

  switch(event)
  {
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

  Serial.begin(115200);
  Serial.println();
  Serial.println();

  Wire.begin(D2, D1);  // SDA=D2=GPIO4 SCL=D1=GPIO5

  ads.setGain(GAIN_ONE);        // 1x gain   1 bit = 2mV
  ads.begin();

  if (!htu.begin())
  {
    Serial.println("Couldn't find HTU21D sensor!");
  }

  Homie.setFirmware(FW_NAME, FW_VERSION);

  Homie.onEvent(onHomieEvent);


  Homie.registerNode(cylinderNode);
  Homie.registerNode(collectorNode);
  Homie.registerNode(cupboardNode);

  //Homie.setSetupFunction(homieSetupHandler);
  Homie.setLoopFunction(homieLoopHandler);

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
