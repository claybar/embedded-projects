TMonESP_MQTT  Temperature via MQTT with ESP8266
https://github.com/claybar/home-automation/

Samples the temperature of up to 4 DS18B20 sensors and reports using MQTT over WiFi

Buildpack required:
  - Arduino >1.6.4 http://www.arduino.cc/en/Main/Software
  - ESP build environment https://github.com/esp8266/Arduino
      Install via Board Manager by following the instructions in main README.md 
      using http://arduino.esp8266.com/package_esp8266com_index.json

Libraries required:
  - DallasTemperature https://github.com/milesburton/Arduino-Temperature-Control-Library/archive/master.zip
  - PubSubClient (MQTT) https://github.com/Imroy/pubsubclient/archive/master.zip

Hardware required:
  - ESP8266 module
  - DS18B20 temperature sensors connected to GPIO2 with pullup to VCC
  - Usual set of IO pull up/downs for programming/running ESP module

Network config required:
  - Edit Secrets.h with network credentials and IP assignments.
  Network and MQTT status is monitored indirectly by counting the number of sequential MQTT transmission 
  failures.  If this count exceeds a threshold (TX_FAIL_LIMIT) the processor is rebooted. 

MQTT messages
  MQTT messages are set to the topic "/ESP8266_DEVICEID/temperature/instant/ONEWIREID.c" with a string 
  payload of the temperature where:
    - DEVICEID is the unique (hopefully) number of the ESP module
    - ONEWIREID is the unique one-wire serial number assigned by Maxim

Temperature sampling and transmission periods  (NOT YET FULLY IMPLEMENTED)
  The sampling and transmission periods can be set independently.  At each transmission the average of all 
  temperature samples taken within the transmission is calculated and sent.  To disable averaging over the
  time period, set SAMPLE_PERIOD = MQTT_TX_PERIOD
  
TODO:
  - Implement temperature averaging using an accumulator.  Need to make sure fresh sample is sent when
    sample and tx period are equal.  If the tx fires just before the sample ticker there will be a delay
    equal to the ticker period.
  - Subscribe to messages from MQTT to configure things.  Also implement a config sending trigger.


