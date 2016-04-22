Code for home automation projects.  Central comms is via MQTT, either via wired ethernet or wireless via WiFi.

# Overall Configuration
All projects attempt to be general purpose and able to be installed onto multiple devices on the same network.  To do this each unit includes some form of unique ID in their communications.  Many also include a user configurable string, normally configured via MQTT.

Network configuration is included at compile-time by including a common header file <Secrets.h> installed as a library.

# EmonCMS

## Raspberry Pi2
EmonCMS installed on a Raspberry Pi2
MQTTWarn used to pull messages from MQTT and feed into EmonCMS

## Power Monitoring

`emon_cont_direct/emon_cont_direct.ino`

EtherTen Arduino clone with a openenergy.org EmonTX Shield V2.  This has been modified from stock with the following changes:
- Ch4 rerouted to Analog pin 6.  This is so the on-board MAC address chip can be addressed via I2C
- Burden resistors on Ch2 & Ch4 changed to 180ohm to improve sensitivity.  These are used for monitoring a heat pump and house lights.


## Temperature Monitoring

Base install
Arduino 1.6.4
ESP build environment
https://github.com/esp8266/Arduino
http://arduino.esp8266.com/package_esp8266com_index.json

Libraries

Arduino Library for Dallas Temperature ICs
https://github.com/milesburton/Arduino-Temperature-Control-Library/archive/master.zip

MQTT pubsubclient
https://github.com/Imroy/pubsubclient/archive/master.zip

## Lighting Control

Libraries

LED Fader
https://github.com/jgillick/arduino-LEDFader/archive/master.zip

MQTT pubsubclient
https://github.com/knolleary/pubsubclient/archive/master.zip

Simple elapsed timer
https://github.com/pfeerick/elapsedMillis

## Other Libraries of interest

LED Abstraction
http://arduino-info.wikispaces.com/HAL-LibrariesUpdates

Finite State Machine
http://arduino-info.wikispaces.com/HAL-LibrariesUpdates


