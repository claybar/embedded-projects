Code for home automation projects.

# EmonCMS

## Raspberry Pi2
EmonCMS installed on a Raspberry Pi2
MQTTWarn used to pull messages from MQTT and feed into EmonCMS

## Power Monitoring

`emon_cont_direct/emon_cont_direct.ino`

EtherTen Arduino clone with a openenergy.org EmonTX Shield V2.  This has been modified from stock with the following changes:
- Ch4 rerouted to Analog pin 6.  This is so the on-board MAC address chip can be addressed via I2C
- Burden resistors on Ch2 & Ch4 changed to 180ohm to improve sensitivity.  These are used for monitoring a heat pump and house lights.

