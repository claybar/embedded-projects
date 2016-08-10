/*
Philip Barclay
26 July 2016

Based on: https://github.com/jgillick/arduino-LEDFader
*/

#ifndef CURVE_H
#define	CURVE_H

#include <avr/pgmspace.h>

class Curve
{
  static const uint8_t etable[] PROGMEM;
  static const uint8_t qetable[] PROGMEM;
public:
  static uint8_t exponential(uint8_t);
  static uint8_t quasiExp(uint8_t);
};

#endif
