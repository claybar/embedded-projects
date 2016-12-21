/*
Philip Barclay
26 July 2016

Based on: https://github.com/jgillick/arduino-LEDFader
*/

#include "Curve.h"

// This table is 101 entries long to allow 0-100% settings
// Calc as =ROUND(EXP(LN(255) * B7 / 100), 0)
const uint8_t Curve::etable[101] PROGMEM =
{
  0,
  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,
  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,
  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
  6,  6,  6,  7,  7,  7,  8,  8,  9,  9,
 10, 10, 11, 11, 12, 13, 14, 14, 15, 16,
 17, 18, 19, 20, 21, 22, 24, 25, 26, 28,
 29, 31, 33, 35, 37, 39, 41, 43, 46, 48,
 51, 54, 57, 60, 64, 67, 71, 75, 80, 84,
 89, 94, 99,105,111,117,124,131,139,147,
155,164,173,183,193,204,216,228,241,255
};

uint8_t Curve::exponential(uint8_t i) {
 return pgm_read_byte(&etable[i]);
}
