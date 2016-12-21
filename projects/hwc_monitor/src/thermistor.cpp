#include <thermistor.h>

double thermistor10k(const double vTherm, const double vSupply, const double rDivider)
{
  // Assumes thermistor is lower part of resistor divider

  // Thermister parameters
  #define THERM_R1 10000
  #define THERM_R2 3950
  #define THERM_T2 2.98E+02

  double rTh = (vSupply / vTherm) * rDivider - rDivider;

  double tAbsInv = (1.0 / THERM_T2) - log(THERM_R1 / rTh) / THERM_R2;
  double tAbs = 1.0 / tAbsInv;

  return tAbs - 273.15;
}
