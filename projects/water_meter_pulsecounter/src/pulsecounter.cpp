
#include <Arduino.h>
#include <Timer.h>

#define PIN_PULSEIN PB0
#define PIN_PULSEOUT PB1

// Max of ~250 otherwise need a counter of larger size
#define PULSEDIVISOR 135
// Milliseconds
#define PULSEDURATION 10

volatile char pulseCount;
Timer timer;

void isr()
{
  pulseCount++;
}

void setup()
{
  pinMode(PIN_PULSEIN, INPUT);
  pinMode(PIN_PULSEOUT, OUTPUT);
  digitalWrite(PIN_PULSEOUT, LOW);

  pulseCount = 0;

  attachInterrupt(PIN_PULSEIN, isr, FALLING);
}

void loop()
{
  if (pulseCount >= PULSEDIVISOR)
  {
    timer.pulse(PIN_PULSEOUT, 10, HIGH);
    pulseCount -= PULSEDIVISOR;
  }

  timer.update();
}
