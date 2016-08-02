## Libraries of interest

Simple way of calculating sunrise/sunset:
http://forum.arduino.cc/index.php/topic,66426.msg487457.html#msg487457

### Timing

- Simple rolling aware elapsed timing (good for simple task duration timing like lighting delays) https://github.com/pfeerick/elapsedMillis

## Timer Usage

All timers default to around 1 kHZ in Arduino firmware.

Timer 0 (8 bit)
- Sketch timing
- Default 976 Hz
- PWM pins 5 and 6

Timer 1 (16 bit)
- Servo library
- Default 490 Hz
- PWM Pins 9 and 10

Timer 2 (8 bit)
- Default 490 Hz
- PWM Pins 11 and 3
