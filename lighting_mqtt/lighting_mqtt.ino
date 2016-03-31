/*
Overall operation implemeted as a FSM.

States:
- Daytime (LEDs always off)
- Dusk (LEDs on dimly)
- Dawn (LEDs on dimly)
- Night (LEDs off)
- Motion (LEDs on bright)
- Alarm (LEDs flashing)

Transitions clock:
Daytime -> Dusk
Dusk -> Night
Night -> Dawn
Dawn -> Daytime

Within states [Dusk, Night, Dawn] motion



*/

//#include <LED.h>
#include <LEDFader.h>
#include <Curve.h>

const int activityLedPin = 13;
const int motionSensorAPin = 7;
const int motionSensorBPin = 6;
const int lightingPin = 9;

int lightingLevelAmbient = 10; // percentage
int lightingLevelBright = 50;  // percentage

int motionAState = 0;
int motionBState = 0;
int previousMotionAState = 0;
int previousMotionBState = 0;

//LED activityLed = LED(activityLedPin);
//LED lightingLed = LED(lightingPin);
//LEDFader activityLed = LEDFader(activityLedPin);

LEDFader lightingLed = LEDFader(lightingPin);

void setup() {
  pinMode(motionSensorAPin, INPUT);
  pinMode(motionSensorBPin, INPUT);
  
  //pinMode(activityLedPin, OUTPUT);
  pinMode(lightingPin, OUTPUT);
  
  
  lightingLed.set_curve(&Curve::exponential);

  Serial.begin(9600);
}

void loop() {
  motionAState = digitalRead(motionSensorAPin);
  motionBState = digitalRead(motionSensorBPin);

  // Trigger on positive edges
  if (motionAState != previousMotionAState)
  {
    if (motionAState == HIGH)
    {
      Serial.println("Motion detected, sensor A");

      //activityLed.fadeIn(1000);
      //lightingLed.fadeIn(1000);
      lightingLed.fade(255, 5000);

      //digitalWrite(activityLedPin, HIGH);
    }
    else
    {
      Serial.println("Motion gone.");

      //activityLed.fadeOut(1000);
      //lightingLed.fadeOut(1000);
      lightingLed.fade(70, 5000);

      //digitalWrite(activityLedPin, LOW);
    }

    previousMotionAState = motionAState;
    // Delay a little bit to avoid bouncing
    delay(50);
  }

  lightingLed.update();
}



