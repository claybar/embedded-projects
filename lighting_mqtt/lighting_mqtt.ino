/*
Overall operation implemeted as a FSM.

States:
- Daytime (LEDs always off)
- Dusk (LEDs on dimly)
- Dawn (LEDs on dimly)
- Night (LEDs off)
- Motion (LEDs on bright)
- Alarm (LEDs flashing)

Transitions triggered by clock:
Daytime -> Dusk
Dusk -> Night
Night -> Dawn
Dawn -> Daytime




*/

#include <LED.h>

const int activityLedPin = 13;
const int motionSensorAPin = 2;
const int motionSensorBPin = 4;
const int lightingPin = 4;

int lightingLevelAmbient = 10; // percentage
int lightingLevelBright = 50;  // percentage

int motionAState = 0;
int motionBState = 0;
int previousMotionAState = 0;
int previousMotionBState = 0;

LED activityLed = LED(activityLedPin);

void setup() {
  pinMode(motionSensorAPin, INPUT);
  pinMode(motionSensorBPin, INPUT);
  
  //pinMode(activityLedPin, OUTPUT);
  pinMode(lightingPin, OUTPUT);
  
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

      activityLed.fadeIn(1000);

      //digitalWrite(activityLedPin, HIGH);
    }
    else
    {
      Serial.println("Motion gone.");

      activityLed.fadeOut(1000);

      //digitalWrite(activityLedPin, LOW);
    }

    previousMotionAState = motionAState;
    // Delay a little bit to avoid bouncing
    delay(50);
  }
}



