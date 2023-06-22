#include <NewPing.h>
#include "MOTOR.h"

#define MAX_DISTANCE 800

NewPing sonar(29, 28, MAX_DISTANCE);
NewPing sonar2(27, 26, MAX_DISTANCE);

MOTOR lmotor(10, 11, 9);
MOTOR rmotor(6, 7, 5);
MOTOR_CONTROL motorcontrol(lmotor, rmotor);

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(100);
  // motorcontrol.swivelLeft(80);

  // lmotor.setSpeed(210, true);
  // rmotor.setSpeed(210, false);
  motorcontrol.swivelLeft(80);
  delay(100);
  motorcontrol.swivelRight(80);
}