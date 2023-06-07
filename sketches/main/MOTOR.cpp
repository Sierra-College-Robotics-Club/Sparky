#include "MOTOR.h"

MOTOR::MOTOR(int inputDirPin1, int inputDirPin2, int inputEnablePin, int input_ANALOG_MAX, int input_ANALOG_MIN, int input_ANALOG_START) {
  pinMode(inputDirPin1, OUTPUT);
  pinMode(inputDirPin2, OUTPUT);
  pinMode(inputEnablePin, OUTPUT);

  digitalWrite(inputDirPin1, LOW);
  digitalWrite(inputDirPin2, LOW);
  analogWrite(inputEnablePin, 0);

  dirPin1 = inputDirPin1;
  dirPin2 = inputDirPin2;
  enablePin = inputEnablePin;

  ANALOG_MAX = input_ANALOG_MAX;
  ANALOG_MIN = input_ANALOG_MIN;
  ANALOG_START = input_ANALOG_START;
}

void MOTOR::setSpeed(int speed, bool direction) {
    if (currentSpeed == 0) speed = ANALOG_START; // give the motor a spike to start if needed

    // ensure speed is within bounds
    if (speed > ANALOG_MAX) speed = ANALOG_MAX;
    else if(speed < ANALOG_MIN) speed = ANALOG_MIN;

    // set direction pins
    if (direction) {
        digitalWrite(dirPin1, HIGH);
        digitalWrite(dirPin2, LOW);
    } else {
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, HIGH);
    }

    // write speed
    currentSpeed = speed;
    analogWrite(enablePin, speed);
}

void MOTOR::stop() {
    currentSpeed = 0;
    analogWrite(enablePin, 0);
}