#include "MOTOR.h"

MOTOR::MOTOR(int inputDirPin1, int inputDirPin2, int inputEnablePin) {
  pinMode(inputDirPin1, OUTPUT);
  pinMode(inputDirPin2, OUTPUT);
  pinMode(inputEnablePin, OUTPUT);

  digitalWrite(inputDirPin1, LOW);
  digitalWrite(inputDirPin2, LOW);
  analogWrite(inputEnablePin, 0);

  dirPin1 = inputDirPin1;
  dirPin2 = inputDirPin2;
  enablePin = inputEnablePin;
}

//direction: true forwards, false backwards
void MOTOR::setSpeed(int speed, bool direction) {
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
    
    // give the motor a spike to start if needed
    if (currentSpeed == 0) {
      speed = ANALOG_START;
      analogWrite(enablePin, speed);
      delay(ANALOG_START_TIME)
    } 
  
    // write speed
    currentSpeed = speed;
    analogWrite(enablePin, speed);
}

void MOTOR::stop() {
    currentSpeed = 0;
    analogWrite(enablePin, 0);
}

/////////////////////////////////
//////////MOTOR CONTROL///////O//
/////////////////////////////////


MOTOR_CONTROL::MOTOR_CONTROL(MOTOR& inputLeftMotor, MOTOR& inputRightMotor, int inputSpeedBalance) {
    speedBalance = inputSpeedBalance;
    leftMotor = inputLeftMotor;
    rightMotor = inputRightMotor;
}

int MOTOR_CONTROL::contextializeSpeed(int speed) {
    if(speed > 100) speed = 100;
    if(speed < 0) speed = 0;

    // bound speed to the max and min speed
    int speedRatio = (int)(speed / ANALOG_MAX - ANALOG_MIN);
    speed = speedRatio * 100 + ANALOG_MIN;

    return speed;
}

void MOTOR_CONTROL::turnLeft(int turnspeed, int speed) {
    speed = contextializeSpeed(speed);

    if (speed-turnspeed <= 0) {
        leftMotor.stop();
    } else {
        leftMotor.setSpeed(speed - turnspeed, true);
    }
    rightMotor.setSpeed(speed, true);
}

void MOTOR_CONTROL::turnRight(int turnspeed, int speed) {
    speed = contextializeSpeed(speed);

    if(speed-turnspeed <= 0) {
        rightMotor.stop();
    } else {
        rightMotor.setSpeed(speed - turnspeed, true);
    }
    leftMotor.setSpeed(speed, true);
}

void MOTOR_CONTROL::stop() {
    rightMotor.stop();
    leftMotor.stop();
}

void MOTOR_CONTROL::swivelLeft(int speed) {
    speed = contextializeSpeed(speed);
    leftMotor.setSpeed(speed, false);
    rightMotor.setSpeed(speed, true);
}

void MOTOR_CONTROL::swivelRight(int speed) {
    speed = contextializeSpeed(speed);
    leftMotor.setSpeed(speed, true);
    rightMotor.setSpeed(speed, false);
}
