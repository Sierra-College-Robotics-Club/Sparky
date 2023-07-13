#include "HardwareSerial.h"
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
      analogWrite(enablePin, ANALOG_START);
      delay(ANALOG_START_TIME);
    } 
  
    // write speed
    currentSpeed = speed;
    Serial.println(currentSpeed);
    analogWrite(enablePin, speed);
}

void MOTOR::stop() {
    currentSpeed = 0;
    analogWrite(enablePin, 0);
}

/////////////////////////////////
//////////MOTOR CONTROL//////////
/////////////////////////////////


MOTOR_CONTROL::MOTOR_CONTROL(MOTOR inputLeftMotor, MOTOR inputRightMotor, int inputSpeedBalance) {
    speedBalance = inputSpeedBalance;
    leftMotor = inputLeftMotor;
    rightMotor = inputRightMotor;
}

bool MOTOR_CONTROL::isTurningLeft() {
    if (leftMotor.currentSpeed > rightMotor.currentSpeed) {
      return true;
    }
    return false;
}

bool MOTOR_CONTROL::isTurningRight() {
    if (leftMotor.currentSpeed > rightMotor.currentSpeed) {
      return false;
    }
    return true;
}

int MOTOR_CONTROL::contextializeSpeed(int speed) {
    if(speed > 100) speed = 100;
    if(speed < 0) speed = 0;

    // bound speed to the max and min speed
    speed = (((float)speed / 100.0) * (ANALOG_MAX - ANALOG_MIN)) + ANALOG_MIN;
    return speed;
}

void MOTOR_CONTROL::turnLeft(int turnspeed, int speed) {
    int contextspeed = contextializeSpeed(speed);

    if (speed == 0) {
        leftMotor.stop();
    } else {
        leftMotor.setSpeed(contextspeed - turnspeed, true);
    }
    rightMotor.setSpeed(contextspeed, true);
}

void MOTOR_CONTROL::turnRight(int turnspeed, int speed) {
    int contextspeed = contextializeSpeed(speed);

    if (speed == 0) {
        leftMotor.stop();
    } else {
        leftMotor.setSpeed(contextspeed, true);
    }
    rightMotor.setSpeed(contextspeed - turnspeed, true);
}

void MOTOR_CONTROL::stop() {
    rightMotor.stop();
    leftMotor.stop();
}

void MOTOR_CONTROL::swivelLeft(int speed) {
    speed = contextializeSpeed(speed);
    // Serial.println(speed);
    leftMotor.setSpeed(speed, false);
    rightMotor.setSpeed(speed, true);
}

void MOTOR_CONTROL::swivelRight(int speed) {
    speed = contextializeSpeed(speed);
    // Serial.println(speed);
    leftMotor.setSpeed(speed, true);
    rightMotor.setSpeed(speed, false);
}
