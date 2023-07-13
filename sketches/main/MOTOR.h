#ifndef ANALOG_MAX
    #define ANALOG_MAX 250
#endif

#ifndef ANALOG_MIN
    #define ANALOG_MIN 50
#endif

#ifndef ANALOG_START
    #define ANALOG_START 200
#endif

#ifndef ANALOG_START_TIME
    #define ANALOG_START_TIME 20
#endif

#ifndef MOTOR_h
#define MOTOR_h

#include <Arduino.h>

class MOTOR {
private:
    int dirPin1, dirPin2;
    int enablePin;

public:
    int currentSpeed = 0;
    MOTOR() = default;
    MOTOR(int inputDirPin1, int inputDirPin2, int inputEnablePin);
    void setSpeed(int speed, bool direction);
    void stop();
};

class MOTOR_CONTROL {
private:
    MOTOR leftMotor;   // Use a reference to MOTOR objects
    MOTOR rightMotor;
    int speedBalance;
    int contextializeSpeed(int speed);

public:
    MOTOR_CONTROL(MOTOR inputLeftMotor, MOTOR inputRightMotor, int inputSpeedBalance = 0);
    bool isTurningLeft();
    bool isTurningRight();
    void turnLeft(int turnspeed, int speed = 255);
    void turnRight(int turnspeed, int speed = 255);
    void stop();
    void swivelLeft(int speed);
    void swivelRight(int speed);
};

#endif
