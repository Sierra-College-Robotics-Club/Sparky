#ifndef MOTOR_h
#define MOTOR_h

#include <Arduino.h>

class MOTOR {
    private:
        int dirPin1, dirPin2;
        int ANALOG_MAX, ANALOG_MIN, ANALOG_START;
        int enablePin;
        int currentSpeed = 0;
    public:
        MOTOR(int inputDirPin1, int inputDirPin2, int inputEnablePin, int input_ANALOG_MAX = 250, int input_ANALOG_MIN = 50, int input_MOTOR_START = 200);
        void setSpeed(int speed, bool direction);
        void stop();
};
#endif
