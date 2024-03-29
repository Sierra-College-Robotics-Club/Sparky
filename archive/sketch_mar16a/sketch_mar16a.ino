#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

// uno r3
// #define leftMotor1 3
// #define leftMotor2 5
// #define rightMotor1 10
// #define rightMotor2 11


// atremis
// B on motor driver
// currently right motor
// Motor A is left Motor B is right
// A2 is pin 2
// A1 is pin 3
// L1 : 2, L2 : 3, R1 : 4, R2 : 5 | old

// A on motor driver
// currently left motor
// B2 is 5
// B1 is 4
#define rightMotor1 2 //A
#define rightMotor2 4 //A
#define rightMotorEnable 10
#define rightEncoder 6

#define leftMotor1 8
#define leftMotor2 9
#define leftMotorEnable 7
#define leftEncoder 3

// IR sensor pins
// #define leftIR 9
// #define rightIR 10

// Possible limitation for the motor driver and PWM signal setup
// analog writing 254 seems to usually work, 253 has never failed
// now in rpm, ultimately these will be experimental
const int maxPossSpeed = 253;
const int minPossSpeed = 150;

const int encoderDelay = 1000; // delay for the encoder count readings for PID in ms
const int EISRDelay = 2; // delay for debouncing encoder ISRs
const int ISRDelay = 100; // delay for debouncing IR ISRs

// these need to be experimental tests, in RPM
// for now lets say 40 and 60
// in analog write that is somewhere around 200-254
const int maxSetSpeed = 45;
const int minSetSpeed = 55; //need to test to find real value
 
// these are also experimental values, needed to find
const int maxAnalog = 254;
const int minAnalog = 200;


volatile bool rightIRState = false;
volatile bool leftIRState = false;

volatile int dCountLIR;
volatile int dCountRIR;

long lastTimeRISR = 0;
long lastTimeLISR = 0;
long lastTimeREISR = 0;
long lastTimeLEISR = 0;




long rPIDInt = millis();
// short sp;
// short err;
// short cv;
// short pv; // calculated from encoder velcory

short kp = 1;
short ki = 0.05;
short kd = 0.01;

short lIntP;
short lErrP;

short rIntP;
short rErrP;

short rPrevPIDTime = millis();
short lPrevPIDTime = millis();

long rEncoderCount = 0;
long lEncoderCount = 0;

// this funciton takes a boolean for polarity and integer speed in RPM for angular velocity
// this also uses a PID loop to compare the encoder values of the wheels to adjust the speed the wheel
void rightMotorSpeed(bool dir, int speed){
  // validating speed as min and max set value in rpm
  if(speed > maxSetSpeed) speed = maxSetSpeed;
  if(speed < minSetSpeed) speed = minSetSpeed;

  long fEncoderCount = rEncoderCount;
  delay(encoderDelay); // wait 100 ms for sample
  long sEncoderCount = rEncoderCount;

  short sp = speed; 

  // pv is the rotations per units time, currently encoderDelay in ms
  short pv = ((sEncoderCount-fEncoderCount)/(74.83*12))/encoderDelay; // 74.83 rotations of motor shaft per wheel rotation (gear box) * 12 encoder rising edges per motor shaft (encoder ticks)
  short err = sp - pv;

  short cycleTime = millis() -  rPrevPIDTime;
  short I = rIntP + ki * err * cycleTime;

  short cv = kp * err + I + kd * (err - rErrP) / cycleTime;

  rPrevPIDTime = millis();
  rErrP = err;
  rIntP = I;

  // validating speed as min and max possible value as rpm
  if(cv > maxPossSpeed) cv = maxPossSpeed;
  if(cv < minPossSpeed) cv = minPossSpeed;

  // mapping the valid range cv (rpm) to valid range analogMin, analogMax
  // assuming analogMax will give maxPossSpeed linearly
  // analogSpeed takes cv(rpm) and multiplies it by the ratio of the range of analog signals to the range of rpm
  int analogSpeed = floor(cv*((maxAnalog-minAnalog)/(maxPossSpeed-minPossSpeed)));
  
  // validating output of analog write mapping as between min and max possible analog signals
  if(analogSpeed > maxAnalog) analogSpeed = maxAnalog;
  if(analogSpeed < minAnalog) analogSpeed = minAnalog;

  //if true, go forwards
  if(dir == true){
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorEnable, analogSpeed);
  }
  //else, go backwards
  else{
    digitalWrite(rightMotor1, LOW); 
    digitalWrite(rightMotor2, HIGH); 
    analogWrite(rightMotorEnable, analogSpeed);
    }
}

void leftMotorSpeed( bool dir, int speed){
  // validating speed as min and max set value in rpm
  if(speed > maxSetSpeed) speed = maxSetSpeed;
  if(speed < minSetSpeed) speed = minSetSpeed;

  long fEncoderCount = lEncoderCount;
  delay(encoderDelay); // wait 100 ms for sample
  long sEncoderCount = lEncoderCount;

  short sp = speed; 

  // pv is the rotations per units time, currently encoderDelay in ms
  short pv = ((sEncoderCount-fEncoderCount)/(74.83*12))/encoderDelay; // 74.83 rotations of motor shaft per wheel rotation (gear box) * 12 encoder rising edges per motor shaft (encoder ticks)
  short err = sp - pv;

  short cycleTime = millis() -  lPrevPIDTime;
  short I = lIntP + ki * err * cycleTime;

  short cv = kp * err + I + kd * (err - lErrP) / cycleTime;

  lPrevPIDTime = millis();
  lErrP = err;
  lIntP = I;

  // validating speed as min and max possible value as rpm
  if(cv > maxPossSpeed) cv = maxPossSpeed;
  if(cv < minPossSpeed) cv = minPossSpeed;

  // mapping the valid range cv (rpm) to valid range analogMin, analogMax
  // assuming analogMax will give maxPossSpeed linearly
  // analogSpeed takes cv(rpm) and multiplies it by the ratio of the range of analog signals to the range of rpm
  int analogSpeed = floor(cv*((maxAnalog-minAnalog)/(maxPossSpeed-minPossSpeed)));

  if(analogSpeed > maxAnalog) analogSpeed = maxAnalog;
  if(analogSpeed < minAnalog) analogSpeed = minAnalog;

  //if true, go forwards
  if(dir == true){
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);  
    analogWrite(leftMotorEnable, analogSpeed);  
  }
  //else, go backwards
  else{
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    analogWrite(leftMotorEnable, analogSpeed);
    }
}

void rotateRobot(bool dir, int speed){
  if(speed > maxPossSpeed) speed = maxPossSpeed;
  if(speed < minPossSpeed) speed = minPossSpeed;

  // we might want to do something tricky with the speeds being passed to each motor

  //if true, roatate clockwise
  if(dir == true){
    leftMotorSpeed(true, speed);
    rightMotorSpeed(false, speed);
  }

  //else, rotate counterclockwise
  if(dir == false){
    leftMotorSpeed(false, speed);
    rightMotorSpeed(true, speed);
  }
}

void RISR(){
  // should only swap 10 times a sec
  if(millis() - lastTimeRISR > ISRDelay){
    if(rightIRState == false) rightIRState = true;
    if(rightIRState == true) rightIRState = false;
    lastTimeRISR = millis();
  }
  
}

void LISR(){
  // should only swap 10 times a sec
  if(millis() - lastTimeLISR > ISRDelay){
    if(leftIRState == false) leftIRState = true;
    if(leftIRState == true) leftIRState = false;
    lastTimeLISR = millis();
  }
}


void REISR(){
  if(millis() - lastTimeREISR > EISRDelay){
    rEncoderCount ++;
    lastTimeREISR = millis();
  }
}

void LEISR(){
  if(millis() - lastTimeLEISR > EISRDelay){
    lEncoderCount ++;
    lastTimeLEISR = millis();
  }
}

// privilege right IR sensor
// turn ccw until only the right IR is active
// the stop and turn cw
// measure time util only left IR is active
// turn ccw for half time
// both IR should be active
// fire should be centered on fan

// drive forwards some constant measured distance, from testing
// robot_move is forwards

int fireExtinguish(bool leftIRState, bool rightIRState, int speed){
  int rotate_start_time = millis();

  if( ((rightIRState == true) && (leftIRState == false)) || ((rightIRState == false) && (leftIRState == true))) {
    if(rightIRState == true){
      rotateRobot(true, 250);
    }
    else if(leftIRState == true){
      rotateRobot(false, 250);
    }
  } else {
    // go forward robot
  }
  return 1;
  
}

void setup() {
 // hi
 // 254 seems to inconsistently work
 // 253 works
  //leftMotorSpeed(true, 254);
  // guess forwards
  //rightMotorSpeed(true, 254);

  attachInterrupt(digitalPinToInterrupt(leftIR), RISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightIR), LISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(leftEncoder), REISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoder), LEISR, FALLING);
}

void loop() {

  //lets test the directions of the motors
  // guess forwards this is true
  //leftMotorSpeed(true, 255);
  // guess forwards
 // rightMotorSpeed(true, 255);
  
  rightMotorSpeed(true, 250);
  leftMotorSpeed(true, 250);
  delay(1000);
  rotateRobot(true, 255);
  delay(1000);
  leftMotorSpeed(true, 250);
  rightMotorSpeed(true, 250);
  delay(1000);
  rotateRobot(false, 255);
  delay(1000);
  leftMotorSpeed(false, 250);
  leftMotorSpeed(false, 250);
  delay(1000);





  /*
  // motors should start from as close to the begginning of the loop
  // as practical to delay power cycle motor delay

  /*

  delay(3000);


  // needed to reset motor power signal on motor power on/off
  */

}