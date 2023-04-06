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
#define rightMotor1 8 //B
#define rightMotor2 9
#define rightMotorEnable 10
#define rightEncoder 18

#define leftMotor1 11 //A
#define leftMotor2 12
#define leftMotorEnable 13
#define leftEncoder 2


// ULTRASONICS
#define trigPin1 22 // fwd
#define echoPin1 23

#define trigPin2 24 // left
#define echoPin2 25

#define trigPin3 26 // right
#define echoPin3 27

// IR sensor pins
// #define leftIR 9
// #define rightIR 10

// Possible limitation for the motor driver and PWM signal setup
// analog writing 254 seems to usually work, 253 has never failed
// now in rpm, ultimately these will be experimental
const int maxPossSpeed = 253;
const int minPossSpeed = 150;

const int encoderDelay = 100; // delay for the encoder count readings for PID in ms
const int EISRDelay = 2; // delay for debouncing encoder ISRs
const int ISRDelay = 100; // delay for debouncing IR ISRs

// these need to be experimental tests, in RPM
// for now lets say 40 and 60
// in analog write that is somewhere around 200-254
const int maxSetSpeed = 45;
const int minSetSpeed = 55; //need to test to find real value
 
// these are also experimental values, needed to find
const int maxAnalog = 230;
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

int get_distance(int echoPin, int trigPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
 
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
 
    digitalWrite(trigPin, LOW);
    int duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.0344 / 2;

    return distance;
}

// this funciton takes a boolean for polarity and integer speed in RPM for angular velocity
// this also uses a PID loop to compare the encoder values of the wheels to adjust the speed the wheel
void rightMotorSpeed(bool dir, int speed){
  // validating speed as min and max set value in rpm
  if(speed > maxSetSpeed) speed = maxSetSpeed;
  if(speed < minSetSpeed) {
    analogWrite(rightMotorEnable, 0);
    return;
  }

  if (rErrP < 1) rErrP = 5;

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
  if(speed < minSetSpeed) {
    analogWrite(leftMotorEnable, 0);
    return;
  }

  if (lErrP < 1) lErrP = 5;

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

int rotationalspeed = 201;
int movespeedstd = 201;

void setup() {
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorEnable, OUTPUT);
  pinMode(rightEncoder, INPUT);

  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorEnable, OUTPUT);
  pinMode(leftEncoder, INPUT);

  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotorEnable, LOW);
  digitalWrite(leftMotorEnable, LOW);

  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(leftEncoder), LEISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoder), REISR, RISING);
}

//cm
int optimalleftdist = 50;
int unoptimalfwddist = 50;
int rightthreashhold = 100;
int leftthreashhold = 100;

bool turning = false;
bool leftturning = false;
int startturnticks = 0;
int desiredticks = 3000;

void left_turn() {
  leftturning = true;
  turning = true;
  startturnticks = lEncoderCount;
  leftMotorSpeed(true, 0);
  rightMotorSpeed(true, 255);
}

void right_turn() {
  leftturning = false;
  turning = true;
  startturnticks = rEncoderCount;
  leftMotorSpeed(true, 255);
  rightMotorSpeed(true, 0);
}

void stop() {
  leftMotorSpeed(true, 0);
  rightMotorSpeed(true, 0);
}

void go_forward() {
  leftMotorSpeed(true, 255);
  rightMotorSpeed(true, 255);
}

void stop_turning() {
  turning = false;
}

void loop() {
  // int fwd = get_distance(echoPin1, trigPin1);
  // int left = get_distance(echoPin2, trigPin2);
  // int right = get_distance(echoPin3, trigPin3);
  // // Serial.print("FWD ");
  // // Serial.println(fwd);
  // // Serial.print("LEFT ");
  // // Serial.println(left);
  // // Serial.print("RIGHT ");
  // // Serial.println(right);
  // if (!turning) {
  //   if (left > leftthreashhold) {
  //     left_turn();
  //     Serial.println("left");
  //   } else if (right > rightthreashhold && fwd <= unoptimalfwddist) {
  //     right_turn();
  //     Serial.println("right");
  //   } else if (fwd > unoptimalfwddist) {
  //     go_forward();
  //     Serial.println("fwd");
  //   } else {
  //     Serial.println("stop");
  //     stop();
  //   }
  // } else {
  //   if (leftturning) {
  //     if ((lEncoderCount - startturnticks) >= desiredticks) {
  //       stop_turning();
  //     }
  //   } else {
  //     if ((lEncoderCount - startturnticks) >= desiredticks) {
  //       stop_turning();
  //     }
  //   }
  // }   
}