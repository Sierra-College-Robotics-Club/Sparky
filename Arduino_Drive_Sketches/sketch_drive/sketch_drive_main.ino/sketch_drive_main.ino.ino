#include <math.h>
#include <string.h>

#define fanenable 40
const int edgedetectorpin = A2;

#define rightMotor1 8 //B
#define rightMotor2 9
#define rightMotorEnable 10
#define rightEncoder 18

#define leftMotor1 11 //A
#define leftMotor2 12
#define leftMotorEnable 13
#define leftEncoder 2

#define leftirpin 30
#define rightirpin 31

// ULTRASONICS
#define trigPin1 22 // fwd
#define echoPin1 23

#define trigPin2 24 // left
#define echoPin2 25

#define trigPin3 26 // right
#define echoPin3 27

const int encoderDelay = 100; // delay for the encoder count readings for PID in ms
const int EISRDelay = 2; // delay for debouncing encoder ISRs
const int ISRDelay = 100; // delay for debouncing IR ISRs

const int maxAnalog = 230;
const int minAnalog = 100;

volatile bool rightIRState = false;
volatile bool leftIRState = false;

volatile int dCountLIR;
volatile int dCountRIR;

long lastTimeRISR = 0;
long lastTimeLISR = 0;
long lastTimeREISR = 0;
long lastTimeLEISR = 0;

long rEncoderCount = 0;
long lEncoderCount = 0;

int navmode = 1;

float get_distance(int echoPin, int trigPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
 
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
 
    digitalWrite(trigPin, LOW);
    float duration = pulseIn(echoPin, HIGH);
    float distance = duration * 0.0344 / 2;

    return distance;
}

// this funciton takes a boolean for polarity and integer speed in RPM for angular velocity
// this also uses a PID loop to compare the encoder values of the wheels to adjust the speed the wheel
void rightMotorSpeed(bool dir, int speed){
  if (speed > maxAnalog) speed = maxAnalog;

  //if true, go forwards
  if(dir == true){
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorEnable, speed);
  }
  //else, go backwards
  else{
    digitalWrite(rightMotor1, LOW); 
    digitalWrite(rightMotor2, HIGH); 
    analogWrite(rightMotorEnable, speed);
    }
}

void leftMotorSpeed( bool dir, int speed){
  if (speed > maxAnalog) speed = maxAnalog;

  // if (speed <= 1) {
  //   digitalWrite(leftMotorEnable, LOW);
  //   return;
  // }

  //if true, go forwards
  if(dir == true){
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);  
    analogWrite(leftMotorEnable, speed);  
  }
  //else, go backwards
  else{
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    analogWrite(leftMotorEnable, speed);
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

void setup() {
  pinMode(fanenable, OUTPUT);
  pinMode(edgedetectorpin, INPUT);
  pinMode(leftirpin, INPUT);
  pinMode(rightirpin, INPUT);

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

void closest_point_on_line(int projection[2], float point1[2], float point2[2]) {
    float position[2] = {0, 0};
    float line_vec[2] = {point2[0] - point1[0], point2[1] - point1[1]};
    float point_vec[2] = {position[0] - point1[0], position[1] - point1[1]};
    float line_len = sqrt(pow(line_vec[0], 2) + pow(line_vec[1], 2));
    float line_unitvec[2] = {line_vec[0] / line_len, line_vec[1] / line_len};
    float projection_len = point_vec[0] * line_unitvec[0] + point_vec[1] * line_unitvec[1];
    projection[0] = point1[0] + line_unitvec[0] * projection_len;
    projection[1] = point1[1] + line_unitvec[1] * projection_len;
}

const float desiredDistance = 10.0; // in centimeters
const int regularSpeed = 135;
const int speedupSpeed = 110;
const int slowdownSpeed = 40;
const int adjustSpeed = 12;

int turningstate = 0;
int leftturning = false;
int startingencoderticks = 0;

int turnthreashold = 46;

void turn_left() {
  leftMotorSpeed(true, 0);
  rightMotorSpeed(true, 0);
  delay(500);
  startingencoderticks = rEncoderCount;
  turningstate = -1;
  leftturning = true;
}

void turn_right() {
  leftMotorSpeed(true, 0);
  rightMotorSpeed(true, 0);
  delay(500);
  leftMotorSpeed(false, 255);
  rightMotorSpeed(false, 255);
  delay(500);
  leftMotorSpeed(true, 0);
  rightMotorSpeed(true, 0);
  delay(500);
  startingencoderticks = lEncoderCount;
  turningstate = 1;
  leftturning = false;
}

void loop() {
  int leftir = digitalRead(leftirpin);
  int rightir = digitalRead(rightirpin);

  int edgedetector = analogRead(edgedetectorpin);
    Serial.println(edgedetector);
  if (edgedetector < 100) {
    edgedetector = 1;
  }
  if (leftir == 1 || rightir == 1) {
    navmode = 2;
  }
  if (navmode == 2) {
    if (edgedetector == 1) {
      digitalWrite(fanenable, HIGH);
      if (0 == 1) {
        leftMotorSpeed(true, 130);
        rightMotorSpeed(false, 130);
        delay(200);
        leftMotorSpeed(true, 0);
        rightMotorSpeed(false, 0);
        delay(1000);

        leftMotorSpeed(false, 130);
        rightMotorSpeed(true, 130);
        delay(200);
        leftMotorSpeed(true, 0);
        rightMotorSpeed(false, 0);
        delay(1000);
      }
    } else {

      if (leftir == 1 && rightir == 1) {
        leftMotorSpeed(true, 0);
        rightMotorSpeed(true, 0);
        delay(300);
        leftMotorSpeed(true, 130);
        rightMotorSpeed(true, 130);
        delay(300);
      } else if (leftir == 1) {
        leftMotorSpeed(true, 0);
        rightMotorSpeed(true, 130);
      } else if (rightir == 1) {
        leftMotorSpeed(true, 130);
        rightMotorSpeed(true, 0);      
      } else {
        leftMotorSpeed(true, 130);
        rightMotorSpeed(false, 130);
        delay(200);
        leftMotorSpeed(true, 0);
        rightMotorSpeed(false, 0);
        delay(1000);

        leftMotorSpeed(false, 130);
        rightMotorSpeed(true, 130);
        delay(200);
        leftMotorSpeed(true, 0);
        rightMotorSpeed(false, 0);
        delay(1000);
      }
    }
  } 
  if (navmode == 1) {
    float fwd = get_distance(echoPin1, trigPin1);  
    float leftfront = get_distance(echoPin2, trigPin2);
    delay(10);
    float leftback = get_distance(echoPin3, trigPin3);
    delay(10);

    int encoderChange;
    if (turningstate != 0) {
      if (leftturning) {
        encoderChange = rEncoderCount - startingencoderticks;
      } else {
        encoderChange = lEncoderCount - startingencoderticks;
      }
      if (turningstate == -1) {
        leftMotorSpeed(false, 255);
        rightMotorSpeed(false, 255);
        delay(500);
        leftMotorSpeed(true, 255);
        rightMotorSpeed(true, 255);
        delay(350);
        leftMotorSpeed(false, 0);
        rightMotorSpeed(false, 0);
        startingencoderticks = rEncoderCount;
        turningstate = 1;
        delay(200);
      } else if (turningstate == 1) {
        if (leftturning) {
          leftMotorSpeed(true, 0);
          rightMotorSpeed(true, 135);
        } else {
          leftMotorSpeed(true, 135);
          rightMotorSpeed(true, 0);
        }
        if (leftturning) {
          if (encoderChange > 350) {
            turningstate = 2;
            startingencoderticks = rEncoderCount;
          }
        } else {
          if (encoderChange > 350) {
            turningstate = 0;
            startingencoderticks = rEncoderCount;
          }
        }
      } else if (turningstate == 2) {    
        leftfront = get_distance(echoPin2, trigPin2); 
        if (leftfront <= 22 && leftback <= 22) {
          turningstate = 0;
        } else if (leftfront <= 400) {
          leftMotorSpeed(true, 255);
          rightMotorSpeed(true, 255);
          delay(700);
          leftMotorSpeed(true, 0);
          rightMotorSpeed(true, 0);
          turningstate = 0;
          delay(200);
        } else {
          // startingencoderticks = rEncoderCount;

          leftMotorSpeed(true, 255);
          rightMotorSpeed(true, 255);

          delay(700);
          leftMotorSpeed(true, 0);
          rightMotorSpeed(true, 0);

          turningstate = -1;
          delay(500);
          // turningstate = 0;
        }
      } else if (turningstate == 3) {

      }
    } else {
      int projection[2];
      float point1[2] = {13, leftfront};
      float point2[2] = {0, leftback};
      closest_point_on_line(projection, point1, point2);
      double diff = leftfront - leftback;
      float distance = hypot(projection[0], projection[1]);
      float offset = distance - desiredDistance;
      
      int speedmodL = 0;
      int speedmodR = 0;

      int maxspeedmox = 125;

      if (offset > 0) {
        speedmodR = abs(offset) * adjustSpeed;
        if (speedmodR > maxspeedmox) {
          speedmodR = maxspeedmox;
        }
      } else {
        speedmodL = (abs(offset) * adjustSpeed);
        if (speedmodL > maxspeedmox) {
          speedmodL = maxspeedmox;
        }
      }

  ////////////
      if (fwd > 900) {
        leftMotorSpeed(false, 130);
        rightMotorSpeed(false, 130);
        delay(1000);
        leftMotorSpeed(false, 0);
        rightMotorSpeed(false, 0);
        delay(200);
      } else if (leftfront >= 22) {
        turn_left();
      } else if (fwd <= 8) {
        turn_right();    
      }

      int cspeedup = abs(diff * 70);

      if (diff > 0) {
        leftMotorSpeed(true, regularSpeed - cspeedup + speedmodL);
        rightMotorSpeed(true, regularSpeed + cspeedup + speedmodR);
      } else if (diff < 0) {
        if (abs(diff) < 280) {
          leftMotorSpeed(true, regularSpeed + cspeedup + speedmodL); // regular right case
          rightMotorSpeed(true, regularSpeed - cspeedup + speedmodR);
        } else {
          leftMotorSpeed(true, regularSpeed - cspeedup);
          rightMotorSpeed(true, regularSpeed + cspeedup);
        }
      }
    } 
  }

}