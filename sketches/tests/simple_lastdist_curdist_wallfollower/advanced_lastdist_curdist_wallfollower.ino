#include <NewPing.h>
#define MAX_DISTANCE 300

NewPing backsonar(29, 28, MAX_DISTANCE);
NewPing frontsonar(27, 26, MAX_DISTANCE);

#define LEnable 9
#define LIN1 10
#define LIN2 11

#define REnable 5
#define RIN1 6
#define RIN2 7

int analogMax = 250;
int analogMin = 50;
int motorStart = 200;
int LcurrentMotorSpeed = 0;
int RcurrentMotorSpeed = 0;
// 1 for cw, 0 for ccw
int currentDirection = 0;

int cwLeftSpeed = 100;
int cwRightSpeed = 80;

int ccwLeftSpeed = 80;
int ccwRightSpeed = 100;

float history[4] = {0, 0, 0, 0};


/*
 . start by turning into the wall, ccw for left wall

loop:

. continousously read distance data from one ultrasonic

 . save the previous 4 distance readings

 . when two consecutively are larger than the previous,
   turn away from the wall, cw

 . continousously read distance data from one ultrasonic

 . save the previous 4 distance readings

 . when two consecutively are larger than the previous,
   turn away from the wall, ccw

*/

void setup() {
  Serial.begin(9600);

  // put your setup code here, to run once:
  pinMode(LIN1, OUTPUT);
  pinMode(LIN2, OUTPUT);
  pinMode(LEnable, OUTPUT);

  pinMode(RIN1, OUTPUT);
  pinMode(RIN2, OUTPUT);
  pinMode(REnable, OUTPUT);

  digitalWrite(LIN1, HIGH);
  digitalWrite(LIN2, LOW);
  analogWrite(LEnable, 250);
  delay(20);
  analogWrite(LEnable, 50);

  digitalWrite(RIN1, HIGH);
  digitalWrite(RIN2, LOW);
  analogWrite(REnable, 250);
  delay(20);
  analogWrite(REnable, 50);

  // start by turning into the wall and setting currentDirection
  rMotor(ccwLeftSpeed, true);
  lMotor(ccwRightSpeed, true);
  currentDirection = 0;
}

void rMotor(int speed, bool fwd) {
  if (RcurrentMotorSpeed == 0){
    speed = motorStart;
  }
  if (speed > analogMax) {
    speed = analogMax;
  }
  else if(speed < 50){
    speed = analogMin;
  }

  if (fwd) {
    digitalWrite(RIN1, HIGH);
    digitalWrite(RIN2, LOW);
  } else {
    digitalWrite(RIN1, LOW);
    digitalWrite(RIN2, HIGH);
  }

  RcurrentMotorSpeed = speed;
  analogWrite(REnable, speed);
}

void lMotor(int speed, bool fwd) {
  if (LcurrentMotorSpeed == 0){
    speed = motorStart;
  }
  if (speed < analogMax) {
    speed = 250;
  }

  else if(speed < 50){
    speed = analogMin;
  }

  if (fwd) {
    digitalWrite(LIN1, HIGH);
    digitalWrite(LIN2, LOW);
  } else {
    digitalWrite(LIN1, LOW);
    digitalWrite(LIN2, HIGH);
  }

  LcurrentMotorSpeed = speed;
  analogWrite(LEnable, speed);
}

void loop() {
  delay(100);
  // Varible Assignment //
  //* float backdist = backsonar.convert_cm(backsonar.ping_median(5)); // this is not required for this method we only need front
  float frontdist = frontsonar.convert_cm(frontsonar.ping_median(5));

  // populate the history for this cycle
  history[3] = history[2];
  history[2] = history[1];
  history[1] = history[0];
  history[0] = frontdist;

  ////////////////////////

  ////////////////////////

  // Debug printing //
  //* delay(50);
  //* Serial.println("============");
  //* Serial.print("back  Ping: ");
  //* Serial.print(backdist);
  //* Serial.println("cm");

  //* delay(20);

  Serial.print("front Ping: ");
  Serial.print(frontdist);
  Serial.println("cm");
  ////////////////////

  // if distance is starting to increese, switch turning direction
  // potential issure if this populates quicker than neccessarliy
  if ((history[0] > history[2]) && (history[1] > history[3])) {
    if(currentDirection) {//turn ccw and set dir
      rMotor(ccwLeftSpeed, true); lMotor(ccwRightSpeed, true);currentDirection = 0; }
    else {
      //turn cw and set dir
      rMotor(cwLeftSpeed, true); lMotor(cwRightSpeed, true);
      currentDirection = 1;
    }
  }
}
