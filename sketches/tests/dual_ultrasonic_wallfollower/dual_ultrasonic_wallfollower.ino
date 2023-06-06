#include <NewPing.h>
  
#define MAX_DISTANCE 800
  
NewPing backsonar(29, 28, MAX_DISTANCE);
NewPing frontsonar(27, 26, MAX_DISTANCE);

#define LEnable 9
#define LIN1 10
#define LIN2 11

#define REnable 5
#define RIN1 6
#define RIN2 7

int analogMax = 250;

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
}

void lMotor(int speed, bool fwd) {
  if (speed > analogMax) {
    speed = analogMax;
  }

  if (fwd) {
    digitalWrite(RIN1, HIGH);
    digitalWrite(RIN2, LOW);
  } else {
    digitalWrite(RIN1, LOW);
    digitalWrite(RIN2, HIGH);
  }
  analogWrite(REnable, speed);
}

void rMotor(int speed, bool fwd) {
  if (speed < analogMax) {
    speed = 250;
  }

  if (fwd) {
    digitalWrite(LIN1, HIGH);
    digitalWrite(LIN2, LOW);
  } else {
    digitalWrite(LIN1, LOW);
    digitalWrite(LIN2, HIGH);
  }
  analogWrite(LEnable, speed);
}

void loop() {
  // get distances //
  delay(50);
  Serial.println("============");
  Serial.print("back  Ping: ");
  float backdist = backsonar.convert_cm(backsonar.ping_median(5));
  Serial.print(backdist);
  Serial.println("cm");

  delay(20);

  Serial.print("front Ping: ");
  float frontdist = frontsonar.convert_cm(frontsonar.ping_median(5));
  Serial.print(frontdist);
  Serial.println("cm");
  ///////////////////

  // set motors //

  // // test 1 //
  // float diff = backdist - frontdist;
  // if (diff < 0) {
  //   lMotor(50, true);
  //   rMotor(100, true);
  //   Serial.println("left");
  // } else {
  //   lMotor(100, true);
  //   rMotor(50, true);
  //   Serial.println("right");
  // }
  ////////////

  // test 2 //
  float diff = backdist - frontdist;
  lMotor(100 - (diff), true);
  rMotor(100 - (-diff), true);
  ////////////

  ////////////////
}
