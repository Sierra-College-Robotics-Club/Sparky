#include <NewPing.h>
#define MAX_DISTANCE 200
NewPing sonar(27, 26, MAX_DISTANCE);

#define LEnable 9
#define LIN1 10
#define LIN2 11

#define REnable 5
#define RIN1 6
#define RIN2 7

int last_dist = 0;

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

  last_dist = sonar.convert_cm(sonar.ping_median(5));
}

void mvleft() {
  analogWrite(LEnable, 50);
  analogWrite(REnable, 110);
}

void mvright() {
  analogWrite(LEnable, 75);
  analogWrite(REnable, 65);
}

void forwards(){
  analogWrite(LEnable, 80);
  analogWrite(REnable, 80);
}

void loop() {
  delay(100);


  int cm = sonar.convert_cm(sonar.ping_median(5));
  int diff = last_dist - cm;
  last_dist = cm;
  
  if (diff > 5) {
    mvright();
  } else if(diff < 1){
    mvleft();
  }
  else{
    forwards();
  }

  delay(500);
}
