
// uno r3
// #define leftMotor1 3
// #define leftMotor2 5
// #define rightMotor1 10
// #define rightMotor2 11


// atremis
// B on motor driver
// currently right motor
#define leftMotor1 2
#define leftMotor2 3
// A on motor driver
// currently left motor
#define rightMotor1 4
#define rightMotor2 5

int maxPossSpeed = 255;
int minPossSpeed = 0;

void rightMotorSpeed(bool dir, int speed){
  if(speed > maxPossSpeed) speed = maxPossSpeed;
  if(speed < minPossSpeed) speed = minPossSpeed;

  //if true, go forwards
  if(dir == true){
    analogWrite(rightMotor1, speed);
    analogWrite(rightMotor2, 0);
  }
  //else, go backwards
  else{
    analogWrite(rightMotor1, 0); 
    analogWrite(rightMotor2, speed); 
    }
}

void leftMotorSpeed( bool dir, int speed){
  if(speed > maxPossSpeed) speed = maxPossSpeed;
  if(speed < minPossSpeed) speed = minPossSpeed;

  //if true, go forwards
  if(dir == true){
    analogWrite(leftMotor1, speed);
    analogWrite(leftMotor2, 0);  
  }
  //else, go backwards
  else{
    analogWrite(leftMotor1, 0);
    analogWrite(leftMotor2, speed);
    }
}

void setup() {
 // hi
 // 254 seems to inconsistently work
 // 253 works
  leftMotorSpeed(true, 255);
  // guess forwards
  rightMotorSpeed(true, 255);


}

void loop() {

  //lets test the directions of the motors
  // guess forwards this is true
  //leftMotorSpeed(true, 255);
  // guess forwards
 // rightMotorSpeed(true, 255);
  delay(1000);


  /*
  // motors should start from as close to the begginning of the loop
  // as practical to delay power cycle motor delay

  /*
  analogWrite(leftMotor1, 255);
  pinMode(leftMotor2, LOW);

  analogWrite(rightMotor1, 255);
  pinMode(rightMotor2, LOW);

  delay(3000);


  // needed to reset motor power signal on motor power on/off
  analogWrite(leftMotor1, 0);
  analogWrite(rightMotor1, 0);
  */

}
