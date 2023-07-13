#include <NewPing.h>
#include "MOTOR.h"

#define MAX_DISTANCE 10000
float target_distance = 20.0;

NewPing LBackSonar(29, 28, MAX_DISTANCE);
NewPing LFrontSonar(27, 26, MAX_DISTANCE);

MOTOR lmotor(10, 11, 9);
MOTOR rmotor(6, 7, 5);
MOTOR_CONTROL motorcontrol(lmotor, rmotor);

float NearestPoint[2] = {0,0};
float NearestDist = 0;

#include <math.h>

struct Point {
  float x;
  float y;
};

float calculateDistance(Point point1, Point point2) {
  float dx = point2.x - point1.x;
  float dy = point2.y - point1.y;
  return sqrt(dx * dx + dy * dy);
}

Point findClosestPointOnLine(Point linePoint1, Point linePoint2, Point spacePoint) {
  Point closestPoint;
  
  // Calculate the direction vector of the line
  float lineDirectionX = linePoint2.x - linePoint1.x;
  float lineDirectionY = linePoint2.y - linePoint1.y;
  
  // Calculate the vector from the line's first point to the space point
  float lineToPointX = spacePoint.x - linePoint1.x;
  float lineToPointY = spacePoint.y - linePoint1.y;
  
  // Calculate the dot product between the line direction and the line-to-point vector
  float dotProduct = (lineDirectionX * lineToPointX) + (lineDirectionY * lineToPointY);
  
  // Calculate the squared length of the line
  float lineLengthSquared = (lineDirectionX * lineDirectionX) + (lineDirectionY * lineDirectionY);
  
  // Calculate the parameter along the line for the closest point
  float t = dotProduct / lineLengthSquared;
  
  // Calculate the closest point on the line
  closestPoint.x = linePoint1.x + (t * lineDirectionX);
  closestPoint.y = linePoint1.y + (t * lineDirectionY);
  
  return closestPoint;
}

void setup() {
  Serial.begin(9600);
}

float LeftBack = 0;
float LeftFront = 0;
Point BackPoint = {0,0};
Point FrontPoint = {6,0};

Point ClosestPoint = {0, 0};
float WallDistance = 0;

bool MovingLeft = false;

bool CriticalState = false;

void turnRoutine() {
  if (LeftBack < 1 && LeftFront < 1) {
    if (CriticalState) return;
    if (motorcontrol.isTurningRight()) {
      motorcontrol.turnLeft(120);
    } else {
      motorcontrol.turnRight(120);
    }
    CriticalState = true;
    return;
  } else {
    CriticalState = false;
  }

  if (WallDistance < target_distance) {
    motorcontrol.turnRight(120);
  } else {
    motorcontrol.turnLeft(120);
  }
  return;
}

void loop() {
  LeftBack = LBackSonar.convert_cm(LBackSonar.ping_median(3));
  LeftFront = LFrontSonar.convert_cm(LFrontSonar.ping_median(3));

  BackPoint.y = LeftBack;
  FrontPoint.y = LeftFront;

  ClosestPoint = findClosestPointOnLine(BackPoint, FrontPoint, {0,0});
  WallDistance = calculateDistance(ClosestPoint, {0,0});
  
  Serial.print(ClosestPoint.x);
  Serial.print(":");
  Serial.print(ClosestPoint.y);
  Serial.print(" ");
  Serial.println(WallDistance);
  
  turnRoutine();
}