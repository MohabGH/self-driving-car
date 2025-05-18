#include "ultrasonic.h"
#include "motor.h"

#define POSITIVE_PIN_M1 A5
#define POSITIVE_PIN_M2 A0
#define NEGATIVE_PIN_M1 A4
#define NEGATIVE_PIN_M2 A1
#define SPEED_PIN_M1 5
#define SPEED_PIN_M2 6
#define ULTRASONIC_TRIGGER_PIN 7
#define ULTRASONIC_ECHO_PIN 8
#define RIGHT 1
#define LEFT 0

void rotateInPlace(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t steeringMode, unsigned int delayTime);
void moveStraight(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t straightMode, unsigned int delayTime);
void obstacleAvoidance(Motor_t *rightMotor, Motor_t *leftMotor, Ultrasonic_t *ultrasonic, uint8_t straightMode, unsigned int delayTime);

Motor_t rightMotor;
Motor_t leftMotor;
Ultrasonic_t ultrasonic;

// Setting up.
void setup() {
  // Initializing motors for controlling them.
  motorInit(&rightMotor, POSITIVE_PIN_M1, NEGATIVE_PIN_M1, NO_PIN);
  motorInit(&leftMotor, POSITIVE_PIN_M2, NEGATIVE_PIN_M2, NO_PIN);

  // Initializing the Ultrasonic for sensing.
  ultrasonicInit(&ultrasonic, ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);
}

// TODO:
void loop() {
  moveStraight(&rightMotor, &leftMotor, FORWARD, 1000);
  stopMoving(&rightMotor, &leftMotor, 1000);
}

/*This function makes the car rotate in place
  rightMotor -> The wheel on the right of the car.
  leftMotor -> The wheel on the left of the car.
  steeringMode -> Decides whether the car is going to move RIGHT or LEFT.
  delayTime -> Decides the delay time the car is going to rotate in.*/
void rotateInPlace(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t steeringMode, unsigned int delayTime)
{
  moveMotor(rightMotor, (steeringMode + 1) % 2, 80);
  moveMotor(leftMotor, steeringMode % 2, 80);
  delay(delayTime);
}

/*Makes the car move in a straight line.
  rightMotor -> The wheel on the right of the car.
  leftMotor -> The wheel on the left of the car.
  straightMode -> Decides wheter the caris going to move FORWARD or BACKWARD
  delayTime -> Decides the delay time (milliSeconds) the car is going to rotate in.*/
void moveStraight(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t straightMode, unsigned int delayTime)
{
  moveMotor(rightMotor, straightMode, 80);
  moveMotor(leftMotor, straightMode, 80);
  delay(delayTime);
}

void stopMoving(Motor_t *rightMotor, Motor_t *leftMotor, unsigned int delayTime)
{
  stopMotor(rightMotor);
  stopMotor(leftMotor);
  delay(delayTime);
}

/*Makes the car make a decision if it found an obstacle in front of it
  initially:
    The car is going to see in its left and right directions and walk in the more distanced direction.*/
void obstacleAvoidance(Motor_t *rightMotor, Motor_t *leftMotor, Ultrasonic_t *ultrasonic, uint8_t straightMode, unsigned int delayTime)
{
  // The variables with which we will get the right and left distance
  unsigned int rightDistance = 0;
  unsigned int leftDistance = 0;

  // Look at the right direction and get the distance.
  rotateInPlace(rightMotor, leftMotor, RIGHT, 500);
  rightDistance = ultrasonicGetDistance(ultrasonic);

  // Look at the left direction and get the distance.
  rotateInPlace(rightMotor, leftMotor, LEFT, 1000);
  leftDistance = ultrasonicGetDistance(ultrasonic);

  if(leftDistance >= rightDistance) moveStraight(rightMotor, leftMotor, FORWARD, 1000);
  else
  {
    rotateInPlace(rightMotor, leftMotor, RIGHT, 1000);
    moveStraight(rightMotor, leftMotor, FORWARD, 1000);
  }
}