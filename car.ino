#include "car.h"

/*Makes the car move in a straight line.
  rightMotor -> The wheel on the right of the car.
  leftMotor -> The wheel on the left of the car.
  straightMode -> Decides wheter the caris going to move FORWARD or BACKWARD.
  delayTime -> Decides the delay time (milliSeconds) the car is going to rotate in.
  rightSpeed -> The speed of the wheel on the right.
  leftSpeed -> The speed of the wheel on the left.*/
void moveStraight(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t straightMode, unsigned int delayTime, uint8_t rightSpeed, uint8_t leftSpeed)
{
  moveMotor(rightMotor, straightMode, rightSpeed);
  moveMotor(leftMotor, straightMode, leftSpeed);
  delay(delayTime);
}

/*This function makes the car rotate in place
  rightMotor -> The wheel on the right of the car.
  leftMotor -> The wheel on the left of the car.
  steeringMode -> Decides whether the car is going to move RIGHT or LEFT.
  delayTime -> Decides the delay time the car is going to rotate in.*/
void rotateInPlace(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t steeringMode, unsigned int delayTime, uint8_t speed)
{
  moveMotor(rightMotor, (steeringMode + 1) % 2, speed);
  moveMotor(leftMotor, steeringMode % 2, speed);
  delay(delayTime);
}

/*Makes the car stop moving
  rightMotor -> The wheel on the right of the car.
  leftMotor -> The wheel on the left of the car.*/
void stopMoving(Motor_t *rightMotor, Motor_t *leftMotor)
{
  stopMotor(rightMotor);
  stopMotor(leftMotor);
}
