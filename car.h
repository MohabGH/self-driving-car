/*This library has a set of functions to control a car with two motor
The library leverages the motor.h library to control the car.*/

#ifndef CAR_H
#define CAR_H

// Rotation definitions.
#define RIGHT 1
#define LEFT 0

#include <stdint.h>
#include "motor.h"

void moveStraight(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t straightMode, unsigned int delayTime, uint16_t rightSpeed, uint16_t leftSpeed);
void rotateInPlace(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t steeringMode, unsigned int delayTime, uint16_t speed);
void stopMoving(Motor_t *rightMotor, Motor_t *leftMotor);

#endif*/