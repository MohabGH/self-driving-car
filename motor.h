/*The motor library is used to make operations with the Arduino and an H bridge module
All functions and structs are defined according to the POV of the arduino*/

#ifndef MOTOR_H
#define MOTOR_H

#define NO_PIN -1
#define FORWARD 1
#define BACKWARD 0
#define RIGHT 1
#define LEFT 0

#include <stdint.h>

/*A motor structure that defines the pins the arduino is going to modify to control the motor using the H bridge.
  The positivePin and negativePin variables are defined according to the voltage across motors.
  If the positivePin is HIGH and negativePin is LOW, the motor rotates FORWARD.
  If the positivePin is LOW and negativePin is HIGH, the motor rotates BACKWARD.
  The speed pin is optional for controlling the speed of the motor using the H bridge.
  If the used didn't need to control the speed then the used should assign the speedPin to NO_PIN and should connect the enable pin to 5V*/
typedef struct{
  uint8_t positivePin;
  uint8_t negativePin;
  uint8_t speedPin;
} Motor_t;

void motorInit(Motor_t *motor, uint8_t positivePin, uint8_t negativePin, uint8_t speedPin);
void rotateMotor(Motor_t *motor, uint8_t rotateMode, uint8_t speed);
void stopMotor(Motor_t *motor);
void steerMotor(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t steerMode, uint8_t steerSpeed);
void changeMotorSpeed(Motor_t *motor, uint8_t speed);

#endif