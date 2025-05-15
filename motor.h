#ifndef Motor_h
#define Motor_h

#define NO_PIN -1
#define FORWARD 1
#define BACKWARD 0
#define RIGHT 1
#define LEFT 0

#include <stdint.h>

typedef struct{
  uint8_t positivePin;
  uint8_t negativePin;
  int8_t speedPin;
} Motor_t;

void motorInit(Motor_t *motor, uint8_t positivePin, uint8_t negativePin, uint8_t speedPin);
void rotateMotor(Motor_t *motor, uint8_t rotateMode, uint8_t speed);
void stopMotor(Motor_t *motor);
void steerMotor(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t steerMode, uint8_t steerSpeed);
void changeMotorSpeed(Motor_t *motor, uint8_t speed);

#endif