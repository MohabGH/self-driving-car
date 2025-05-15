#include "motor.h"

/*Initialize the motor terminals according to the pins used from H bridge motor driver.*/
void motorInit(Motor_t *motor, uint8_t positivePin, uint8_t negativePin, uint8_t speedPin)
{
  Serial.begin(9600);
  uint8_t pins[] = {positivePin, negativePin, speedPin};
  for (int i = 0; i < 3; i++) pinMode(pins[i], OUTPUT);
  motor->positivePin = positivePin;
  motor->negativePin = negativePin;
  motor->speedPin = speedPin;
}

/*Rotates a motor in a specific direction:
  rotateMode == 1 -> forward,
  rotateMode == 0 -> backward,
  rotateMode != 1, 0 -> Invalid.*/
void rotateMotor(Motor_t *motor, uint8_t rotateMode, uint8_t speed)
{
  if(rotateMode != FORWARD && rotateMode != BACKWARD)
  {
    Serial.println("Invalid rotate mode!");
    return;
  }

  digitalWrite(motor->positivePin, rotateMode % 2);
  digitalWrite(motor->negativePin, (rotateMode + 1) % 2);
  if(motor->speedPin != NO_PIN && speed < 256 && speed > 0) analogWrite(motor->speedPin, speed);
}

// Stops the motor by making both the pins it is connected to into low.
void stopMotor(Motor_t *motor)
{
  digitalWrite(motor->positivePin, LOW);
  digitalWrite(motor->negativePin, LOW);
}

/*Steers the motors of the car:
  steerMode = 1 -> steer right,
  steerMode = 0 -> steer left,
  steerMode != 1, 0 -> Invalid.*/
void steerMotor(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t steerMode, uint8_t steerSpeed)
{
  if(steerMode != RIGHT && steerMode != LEFT)
  {
    Serial.println("Invalid steer mode!");
    return;
  }

  rotateMotor(rightMotor, (steerMode + 1) % 2, steerSpeed);
  rotateMotor(leftMotor, steerMode % 2, steerSpeed);
}

/*Changes the speed of a specific motor without using rotateMotor function again.*/
void changeMotorSpeed(Motor_t *motor, uint8_t speed)
{
  if(motor->speedPin != NO_PIN) analogWrite(motor->speedPin, speed);
}