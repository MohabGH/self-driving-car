#include "motor.h"

/*Initialize the H bridge module to control the motor using another functions.
  motor -> A pointer to a motor that is going to be used.
  positivePin -> The pin which when is HIGH and negativePin is LOW, the motor would rotate forward. (and vice versa).
  speedPin -> The pin that controls the speed of the motor.
  If the speed pin is NO_PIN all the functions are going to ignore any speed the user specifies.*/
void motorInit(Motor_t *motor, uint8_t positivePin, uint8_t negativePin, uint8_t speedPin)
{
  uint8_t pins[] = {positivePin, negativePin, speedPin}; // Puts the pins in an array before starting to assign the pins' modes.
  for (int i = 0; i < 3; i++) pinMode(pins[i], OUTPUT); // Assigning the pins' modes using a for loop to OUTPUT.
  /*Assigning the pins to the ones the motor uses.*/
  motor->positivePin = positivePin;
  motor->negativePin = negativePin;
  motor->speedPin = speedPin;
}

/*Rotates a motor in a specific direction
  motor -> a pointer to the motor that needs modification.
  movingMode -> It is the direction the motor will move in. It can either be FORWARD or BACKWARD.
  speed -> The speed the motor will use if it contains a pin. This would be useless if there's no pin for controlling speed.
speed is a value between 0 and 255*/
void moveMotor(Motor_t *motor, uint8_t movingMode, uint16_t speed)
{
  // Checking that the movingMode is either FORWARD or BACKWARD.
  if(movingMode != FORWARD && movingMode != BACKWARD) return;

  /*Checks if there's a speed pin or not.
    If it exists, the speed is going to get modified by the value that got passed
    If not, the function will ignore the value of the speed passed into it.*/
  if(motor->speedPin != NO_PIN || (speed < 256 && speed >= 0)) analogWrite(motor->speedPin, speed);

  /*For the motor to move forward, the positivePin needs to be HIGH and the negative pin needs to be LOW.
    This section of code make use of the fact that HIGH = 0x1 and LOW = 0x0.
    If movingMode = FORWARD, this means movingMode = 1, then the postivePin in the motor
  will be 1 (HIGH) and negativePin will be 0 (LOW), and vice versa.*/
  digitalWrite(motor->positivePin, movingMode % 2);
  digitalWrite(motor->negativePin, (movingMode + 1) % 2);
}

/*Stops a specific motor from moving.
  motor ->A pointer to the motor that needs to be modified.*/
void stopMotor(Motor_t *motor)
{
  /*The function assgines both positive and negative pins to LOW to stop the motor from moving.*/
  digitalWrite(motor->positivePin, LOW);
  digitalWrite(motor->negativePin, LOW);
}

/*Changes the speed of a specific motor without moving or stopping the motor.
  motor -> A pointer to the motor that needs to be modified.
  speed -> The new speed of the motor.*/
void changeMotorSpeed(Motor_t *motor, uint16_t speed)
{
  /*If there's a pin for the speed, and the speed is in the valid range, The motor speed will change.
    If not, this function does nothing.*/
  if(motor->speedPin != NO_PIN || (speed < 256 && speed >= 0)) analogWrite(motor->speedPin, speed);
}
