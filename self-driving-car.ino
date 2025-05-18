#include "ultrasonic.h"
#include "motor.h"
#include <Servo.h>

#define POSITIVE_PIN_M1 A5
#define POSITIVE_PIN_M2 A0
#define NEGATIVE_PIN_M1 A4
#define NEGATIVE_PIN_M2 A1
#define SPEED_PIN_M1 5
#define SPEED_PIN_M2 6
#define ULTRASONIC_TRIGGER_PIN 4
#define ULTRASONIC_ECHO_PIN 2
#define SERVO_PIN 3
#define RIGHT 1
#define LEFT 0
#define SPEED 80
#define STRAIGHT -2
#define ROTATE -3

void rotateInPlace(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t steeringMode, unsigned int delayTime, uint8_t speed);
void moveStraight(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t straightMode, unsigned int delayTime, uint8_t speed);
void stopMoving(Motor_t *rightMotor, Motor_t *leftMotor);
void obstacleAvoidance(Motor_t *rightMotor, Motor_t *leftMotor, Ultrasonic_t *ultrasonic, uint8_t speed);

Motor_t rightMotor;
Motor_t leftMotor;
Ultrasonic_t ultrasonic;
Servo servo;

// Setting up.
void setup() {
  Serial.begin(9600);
  // Initializing motors for controlling them.
  motorInit(&rightMotor, POSITIVE_PIN_M1, NEGATIVE_PIN_M1, SPEED_PIN_M1);
  motorInit(&leftMotor, POSITIVE_PIN_M2, NEGATIVE_PIN_M2, SPEED_PIN_M2);

  // Initializing the Ultrasonic for sensing.
  ultrasonicInit(&ultrasonic, ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);

  // Initializing servo
  servo.attach(SERVO_PIN);
  servo.write(90);
  delay(500);
}

// TODO:
float objectDistance = 0;
unsigned long degrees = 0;
void loop() {
  objectDistance = ultrasonicGetDistance(&ultrasonic);
  moveStraight(&rightMotor, &leftMotor, FORWARD, 0, SPEED);
  Serial.println(objectDistance);
  if(objectDistance <= 20)
  {
    Serial.println(objectDistance);
    obstacleAvoidance(&rightMotor, &leftMotor, &ultrasonic, SPEED);
  }
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

/*Makes the car move in a straight line.
  rightMotor -> The wheel on the right of the car.
  leftMotor -> The wheel on the left of the car.
  straightMode -> Decides wheter the caris going to move FORWARD or BACKWARD.
  delayTime -> Decides the delay time (milliSeconds) the car is going to rotate in.*/
void moveStraight(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t straightMode, unsigned int delayTime, uint8_t speed)
{
  moveMotor(rightMotor, straightMode, speed);
  moveMotor(leftMotor, straightMode, speed);
  delay(delayTime);
}

void stopMoving(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t previousMode, uint8_t speed)
{
    for(int i = speed; i > 50; i--)
    {
      moveStraight(rightMotor, leftMotor, FORWARD, 0, speed);
      delay(50);
    }
    stopMotor(rightMotor);
    stopMotor(leftMotor);
}

/*Makes the car make a decision if it found an obstacle in front of it
  initially:
    The car is going to see in its left and right directions and walk in the more distanced direction.*/
void obstacleAvoidance(Motor_t *rightMotor, Motor_t *leftMotor, Ultrasonic_t *ultrasonic, uint8_t speed)
{
  stopMoving(rightMotor, leftMotor);
  // The variables with which we will get the right and left distance
  unsigned int rightDistance = 0;
  unsigned int leftDistance = 0;

  // Look at the right direction and get the distance.
  servo.write(0);
  delay(500);
  rightDistance = ultrasonicGetDistance(ultrasonic);
  delay(150);

  // Look at the left direction and get the distance.
  servo.write(180);
  delay(500);
  leftDistance = ultrasonicGetDistance(ultrasonic);
  delay(150);

  // Go back to starting position.
  servo.write(90);
  delay(500);

  if(leftDistance >= rightDistance) rotateInPlace(rightMotor, leftMotor, LEFT, 500, speed);
  else rotateInPlace(rightMotor, leftMotor, RIGHT, 500, speed);
  stopMoving(rightMotor, leftMotor);
  delay(2000);
}