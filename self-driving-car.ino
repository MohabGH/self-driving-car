#include "ultrasonic.h"
#include "motor.h"
#include "infrared-sensor.h"
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
#define INFRARED_RIGHT_PIN 8
#define INFRARED_LEFT_PIN 7
#define DARK 1
#define LIGHT 0

uint8_t rightSpeed = 0;
uint8_t leftSpeed = 0;

void rotateInPlace(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t steeringMode, unsigned int delayTime, uint8_t speed);
void moveStraight(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t straightMode, unsigned int delayTime, uint8_t rightSpeed, uint8_t leftSpeed);
void stopMoving(Motor_t *rightMotor, Motor_t *leftMotor);
void avoidObstacle(Motor_t *rightMotor, Motor_t *leftMotor, Ultrasonic_t *ultrasonic, uint8_t speed);
void oneLineTraceMode(InfraredSensor_t *infraredSensorRight, InfraredSensor_t *infraredSensorLeft, Motor_t *rightMotor, Motor_t *leftMotor, uint8_t originalRightSpeed, uint8_t originalLeftSpeed);

Motor_t rightMotor;
Motor_t leftMotor;
Ultrasonic_t ultrasonic;
InfraredSensor_t infraredSensorRight;
InfraredSensor_t infraredSensorLeft;
Servo servo;

// Setting up.
void setup() {
  Serial.begin(9600);
  // Initializing the infrared sensor
  infraredInit(&infraredSensorRight, INFRARED_RIGHT_PIN);
  infraredInit(&infraredSensorLeft, INFRARED_LEFT_PIN);
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
  leftSpeed = 70 ;
  rightSpeed = 70 ;
  //oneLineTraceMode(&infraredSensorRight, &infraredSensorLeft, &rightMotor, &leftMotor, rightSpeed, leftSpeed); -> for always tracing while moving
  oneLineTraceModeModified(&infraredSensorRight, &infraredSensorLeft, &rightMotor, &leftMotor, rightSpeed, leftSpeed); // stop to steer then move again
  /*
  This code is going to be for the obstacle avoidance
  leftSpeed = 76;
  rightSpeed = 87;
  objectDistance = ultrasonicGetDistance(&ultrasonic);
  moveStraight(&rightMotor, &leftMotor, FORWARD, 0, rightSpeed, leftSpeed);
  if(objectDistance <= 20)
  {
    rightSpeed = 150;
    avoidObstacle(&rightMotor, &leftMotor, &ultrasonic, rightSpeed);
  }*/
  /*leftSpeed = 76;
  rightSpeed = 87;

  int leftIR = digitalRead(IR_LEFT_PIN);
  int rightIR = digitalRead(IR_RIGHT_PIN);

  if (leftIR == HIGH && rightIR == HIGH) {
    moveStraight(&rightMotor, &leftMotor, FORWARD, 0, rightSpeed, leftSpeed);
  }
  else if (leftIR == LOW && rightIR == HIGH) {
    rotateInPlace(&rightMotor, &leftMotor, RIGHT, 0, speed);
  }
  else if (leftIR == HIGH && rightIR == LOW) {
    rotateInPlace(&rightMotor,&leftMotor, LEFT, 0, speed);
  }
  else {
    stopMotor(&rightMotor);
    stopMotor(&leftMotor);
  }
  delay(30);*/
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
  delayTime -> Decides the delay time (milliSeconds) the car is going to rotate in.
  rightSpeed -> The speed of the wheel on the right.
  leftSpeed -> The speed of the wheel on the left.*/
void moveStraight(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t straightMode, unsigned int delayTime, uint8_t rightSpeed, uint8_t leftSpeed)
{
  moveMotor(rightMotor, straightMode, rightSpeed);
  moveMotor(leftMotor, straightMode, leftSpeed);
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

/*Makes the car make a decision if it found an obstacle in front of it
  initially:
    The car is going to see in its left and right directions and walk in the more distanced direction.*/
void avoidObstacle(Motor_t *rightMotor, Motor_t *leftMotor, Ultrasonic_t *ultrasonic, uint8_t speed)
{
  stopMoving(rightMotor, leftMotor);
  delay(1000);
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

  if(leftDistance >= rightDistance) rotateInPlace(rightMotor, leftMotor, LEFT, 180, speed);
  else rotateInPlace(rightMotor, leftMotor, RIGHT, 180, speed);
  stopMoving(rightMotor, leftMotor);
  delay(2000);
}

/*void lineTraceMode(InfraredSensor_t *infraredSensorRight, InfraredSensor_t *InfraredSensorLeft, Motor_t *rightMotor, Motor_t *leftMotor)
{
  // If right light
  if(getPinState(infraredSensorRight) == DARK && getPinState(infraredSensorLeft) == DARK)
  {
    stopMoving(rightMotor, leftMotor);
  }
  // make right wheel more speed than left wheel
  else if(getPinState(infraredSensorRight) == DARK && getPinState(infraredSensorLeft) == LIGHT)
  {
    changeMotorSpeed(, );
  }
  // If not right light
    // make left wheel more speed than right wheel
  // If right and left light
    // stop
}*/

void oneLineTraceMode(InfraredSensor_t *infraredSensorRight, InfraredSensor_t *infraredSensorLeft, Motor_t *rightMotor, Motor_t *leftMotor, uint8_t originalRightSpeed, uint8_t originalLeftSpeed)
{
  while(getInfraredState(infraredSensorRight) == DARK)
  {
    if(getInfraredState(infraredSensorLeft) == DARK)
    {
      stopMoving(rightMotor, leftMotor);
      break;
    }
    Serial.println("DARK LEFT");
    changeMotorSpeed(leftMotor, originalLeftSpeed + 35);
  }
  changeMotorSpeed(leftMotor, originalLeftSpeed);
  while(getInfraredState(infraredSensorLeft) == DARK)
  {
    if(getInfraredState(infraredSensorRight) == DARK)
    {
      stopMoving(rightMotor, leftMotor);
      break;
    }
    Serial.println("DARK RIGHT");
    changeMotorSpeed(rightMotor, originalRightSpeed + 35);
  }
  changeMotorSpeed(rightMotor, originalRightSpeed);
}

void oneLineTraceModeModified(InfraredSensor_t *infraredSensorRight, InfraredSensor_t *infraredSensorLeft, Motor_t *rightMotor, Motor_t *leftMotor, uint8_t originalRightSpeed, uint8_t originalLeftSpeed)
{
  while(getInfraredState(infraredSensorRight) == DARK)
  {
    if(getInfraredState(infraredSensorLeft) == DARK)
    {
      stopMoving(rightMotor, leftMotor);
      break;
    }
    rotateInPlace(rightMotor, leftMotor, RIGHT, 0, originalRightSpeed);
  }
  moveStraight(rightMotor, leftMotor, FORWARD, 0, originalRightSpeed, originalLeftSpeed);
  while(getInfraredState(infraredSensorLeft) == DARK)
  {
    if(getInfraredState(infraredSensorRight) == DARK)
    {
      stopMoving(rightMotor, leftMotor);
      break;
    }
    rotateInPlace(rightMotor, leftMotor, LEFT, 0, originalRightSpeed);
  }
  moveStraight(rightMotor, leftMotor, FORWARD, 0, originalRightSpeed, originalLeftSpeed);
}