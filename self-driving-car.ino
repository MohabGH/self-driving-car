#include "ultrasonic.h"
#include "motor.h"
#include "infrared-sensor.h"
#include <Servo.h>

// Motors definitions.
#define POSITIVE_PIN_M1 A5
#define POSITIVE_PIN_M2 A0
#define NEGATIVE_PIN_M1 A4
#define NEGATIVE_PIN_M2 A1
#define SPEED_PIN_M1 5
#define SPEED_PIN_M2 6

// Ultrasonic definitions.
#define ULTRASONIC_TRIGGER_PIN 4
#define ULTRASONIC_ECHO_PIN 2

// Servo definitions.
#define SERVO_PIN 3

// Rotation definitions.
#define RIGHT 1
#define LEFT 0

// Infrared definitions.
#define INFRARED_RIGHT_PIN 8
#define INFRARED_LEFT_PIN 7
#define DARK 1
#define LIGHT 0

// Modes definitions.
#define OBSTACLE_AVOIDANCE 0
#define LINE_FOLLOWING_NO_STOP 1
#define LINE_FOLLOWING_STOP 2
#define MODE_ALTERATION_PIN 9

// Modes' leds definitions.
#define OBSTACLE_LED 11
#define TRACE_STOP_LED 10
#define TRACE_NO_STOP_LED 12


void rotateInPlace(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t steeringMode, unsigned int delayTime, uint8_t speed);
void moveStraight(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t straightMode, unsigned int delayTime, uint8_t rightSpeed, uint8_t leftSpeed);
void stopMoving(Motor_t *rightMotor, Motor_t *leftMotor);
void obstacleAvoidance(Motor_t *rightMotor, Motor_t *leftMotor, Ultrasonic_t *ultrasonic, uint8_t rightSpeed, uint8_t leftSpeed, unsigned int thresholdDistance);
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

  // Setting up modes changer button.
  pinMode(MODE_ALTERATION_PIN, INPUT_PULLUP);

  // Setting up leds for modes.
  pinMode(OBSTACLE_LED, OUTPUT);
  pinMode(TRACE_NO_STOP_LED, OUTPUT);
  pinMode(TRACE_STOP_LED, OUTPUT);

  // Initializing servo.
  servo.attach(SERVO_PIN);
  servo.write(90);
  delay(500);
}

// TODO:
unsigned int thresholdDistance = 0;
unsigned int rotationSpeed = 0;
uint8_t rightSpeed = 0;
uint8_t leftSpeed = 0;
uint8_t mode = 0;
void loop() {
  // making the modes variale be scaled to the selected modes
  if(digitalRead(MODE_ALTERATION_PIN) == LOW)
  {
    mode++;
    mode = mode % 3;
    Serial.println(mode);
    delay(200);
  }


  if(mode == OBSTACLE_AVOIDANCE)
  {
    digitalWrite(OBSTACLE_LED, HIGH);
    digitalWrite(TRACE_NO_STOP_LED, LOW);
    digitalWrite(TRACE_STOP_LED, LOW);
    rightSpeed = 80;
    leftSpeed = 80;
    thresholdDistance = 20;
    rotationSpeed = 100;
    obstacleAvoidance(&rightMotor, &leftMotor, &ultrasonic, rightSpeed, leftSpeed, thresholdDistance, rotationSpeed);
  }
  else if(mode == LINE_FOLLOWING_NO_STOP)
  {
    digitalWrite(OBSTACLE_LED, LOW);
    digitalWrite(TRACE_NO_STOP_LED, HIGH);
    digitalWrite(TRACE_STOP_LED, LOW);
    rightSpeed = 90;
    leftSpeed = 90;
    oneLineTraceModeModified(&infraredSensorRight, &infraredSensorLeft, &rightMotor, &leftMotor, rightSpeed, leftSpeed);
  }
  else if(mode == LINE_FOLLOWING_STOP)
  {
    digitalWrite(OBSTACLE_LED, LOW);
    digitalWrite(TRACE_NO_STOP_LED, LOW);
    digitalWrite(TRACE_STOP_LED, HIGH);
    rightSpeed = 90;
    leftSpeed = 90;
    oneLineTraceMode(&infraredSensorRight, &infraredSensorLeft, &rightMotor, &leftMotor, rightSpeed, leftSpeed);
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
void obstacleAvoidance(Motor_t *rightMotor, Motor_t *leftMotor, Ultrasonic_t *ultrasonic, uint8_t rightSpeed, uint8_t leftSpeed, unsigned int thresholdDistance, unsigned int rotationSpeed)
{
  moveStraight(rightMotor, leftMotor, FORWARD, 0, rightSpeed, leftSpeed);
  unsigned int distance = ultrasonicGetDistance(ultrasonic);

  if(distance < thresholdDistance)
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

    if(leftDistance >= rightDistance) rotateInPlace(rightMotor, leftMotor, LEFT, 180, rotationSpeed);
    else rotateInPlace(rightMotor, leftMotor, RIGHT, 180, rotationSpeed);
    stopMoving(rightMotor, leftMotor);
    delay(2000);
  }
}

void oneLineTraceMode(InfraredSensor_t *infraredSensorRight, InfraredSensor_t *infraredSensorLeft, Motor_t *rightMotor, Motor_t *leftMotor, uint8_t originalRightSpeed, uint8_t originalLeftSpeed)
{
  if(getInfraredState(infraredSensorRight) == LIGHT && getInfraredState(infraredSensorLeft) == LIGHT)
  {
    moveStraight(rightMotor, leftMotor, FORWARD, 0, originalRightSpeed, originalLeftSpeed);
  }
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
  if(getInfraredState(infraredSensorRight) == LIGHT && getInfraredState(infraredSensorLeft) == LIGHT)
  {
    moveStraight(rightMotor, leftMotor, FORWARD, 0, originalRightSpeed, originalLeftSpeed);
  }
  while(getInfraredState(infraredSensorRight) == DARK)
  {
    if(getInfraredState(infraredSensorLeft) == DARK)
    {
      stopMoving(rightMotor, leftMotor);
      break;
    }
    rotateInPlace(rightMotor, leftMotor, RIGHT, 0, originalRightSpeed);
  }
  while(getInfraredState(infraredSensorLeft) == DARK)
  {
    if(getInfraredState(infraredSensorRight) == DARK)
    {
      stopMoving(rightMotor, leftMotor);
      break;
    }
    rotateInPlace(rightMotor, leftMotor, LEFT, 0, originalRightSpeed);
  }
}