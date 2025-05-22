#include "ultrasonic.h"
#include "motor.h"
#include "infrared-sensor.h"
#include "car.h"
#include <Servo.h>
#include <LiquidCrystal.h>

// LCD Display pins.
#define RS A2
#define EN A3
#define D4 9
#define D5 10
#define D6 11
#define D7 12

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
#define MODE_ALTERATION_PIN 13

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void obstacleAvoidance(Motor_t *rightMotor, Motor_t *leftMotor, Ultrasonic_t *ultrasonic, uint8_t rightSpeed, uint8_t leftSpeed, unsigned int thresholdDistance);
void oneLineTraceMode(InfraredSensor_t *infraredSensorRight, InfraredSensor_t *infraredSensorLeft, Motor_t *rightMotor, Motor_t *leftMotor, uint8_t rightSpeed, uint8_t leftSpeed);
void oneLineTraceModeModified(InfraredSensor_t *infraredSensorRight, InfraredSensor_t *infraredSensorLeft, Motor_t *rightMotor, Motor_t *leftMotor, uint8_t rightSpeed, uint8_t leftSpeed);

Motor_t rightMotor;
Motor_t leftMotor;
Ultrasonic_t ultrasonic;
InfraredSensor_t infraredSensorRight;
InfraredSensor_t infraredSensorLeft;
Servo servo;

// Setting up.
void setup() {
  // Serial connection for debugging.
  Serial.begin(9600);

  // LCD setup.
  lcd.begin(16, 2);
  lcd.setCursor(5, 1);
  lcd.print(">___<");

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

  // Initializing servo.
  servo.attach(SERVO_PIN);
  servo.write(90);
  delay(500);
}

// Necessary variables for car operation.
unsigned int thresholdDistance = 0;
unsigned int rotationSpeed = 0;
uint8_t rightSpeed = 0;
uint8_t leftSpeed = 0;
uint8_t mode = 0;
void loop() {
  // Making the modes variale be scaled to the selected modes
  if(digitalRead(MODE_ALTERATION_PIN) == LOW)
  {
    mode++;
    mode = mode % 3;
    Serial.println(mode);
    delay(200);
  }

  // The abstacle avoidance mode.
  if(mode == OBSTACLE_AVOIDANCE)
  {
    Serial.println("Obstacle");
    lcd.setCursor(0, 0);
    lcd.print(" OBSTACLE AVOID ");
    rightSpeed = 80;
    leftSpeed = 80;
    thresholdDistance = 20;
    rotationSpeed = 100;
    obstacleAvoidance(&rightMotor, &leftMotor, &ultrasonic, rightSpeed, leftSpeed, thresholdDistance, rotationSpeed);
  }
  // The line following mode. Stops to modify the orientation then moves again.
  else if(mode == LINE_FOLLOWING_STOP)
  {
    Serial.println("Trace No");
    lcd.setCursor(0, 0);
    lcd.print("  LINE TRACE 1  ");
    rightSpeed = 90;
    leftSpeed = 90;
    oneLineTraceModeModified(&infraredSensorRight, &infraredSensorLeft, &rightMotor, &leftMotor, rightSpeed, leftSpeed);
  }
  // The line follwing mode. Doesn't stop to modify the orientation. It modifies the orientation while moving.
  else if(mode == LINE_FOLLOWING_NO_STOP)
  {
    Serial.println("Trace Stop");
    lcd.setCursor(0, 0);
    lcd.print("  LINE TRACE 2  ");
    rightSpeed = 90;
    leftSpeed = 90;
    oneLineTraceMode(&infraredSensorRight, &infraredSensorLeft, &rightMotor, &leftMotor, rightSpeed, leftSpeed);
  }
}

/*Makes the car make a decision if it found an obstacle in front of it
  The car is going to see in its left and right directions and walk in the more distanced direction.
  rightMotor -> The right motor of the car.
  leftMotor -> The left motor of the car.
  rightSpeed -> The speed of the motor on the right
  leftSpeed -> The speed of the motor on the left
  thresholdDistance -> The distance on which it will stop and look left and right.
  rotationSpeed -> The speed the car is going to rotate in towards the direction that has more distance*/
void obstacleAvoidance(Motor_t *rightMotor, Motor_t *leftMotor, Ultrasonic_t *ultrasonic, uint8_t rightSpeed, uint8_t leftSpeed, unsigned int thresholdDistance, unsigned int rotationSpeed)
{
  moveStraight(rightMotor, leftMotor, FORWARD, 0, rightSpeed, leftSpeed);
  float distance = ultrasonicGetDistance(ultrasonic);
  Serial.println(distance);
  if(distance < thresholdDistance)
  {
    // Stop before looking left or right.
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

    /*If the ultrasonic found the leftDistance is more or equal to the rightDistance, the car is going to rotate left.
      Otherwise, it is going to rotatoe to the right.*/
    if(leftDistance >= rightDistance) rotateInPlace(rightMotor, leftMotor, LEFT, 180, rotationSpeed);
    else rotateInPlace(rightMotor, leftMotor, RIGHT, 180, rotationSpeed);

    // Stopping for 2 seconds before moving again.
    stopMoving(rightMotor, leftMotor);
    delay(2000);
  }
}

/*This mode traces a line and continues moving while changing the orientation.
  infraredSensorRight -> The sensor on the right of the car.
  infraredSensorLeft -> The sensor on the left of the car.
  rightMotor -> The motor on the right of the car.
  leftMotor -> The motor on the left of the car.
  rightSpeed -> The speed of the right wheel.
  leftSpeed -> The speed of the left wheel.*/
void oneLineTraceMode(InfraredSensor_t *infraredSensorRight, InfraredSensor_t *infraredSensorLeft, Motor_t *rightMotor, Motor_t *leftMotor, uint8_t rightSpeed, uint8_t leftSpeed)
{
  // As long as the right and left sensors are reading LIGHT (LOW), it is going to move straight forward.
  if(getInfraredState(infraredSensorRight) == LIGHT && getInfraredState(infraredSensorLeft) == LIGHT)
  {
    moveStraight(rightMotor, leftMotor, FORWARD, 0, rightSpeed, leftSpeed);
  }
  /*If the sensor on the right found the ground to be DARK (HIGH), it is going to enter the loop.*/
  while(getInfraredState(infraredSensorRight) == DARK)
  {
    /*If the left sensor is reading DARK like the right sensor, the car is going to stop moving and break.*/
    if(getInfraredState(infraredSensorLeft) == DARK)
    {
      stopMoving(rightMotor, leftMotor);
      break;
    }
    /*As long as the right sensor is DARK, the car is going to rotate until the right sensor becomes LIGHT again or left sensor becomes DARK.*/
    Serial.println("DARK LEFT");
    changeMotorSpeed(leftMotor, leftSpeed + 35);
  }
  // Changing the motor speed to the normal speed again.
  changeMotorSpeed(leftMotor, leftSpeed);

  /*If the sensor on the left found the ground to be DARK (HIGH), it is going to enter the loop.*/
  while(getInfraredState(infraredSensorLeft) == DARK)
  {
    /*If the right sensor is reading DARK like the right sensor, the car is going to stop moving and break.*/
    if(getInfraredState(infraredSensorRight) == DARK)
    {
      stopMoving(rightMotor, leftMotor);
      break;
    }
    /*As long as the left sensor is DARK, the car is going to rotate until the left sensor becomes LIGHT again or right sesnor becomes DARK.*/
    Serial.println("DARK RIGHT");
    changeMotorSpeed(rightMotor, rightSpeed + 35);
  }
  // Changing the motor speed to the normal speed again.
  changeMotorSpeed(rightMotor, rightSpeed);
}

/*This mode traces a line and stops while changing the orientation.
  infraredSensorRight -> The sensor on the right of the car.
  infraredSensorLeft -> The sensor on the left of the car.
  rightMotor -> The motor on the right of the car.
  leftMotor -> The motor on the left of the car.
  rightSpeed -> The speed of the right wheel.
  leftSpeed -> The speed of the left wheel.*/
void oneLineTraceModeModified(InfraredSensor_t *infraredSensorRight, InfraredSensor_t *infraredSensorLeft, Motor_t *rightMotor, Motor_t *leftMotor, uint8_t rightSpeed, uint8_t leftSpeed)
{
  // As long as the right and left sensors are reading LIGHT (LOW), it is going to move straight forward.
  if(getInfraredState(infraredSensorRight) == LIGHT && getInfraredState(infraredSensorLeft) == LIGHT)
  {
    moveStraight(rightMotor, leftMotor, FORWARD, 0, rightSpeed, leftSpeed);
  }
  /*If the sensor on the right found the ground to be DARK (HIGH), it is going to enter the loop.*/
  while(getInfraredState(infraredSensorRight) == DARK)
  {
    /*If the left sensor is reading DARK like the right sensor, the car is going to stop moving and break.*/
    if(getInfraredState(infraredSensorLeft) == DARK)
    {
      stopMoving(rightMotor, leftMotor);
      break;
    }
    /*As long as the right sensor is DARK, the car is going to rotate until the right sensor becomes LIGHT again or left sensor becomes DARK.*/
    rotateInPlace(rightMotor, leftMotor, RIGHT, 0, rightSpeed);
  }

  /*If the sensor on the left found the ground to be DARK (HIGH), it is going to enter the loop.*/
  while(getInfraredState(infraredSensorLeft) == DARK)
  {
    /*If the right sensor is reading DARK like the right sensor, the car is going to stop moving and break.*/
    if(getInfraredState(infraredSensorRight) == DARK)
    {
      stopMoving(rightMotor, leftMotor);
      break;
    }
    /*As long as the left sensor is DARK, the car is going to rotate until the left sensor becomes LIGHT again or right sesnor becomes DARK.*/
    rotateInPlace(rightMotor, leftMotor, LEFT, 0, rightSpeed);
  }
}