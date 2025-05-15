#include "ultrasonic.h"
#include "motor.h"

#define OUTPUT_DIGITAL_PINS 7
#define INPUT_DIGITAL_PINS 1
#define POSITIVE_PIN_M1 11
#define NEGATIVE_PIN_M1 12
#define SPEED_PIN_M1 5
#define SPEED_PIN_M2 6
#define POSITIVE_PIN_M2 10
#define NEGATIVE_PIN_M2 9
#define ULTRASONIC_TRIGGER 8
#define ULTRASONIC_ECHO 7

Motor_t rightMotor;
Motor_t leftMotor;
Ultrasonic_t ultrasonic;

// Setting up.
void setup() {
  Serial.begin(9600);

  // Setting up motors.
  motorInit(&rightMotor, POSITIVE_PIN_M1, NEGATIVE_PIN_M1, SPEED_PIN_M1);
  motorInit(&leftMotor, POSITIVE_PIN_M2, NEGATIVE_PIN_M2, SPEED_PIN_M2);

  // Setting up ultrasonic
  ultrasonicInit(&ultrasonic, ULTRASONIC_TRIGGER, ULTRASONIC_ECHO);
}

// TODO: 
void loop() {
  
}

