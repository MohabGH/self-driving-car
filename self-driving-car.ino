#define DIGITAL_PINS 5
#define POSITIVE_PIN_M1 11
#define NEGATIVE_PIN_M1 12
#define SPEED_PIN_M1 5
#define SPEED_PIN_M2 6
#define POSITIVE_PIN_M2 10
#define NEGATIVE_PIN_M2 9
#define NO_PIN -1
#define FORWARD 1
#define BACKWARD 0
#define RIGHT 1
#define LEFT 0

/*A motor data type the that its pins get information from an H bridge.*/
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

Motor_t rightMotor;
Motor_t leftMotor;

// Setting up.
void setup() {
  Serial.begin(9600);

  // Setting up the pins.
  uint8_t pins[DIGITAL_PINS] = {
    POSITIVE_PIN_M1,
    NEGATIVE_PIN_M1,
    POSITIVE_PIN_M2,
    NEGATIVE_PIN_M2,
    SPEED_PIN_M1
  };
  for (uint8_t i = 0; i < DIGITAL_PINS; i++)
  {
    pinMode(pins[i], OUTPUT);
  }

  // Setting up motors.
  motorInit(&rightMotor, POSITIVE_PIN_M1, NEGATIVE_PIN_M1, SPEED_PIN_M1);
  motorInit(&leftMotor, POSITIVE_PIN_M2, NEGATIVE_PIN_M2, SPEED_PIN_M2);
}

// TODO: 
void loop() {
  // put your main code here, to run repeatedly:
  rotateMotor(&rightMotor, FORWARD, 255);
  delay(1000);
  stopMotor(&rightMotor);
  delay(1000);
  rotateMotor(&leftMotor, BACKWARD, 128);
  delay(1000);
  stopMotor(&leftMotor);
  delay(1000);
}

/*Initialize the motor terminals according to the pins used from H bridge motor driver.*/
void motorInit(Motor_t *motor, uint8_t positivePin, uint8_t negativePin, uint8_t speedPin)
{
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
  if(motor->speedPin != -1) analogWrite(motor->speedPin, speed);
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
  if(motor->speedPin != -1) analogWrite(motor->speedPin, speed);
}