#define DIGITAL_PINS 4
#define POSITIVE_PIN_M1 11
#define NEGATIVE_PIN_M1 12
#define POSITIVE_PIN_M2 10
#define NEGATIVE_PIN_M2 9
#define FORWARD 1
#define BACKWARD 0
#define RIGHT 1
#define LEFT 0

typedef struct{
  uint8_t positivePin;
  uint8_t negativePin;
} Motor_t;

void motorInit(Motor_t *motor, uint8_t positivePin, uint8_t negativePin);
void rotateMotor(Motor_t *motor, uint8_t rotateMode);
void stopMotor(Motor_t *motor);
void steerMotor(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t steerMode);

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
    NEGATIVE_PIN_M2
  };
  for (uint8_t i = 0; i < DIGITAL_PINS; i++)
  {
    pinMode(pins[i], OUTPUT);
  }

  // Setting up motors.
  motorInit(&rightMotor, POSITIVE_PIN_M1, NEGATIVE_PIN_M1);
  motorInit(&leftMotor, POSITIVE_PIN_M2, NEGATIVE_PIN_M2);
}

// TODO: 
void loop() {
  // put your main code here, to run repeatedly:
  rotateMotor(&rightMotor, FORWARD);
  delay(1000);
  stopMotor(&rightMotor);
  delay(1000);
  rotateMotor(&leftMotor, BACKWARD);
  delay(1000);
  stopMotor(&leftMotor);
  delay(1000);
}

/*Initialize the motor terminals according to the pins used from H bridge motor driver.*/
void motorInit(Motor_t *motor, uint8_t positivePin, uint8_t negativePin)
{
  motor->positivePin = positivePin;
  motor->negativePin = negativePin;
}

/*Rotates a motor in a specific direction:
  rotateMode == 1 -> forward,
  rotateMode == 0 -> backward,
  rotateMode != 1, 0 -> Invalid.*/
void rotateMotor(Motor_t *motor, uint8_t rotateMode)
{
  if(rotateMode != FORWARD && rotateMode != BACKWARD)
  {
    Serial.println("Invalid rotate mode!");
    return;
  }

  digitalWrite(motor->positivePin, rotateMode % 2);
  digitalWrite(motor->negativePin, (rotateMode + 1) % 2);
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
void steerMotor(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t steerMode)
{
  if(steerMode != RIGHT && steerMode != LEFT)
  {
    Serial.println("Invalid steer mode!");
    return;
  }

  rotateMotor(rightMotor, (steerMode + 1) % 2);
  rotateMotor(leftMotor, steerMode % 2);
}