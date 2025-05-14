#define DIGITAL_PINS 4
#define ROTATE_PIN1_M1 12
#define ROTATE_PIN2_M1 11
#define ROTATE_PIN1_M2 9
#define ROTATE_PIN2_M2 10

typedef struct{
  uint8_t positivePin;
  uint8_t negativePin;
} Motor_t;

Motor_t motor1;
Motor_t motor2;

// Setting up.
void setup() {
  Serial.begin(9600);

  // Setting up the pins.
  uint8_t pins[DIGITAL_PINS] = {
    ROTATE_PIN1_M1,
    ROTATE_PIN2_M2,
    ROTATE_PIN1_M2,
    ROTATE_PIN2_M2
  };
  for (uint8_t i = 0; i < DIGITAL_PINS; i++)
  {
    pinMode(pins[i], OUTPUT);
  }

  // Setting up motors.
  motorInit(&motor1, ROTATE_PIN1_M1, ROTATE_PIN2_M1);
  motorInit(&motor2, ROTATE_PIN1_M2, ROTATE_PIN2_M2);
}

// TODO: 
void loop() {
  // put your main code here, to run repeatedly:
  rotate(&motor1, 1);
  stopMotor(&motor1, 1000);
  rotate(&motor2, 0);
  stopMotor(&motor2, 2000);
}

void motorInit(Motor_t *motor, uint8_t positivePin, uint8_t negativePin)
{
  motor->positivePin = positivePin;
  motor->negativePin = negativePin;
}

/*Rotates a motor in a specific direction:
  rotateMode == 1 -> forward,
  rotateMode == 0 -> backward,
  rotateMode != 1, 0 -> Invalid.*/
void rotate(Motor_t *motor, uint8_t rotateMode)
{
  if(rotateMode != 1 || rotateMode != 0)
  {
    Serial.println("Invalid rotate mode!");
    return;
  }

  digitalWrite(motor->positivePin, rotateMode % 2);
  digitalWrite(motor->negativePin, (rotateMode + 1) % 2);
}

// Stops the motor by making both the pins it is connected to into low.
void stopMotor(Motor_t *motor, unsigned long delayTime)
{
  digitalWrite(motor->positivePin, LOW);
  digitalWrite(motor->negativePin, LOW);
  delay(delayTime);
}
