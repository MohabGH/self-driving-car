#define DIGITAL_PINS 4
#define ROTATE_RIGHT_PIN_M1 12
#define ROTATE_LEFT_PIN_M1 11
#define ROTATE_RIGHT_PIN_M2 9
#define ROTATE_LEFT_PIN_M2 10




void setup() {
  // put your setup code here, to run once:
  uint8_t pins[DIGITAL_PINS] = {
    ROTATE_RIGHT_PIN_M1,
    ROTATE_LEFT_PIN_M1,
    ROTATE_RIGHT_PIN_M2,
    ROTATE_LEFT_PIN_M2
  };
  for (uint8_t i = 0; i < DIGITAL_PINS; i++)
  {
    pinMode(pins[i], OUTPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}


void rotateRight(uint8_t highPin, uint8_t lowPin, unsigned long delayTime)
{
  digitalWrite(highPin, HIGH);
  digitalWrite(lowPin, LOW);
  delay(delayTime);
}

void
