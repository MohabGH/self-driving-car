#define DIGITAL_PINS 4
#define ROTATE_RIGHT_PIN_M1 12
#define ROTATE_LEFT_PIN_M1 11
#define ROTATE_RIGHT_PIN_M2 9
#define ROTATE_LEFT_PIN_M2 10



// Setting up the pins.
void setup() {
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

// TODO: 
void loop() {
  // put your main code here, to run repeatedly:

}

// Rotates the motor in a specific direction according to low and high voltage logic digital pin.
void rotate(uint8_t highPin, uint8_t lowPin, unsigned long delayTime)
{
  digitalWrite(highPin, HIGH);
  digitalWrite(lowPin, LOW);
  delay(delayTime);
}

// Stops the motor by making both the pins it is connected to into low.
void stop(uint8_t pin1, uint8_t pin2, unsigned long delayTime)
{
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
  delay(delayTime);
}
