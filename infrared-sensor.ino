#include "infrared-sensor.h"

void infraredInit(InfraredSensor_t *infraredSensor, uint8_t infraredPin)
{
  infraredSensor->infraredPin = infraredPin;
  pinMode(infraredPin, INPUT_PULLUP);
}

uint8_t getInfraredState(InfraredSensor_t *infraredSensor)
{
  return digitalRead(infraredSensor->infraredPin);
}