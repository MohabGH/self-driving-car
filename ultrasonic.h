#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

typedef struct{
  uint8_t triggerPin;
  uint8_t echoPin;
} Ultrasonic_t;

void ultrasonicInit(Ultrasonic_t *ultrasonic, uint8_t triggerPin, uint8_t echoPin);
unsigned int ultrasonicGetDistance(Ultrasonic_t *ultrasonic);

#endif