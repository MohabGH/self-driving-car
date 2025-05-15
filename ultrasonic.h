#ifndef Ultrasonic_h
#define Ultrasonic_h

#include <stdint.h>

typedef struct{
  uint8_t triggerPin;
  uint8_t echoPin;
} Ultrasonic_t;

void ultrasonicInit(Ultrasonic_t *ultrasonic, uint8_t triggerPin, uint8_t echoPin);

#endif