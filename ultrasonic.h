#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

/*a struct the defines the Ultrasonic pins the Arduino is going to use the Ultrasonic with.
  All pins are assgined in the POV of the arduino and not the Ultrasonic sensor.
  triggerPin is the trigger pin in the Ultrasonic sensor that is used to start reading with the sensor.
  echoPin is the echo pin with which the Ultrasonnic sensor is going to send back the reading to the arduino.*/
typedef struct{
  uint8_t triggerPin;
  uint8_t echoPin;
} Ultrasonic_t;

void ultrasonicInit(Ultrasonic_t *ultrasonic, uint8_t triggerPin, uint8_t echoPin);
unsigned int ultrasonicGetDistance(Ultrasonic_t *ultrasonic);

#endif