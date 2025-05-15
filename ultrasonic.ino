#include "ultrasonic.h"

/*Initialize the ultrasonic pins*/
void ultrasonicInit(Ultrasonic_t *ultrasonic, uint8_t triggerPin, uint8_t echoPin)
{
  pinMode(triggerPin, OUTPUT);
  pinMode(triggerPin, INPUT);
  ultrasonic->triggerPin = triggerPin;
  ultrasonic->echoPin = echoPin;
}