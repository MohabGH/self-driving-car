#include "ultrasonic.h"

/*Initialize the ultrasonic pins*/
void ultrasonicInit(Ultrasonic_t *ultrasonic, uint8_t triggerPin, uint8_t echoPin)
{
  pinMode(triggerPin, OUTPUT);
  pinMode(triggerPin, INPUT);
  ultrasonic->triggerPin = triggerPin;
  ultrasonic->echoPin = echoPin;
}

/*return the object distance in CentiMeters.*/
unsigned int ultrasonicGetDistance(Ultrasonic_t *ultrasonic)
{
  digitalWrite(ultrasonic->triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonic->triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonic->triggerPin, LOW);

  long distance = pulseIn(ultrasonic->echoPin, HIGH);
  return distance / 58;
}