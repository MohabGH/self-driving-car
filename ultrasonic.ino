#include "ultrasonic.h"

/*Initialize the ultrasonic pins to start using the ultrasonic.
  ultrasonic -> a pointer to an ultrasonic that is going to be used.
  triggerPin -> The pin the arduino is going to make the ultrasonic sensor start reading.
  echoPin -> The pin the arduino is going to get back the reading of the sensor with.
  */
void ultrasonicInit(Ultrasonic_t *ultrasonic, uint8_t triggerPin, uint8_t echoPin)
{
  /*Assigning the passed pins to the correct pin modes*/
  pinMode(triggerPin, OUTPUT);
  pinMode(triggerPin, INPUT);
  /*Assgining these same pins to the ultrasonic struct.*/
  ultrasonic->triggerPin = triggerPin;
  ultrasonic->echoPin = echoPin;
}

/*This function calculates the distance by getting the ultrasonic to start reading
and getting back the reading using the echoPin.
  The reading the Arduino got from the echoPin will be put in an equation to get the distance in centimeters.
  ultrasonic -> an ultrasonic that is being used.*/
unsigned long ultrasonicGetDistance(Ultrasonic_t *ultrasonic)
{
  /*Starting the ultrasonic according to the datasheet instructions.*/
  digitalWrite(ultrasonic->triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonic->triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonic->triggerPin, LOW);

  /*Getting the time the echoPin stayed HIGH in.*/
  unsigned long pulseTime = pulseIn(ultrasonic->echoPin, HIGH);

  /*The equation with which the distance is being determined
    distance (centimeters) = speed of sound (centimeters in microSecond) * (overall time of travel * 0.5)
    distance -> the distance the sound waves traveled without going back.
    speed of sound -> 0.0343 centimeters / microSecond.
    time of travel -> The time the sound took to travel away and into the Ultrasonic sensor.
    Time of travelling away from the Ultrasonic = Time of travelling into the Ultrasonic. This is the reason why 0.5 is in the equation*/
  unsigned int distance = 0.0343 * (pulseTime * 0.5);
  
  return distance;
}