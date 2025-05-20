#ifndef INFRARED_H
#define INFRARED_H

typedef struct {
  uint8_t infraredPin;
} InfraredSensor_t;

uint8_t getInfraredState(InfraredSensor_t *infraredSensor);
void infraredInit(InfraredSensor_t *infraredSensor, uint8_t infraredPin);

#endif