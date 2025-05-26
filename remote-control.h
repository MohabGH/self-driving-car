/*This library contains multiple functions to control the car with the remote.
  This function uses LiquidCrystal.h library and assumes that the LCD display will be an interface for the user.*/

#ifndef REMOTE_H
#define REMOTE_H
#include <IRremote.hpp>
#include "motor.h"
#include "car.h"

#define ZERO 0xE619FF00
#define ONE 0xBA45FF00
#define TWO 0xB946FF00
#define THREE 0xB847FF00
#define FOUR 0xBB44FF00
#define FIVE 0xBF40FF00
#define SIX 0xBC43FF00
#define SEVEN 0xF807FF00
#define EIGHT 0xEA15FF00
#define NINE 0xF609FF00
#define OK 0xE31CFF00
#define UP 0xE718FF00
#define DOWN 0xAD52FF00
#define LEFT_SIDE 0xF708FF00
#define RIGHT_SIDE 0xA55AFF00


unsigned long REMOTE_BUTTONS[]
{
  ZERO,
  ONE,
  TWO,
  THREE,
  FOUR,
  FIVE,
  SIX,
  SEVEN,
  EIGHT,
  NINE,
  OK,
  UP,
  DOWN,
  LEFT_SIDE,
  RIGHT_SIDE
};

void remoteInit(uint8_t receiverPin);
unsigned long remoteControl(Motor_t *rightMotor, Motor_t *leftMotor, uint16_t rotationSpeed, uint16_t rightSpeed, uint16_t leftSpeed);
unsigned long remoteGetSignal();

#endif