#include "remote-control.h"

void remoteInit(uint8_t receiverPin)
{
  IrReceiver.begin(receiverPin, ENABLE_LED_FEEDBACK);
}

void remoteControl(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t rotationSpeed, uint8_t rightSpeed, uint8_t leftSpeed)
{
  unsigned long code = 0;
  unsigned long lastCode = 0;            
  unsigned long lastSignalTime = 0;
  const unsigned long timeout = 200;
  unsigned long repeatCode = 0;
  while(1)
  {
    if (IrReceiver.decode()) 
    {
      code = IrReceiver.decodedIRData.decodedRawData; // Read the raw code
      Serial.println(code, HEX);  // Print the received IR code in hexadecimal

      // If the received code is not the repeat code, update the lastCode
      if (code != repeatCode) lastCode = code;

      // Update the timestamp to the current time (signal received)
      lastSignalTime = millis();

      // Ready to receive the next IR signal
      IrReceiver.resume();
    }

    if (millis() - lastSignalTime <= timeout) {
      if (lastCode == UP) moveStraight(rightMotor, leftMotor, FORWARD, 0, rightSpeed, leftSpeed);
      else if(lastCode == DOWN) moveStraight(rightMotor, leftMotor, BACKWARD, 0, rightSpeed, leftSpeed);
      else if(lastCode == RIGHT_SIDE) rotateInPlace(rightMotor, leftMotor, RIGHT, 0, rotationSpeed);
      else if(lastCode == LEFT_SIDE) rotateInPlace(rightMotor, leftMotor, LEFT, 0, rotationSpeed);
      else if (lastCode == OK) break;
    } 
    else stopMoving(rightMotor, leftMotor); 
  }
}