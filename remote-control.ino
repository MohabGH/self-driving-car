#include "remote-control.h"

void remoteInit(uint8_t receiverPin)
{
  IrReceiver.begin(receiverPin, ENABLE_LED_FEEDBACK);
}

uint8_t modes(Motor_t rightMotor, Motor_t leftMotor, LiquidCrystal *lc)
{
  uint8_t cursor = 0;
  uint8_t cursorIndex = 0;
  stopMoving(rightMotor, leftMotor);
  lcd->clear();
  lcd->setCursor(5, 0)
  lcd->print("Modes:");
  lcd->setCursor(2, 1);
  for(int i = 1; i <= 4; i++)
  {
    lcd->print(i); lcd->print("  ");
  }
  lcd->setCursor(3, 1);
  lcd->print("<");

  unsigned long cursorSignal;
  while(1)
  {
    cursorSignal = remoteGetSignal();

    if(cursorSignal == RIGHT_SIDE)
    {
      lcd->setCursor(1, cursorIndex);
      lcd->print(" ");
      cursor = ++cursor % 4;
      cursorIndex = 3*(cursor) + 3;
      lcd->setCursor(1, cursorIndex);
      lcd->print("<");
    }
    else if(cursorSignal == RIGHT_SIDE)
    {
      lcd->setCursor(1, cursorIndex);
      lcd->print(" ");
      cursor = abs(--cursor) % 4;
      cursorIndex = 3*(cursor) + 3;
      lcd->setCursor(1, cursorIndex);
      lcd->print("<");
    }
    else if(cursorSignal == OK) return cursor;
  }
}


Status_t obstacleAvoidanceSettings(Motor_t *rightMotor, Motor_t *leftMotor, LiquidCrystal *lcd, Ultrasonic_t *ultrasonic)
{
  uint8_t cursor = 0;
  uint8_t cursorIndex = 0;
  uint8_t cursorSignal = 5;


  lcd->clear();
  lcd->setCursor(4, 0);
  lcd->print("Settings");
  lcd->setCursor(0, 1);
  lcd->print("Speed< Distance ");

  // Get the speed or distance settings
  while(cursorSignal != OK && cursorSignal != ZERO)
  {
    cursorSignal = remoteGetSignal();

    if(cursorSignal == RIGHT_SIDE || cursorSignal == LEFT_SIDE)
    {
      lcd->setCursor(cursorIndex, 1);
      lcd->print(" ");
      cursor = (cursor == 0) ? 1 : 0;
      cursorIndex = (cursor == 0) ? 5 : 15;
      lcd->setCursor(cursorIndex, 1);
      lcd->print("<");
    }
  }

  if(cursorSignal == ZERO)
  {
    return EXIT;
  }

  cursorSignal = 0;
  cursor = 0
  cursorIndex = 0;

  // Speed Settings
  if(cursor == 0)
  {
    cursorIndex = 15;
    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("rightSpeed:    <"); // 12 arrow at 15
    lcd->print(getMotorSpeed(rightMotor));
    lcd->setCursor(0, 1);
    lcd->print("leftSpeed:     "); // 12 arrow at 15
    lcd->print(getMotorSpeed(leftMotor));

    while(cursorSignal != OK)
    {
      cursorSignal = remoteGetSignal();

      if(cursorSignal == UP || cursorSignal == DOWN)
      {
        lcd->setCursor(cursorIndex, cursor);
        lcd->print(" ");
        cursor = (cursor == 0) ? 1 : 0;
        lcd->setCursor(cursorIndex, cursor);
        lcd->print("<");
      }

      if(cursorSignal == RIGHT_SIDE)
      {
        if(cursor == 0)
        {
          changeMotorSpeed(rightMotor, (rightMotor->speed) + 1);
          lcd->setCursor(cursorIndex - 3, cursor);
          lcd->print("   ");
          lcd->setCursor(cursorIndex - 3, cursor);
          lcd->print(getMotorSpeed(rightMotor));
        }
        else
        {
          changeMotorSpeed(leftMotor, (leftMotor->speed) + 1);
          lcd->setCursor(cursorIndex - 3, cursor);
          lcd->print("   ");
          lcd->setCursor(cursorIndex - 3, cursor);
          lcd->print(getMotorSpeed(leftMotor));
        }
      }
      else if(cursorSignal == LEFT_SIDE)
      {
        if(cursor == 0)
        {
          changeMotorSpeed(rightMotor, (rightMotor->speed) - 1);
          lcd->setCursor(cursorIndex - 3, cursor);
          lcd->print("   ");
          lcd->setCursor(cursorIndex - 3, cursor);
          lcd->print(getMotorSpeed(rightMotor));
        }
        else
        {
          changeMotorSpeed(leftMotor, (leftMotor->speed) - 1);
          lcd->setCursor(cursorIndex - 3, cursor);
          lcd->print("   ");
          lcd->setCursor(cursorIndex - 3, cursor);
          lcd->print(getMotorSpeed(leftMotor));
        }
      }
    }
  }

  // Ultrasonic Settings
  else
  {
    lcd->setCursor(0, 0);
    lcd->print("minDistance:  "); // 13
    lcd->setCursor(13, 1);
    lcd->print("^");

    while(cursorSignal != OK)
    {

    }
  }
}

void settings(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t mode, LiquidCrystal *lcd)
{
  uint8_t cursor = 0;
  uint8_t cursorIndex = 0;
  uint8_t cursorSignal = 5;
  if(mode == OBSTACLE_AVOIDANCE)
  {
    lcd->clear();
    lcd->setCursor(4, 0);
    lcd->print("Settings");
    lcd->setCursor(0, 1);
    lcd->print("Speed< Distance ");

    while(1)
    {
      cursorSignal = remoteGetSignal();

      if(cursorSignal == RIGHT_SIDE || cursorSignal == LEFT_SIDE)
      {
        lcd->setCursor(cursorIndex, 1);
        lcd->print(" ");
        cursor = (cursor == 0) ? 1 : 0;
        cursorIndex = (cursor == 0) ? 5 : 15;
        lcd->setCursor(cursorIndex, 1);
        lcd->print("<");
      }
      else if(cursorSignal == OK) break;
    }

    if(cursor == 0)
    {
      lcd->clear()
      lcd->setCursor(0, 0);
      lcd->print("rightSpeed: ")
    }
    else
    {

    }
  }
  else if(mode == LINE_FOLLOWING_NO_STOP)
  {

  }
  else if(mode == LINE_FOLLOWING_STOP)
  {

  }
  else if(mode == REMOTE_CONTROL)
  {

  }
}

Status_t pause(Motor_t *rightMotor, Motor_t *leftMotor, LiquidCrystal *lcd, void (*mainMenu)(LiquidCrystal *lcd))
{
  stopMoving(rightMotor, leftMotor);
  lcd->clear();
  lcd->setCursor(5, 0);
  lcd->print("Paused");
  uint8_t cursor = 0;
  uint8_t cursorIndex = 0;
  lcd->setCursor(0, 1);
  lcd->print("Continue.<");
  lcd->setCursor(10, 1);
  lcd->print("Exit.");
  unsigned long cursorSignal
  while(1)
  {
    cursirSignal = remoteGetSignal();

    // Adjusting the cursor.
    if(cursorSignal == RIGHT_SIDE || cursroSignal == LEFT_SIDE)
    {
      lcd->setCursor(1, cursorIndex);
      lcd->print(" ");
      cursor = (cursor == 0) ? 1 : 0;
      cursorIndex = (cursor == 0) ? 9 : 15;
      lcd->setCursor(1, cursorIndex);
      lcd->print("<");
    }
    else if(cursorSignal == OK && cursor == 0) return EXIT;
    else if(cursorSignal == OK && cursor == 1) return CONTINUE;
  }
  mainMenu(lcd);
}

void remoteChangeSpeed(Motor_t *rightMotor, Motor_t *leftMotor, uint8_t speed)
{
  changeMotorSpeed(rightMotor, speed);
  changeMotorSpeed(leftMotor, speed);
}

void remoteChangeMode(uint8_t *mode, uint8_t numberOfModes)
{
  *mode = ++(*mode) % numberOfModes;
}

uint8_t remoteToNumber(unsigned long remoteInput)
{
  for(int i = 0; i < NUMBER_OF_BUTTONS - 5; i++)
  {
    if(remoteInput == REMOTE_BUTTONS[i])
    {
      return i;
    }
  }
  return -1;
}

unsigned long remoteGetSignal()
{
  unsigned long code;
  while(1)
  { 
    if(IrReceiver.decode())
    {
      code = IrReceiver.decodedIRData.decodedRawData;
      IrReceiver.resume();
      break;
    }
  }
  return code;
}