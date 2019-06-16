// Description: Test program to analyze the output of the DTFM decoder MT8870D module
#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;

// ---- Pin Definitions ----
#define Q1 13
#define Q2 14
#define Q3 17
#define Q4 1
#define STD 0 // delayed steering: high when a tone has been registered, low otherwise
#define MAXSPEED 300

void setup()
{
  pinMode(Q1, INPUT);
  pinMode(Q2, INPUT);
  pinMode(Q3, INPUT);
  pinMode(Q4, INPUT);
  pinMode(STD, INPUT);
}

void loop()
{
  uint8_t number;
  bool signal;
  signal = digitalRead(STD);  
  if (!signal)
  {
    number = (0x00 | (digitalRead(Q1) << 0) | (digitalRead(Q2) << 1) | (digitalRead(Q3) << 2) | (digitalRead(Q4) << 3));
    switch (number)
    {
    case 2: // go forward
      motors.setLeftSpeed(MAXSPEED);
      motors.setRightSpeed(MAXSPEED);
      break;
    case 5: // stop motors
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      break;
    case 8: // go backward
      motors.setLeftSpeed(-MAXSPEED);
      motors.setRightSpeed(-MAXSPEED);
      break;
    case 4: // go left
      motors.setLeftSpeed(-MAXSPEED);
      motors.setRightSpeed(MAXSPEED);
      break;
    case 6: // go left
      motors.setLeftSpeed(MAXSPEED);
      motors.setRightSpeed(-MAXSPEED);
      break;
    }
  }
  else
  {
    motors.setLeftSpeed(0);
    motors.setRightSpeed(0);
  }
}