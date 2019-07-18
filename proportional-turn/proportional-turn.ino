


#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;


#define pi 3.14159265359

const int samplingDelay = 10; // in ms
const double kpe = 0.8;  // proportional gain for encoders
const int turn180 = 820; // total distance to travel in counts
const int turn90 = 395;// total distance to travel in counts
const int rWheel = 19; // wheel radius in mm
const int ticksPerRev = 610;
const int initialSpeed = 175;

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting...");
  delay(1000);
//  turnLeft();
//  delay(1000);
//  turnRight();
//  delay(1000);
  turnAround();
}

void turnLeft(){
  int x = 0;
  int speedLeft = -initialSpeed;
  int speedRight = initialSpeed;
  int leftCal = encoders.getCountsLeft();
  int rightCal = encoders.getCountsRight();
  motors.setSpeeds(speedLeft, speedRight);
  while(x < turn90){
    int16_t countsLeft = encoders.getCountsLeft()-leftCal;
    int16_t countsRight = encoders.getCountsRight()-rightCal;   
    speedLeft = - round(initialSpeed - (kpe * (countsLeft + countsRight)));
    speedRight = round(initialSpeed - (kpe * (countsRight + countsLeft)));
    speedLeft = max(-400,speedLeft);
    speedLeft = min(0,speedLeft);
    speedRight = max(0,speedRight);
    speedRight = min(400,speedRight);
    motors.setSpeeds(speedLeft, speedRight);
    x = (abs(countsLeft) + abs(countsRight))/2;
    delay(samplingDelay); 
  }
  motors.setSpeeds(0, 0);
}

void turnRight(){
  int x = 0;
  int speedLeft = initialSpeed;
  int speedRight = - initialSpeed;
  int leftCal = encoders.getCountsLeft();
  int rightCal = encoders.getCountsRight();
  motors.setSpeeds(speedLeft, speedRight);
  while(x < turn90){
    int16_t countsLeft = encoders.getCountsLeft()-leftCal;
    int16_t countsRight = encoders.getCountsRight()-rightCal;   
    speedLeft = round(initialSpeed - (kpe * (countsLeft + countsRight)));
    speedRight = - round(initialSpeed - (kpe * (countsRight + countsLeft)));
    speedLeft = max(0,speedLeft);
    speedLeft = min(400,speedLeft);
    speedRight = max(-400,speedRight);
    speedRight = min(0,speedRight);
    motors.setSpeeds(speedLeft, speedRight);
    x = (abs(countsLeft) + abs(countsRight))/2;
    delay(samplingDelay); 
  }
  motors.setSpeeds(0, 0);
}


void turnAround(){
  int x = 0;
  int speedLeft = initialSpeed;
  int speedRight = - initialSpeed;
  int leftCal = encoders.getCountsLeft();
  int rightCal = encoders.getCountsRight();
  motors.setSpeeds(speedLeft, speedRight);
  while(x < turn180){
    int16_t countsLeft = encoders.getCountsLeft()-leftCal;
    int16_t countsRight = encoders.getCountsRight()-rightCal;   
    speedLeft = round(initialSpeed - (kpe * (countsLeft + countsRight)));
    speedRight = - round(initialSpeed - (kpe * (countsRight + countsLeft)));
    speedLeft = max(0,speedLeft);
    speedLeft = min(400,speedLeft);
    speedRight = max(-400,speedRight);
    speedRight = min(0,speedRight);
    motors.setSpeeds(speedLeft, speedRight);
    x = (abs(countsLeft) + abs(countsRight))/2;
    delay(samplingDelay); 
  }
  motors.setSpeeds(0, 0);
}

void loop()
{
  

}
