#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;

#define pi 3.14159265359
char report[80];

const int leftTrigPin = 5;
const int leftEchoPin = 11;
long leftDuration;
int leftDistance;

const int frontTrigPin = 2;
const int frontEchoPin = 0;
long frontDuration;
int frontDistance;

const int rightTrigPin = 3;
const int rightEchoPin = 4;
long rightDuration;
int rightDistance;



const int samplingDelay = 0; // in ms
const double k = 3.5;
const int distance = 380; // total distance to travel in mm
const int rWheel = 19; // wheel radius in mm
const int ticksPerRev = 610;
const int initialSpeed = 200;
int speedLeft = initialSpeed;
int speedRight = initialSpeed;

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting...");
  delay(1000);
  motors.setSpeeds(speedLeft, speedRight);
  pinMode(leftTrigPin,OUTPUT);
  pinMode(leftEchoPin,INPUT);
  
  pinMode(frontTrigPin,OUTPUT);
  pinMode(frontEchoPin,INPUT);
  
  pinMode(rightTrigPin,OUTPUT);
  pinMode(rightEchoPin,INPUT);
}

void loop()
{
  readUltrasonic();
  speedLeft = round(initialSpeed - (k * (leftDistance - rightDistance)));
  speedRight = round(initialSpeed - (k * (- leftDistance + rightDistance)));
  speedLeft = max(0,speedLeft);
  speedLeft = min(400,speedLeft);
  speedRight = max(0,speedRight);
  speedRight = min(400,speedRight);
  motors.setSpeeds(speedLeft, speedRight);    
  snprintf_P(report, sizeof(report),
        PSTR("Distances: L%6d F%6d R%6d | Speeds: L%6d R%6d"),
        leftDistance,frontDistance,rightDistance,speedLeft,speedRight);
  
  Serial.println(report);
    delay(samplingDelay); 
}

void readUltrasonic(){
   // Clear the trigger pins
  digitalWrite(leftTrigPin,LOW);
  digitalWrite(frontTrigPin,LOW);
  digitalWrite(rightTrigPin,LOW);
  delayMicroseconds(2);

  // Read left sensor
  digitalWrite(leftTrigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(leftTrigPin,LOW);
  leftDuration = pulseIn(leftEchoPin,HIGH);

  // Read front sensor
  digitalWrite(frontTrigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(frontTrigPin,LOW);
  frontDuration = pulseIn(frontEchoPin,HIGH);

  // Read right sensor
  digitalWrite(rightTrigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(rightTrigPin,LOW);
  rightDuration = pulseIn(rightEchoPin,HIGH);
  
  // Calculate distances
  leftDistance = leftDuration*0.034/2;
  frontDistance = frontDuration*0.034/2;
  rightDistance = rightDuration*0.034/2; 
}

