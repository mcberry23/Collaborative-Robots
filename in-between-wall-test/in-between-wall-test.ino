#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;

#define pi 3.14159265359

// ---- Pin Setup ----
const int leftTrigPin = 5;
const int leftEchoPin = 11;
const int frontTrigPin = 2;
const int frontEchoPin = 0;
const int rightTrigPin = 3;
const int rightEchoPin = 4;

// ---- Performance Constants ----
const int samplingDelay = 0; // in ms
const long pulseTimeout = 10000; // in us
const double kp = 1;
const double ki = 0;
const double kd = 6;
const int rWheel = 19; // wheel radius in mm
const int ticksPerRev = 610;
const int initialSpeed = 200;

// ---- Variables ----
long leftDuration;
long frontDuration;
long rightDuration;
int leftDistance;
int frontDistance;
int rightDistance;
int error = 0;
int lastError = 0;
int integral = 0;
int derivative = 0;
char report[80];
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

  while(abs(error) < 7){
    readUltrasonic();
    driveStraight();  
    snprintf_P(report, sizeof(report),
          PSTR("Distances: L%6d F%6d R%6d | Speeds: L%6d R%6d | Error: %6d"),
          leftDistance,frontDistance,rightDistance,speedLeft,speedRight,error);
    Serial.println(report);
    delay(samplingDelay);
  }
  stopMotors();
}

void loop()
{
   
}

void stopMotors(){
  motors.setSpeeds(0, 0);
}

void driveStraight(){
  error = leftDistance - rightDistance;
  integral = integral + error;
  derivative = error - lastError;
  lastError = error;
  speedLeft = round(initialSpeed - (kp * error) - (ki * integral) - (kd * derivative));
  speedRight = round(initialSpeed + (kp * error) + (ki * integral) + (kd * derivative));
  speedLeft = max(0,speedLeft);
  speedLeft = min(400,speedLeft);
  speedRight = max(0,speedRight);
  speedRight = min(400,speedRight);
  motors.setSpeeds(speedLeft, speedRight);  
}

void clearPID(){
  lastError = 0;
  derivative = 0;
  integral = 0;
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
  leftDuration = pulseIn(leftEchoPin,HIGH,pulseTimeout);

  // Read front sensor
  digitalWrite(frontTrigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(frontTrigPin,LOW);
  frontDuration = pulseIn(frontEchoPin,HIGH,pulseTimeout);

  // Read right sensor
  digitalWrite(rightTrigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(rightTrigPin,LOW);
  rightDuration = pulseIn(rightEchoPin,HIGH,pulseTimeout);
  
  // Calculate distances
  leftDistance = leftDuration*0.034/2;
  frontDistance = frontDuration*0.034/2;
  rightDistance = rightDuration*0.034/2; 
}

