#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4ButtonC buttonC;

#define pi 3.14159265359

// ---- Pin Setup ----
const int leftTrigPin = 5;
const int leftEchoPin = 11;
const int frontTrigPin = 2;
const int frontEchoPin = 0;
const int rightTrigPin = 3;
const int rightEchoPin = 4;

// ---- Performance Constants ----
const int samplingDelay = 1; // in ms
const long pulseTimeout = 10000; // in us
const double kp = 1;
const double ki = 0;
const double kd = 6;
const double rWheel = 19/25.4; // wheel radius in mm
const int ticksPerRev = 610;
const int initialSpeed = 200;

// ---- Imported Constants for Turning ----
const int samplingDelayE = 10; // in ms
const double kpe = 0.8;  // proportional gain for encoders
const int turn180 = 820; // total distance to travel in counts
const int turn90 = 395;// total distance to travel in counts

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

void setup(){
  Serial.begin(9600);
  Serial.println("Starting...");
  stopMotors();
  delay(1000);  
  pinMode(leftTrigPin,OUTPUT);
  pinMode(leftEchoPin,INPUT);
  
  pinMode(frontTrigPin,OUTPUT);
  pinMode(frontEchoPin,INPUT);
  
  pinMode(rightTrigPin,OUTPUT);
  pinMode(rightEchoPin,INPUT);
   
}

void loop(){
  driveStraightUntilOpening();
  delay(200);
  if (rightDistance > 9){
    turnRight();
  }else{
    turnLeft();
  }
  delay(200);
  driveInches(6.5);
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
    delay(samplingDelayE); 
  }
  motors.setSpeeds(0, 0);
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
    delay(samplingDelayE); 
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
    delay(samplingDelayE); 
  }
  motors.setSpeeds(0, 0);
}

void driveStraightUntilOpening(){
  readUltrasonic();
  error = leftDistance - rightDistance;
  if (abs(error) < 7){
    motors.setSpeeds(speedLeft, speedRight);
    while(abs(error) < 7){
      readUltrasonic();
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
      snprintf_P(report, sizeof(report),
            PSTR("Distances: L%6d F%6d R%6d | Speeds: L%6d R%6d | Error: %6d"),
            leftDistance,frontDistance,rightDistance,speedLeft,speedRight,error);
      Serial.println(report);
    }
    clearPID();
    driveInches(3);
  }
}

void driveInches(double inches){
  Serial.println("Centering...");
  int16_t leftEncoderCal = encoders.getCountsLeft();
  int16_t rightEncoderCal = encoders.getCountsRight();
  speedLeft = initialSpeed;
  speedRight = initialSpeed;
  motors.setSpeeds(speedLeft, speedRight);  
  double leftDistanceTravelled = 0;
  double rightDistanceTravelled = 0;
  double avgDistance = 0;
  while(avgDistance < inches){
    leftDistanceTravelled = (encoders.getCountsLeft()-leftEncoderCal) * 2 * pi * rWheel /ticksPerRev; // distance left wheel travelled in mm
    rightDistanceTravelled = (encoders.getCountsRight()-rightEncoderCal) * 2 * pi * rWheel /ticksPerRev; // distance right wheel travelled in mm
    avgDistance = (leftDistanceTravelled+rightDistanceTravelled)/2;
    delay(samplingDelay);
  }  
  stopMotors();
}

void clearPID(){
  error = 0;
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

void stopMotors(){
  motors.setSpeeds(0, 0);
}
