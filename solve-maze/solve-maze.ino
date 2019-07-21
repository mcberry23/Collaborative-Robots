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
const int frontEchoPin = 17;
const int rightTrigPin = 3;
const int rightEchoPin = 4;

// ---- Performance Constants ----
const int samplingDelay = 1; // in ms
const long pulseTimeout = 10000; // in us
const double kp = 6;
const double ki = 0;
const double kd = 10;
const double rWheel = 19/25.4; // wheel radius in mm
const int ticksPerRev = 610;
const int initialSpeed = 175;
const char wallOpeningThresh = 14;
const int samplingDelayE = 10; // in ms
const double kpe = 0.8;  // proportional gain for encoders
const int turn180 = 840; // total distance to travel in counts
const int turn90 = 390;// total distance to travel in counts

// ---- Variables ----
long leftDuration;
long frontDuration;
long rightDuration;
double leftDistance;
double frontDistance;
double rightDistance;
double error = 0;
int lastError = 0;
int integral = 0;
int derivative = 0;
char report[80];
int speedLeft = initialSpeed;
int speedRight = initialSpeed;
boolean mazeComplete = false;


void setup(){
  Serial1.begin(9600);
  Serial1.println("----Solving Maze----");
  stopMotors();
  pinMode(leftTrigPin,OUTPUT);
  pinMode(leftEchoPin,INPUT);
  
  pinMode(frontTrigPin,OUTPUT);
  pinMode(frontEchoPin,INPUT);
  
  pinMode(rightTrigPin,OUTPUT);
  pinMode(rightEchoPin,INPUT);
  delay(1500);
  while(mazeComplete == false){
      for(int temp = 0;temp<7;temp++){
        driveStraightUntilOpening();
        delay(500);
        makeDecision();   
      }   
      mazeComplete = true;
  }
}

void loop(){

}

void makeDecision(){
  Serial1.write("making decision...\n");
  readUltrasonic();
  boolean openRight = rightDistance > wallOpeningThresh;
  boolean openLeft = leftDistance > wallOpeningThresh;
  boolean openForward = frontDistance > wallOpeningThresh;
  if ((openRight && openLeft) && openForward){
    mazeComplete = true;
    Serial1.write("maze completed\n");
  }
  else if (openRight){
    turnRight();
    delay(500);
    driveForwardOneBlock();
  }
  else if (openForward){
    driveStraightUntilOpening();
  }
  else if (openLeft){
    turnLeft();
    delay(500);
    driveForwardOneBlock();
  }
  else{
    turnAround();
  }
  delay(500);
}

void turnRight(){
  Serial1.write("turning right\n");
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
  Serial1.write("turning left\n");
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
  Serial1.write("turning around\n");
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
    Serial1.write("driving straight until opening\n");
    motors.setSpeeds(speedLeft, speedRight);
    while(abs(error) < 7){
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
      readUltrasonic();
    }
    stopMotors();
    clearPID();
    delay(500);
    driveInches(3);
  }
  Serial1.write("I see and open path\n");
}

void driveForwardOneBlock(){
  readUltrasonic();
  if (frontDistance < 30){
    Serial1.write("driving 1 block towards wall\n");
//    tone(6, 440, 200);
    speedLeft = initialSpeed;
    speedRight = initialSpeed;
    motors.setSpeeds(speedLeft, speedRight);   
    while(frontDistance > 5){
       readUltrasonic();
    }
    stopMotors();
  }
  else{
    Serial1.write("driving 1 block with encoders\n");
    driveInches(7);
  }
}

void driveInches(double inches){
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
  snprintf_P(report, sizeof(report),
        PSTR("Distances: L%3d F%3d R%3d\n"),
        int(leftDistance),int(frontDistance),int(rightDistance));  
  Serial1.write(report);
  if ((leftDistance == 0 || frontDistance == 0)  || rightDistance == 0){
    tone(6, 1000, 200);
    noTone(6);
    delay(200);
    tone(6, 1000, 200);
    delay(200);
  }
}

void stopMotors(){
  motors.setSpeeds(0, 0);
}
