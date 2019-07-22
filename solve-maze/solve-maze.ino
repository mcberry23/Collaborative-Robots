#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;

#define pi 3.14159265359
//
//extern void makeDecision();
//extern void turnRight();
//extern void turnLeft();
//extern void turnAround();
//extern void driveStraightUntilOpening();
//extern void driveForwardOneBlock();
//extern void driveTowardsWall();
//extern void driveInches(double inches);
//extern void clearPID();
//extern void readUltrasonic();
//extern void stopMotors();

// ---- Pin Setup ----
const int leftTrigPin = 5;
const int leftEchoPin = 11;
const int frontTrigPin = 2;
const int frontEchoPin = 17;
const int rightTrigPin = 3;
const int rightEchoPin = 4;

// ---- Performance Constants ----
const int samplingDelay = 1; // in ms
const int pauseDelay = 100;
const long pulseTimeout = 10000; // in us
const double kp = 8;
const double ki = 0;
const double kd = 4;
const int maxError = 7; // cm
const double rWheel = 19/25.4; // wheel radius in mm
const int ticksPerRev = 610;
const int initialSpeed = 190;
const int turnSpeed = 175;
const char wallOpeningThresh = 14;
const int samplingDelayE = 10; // in ms
const double kpe = 0.8;  // proportional gain for encoders
const int turn180 = 840; // total distance to travel in counts
const int turn90 = 385;// total distance to travel in counts
const int oneBlockAwayThresh = 27; // cm
const int approachingWallCutoff = 6; // cm
const int driveOneBlockDistance = 7; // inches
const int centeringAdjustmentDistance = 1; // inches

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
  Serial1.write("\n\n--------Solving Maze--------\n");
  stopMotors();
  pinMode(leftTrigPin,OUTPUT);
  pinMode(leftEchoPin,INPUT);
  
  pinMode(frontTrigPin,OUTPUT);
  pinMode(frontEchoPin,INPUT);
  
  pinMode(rightTrigPin,OUTPUT);
  pinMode(rightEchoPin,INPUT);
  delay(1500);
  while(mazeComplete == false){
      for(int temp = 0;temp<70;temp++){
        snprintf_P(report, sizeof(report),
          PSTR("Iteration: %3d\n"),
          temp);  
        Serial1.write(report);
        driveStraightUntilOpening();
        delay(pauseDelay);
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
    if (!openLeft){
      motors.setSpeeds(-turnSpeed, -turnSpeed);
      delay(200);
    }
    stopMotors();
    delay(pauseDelay);
    driveForwardOneBlock();
  }
  else if (openForward){
    driveForwardOneBlock();
  }
  else if (openLeft){
    turnLeft();
    if (!openRight){
      motors.setSpeeds(-turnSpeed, -turnSpeed);
      delay(200);
    }
    delay(pauseDelay);
    driveForwardOneBlock();
  }
  else{
    turnAround();
    motors.setSpeeds(-turnSpeed, -turnSpeed);
    delay(200);
    stopMotors();
  }
  delay(pauseDelay);
}

void turnRight(){
  Serial1.write("turning right\n");
  int x = 0;
  int speedLeft = turnSpeed;
  int speedRight = - turnSpeed;
  int leftCal = encoders.getCountsLeft();
  int rightCal = encoders.getCountsRight();
  motors.setSpeeds(speedLeft, speedRight);
  while(x < turn90){
    int16_t countsLeft = encoders.getCountsLeft()-leftCal;
    int16_t countsRight = encoders.getCountsRight()-rightCal;   
    speedLeft = round(turnSpeed - (kpe * (countsLeft + countsRight)));
    speedRight = - round(turnSpeed - (kpe * (countsRight + countsLeft)));
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
  int speedLeft = -turnSpeed;
  int speedRight = turnSpeed;
  int leftCal = encoders.getCountsLeft();
  int rightCal = encoders.getCountsRight();
  motors.setSpeeds(speedLeft, speedRight);
  while(x < turn90){
    int16_t countsLeft = encoders.getCountsLeft()-leftCal;
    int16_t countsRight = encoders.getCountsRight()-rightCal;   
    speedLeft = - round(turnSpeed - (kpe * (countsLeft + countsRight)));
    speedRight = round(turnSpeed - (kpe * (countsRight + countsLeft)));
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
  int leftCal = encoders.getCountsLeft();
  int rightCal = encoders.getCountsRight();
  motors.setSpeeds(-turnSpeed, -turnSpeed);
  delay(200);
  int speedLeft = turnSpeed*0.5;
  int speedRight = -turnSpeed*0.5;
  motors.setSpeeds(speedLeft, speedRight);
  while(x < turn180){
    int16_t countsLeft = encoders.getCountsLeft()-leftCal;
    int16_t countsRight = encoders.getCountsRight()-rightCal;   
    speedLeft = round((turnSpeed*0.5) - (kpe * (countsLeft + countsRight)));
    speedRight = - round((turnSpeed*0.5) - (kpe * (countsRight + countsLeft)));
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
  if ((abs(error) < maxError) && (frontDistance > approachingWallCutoff)){
    Serial1.write("driving straight until opening\n");
    motors.setSpeeds(speedLeft, speedRight);
    while((abs(error) < maxError) && (frontDistance > approachingWallCutoff)){
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
//    stopMotors();
    motors.setSpeeds(speedRight, speedLeft);// fix error from last iteration
    delay(40);  
    clearPID();
    driveInches(centeringAdjustmentDistance);
  }
  Serial1.write("I see an open path\n");
}

void driveForwardOneBlock(){
  readUltrasonic();
  if (frontDistance < oneBlockAwayThresh){
    Serial1.write("driving 1 block towards wall\n");
    driveTowardsWall();
  }
  else{
    Serial1.write("driving 1 block with encoders\n");
    driveInches(driveOneBlockDistance);
  }
}

void driveTowardsWall(){
  int16_t leftCal = encoders.getCountsLeft();
  int16_t rightCal = encoders.getCountsRight();
  speedLeft = initialSpeed;
  speedRight = initialSpeed;
  motors.setSpeeds(speedLeft, speedRight);  
  while(frontDistance > approachingWallCutoff){
    int16_t countsLeft = encoders.getCountsLeft()-leftCal;
    int16_t countsRight = encoders.getCountsRight()-rightCal;   
    speedLeft = round(initialSpeed - (kpe * (countsLeft - countsRight)));
    speedRight = round(initialSpeed - (kpe * (countsRight - countsLeft)));
    speedLeft = max(0,speedLeft);
    speedLeft = min(400,speedLeft);
    speedRight = max(0,speedRight);
    speedRight = min(400,speedRight);
    motors.setSpeeds(speedLeft, speedRight); 
    readUltrasonic();
  }  
  stopMotors();
}

void driveInches(double inches){
  int16_t leftCal = encoders.getCountsLeft();
  int16_t rightCal = encoders.getCountsRight();
  motors.setSpeeds(initialSpeed, initialSpeed);  
  double leftDistanceTravelled = 0;
  double rightDistanceTravelled = 0;
  double avgDistance = 0;
  while(avgDistance < inches){
    int16_t countsLeft = encoders.getCountsLeft()-leftCal;
    int16_t countsRight = encoders.getCountsRight()-rightCal;   
    speedLeft = round(initialSpeed - (kpe * (countsLeft - countsRight)));
    speedRight = round(initialSpeed - (kpe * (countsRight - countsLeft)));
    speedLeft = max(0,speedLeft);
    speedLeft = min(400,speedLeft);
    speedRight = max(0,speedRight);
    speedRight = min(400,speedRight);
    motors.setSpeeds(speedLeft, speedRight);    
    leftDistanceTravelled = countsLeft * 2 * pi * rWheel /ticksPerRev; // distance left wheel travelled in mm
    rightDistanceTravelled = countsRight * 2 * pi * rWheel /ticksPerRev; // distance right wheel travelled in mm
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
  leftDistance = 0;
  frontDistance = 0;
  rightDistance = 0;
  while((leftDistance == 0 || frontDistance == 0)  || rightDistance == 0){
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
      Serial1.write("!!!!!!!!  Distance Error !!!!!!!!!!!!\n");
    }    
  }
}

void stopMotors(){
  motors.setSpeeds(0, 0);
}
