#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;

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
const int pauseDelay = 50;
const long pulseTimeout = 10000; // in us
const double kp = 7;
const double ki = 0;
const double kd = 4;
const int maxError = 7; // cm
const double rWheel = 19/25.4; // wheel radius in mm
const int ticksPerRev = 610;
const int initialSpeed = 185;
const int turnSpeed = 170;
const char wallOpeningThresh = 14;
const int samplingDelayE = 10; // in ms
const double kpe = 0.8;  // proportional gain for encoders
const int turn180 = 840; // total distance to travel in counts
const int turn90 = 385;// total distance to travel in counts
const int oneBlockAwayThresh = 27; // cm
const int approachingWallCutoff = 5; // cm
const int driveOneBlockDistance = 8; // inches
const int centeringAdjustmentDistance = 1; // inches
const int isLeftWallFollowing = 1;
const int postPIDAdjustment = 60;

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
char robot = 'A';

// ---- Map Parameters ----
uint8_t stateMap[6][12];
int16_t rHeading;
uint8_t blocksTravelled;
uint8_t x = 0;
uint8_t y = 0;

void makeDecisionRight(){
  Serial1.write("making decision...\n");
  readSideUltrasonic();
  readFrontUltrasonic();
  boolean openRight = rightDistance > wallOpeningThresh;
  boolean openLeft = leftDistance > wallOpeningThresh;
  boolean openForward = frontDistance > wallOpeningThresh;
  if (openRight){
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
    delay(pauseDelay);
    motors.setSpeeds(-turnSpeed, -turnSpeed);
    delay(550);
    stopMotors();
  }
  delay(pauseDelay);
}

void makeDecisionLeft(){
  Serial1.write("making decision...\n");
  readSideUltrasonic();
  readFrontUltrasonic();
  boolean openRight = rightDistance > wallOpeningThresh;
  boolean openLeft = leftDistance > wallOpeningThresh;
  boolean openForward = frontDistance > wallOpeningThresh;
  if (openLeft){    
    turnLeft();
    if (!openRight){
      motors.setSpeeds(-turnSpeed, -turnSpeed);
      delay(200);
    }
    delay(pauseDelay);
    driveForwardOneBlock();
  }
  else if (openForward){
    driveForwardOneBlock();
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
  else{
    turnAround();
    delay(pauseDelay);
    motors.setSpeeds(-turnSpeed, -turnSpeed);
    delay(550);
    stopMotors();
  }
  delay(pauseDelay);
}

void turnRight(){
  rHeading = rHeading - 90;
  if(rHeading<0){
    rHeading = rHeading + 360;
  }
  //Serial1.print(rHeading);
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
  rHeading = rHeading + 90;
  if(rHeading>=360){
    rHeading = rHeading - 360;
  }
  //Serial1.print(rHeading);
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
  rHeading = rHeading + 180;
  if(rHeading>=360){
    rHeading = rHeading - 360;
  }
  else if (rHeading<0){
    rHeading = rHeading + 360;
  }
  //Serial1.print(rHeading);
  Serial1.write("turning around\n");
  int x = 0; 
  motors.setSpeeds(-turnSpeed, -turnSpeed);
  delay(100);
  stopMotors();
  int speedLeft = turnSpeed;
  int speedRight = -turnSpeed;
  motors.setSpeeds(speedLeft, speedRight);
  int leftCal = encoders.getCountsLeft();
  int rightCal = encoders.getCountsRight();
  while(x < turn180){
    int16_t countsLeft = encoders.getCountsLeft()-leftCal;
    int16_t countsRight = encoders.getCountsRight()-rightCal;   
    speedLeft = round((turnSpeed) - (kpe * (countsLeft + countsRight)));
    speedRight = - round((turnSpeed) - (kpe * (countsRight + countsLeft)));
    speedLeft = max(0,speedLeft);
    speedLeft = min(400,speedLeft);
    speedRight = max(-400,speedRight);
    speedRight = min(0,speedRight);
    motors.setSpeeds(speedLeft, speedRight);
    x = (abs(countsLeft) + abs(countsRight))/2;
    delay(samplingDelayE); 
  }
  stopMotors();
}

void driveStraightUntilOpening(){
  int countLeftTemp = encoders.getCountsLeft();
  int countRightTemp = encoders.getCountsRight();
  int avgCountTemp = 0;
  readSideUltrasonic();
  readFrontUltrasonic();
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
      readSideUltrasonic();
      readFrontUltrasonic();
    }
//    stopMotors();
    motors.setSpeeds(speedRight, speedLeft);// fix error from last iteration
    delay(postPIDAdjustment);  
    clearPID();
    driveInches(centeringAdjustmentDistance);
  }
  Serial1.write("I see an open path\n");
  countLeftTemp = encoders.getCountsLeft() - countLeftTemp;
  countRightTemp = encoders.getCountsRight() - countRightTemp;
  avgCountTemp = (countLeftTemp+countRightTemp)/2;
  if(avgCountTemp > 905){
    blocksTravelled = (int)(avgCountTemp/905);
  }
  else if((avgCountTemp > 650)&&(avgCountTemp < 905)){
    blocksTravelled = 1;
  }
  Serial1.write("-----\n");
  Serial1.print(avgCountTemp);
  Serial1.write("\n-----\n");
  Serial1.print(blocksTravelled);
  Serial1.print("\n-----\n");
  updateMap();
  printMap();
}

void driveForwardOneBlock(){
  readFrontUltrasonic();
  if (frontDistance < oneBlockAwayThresh){
    Serial1.write("driving 1 block towards wall\n");
    driveTowardsWall();
  }
  else{
    Serial1.write("driving 1 block with encoders\n");
    driveInches(driveOneBlockDistance);
  }
  blocksTravelled = 1;
  updateMap();
  printMap();
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
    readFrontUltrasonic();
  }  
  stopMotors();
}

void driveInches(double inches){
  Serial1.write("Driving inches\n ");
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

void readSideUltrasonic(){
  leftDistance = 0;
  rightDistance = 0;
  while(leftDistance == 0 || rightDistance == 0){
     // Clear the trigger pins
    digitalWrite(leftTrigPin,LOW);
    digitalWrite(rightTrigPin,LOW);
    delayMicroseconds(2);
  
    // Read left sensor
    digitalWrite(leftTrigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(leftTrigPin,LOW);
    leftDuration = pulseIn(leftEchoPin,HIGH,pulseTimeout);
    
    // Read right sensor
    digitalWrite(rightTrigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(rightTrigPin,LOW);
    rightDuration = pulseIn(rightEchoPin,HIGH,pulseTimeout);
    
    // Calculate distances
    leftDistance = leftDuration*0.034/2;
    rightDistance = rightDuration*0.034/2; 
    snprintf_P(report, sizeof(report),
          PSTR("Distances: L%3d R%3d\n"),
          int(leftDistance),int(rightDistance));  
    Serial1.write(report);
    if (leftDistance == 0 || rightDistance == 0){
      Serial1.write("!!!!!!!!  Distance Error !!!!!!!!!!!!\n");
    }    
  }
}

void readFrontUltrasonic(){
  frontDistance = 0;
  while(frontDistance == 0){
     // Clear the trigger pins
    digitalWrite(frontTrigPin,LOW);
    delayMicroseconds(2);

    // Read front sensor
    digitalWrite(frontTrigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(frontTrigPin,LOW);
    frontDuration = pulseIn(frontEchoPin,HIGH,pulseTimeout);

    // Calculate distances
    frontDistance = frontDuration*0.034/2;
    snprintf_P(report, sizeof(report),
          PSTR("Distances: F%3d\n"),
          int(frontDistance));  
    Serial1.write(report);
    if (frontDistance == 0){
      Serial1.write("!!!!!!!!  Distance Error !!!!!!!!!!!!\n");
    }     
  }
}

void stopMotors(){
  motors.setSpeeds(0, 0);
}

void updateMap(){
  if(rHeading == 0){
    while(blocksTravelled > 0){
      x++;
      stateMap[x][y]++;
      blocksTravelled--;
    }
    blocksTravelled = 0;
  }
  else if(rHeading == 90){
    while(blocksTravelled > 0){
      y++;
      stateMap[x][y]++;
      blocksTravelled--;
    }
    blocksTravelled = 0;
  }
  else if(rHeading == 180){
    while(blocksTravelled > 0){
      x--;
      stateMap[x][y]++;
      blocksTravelled--;
    }
    blocksTravelled = 0;
  }
  else if(rHeading == 270){
    while(blocksTravelled > 0){
      y--;
      stateMap[x][y]++;
      blocksTravelled--;
    }
    blocksTravelled = 0;
  }
}

void printMap(){
  Serial1.println("----x,y----");
  Serial1.print("x: ");
  Serial1.print(x);
  Serial1.print(", y: ");
  Serial1.println(y);
  Serial1.println("----Map Matrix----");
  for(int i=11;i>=0;i--){
    for(int j=0;j<6;j++){
      Serial1.print(stateMap[j][i]);
      Serial1.print(" ");
    }
    Serial1.println();
  }
  Serial1.println("----END----");
}

void txMap(char toRobot, char fromRobot, char dataType){
  char throwAway;
  while(true){
    int check = Serial1.available();
    if(check == 0){
      Serial1.print("T ");
      Serial1.print(toRobot);
      Serial1.print(" F ");
      Serial1.print(fromRobot);
      Serial1.print(" D ");
      Serial1.print(dataType);
      Serial1.print(" ");
      for(int i=0;i<12;i++){
        for(int j=0;j<6;j++){
          Serial1.print(stateMap[j][i]);
          Serial1.print(" ");
        }
      }
      break;
    }
    throwAway = Serial1.read();
  }
}

void setup(){
  
  // starting conditions
  rHeading = 90;
  x = 0;
  y = 0;
  blocksTravelled = 0;
  stateMap[x][y]++;
  
  Serial1.begin(9600);
  Serial1.write("\n\n--------Solving Maze--------\n ");
  Serial1.print(rHeading);
  Serial1.write("\n-----\n");
  stopMotors();
  pinMode(leftTrigPin,OUTPUT);
  pinMode(leftEchoPin,INPUT);
  
  pinMode(frontTrigPin,OUTPUT);
  pinMode(frontEchoPin,INPUT);
  
  pinMode(rightTrigPin,OUTPUT);
  pinMode(rightEchoPin,INPUT);
  delay(1500);
  
    while(mazeComplete == false){
      driveStraightUntilOpening();
      if (x >= 5 && y >= 11){
        break;
      }
      delay(pauseDelay);
      if (isLeftWallFollowing){
        makeDecisionLeft();  
      } 
      else  {
        makeDecisionRight(); 
      }  
      if (x >= 5 && y >= 11){
        mazeComplete = true;
      }
    }
    Serial1.write("Maze Completed\n ");
    delay(pauseDelay);
    turnLeft();
    delay(pauseDelay);
    driveInches(driveOneBlockDistance);
    delay(pauseDelay);
    turnLeft();
    delay(pauseDelay);
    driveInches(driveOneBlockDistance);
    if (isLeftWallFollowing){
      delay(pauseDelay);
      driveInches(driveOneBlockDistance);
    }
    txMap('C',robot,'M');
}

void loop(){

}
