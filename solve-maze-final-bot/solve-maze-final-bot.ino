#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;

#define pi 3.14159265359
// ---- Pin Setup ----
const int leftSigPin = 11;
const int frontSigPin = 14;
const int rightSigPin = 4;

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
const int isLeftWallFollowing = 0;
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

// ---- Map Parameters ----
uint8_t stateMap[6][12];
uint8_t stateMapA[6][12];
uint8_t stateMapB[6][12];
uint8_t stateMapC[6][12];
uint8_t optimalPathMap[6][12];
int16_t rHeading;
uint8_t blocksTravelled;
uint8_t x = 0;
uint8_t y = 0;

// ---- Rx Parameters ----
char robot = 'C';
char packet[157];
boolean recievedMapA = false;
boolean recievedMapB = false;

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
  leftDuration = 0;
  rightDuration = 0;
//  while((leftDistance == 0 || leftDistance == 1)  || (rightDistance == 0 || rightDistance == 1)){
    // Read left sensor
  delay(1); 
  pinMode(leftSigPin, OUTPUT);
  digitalWrite(leftSigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(leftSigPin,HIGH);
  delayMicroseconds(5);
  digitalWrite(leftSigPin,LOW);
  pinMode(leftSigPin, INPUT);
  leftDuration += pulseIn(leftSigPin,HIGH);
  delay(1);
  pinMode(leftSigPin, OUTPUT);
  digitalWrite(leftSigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(leftSigPin,HIGH);
  delayMicroseconds(5);
  digitalWrite(leftSigPin,LOW);
  pinMode(leftSigPin, INPUT);
  leftDuration += pulseIn(leftSigPin,HIGH);
  delay(1);
  pinMode(leftSigPin, OUTPUT);
  digitalWrite(leftSigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(leftSigPin,HIGH);
  delayMicroseconds(5);
  digitalWrite(leftSigPin,LOW);
  pinMode(leftSigPin, INPUT);
  leftDuration += pulseIn(leftSigPin,HIGH);
    
  // Read right sensor
  delay(1);
  pinMode(rightSigPin, OUTPUT);
  digitalWrite(rightSigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rightSigPin,HIGH);
  delayMicroseconds(5);
  digitalWrite(rightSigPin,LOW);
  pinMode(rightSigPin, INPUT);
  rightDuration += pulseIn(rightSigPin,HIGH);
  delay(1);
  pinMode(rightSigPin, OUTPUT);
  digitalWrite(rightSigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rightSigPin,HIGH);
  delayMicroseconds(5);
  digitalWrite(rightSigPin,LOW);
  pinMode(rightSigPin, INPUT);
  rightDuration += pulseIn(rightSigPin,HIGH);
  delay(1);
  pinMode(rightSigPin, OUTPUT);
  digitalWrite(rightSigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rightSigPin,HIGH);
  delayMicroseconds(5);
  digitalWrite(rightSigPin,LOW);
  pinMode(rightSigPin, INPUT);
  rightDuration += pulseIn(rightSigPin,HIGH);
  delay(1);  
  
    // Calculate distances
    leftDistance = (leftDuration/3.0)*0.034/2;
    rightDistance = (rightDuration/3.0)*0.034/2; 
    snprintf_P(report, sizeof(report),
          PSTR("Distances: L%3d R%3d\n"),
          int(leftDistance),int(rightDistance));  
    Serial1.write(report);
//    if (leftDistance == 0 || rightDistance == 0){
//      Serial1.write("!!!!!!!!  Distance Error !!!!!!!!!!!!\n");
//    }    
//  }
}

void readFrontUltrasonic(){
  frontDistance = 0;
  frontDuration = 0;
//  while(frontDistance == 0){
      // Read front sensor
    delay(1); 
    pinMode(frontSigPin, OUTPUT);
    digitalWrite(frontSigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(frontSigPin,HIGH);
    delayMicroseconds(5);
    digitalWrite(frontSigPin,LOW);
    pinMode(frontSigPin, INPUT);
    frontDuration += pulseIn(frontSigPin,HIGH);
    delay(1);
    pinMode(frontSigPin, OUTPUT);
    digitalWrite(frontSigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(frontSigPin,HIGH);
    delayMicroseconds(5);
    digitalWrite(frontSigPin,LOW);
    pinMode(frontSigPin, INPUT);
    frontDuration += pulseIn(frontSigPin,HIGH);
    delay(1);
    pinMode(frontSigPin, OUTPUT);
    digitalWrite(frontSigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(frontSigPin,HIGH);
    delayMicroseconds(5);
    digitalWrite(frontSigPin,LOW);
    pinMode(frontSigPin, INPUT);
    frontDuration += pulseIn(frontSigPin,HIGH);
    delay(1);  
    // Calculate distances
    frontDistance = (frontDuration/3.0)*0.034/2;
    snprintf_P(report, sizeof(report),
          PSTR("Distances: F%3d\n"),
          int(frontDistance));  
    Serial1.write(report);  
//  }
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

void rxPacket(){
  char incomingByte;
  uint8_t index = 0;
  while(true){
    if(Serial1.available()>0){
      incomingByte = char(Serial1.read());
      packet[index] = incomingByte;
      index++;
    }
    if(index==157){
      packet[index] = '\0';
      break;
    }
  }

  if(packet[2] == robot){ // if to me
    if(packet[6] == 'A'){ // if from A
      if(packet[10] == 'M'){ // if matrix
        recievedMapA = true;
//        Serial.print("\n----Recieving----\n");
//        Serial.print("To: ");
//        Serial.print(packet[2]);
//        Serial.print("\nFrom: ");
//        Serial.print(packet[6]);
//        Serial.print("\nData: ");
//        Serial.print(packet[10]);
//        Serial.print("\n----END----\n");
        int tempIndex = 0;
        for(int i=12;i<155;i++){
          if(packet[i] != ' '){
            int x = ((i%12)/2);
            int y = ((i/12)-1);
            int val = int(packet[i])-48;
          stateMapA[x][y] = val;
          }
        }
      }
    }
    else if(packet[6] == 'B'){ // if from B
      if(packet[10] == 'M'){ // if matrix
        recievedMapB = true;
//        Serial.print("\n----Recieving----\n");
//        Serial.print("To: ");
//        Serial.print(packet[2]);
//        Serial.print("\nFrom: ");
//        Serial.print(packet[6]);
//        Serial.print("\nData: ");
//        Serial.print(packet[10]);
//        Serial.print("\n----END----\n");
        int tempIndex = 0;
        for(int i=12;i<155;i++){
          if(packet[i] != ' '){
            int x = ((i%12)/2);
            int y = ((i/12)-1);
            int val = int(packet[i])-48;
          stateMapB[x][y] = val;
          }
        }
      }
    }
    else if(packet[6] == 'C'){ // if from C
      if(packet[10] == 'M'){ // if matrix
        Serial.print("\n----Recieving----\n");
        Serial.print("To: ");
        Serial.print(packet[2]);
        Serial.print("\nFrom: ");
        Serial.print(packet[6]);
        Serial.print("\nData: ");
        Serial.print(packet[10]);
        Serial.print("\n----END----\n");
        int tempIndex = 0;
        for(int i=12;i<155;i++){
          if(packet[i] != ' '){
            int x = ((i%12)/2);
            int y = ((i/12)-1);
            int val = int(packet[i])-48;
          stateMapC[x][y] = val;
          }
        }
      }
    }
  }
}

void compareMaps(){
  for(int i=0;i<12;i++){
    for(int j=0;j<6;j++){
      if((stateMapA[j][i] == 0)||(stateMapB[j][i] == 0)){
        optimalPathMap[j][i] = 0;
      }
      else if((stateMapA[j][i] > 1)&&(stateMapB[j][i] > 1)){
        optimalPathMap[j][i] = 0;
      }
      else{
        optimalPathMap[j][i] = 1;
      }
    }
  }
}

int lookLeft(int _x, int _y){
  int nextX = _x - 1;
  if(nextX < 0){ // if out of bounds
    return 0;
  }
  else{ // else within bounds
    int nextY = _y;
    int nextBlock = optimalPathMap[nextX][nextY];
    if(nextBlock = 0){ // if left isn't part of optimal path
      return 0;
    }
    else{ // else left is part of optimal path
      readSideUltrasonic();
      boolean openLeft = leftDistance > wallOpeningThresh;
      if(openLeft = false){ // if left isn't open
        return 0;
      }
      else{ // else left is open
        return 1;
      }
    }
  }
}

int lookFront(int _x, int _y){
  int nextY = _x + 1;
  if(nextY > 11){ // if out of bounds
    return 0;
  }
  else{
    int nextX = _x;
    int nextBlock = optimalPathMap[nextX][nextY];
    if(nextBlock = 0){
      return 0;
    }
    else{
      readFrontUltrasonic();
      boolean openFront = frontDistance > wallOpeningThresh;
      if(openFront = false){
        return 0;
      }
      else{
        return 1;
      }
    }
  }
}

int lookRight(int _x, int _y){
  int nextX = _x + 1;
  if(nextX > 5){ // if out of bounds
    return 0;
  }
  else{
    int nextY = _y;
    int nextBlock = optimalPathMap[nextX][nextY];
    if(nextBlock = 0){
      return 0;
    }
    else{
      readSideUltrasonic();
      boolean openRight = rightDistance > wallOpeningThresh;
      if(openRight = false){
        return 0;
      }
      else{
        return 1;
      }
    }
  }
}

void followOptimalPath(){
  while(true){
    if((x>=5)&&(y>=11)){ // if at end
      mazeComplete = true;
      break;
    }
    else{ // else not at end
      if(lookLeft(x,y) == 1){
        turnLeft();
        driveForwardOneBlock();
      }
      else if(lookFront(x,y) == 1){
        driveForwardOneBlock();
      }
      else if(lookRight(x,y) == 1){ // if right is open
        turnRight();
        driveForwardOneBlock();
      }
      else{ // error - exit - maze not completed
        break;
      } 
    }
  }
  if(mazeComplete == true){
    turnLeft();
    driveForwardOneBlock();
  }
  else{
    // error
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
  if (isLeftWallFollowing)
  {
    delay(pauseDelay);
    driveInches(driveOneBlockDistance);
  }
}

void loop(){

}
