
// parameters
uint8_t stateMap[6][12];
int rHeading;
uint8_t x;
uint8_t y;
uint8_t blocksTravelled;

void setup() {
  Serial1.begin(9600);
  Serial1.println("Starting...");

  // test values
  x = 2;
  y = 1;
  rHeading = 270;
  blocksTravelled = 2;
  
  // zero out the map
  for(int i=0;i<12;i++){
    for(int j=0;j<6;j++){
      stateMap[i][j] = 0;
    }
  }

  turnAround();
  
}

void loop() {
  // put your main code here, to run repeatedly:

}

void printMap(){
  for(int i=0;i<12;i++){
    for(int j=0;j<6;j++){
      Serial1.print(stateMap[j][i]);
      Serial1.print(" ");
    }
    Serial1.println();
  }
}

void updateMap(){
  if(rHeading == 0){
    stateMap[x][y]++;
    while(blocksTravelled > 0){
      x++;
      stateMap[x][y]++;
      blocksTravelled--;
    }
    blocksTravelled = 0;
  }
  else if(rHeading == 90){
    stateMap[x][y] = 1;
    while(blocksTravelled > 0){
      y++;
      stateMap[x][y]++;
      blocksTravelled--;
    }
    blocksTravelled = 0;
  }
  else if(rHeading == 180){
    stateMap[x][y]++;
    while(blocksTravelled > 0){
      x--;
      stateMap[x][y]++;
      blocksTravelled--;
    }
    blocksTravelled = 0;
  }
  else if(rHeading == 270){
    stateMap[x][y]++;
    while(blocksTravelled > 0){
      y--;
      stateMap[x][y]++;
      blocksTravelled--;
    }
    blocksTravelled = 0;
  }
  printMap();
}

void turnRight(){
  Serial1.println(rHeading);
  rHeading = rHeading - 90;
  Serial1.println(rHeading);
  if(rHeading<0){
    rHeading = rHeading + 360;
  }
  Serial1.println(rHeading);
}

void turnLeft(){
  Serial1.println(rHeading);
  rHeading = rHeading + 90;
  Serial1.println(rHeading);
  if(rHeading>=360){
    rHeading = rHeading - 360;
  }
  Serial1.println(rHeading);
}

void turnAround(){
  Serial1.println(rHeading);
  rHeading = rHeading + 180;
  Serial1.println(rHeading);
  if(rHeading>=360){
    rHeading = rHeading - 360;
  }
  else if (rHeading<0){
    rHeading = rHeading + 360;
  }
  Serial1.println(rHeading);
}
