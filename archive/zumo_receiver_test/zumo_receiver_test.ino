/* This example drives each motor on the Zumo forward, then
backward.  The yellow user LED is on when a motor should be
running forward and off when a motor should be running backward.
If a motor on your Zumo has been flipped, you can correct its
direction by uncommenting the call to flipLeftMotor() or
flipRightMotor() in the setup() function. */


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Zumo32U4.h>

RF24 radio(13, 17); // CE, CSN

const byte address[6] = "00001";
Zumo32U4Motors motors;
//Zumo32U4ButtonA buttonA;

void setup()
{  
//  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
//  Serial.println("receiver starting...");
}

void loop()
{

  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    int x = text[0]-48; // converts char to int
    int y = text[1]-48;// converts char to int
    int speed = 300;
    int motorLeft = 0;
    int motorRight = 0;
    if (x == 1 && y == 0){
      motorLeft = speed;
      motorRight = speed;
    }
    else if (x == 2 && y == 0){
      motorLeft = -speed;
      motorRight = -speed;
    }
    else if (x == 0 && y == 1){
      motorLeft = speed/1.5;
      motorRight = -speed/1.5;
    }
    else if (x == 0 && y == 2){
      motorLeft = -speed/1.5;
      motorRight = speed/1.5;
    }
    else if (x == 1 && y == 1){
      motorLeft = speed;
      motorRight = speed-(speed/4);
    }
    else if (x == 1 && y == 2){
      motorLeft =  speed-(speed/4);
      motorRight = speed;
    }
    else if (x == 2 && y == 2){
      motorLeft =  -(speed-(speed/4));
      motorRight = -speed;
    }
    else if (x == 2 && y == 1){
      motorLeft =  -speed;
      motorRight = -(speed-(speed/4));
    }
//    Serial.print(motorLeft);
//    Serial.print(" ");
//    Serial.println(motorRight);
    motors.setLeftSpeed(motorLeft);
    motors.setRightSpeed(motorRight);
    delay(100);
  }
}
