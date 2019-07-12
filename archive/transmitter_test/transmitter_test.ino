/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  int x = analogRead(0);
  int y = analogRead(1);
  int xCommand, yCommand;
  if (x < 400) // forward
    xCommand = 1; 
  else if ( x > 600) // backward
    xCommand = 2;
  else
    xCommand = 0;
  if (y < 400) // right
    yCommand = 1;
  else if (y > 600) // left
    yCommand = 2;
  else
    yCommand = 0;  
  String text = "";
  text = text + xCommand;
  text = text + yCommand;
  int len = text.length()+1;
  char output[len];
  text.toCharArray(output,len);
  radio.write(&output,len);
  Serial.println(output);
  delay(100);
}
