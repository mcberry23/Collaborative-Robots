#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
}

void loop() {
  uint8_t number;
  bool signal ;  
  signal = digitalRead(7);
  if(signal == HIGH)  /* If new pin pressed */
   {
    delay(250);
    number = ( 0x00 | (digitalRead(3)<<0) | (digitalRead(4)<<1) | (digitalRead(5)<<2) | (digitalRead(4)<<3) );
      switch (number)
      {
        case 0x01:
        Serial.println("Pin Pressed : 1");
        break;
        case 0x02:
        Serial.println("Pin Pressed : 2");
        break;
        case 0x03:
        Serial.println("Pin Pressed : 3");
        break;
        case 0x04:
        Serial.println("Pin Pressed : 4");
        break;
        case 0x05:
        Serial.println("Pin Pressed : 5");
        break;
        case 0x06:
        Serial.println("Pin Pressed : 6");
        break;
        case 7:
        Serial.println("Pin Pressed : 7");
        break;
        case 0x08:
        Serial.println("Pin Pressed : 8");
        break;
        case 0x09:
        Serial.println("Pin Pressed : 9");
        break;
        case 0x0A:
        Serial.println("Pin Pressed : 0");
        break;
        case 0x0B:
        Serial.println("Pin Pressed : *");
        break;
        case 0x0C:
        Serial.println("Pin Pressed : #");
        break;    
      }
  }
}