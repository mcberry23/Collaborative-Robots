#include <SoftwareSerial.h>
SoftwareSerial HC12(13, 14); // HC-12 TX Pin, HC-12 RX Pin
int i = 0;
void setup() {
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               // Serial port to HC12
}

void loop() {
  HC12.write("Hello World");      // Send that data to HC-12
  HC12.write(i);      // Send that data to HC-12
  delay(100);
}

