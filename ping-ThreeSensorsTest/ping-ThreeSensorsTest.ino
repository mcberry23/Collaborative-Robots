
char report[80];

const int leftSigPin = 11;
long leftDuration;
int leftDistance;

const int frontSigPin = 14;
long frontDuration;
int frontDistance;

const int rightSigPin = 4;
long rightDuration;
int rightDistance;


void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read left sensor
  pinMode(leftSigPin, OUTPUT);
  digitalWrite(leftSigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(leftSigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(leftSigPin,LOW);
  pinMode(leftSigPin, INPUT);
  leftDuration = pulseIn(leftSigPin,HIGH);

  // Read front sensor
  pinMode(frontSigPin, OUTPUT);
  digitalWrite(frontSigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(frontSigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(frontSigPin,LOW);
  pinMode(frontSigPin, INPUT);
  frontDuration = pulseIn(frontSigPin,HIGH);

  // Read right sensor
  pinMode(rightSigPin, OUTPUT);
  digitalWrite(rightSigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rightSigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(rightSigPin,LOW);
  pinMode(rightSigPin, INPUT);
  rightDuration = pulseIn(rightSigPin,HIGH);
  
  // Calculate distances
  leftDistance = leftDuration*0.034/2;
  frontDistance = frontDuration*0.034/2;
  rightDistance = rightDuration*0.034/2;

  snprintf_P(report, sizeof(report),
        PSTR("%6d %6d %6d"),
        leftDistance,frontDistance,rightDistance);
  
  Serial.println(report);
//  
//  Serial.print("Distance: ");
//  Serial.println(frontDistance);

}
