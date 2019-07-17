
char report[80];

const int leftTrigPin = 5;
const int leftEchoPin = 11;
long leftDuration;
int leftDistance;

const int frontTrigPin = 2;
const int frontEchoPin = 0;
long frontDuration;
int frontDistance;

const int rightTrigPin = 3;
const int rightEchoPin = 4;
long rightDuration;
int rightDistance;


void setup() {
  pinMode(leftTrigPin,OUTPUT);
  pinMode(leftEchoPin,INPUT);
  
  pinMode(frontTrigPin,OUTPUT);
  pinMode(frontEchoPin,INPUT);
  
  pinMode(rightTrigPin,OUTPUT);
  pinMode(rightEchoPin,INPUT);
  
  Serial.begin(9600);
}

void loop() {
  // Clear the trigger pins
  digitalWrite(leftTrigPin,LOW);
  digitalWrite(frontTrigPin,LOW);
  digitalWrite(rightTrigPin,LOW);
  delayMicroseconds(2);

  // Read left sensor
  digitalWrite(leftTrigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(leftTrigPin,LOW);
  leftDuration = pulseIn(leftEchoPin,HIGH);

  // Read front sensor
  digitalWrite(frontTrigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(frontTrigPin,LOW);
  frontDuration = pulseIn(frontEchoPin,HIGH);

  // Read right sensor
  digitalWrite(rightTrigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(rightTrigPin,LOW);
  rightDuration = pulseIn(rightEchoPin,HIGH);
  
  // Calculate distances
  leftDistance = leftDuration*0.034/2;
  frontDistance = frontDuration*0.034/2;
  rightDistance = rightDuration*0.034/2;

  snprintf_P(report, sizeof(report),
        PSTR("%6d %6d %6d"),
        leftDistance,frontDistance,rightDistance);
  
  Serial.println(report);
  
  //Serial.print("Distance: ");
  //Serial.println(frontDistance);

}
