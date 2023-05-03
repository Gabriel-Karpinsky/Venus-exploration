#include <Servo.h>
Servo gripServo;
Servo ultraServo;
Servo servoLeft;
Servo servoRight;
Servo myServo;
const int servoPin = 11;

void forward(int time);

int pos = 80;
int startPos = 80;
int incrementInDegrees = 5;
int angleBoundry = 20;


int upGripper(){
  gripServo.write(0);
  delay(2000);
}
int closeGripper(){
  gripServo.write(70);
  delay(2000);
}
int openGripper(){
  gripServo.write(170);
  delay(2000);
}

void forward(int time)                       // Forward function
{
  servoLeft.writeMicroseconds(1700);         // Left wheel counterclockwise
  servoRight.writeMicroseconds(1300);        // Right wheel clockwise
  delay(time);                               // Maneuver for time ms
}

void turnLeft(int time)                      // Left turn function
{
  servoLeft.writeMicroseconds(1300);         // Left wheel clockwise
  servoRight.writeMicroseconds(1300);        // Right wheel clockwise
  delay(time);                               // Maneuver for time ms
}

void turnRight(int time)                     // Right turn function
{
  servoLeft.writeMicroseconds(1700);         // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700);        // Right wheel counterclockwise
  delay(time);                               // Maneuver for time ms
}

void backward(int time)                      // Backward function
{
  servoLeft.writeMicroseconds(1300);         // Left wheel clockwise
  servoRight.writeMicroseconds(1700);        // Right wheel counterclockwise
  delay(time);                               // Maneuver for time ms
}
void stopIt(int time){
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(time);
}

const int pingPin = 9;
long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

void setup() {
  gripServo.attach(10);
  servoLeft.attach(12);
  servoRight.attach(13); 
  Serial.begin(9600);

  myServo.attach(servoPin);
  myServo.write(startPos);
}

int look(){
  long duration, inches, cm;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  cm = microsecondsToCentimeters(duration);
  Serial.print(cm);
  Serial.println("cm// CurrentDegrees");
  return cm;
}

void loop() {
  forward(0);
  if(look()<=20){
    stopIt(0);
    backward(750);
    stopIt(0);
    turnRight(1000);
    //delay(250);
    forward(0);
  }
  int newPos = pos + incrementInDegrees;
  if (newPos > startPos + angleBoundry){
    incrementInDegrees = incrementInDegrees * -1;
    }  
  if (newPos < startPos - angleBoundry){
    incrementInDegrees = incrementInDegrees * -1;
    }
  pos += incrementInDegrees;
  myServo.write(pos);
  delay(100);
  //openGripper();
  //closeGripper();
  //upGripper();
 // closeGripper();
  //openGripper();
  
}
