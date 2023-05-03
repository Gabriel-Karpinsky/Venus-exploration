#include <Arduino.h>
#include <Servo.h>                           // Include servo library
 
Servo servoLeft;                             // Declare left and right servos
Servo servoRight;
void forward(int time);
void setup()                                 // Built-in initialization block
{
  /*pinMode(10, INPUT);  pinMode(9, OUTPUT);   // Left IR LED & Receiver
  pinMode(3, INPUT);  pinMode(2, OUTPUT);    // Right IR LED & Receiver
 */  
  tone(4, 3000, 1000);  
  Serial.begin(115200);   
  // Play tone for 1 second
  delay(1000);                               // Delay to finish tone

  pinMode(8, INPUT);
  pinMode(7, INPUT);

  servoLeft.attach(12);                      // Attach left signal to pin 13
  servoRight.attach(13);                     // Attach right signal to pin 12
}  
 
void loop()                                  // Main loop auto-repeats
{
  //delay(1000);
  //servoLeft.writeMicroseconds(1700);         // Left wheel counterclockwise
  //servoRight.writeMicroseconds(0);
  //delay(1000);
  // servoLeft.writeMicroseconds(1500);
  int pin8 = digitalRead(8);
  int pin7 = digitalRead(7);
  char buffer[128];
  sprintf(buffer, "pin8 read: %d\n pin7 read: %d\n\n", pin8, pin7);
  //Serial.println(buffer);
}

int irDetect(int irLedPin, int irReceiverPin, long frequency)
{
  tone(irLedPin, frequency, 8);              // IRLED 38 kHz for at least 1 ms
  delay(1);                                  // Wait 1 ms
  int ir = digitalRead(irReceiverPin);       // IR receiver -> ir variable
  delay(1);                                  // Down time before recheck
  return ir;                                 // Return 1 no detect, 0 detect
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
 /* int irLeft = irDetect(9, 10, 38000);       // Check for object on left
  int irRight = irDetect(2, 3, 38000);       // Check for object on right

  if((irLeft == 0) && (irRight == 0))        // If both sides detect
  {
    backward(1000);                          // Back up 1 second
    turnLeft(800);                           // Turn left about 120 degrees
  }
  else if(irLeft == 0)                       // If only left side detects
  {
    backward(1000);                          // Back up 1 second
    turnRight(400);                          // Turn right about 60 degrees
  }
  else if(irRight == 0)                      // If only right side detects
  {
    backward(1000);                          // Back up 1 second
    turnLeft(400);                           // Turn left about 60 degrees
  }
  else                                       // Otherwise, no IR detected
  {
    forward(20);                             // Forward 1/50 of a second
  }
  */