#include <Servo.h>
#include <Arduino.h>

Servo gripperServo;
int gripper(int a,int b){
  //Servo called: gripperServo
  //a=robot,b=action (0==close,1==open,2==up)
  if(a==0){
    if(b==0) gripperServo.write(70);
    else if (b==1) gripperServo.write(170);
    else if (b==2) gripperServo.write(0);
    delay(2000);
  }
  else if(a==1){
    if(b==0) gripperServo.write(0);
    else if (b==1) gripperServo.write(80);
    else if (b==2) gripperServo.write(180);
    delay(2000);
  }
}
Servo leftServo;
Servo rightServo;
int movement(int a,int b){
  //Servo called: leftServo and rightServo
  //a==robot,b==action (0==stop,1==forward,2==backward,3==left,4==right)
  if(a==0){
    if(b==0){
      leftServo.writeMicroseconds(1500);
      rightServo.writeMicroseconds(1500);
    }
    else if(b==1){
      leftServo.writeMicroseconds(1700);
      rightServo.writeMicroseconds(1300);
    }
    else if(b==2){
      leftServo.writeMicroseconds(1300);
      rightServo.writeMicroseconds(1700);
    }
    else if(b==3){
      leftServo.writeMicroseconds(1300);
      rightServo.writeMicroseconds(1300);
    }
    else if(b==4){
      leftServo.writeMicroseconds(1700);
      rightServo.writeMicroseconds(1700);
    }
  }
  else if(a==1){
    if(b==0){
      leftServo.writeMicroseconds(1460);
      rightServo.writeMicroseconds(1520);
    }
    else if(b==1){
       leftServo.writeMicroseconds(1700);
      rightServo.writeMicroseconds(1300);
    }
    else if(b==2){
      leftServo.writeMicroseconds(1300);
      rightServo.writeMicroseconds(1700);
    }
    else if(b==3){
      leftServo.writeMicroseconds(1300);
      rightServo.writeMicroseconds(1300);
    }
    else if(b==4){
      leftServo.writeMicroseconds(1700);
      rightServo.writeMicroseconds(1700);
    }
  }
}
Servo ultrasonicServo;
int posServoUltra=0;
int posServoUltraDir=0;

int ultrasonic(int a,int b,int c,int d){
  //Servo called: ultrasonicServo
  //a==robot,b==boundA,c==boundB,d=increment
  if(a==0){
    if(posServoUltra<b || posServoUltra>c){
      if(posServoUltraDir==0)posServoUltraDir=1;
      else if(posServoUltraDir==1)posServoUltraDir=0;
    }
    if(posServoUltraDir==0){
      for(int i=posServoUltra;i<=c;i++){
        ultrasonicServo.write(i);
        posServoUltra=i+d;
        break;
      }
    }
    else if(posServoUltraDir==1){
      for(int i=posServoUltra;i>=b;i--){
        ultrasonicServo.write(i);
        posServoUltra=i-d;
        break;
      }
    }
  }
  else if(a==1){
    if(posServoUltra<b || posServoUltra>c){
      if(posServoUltraDir==0)posServoUltraDir=1;
      else if(posServoUltraDir==1)posServoUltraDir=0;
    }
    if(posServoUltraDir==0){
      for(int i=posServoUltra;i<=c;i++){
        ultrasonicServo.write(i);
        posServoUltra=i+d;
        break;
      }
    }
    else if(posServoUltraDir==1){
      for(int i=posServoUltra;i>=b;i--){
         ultrasonicServo.write(i);
        posServoUltra=i-d;
        break;
      }
    }
  }
//Moving UltrasonicServo can be probably cleaner/better (but it works)
  
  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);
  delayMicroseconds(2);
  digitalWrite(9, HIGH);
  delayMicroseconds(5);
  digitalWrite(9, LOW);
  pinMode(9, INPUT);
  int duration=pulseIn(9, HIGH);
  int dis_cm=duration/29/2;
  Serial.println("Distance="+String(dis_cm)+" cm   Angle="+String(posServoUltra));
  return dis_cm;
}


void setup() {
  Serial.begin(9600);
  gripperServo.attach(10);
  leftServo.attach(12);
  rightServo.attach(13);
  ultrasonicServo.attach(11);
}

void loop() {
  movement(0,1);
    if(ultrasonic(0,50,120,5)<=20){
      if(posServoUltra>90){
        movement(0,4);
        delay(200);
      } else if(posServoUltra<90){ 
        movement(0,3);
        delay(200);
        }
    }
}