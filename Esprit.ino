#include <QTRSensors.h>
#include <Encoder.h>
#include "pins_arduino.h"

const uint8_t SensorCount = 14;
int leee=52;

uint16_t s[SensorCount];
QTRSensorsRC qtr((unsigned char[]){A14,A13,A12,A11,A10,A8,A7,A6,A5,A4,A3,A2,A1,A0}, SensorCount);
int position;
int npR= -0;
int npL= -0;
int n = 0, p = 0, m = 0;
//PID
int right_speed, left_speed;
float kp = 0.22, kd = 1.5 , ki =0, P, D, I; //vitesse 150 kp0.105 kd0.97   vitesse 255 kp0.11 kd0.38
float PIDvalue, lasterror, error;
int left_base=255, right_base=255;
//motors
int leftMotorPin1 = 6;
int leftMotorPin2 = 7;
int rightMotorPin1 = 9;
int rightMotorPin2 = 8;
//time
unsigned int current_time=0, last_time=0, lunch_time;

//kp=0.07;kd=0.12; high speed
//kp=0.06;kd=0.11; medium speed
bool test=false;

int led = 48;

int comp=20;
int pos;


int time=0;
//caps
  int c1;
  int c2;
  int c3;
  int c4;
  int c5;
  int c6;
  int c7;
  int c8;
  int c9;
  int c10;
  int c11;
  int c12;
  int c13;
  int c14;
Encoder myEncl(21,20);
Encoder myEncr(18,19);
void setup() {
  int tst = 0;
  //moteurs
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(47, INPUT);
  pinMode(led, OUTPUT);
  pinMode(leee, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(53,OUTPUT);
  pinMode(51,OUTPUT);
  digitalWrite(3, LOW);
  //ledon(4);
  //forwardm(11111);
  //left(10000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 150; i++) {
    qtr.calibrate();
    tst = 1;
    delay(10);
  }
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  pinMode(30, OUTPUT);
current_time=millis();
    int x=0;
    digitalWrite(22,0);
    while(1){
    x=digitalRead(30);
    //Serial.println(x);
      if(x==HIGH){
          lunch_time=millis();
        if(lunch_time-current_time>200)
        break;
      }
    }
  current_time=millis();
  last_time=current_time;
  n=1;
  forward(300);

}

void loop() {
  qtr.readLine(s);
  //position = qtr.readLineM(s,QTR_EMITTERS_ON,0,false);
  current_time=millis();
  //Serial.println(current_time-last_time);
  //delay(1000);
  for(int i=0;i<14;i++){
    if(s[i]>500)s[i]=1000;
    else s[i]=0;
  }
  int c1=s[0];
  int c2=s[1];
  int c3=s[2];
  int c4=s[3];
  int c5=s[4];
  int c6=s[5];
  int c7=s[6];
  int c8=s[7];
  int c9=s[8];
  int c10=s[9];
  int c11=s[10];
  int c12=s[11];
  int c13=s[12];
  int c14=s[13];
  int somme=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12+c13+c14;
  int rr=c1+c2+c3+c4+c5+c6+c7;
  int ll=c8+c9+c10+c11+c12+c13+c14;
  /*myEncl.readAndReset();
  myEncr.readAndReset();
  forwardCM(100);
  stop(50000);*/
  


 if(n==1 && c10+c11+c12+c13>=3000&&current_time-last_time>=3700){
    digitalWrite(13,1);
    dour(-37);
    n=2;
    last_time=current_time;
    digitalWrite(13,0);
  }
  else if(n==2&& somme<=1000&&current_time-last_time>=500){
    digitalWrite(13,1);
    dour(235);
    last_time=current_time;
    n=3;
    digitalWrite(13,0);
  }
  else if(n==3 &&c10+c11+c12+c13>=3000&&current_time-last_time>=800){
    digitalWrite(13,1);
    dour(-65);
    n=4;
    last_time=current_time;
    digitalWrite(13,0);
  }
  else if(n==4 && current_time-last_time>=2000){
    digitalWrite(13,1);
    stop(200);
    digitalWrite(13,0);
    n=900;
    last_time=current_time;
  }
  else if (n==900 && somme<=0 && current_time-last_time>=600){
    left(9);
    digitalWrite(13,1);
    //stop(500);
    forward(1100);
    //stop(500);
    right_fblstou(275);
    //stop(500);
    forward(450);
    stop(500);
    n=17;
    last_time=current_time;
    digitalWrite(13,0);
  }
  /*else if(n==16&&somme>=100){
    digitalWrite(13,1);
    n=17;
    last_time=current_time;
  }*/
  else if(n==17&&somme<=1000&&current_time-last_time>=0){
    digitalWrite(13,1);
    forward(100);
    left(100);
    stop(200);
    n=18;
    digitalWrite(13,0);
    last_time=current_time;
  }
  else if (n==18 && somme>=8000 && current_time-last_time>=600){
    digitalWrite(13,1);
    forward(100);
    left(15);
    forward(25);
    stop(100);
    n=19;
    digitalWrite(13,0);
    last_time=current_time;
  }
  else if (n==19 && somme>=11000 && current_time-last_time>=1100){
    digitalWrite(13,1);
    //forward(100);
    right(75);
    forward(280);
    stop(100);
    n=20;
    digitalWrite(13,0);
    last_time=current_time;
  }
  else if (n==20 && somme>=8000 && current_time-last_time>=800){
    //digitalWrite(13,1);
    forward(200);
    stop(1);
    digitalWrite(13, 1);
    stop(2000);
    digitalWrite(53, 1);
    stop(2000);
    digitalWrite(3, 1);
    stop(1000);
    n=21;
    digitalWrite(13,0);
    digitalWrite(53,0);
    digitalWrite(3,0);
    forward(600);
    //digitalWrite(13,0);
    last_time=current_time;
  }
  else if(n==21 && somme>=8000 && current_time-last_time>=600){
    digitalWrite(13, 1);
    stop(200);
    forward(50);
    last_time=current_time;
    n=22;
    digitalWrite(13, 0);
  }
  else if(n==22 && rr<=4000 && current_time-last_time>=600){
    digitalWrite(13, 1);
    left_fblstou(90);
    last_time=current_time;
    n=23;
    digitalWrite(13, 0);
  }
  else if(n==23 && somme<=5000 && current_time-last_time>=1000){
    digitalWrite(13, 1);
    stop(200);
    last_time=current_time;
    n=24;
    digitalWrite(13, 0);
  }
  else if(n==24 && somme >=8000 && current_time-last_time>=1000){
    digitalWrite(13, 0);
    stop(500000);
  }



//*************************************************************************
    if (n==1||n==2||n==3){
     left_base=200; right_base=200;
     kp = 0.58; kd = 2.35;
     PID();
     forwardPID();}
       else  if (n==4||n==900||n==19){
     left_base=180; right_base=180;
     kp = 0.31; kd = 2.1;
     PID();
     forwardPID();}

      else if (n==20||n==21||n==16||n==17||n==18||n==24){
     left_base=150; right_base=150;
     kp = 0.22; kd = 2;
          PID();
     forwardPID();}

      else if (n==22||n==23){
     left_base=180; right_base=180;
     kp = 0.22; kd = 1.7;
          PID_noir();
     forwardPID();}



}





/////////////////////////////////////////////////////////////////////

void PID_force() {
  position = qtr.readLine(s);
  qtr.readLine(s);
  for(int i=0;i<14;i++){
    if(s[i]>500)s[i]=1000;
    else s[i]=0;
  }
 int c1=s[0];
  int c2=s[1];
  int c3=s[2];
  int c4=s[3];
  int c5=s[4];
  int c6=s[5];
  int c7=s[6];
  int c8=s[7];
  int c9=s[8];
  int c10=s[9];
  int c11=s[10];
  int c12=s[11];
  int c13=s[12];
  int c14=s[13];
//int somme=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12+c13+c14;
if(c1+c2>=1000&&c14+c13+c12+c11+c10<=1000)
  {left_fblstou(3);}
   else if(c14+c13>=1000&&c1+c2+c3+c4+c5<=1000)
  {right_fblstou(3);}
//else if(c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12+c13+c14<=0){back(10);}
   
else{
  error = position - 6500;
  P = error;
  D = error - lasterror;
  I = error + I;
  PIDvalue = (kp * P) + (kd * D) + (ki * I);
  lasterror = error;
  right_speed = right_base +PIDvalue;
  left_speed = left_base - PIDvalue;
  left_speed = min(255, max(left_speed, 0));
  right_speed = min(255, max(right_speed, 0));}
  //delay(200);
  //Serial.println(error);
}







void PID() {
  position = qtr.readLine(s);
  error = position - 6500;
  P = error;
  D = error - lasterror;
  I = error + I;
  PIDvalue = (kp * P) + (kd * D) + (ki * I);
  lasterror = error;
  right_speed = right_base +PIDvalue;
  left_speed = left_base - PIDvalue;
  left_speed = min(200, max(left_speed, 0));
  right_speed = min(200, max(right_speed, 0));
  //delay(200);
  //Serial.println(error);
}

void PID_noir() {
  position = qtr.readLine(s, QTR_EMITTERS_ON, 1, true);
  error = position - 6500;
  P = error;
  D = error - lasterror;
  I = error + I;
  PIDvalue = (kp * P) + (kd * D) + (ki * I);
  lasterror = error;
  right_speed = right_base + PIDvalue;
  left_speed = left_base - PIDvalue;
  left_speed = min(255, max(left_speed, 0));
  right_speed = min(255, max(right_speed, 0));
  //delay(200);
  //Serial.println(error);
}
void PID_special() {
  bool v = false;
  position = qtr.readLine(s, QTR_EMITTERS_ON, 0, v);
  error = position - 6500;
  P = error;
  D = error - lasterror;
  I = error + I;
  PIDvalue = (kp * P) + (kd * D) + (ki * I);
  lasterror = error;
  right_speed = right_base + PIDvalue;
  left_speed = left_base - PIDvalue;
  left_speed = min(255, max(left_speed, 0));
  right_speed = min(255, max(right_speed, 0));
  //delay(200);
  //Serial.println(error);
}
void PID_special_force() {
  bool v = false;
  position = qtr.readLine(s, QTR_EMITTERS_ON, 0, v);
  qtr.readLine(s);
  for(int i=0;i<14;i++){
    if(s[i]>500)s[i]=1000;
    else s[i]=0;
  }
 int c1=s[0];
  int c2=s[1];
  int c3=s[2];
  int c4=s[3];
  int c5=s[4];
  int c6=s[5];
  int c7=s[6];
  int c8=s[7];
  int c9=s[8];
  int c10=s[9];
  int c11=s[10];
  int c12=s[11];
  int c13=s[12];
  int c14=s[13];
//int somme=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12+c13+c14;
if(c1+c2>=1000)
  {left_fblstou(1);}
   else if(c14+c13>=1000)
  {right_fblstou(1);}
  error = position - 6500;
  P = error;
  D = error - lasterror;
  I = error + I;
  PIDvalue = (kp * P) + (kd * D) + (ki * I);
  lasterror = error;
  right_speed = right_base + PIDvalue;
  left_speed = left_base - PIDvalue;
  left_speed = min(255, max(left_speed, 0));
  right_speed = min(255, max(right_speed, 0));
  //delay(200);
  //Serial.println(error);
}
void PID_touhami() {
  bool v = true;
  position = qtr.readLineM(s, QTR_EMITTERS_ON, 0, v);
  error = position - 2500;
  P = error;
  D = error - lasterror;
  I = error + I;
  PIDvalue = (kp * P) + (kd * D) + (ki * I);
  lasterror = error;
  right_speed = right_base - PIDvalue;
  left_speed = left_base + PIDvalue;
  left_speed = min(250, max(left_speed, 0));
  right_speed = min(250, max(right_speed, 0));
  //delay(200);
  //Serial.println(error);
}
void PID_touhamiN() {
  bool v = true;
  position = qtr.readLineM(s, QTR_EMITTERS_ON, 1, v);
  error = position - 2500;
  P = error;
  D = error - lasterror;
  I = error + I;
  PIDvalue = (kp * P) + (kd * D) + (ki * I);
  lasterror = error;
  right_speed = right_base - PIDvalue;
  left_speed = left_base + PIDvalue;
  left_speed = min(180, max(left_speed, 0));
  right_speed = min(180, max(right_speed, 0));
  //delay(200);
  //Serial.println(error);
}
void PID_touhamiS() {
  bool v = false;
  position = qtr.readLineM(s, QTR_EMITTERS_ON, 0, v);
  error = position - 2500;
  P = error;
  D = error - lasterror;
  I = error + I;
  PIDvalue = (kp * P) + (kd * D) + (ki * I);
  lasterror = error;
  right_speed = right_base - PIDvalue;
  left_speed = left_base + PIDvalue;
  left_speed = min(250, max(left_speed, 0));
  right_speed = min(250, max(right_speed, 0));
  //delay(200);
  //Serial.println(error);
}
void forwardPID() {

  analogWrite(rightMotorPin1, left_speed);
  analogWrite(leftMotorPin1, right_speed);
  analogWrite(rightMotorPin2, 0);
  analogWrite(leftMotorPin2, 0);
}
void back(int x) {

  analogWrite(rightMotorPin1, 0);
  analogWrite(leftMotorPin1, 0);
  analogWrite(rightMotorPin2, 200);
  analogWrite(leftMotorPin2, 200);
  delay(x);
}
void forward(int x) {

  analogWrite(rightMotorPin1, 120);
  analogWrite(leftMotorPin1, 120);
  analogWrite(rightMotorPin2, 0);
  analogWrite(leftMotorPin2, 0);
  delay(x);
}
void right(int x) {

  analogWrite(rightMotorPin1, 180);
  analogWrite(leftMotorPin1, 0);
  analogWrite(rightMotorPin2, 0);
  analogWrite(leftMotorPin2, 0);
  delay(x);
}
void rightsafe(int x) {

  analogWrite(rightMotorPin1, 100);
  analogWrite(leftMotorPin1, 0);
  analogWrite(rightMotorPin2, 0);
  analogWrite(leftMotorPin2, 0);
  delay(x);
}
void right_fblstou(int x) {

  analogWrite(rightMotorPin1, 180);
  analogWrite(leftMotorPin1, 0);
  analogWrite(rightMotorPin2, 0);
  analogWrite(leftMotorPin2, 180);
  delay(x);
}
void right_fblstousafe(int x) {

  analogWrite(rightMotorPin1, 70);
  analogWrite(leftMotorPin1, 0);
  analogWrite(rightMotorPin2, 0);
  analogWrite(leftMotorPin2, 70);
  delay(x);
}
void left_fblstou(int x) {

  analogWrite(rightMotorPin1, 0);
  analogWrite(leftMotorPin1, 180);
  analogWrite(rightMotorPin2, 180);
  analogWrite(leftMotorPin2, 0);
  delay(x);
}
void left(int x) {

  analogWrite(rightMotorPin1, 0);
  analogWrite(leftMotorPin1, 180);
  analogWrite(rightMotorPin2, 0);
  analogWrite(leftMotorPin2, 0);
  delay(x);
}
void leftsafe(int x) {

  analogWrite(rightMotorPin1, 0);
  analogWrite(leftMotorPin1, 100);
  analogWrite(rightMotorPin2, 0);
  analogWrite(leftMotorPin2, 0);
  delay(x);
}
void afsaleft(int x) {

  analogWrite(rightMotorPin1, 0);
  analogWrite(leftMotorPin1, 80);
  analogWrite(rightMotorPin2, 0);
  analogWrite(leftMotorPin2, 0);
  delay(x);
}
void afsalefts(int x) {

  analogWrite(rightMotorPin1, 0);
  analogWrite(leftMotorPin1, 100);
  analogWrite(rightMotorPin2, 0);
  analogWrite(leftMotorPin2, 0);
  delay(x);
}
void stop(int x) {

  analogWrite(rightMotorPin1, 0);
  analogWrite(leftMotorPin1, 0);
  analogWrite(rightMotorPin2, 0);
  analogWrite(leftMotorPin2, 0);
  delay(x);
}
void dour(int degreesToTurn) {
  // Calculate encoder ticks needed for the turn
  //Serial.print("target position ");
  int targetposition = degreesToTurn * (3600 / 455);
  //Serial.println(targetposition);
  
  npL=0;
  npR=0;
  myEncl.readAndReset();
  myEncr.readAndReset();
  if(degreesToTurn>0){
  while(npR<targetposition){
    npL=myEncl.read();
    npR=myEncr.read();
    /*Serial.print(npL);
  Serial.print(' ');
  Serial.println(npR);*/
    analogWrite(rightMotorPin1, 180);
    analogWrite(leftMotorPin1, 0);
    analogWrite(rightMotorPin2, 0);
    analogWrite(leftMotorPin2, 180);
    delay(1);
  }
  while(npR>=targetposition){
    npL=myEncl.read();
    npR=myEncr.read();
    /*Serial.print(npL);
  Serial.print(' ');
  Serial.println(npR);*/
    analogWrite(rightMotorPin1, 0);
    analogWrite(leftMotorPin1, 180);
    analogWrite(rightMotorPin2, 180);
    analogWrite(leftMotorPin2, 0);
    delay(1);
  }
  }
  else {
    while(npR<targetposition){
    npL=myEncl.read();
    npR=myEncr.read();
    /*Serial.print(npL);
  Serial.print(' ');
  Serial.println(npR);*/
    analogWrite(rightMotorPin1, 180);
    analogWrite(leftMotorPin1, 0);
    analogWrite(rightMotorPin2, 0);
    analogWrite(leftMotorPin2, 180);
    delay(1);
   }
  while(npR>=targetposition){
    npL=myEncl.read();
    npR=myEncr.read();
    /*Serial.print(npL);
  Serial.print(' ');
  Serial.println(npR);*/
    analogWrite(rightMotorPin1, 0);
    analogWrite(leftMotorPin1, 180);
    analogWrite(rightMotorPin2, 180);
    analogWrite(leftMotorPin2, 0);
    delay(1);
   }
  }
  myEncl.readAndReset();
  myEncr.readAndReset();
  // Control the motors to make the turn
  // Implement your motor control logic here
}

// Function to move the robot forward by a specified distance in centimeters
void forwardCM(int distanceCm) {
  int target= distanceCm* 1080 / 25;
  npL=0;
  npR=0;
  myEncl.readAndReset();
  myEncr.readAndReset();
  while(npL<target||npR<target){
    npL=myEncl.read();
    npR=myEncr.read();
    analogWrite(rightMotorPin1, 183);
    analogWrite(leftMotorPin1, 180);
    analogWrite(rightMotorPin2, 0);
    analogWrite(leftMotorPin2, 0);
    delay(1);
  }
  npL=0;
  npR=0;
  myEncl.readAndReset();
  myEncr.readAndReset();
}