#include <QTRSensors.h>

const uint8_t SensorCount = 14;

uint16_t s[SensorCount];
QTRSensorsAnalog qtr((unsigned char[]){ A8,A10,A11,A12,A13,A14,A15,A0,A1,A2,A3,A4,A5,A6 }, SensorCount);
int position;
int n = 0, p = 0, m = 0;
//PID
int right_speed, left_speed;
float kp = 0.06, kd = 0.11, ki = 0, P, D, I;
float PIDvalue, lasterror, error;
int left_base, right_base;
//motors
int leftF = 9;
int leftR = 8;
int rightF = 10;
int rightR = 11;
//time
unsigned int current_time, last_time, lunch_time;

//kp=0.07;kd=0.12; high speed
//kp=0.06;kd=0.11; medium speed
bool test=false;

int led = 48;

int comp=20;
int pos;
//leds
int ledi[4]={4,5,6,7};


int time=0;
//caps
 int capt1;
  int capt2;
  int capt3;
  int capt4;
  int capt5;
  int capt6;
  int capt7;
  int capt8;
  int capt9;
  int capt10;
  int capt11;
  int capt12;
  int capt13;
  int capt14;
  int capt15;
  int capt16;


void setup() {

  Serial.begin(9600);
  int tst = 0;
  //moteurs
  pinMode(rightF, OUTPUT);
  pinMode(leftF, OUTPUT);
  pinMode(rightR, OUTPUT);
  pinMode(leftR, OUTPUT);
  pinMode(ledi[0], OUTPUT);
  pinMode(ledi[1], OUTPUT);
  pinMode(ledi[2], OUTPUT);
  pinMode(ledi[3], OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(47, INPUT);
  pinMode(led, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  //ledon(4);
  //forwardm(11111);
  //left(10000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 120; i++) {
    qtr.calibrate();
    tst = 1;
    delay(10);
  }
  digitalWrite(LED_BUILTIN, LOW);
  current_time = millis();
  int x = 0;
  while (1) {
    x = digitalRead(47);
    Serial.println(x);
    if (x == HIGH) {
      lunch_time = millis();
      if (lunch_time - current_time > 200)
        break;
    }
  }
  delay(400);
  forward(150);
  //Serial.begin(9600);



}
int rtime;

void loop() {
  //position = qtr.readLineM(s,QTR_EMITTERS_ON,0,false);
 int current_time = millis();
  qtr.readLine(s);
  int capt1=s[1];
  int capt2=s[0];
  int capt3=s[2];
  int capt4=s[3];
  int capt5=s[4];
  int capt6=s[5];
  int capt7=s[6];
  int capt8=s[7];
  int capt9=s[8];
  int capt10=s[9];
  int capt11=s[10];
  int capt12=s[11];
  int capt13=s[12];
  int capt14=s[13];
  int capt15=s[14];
  int capt16=s[15];

  int somme=capt1+capt2+capt3+capt4+capt5+capt6+capt7+capt8+capt9+capt10+capt11+capt12+capt13+capt14+capt15+capt16;

  if (n==0 &&capt1+capt2+capt3+capt4+capt5>=3000)// dora 9bal l7nach
  {
    stp(50);
    left(290);
    n=1;
    last_time=current_time;

  
  
  }
else if(n==1 && capt11+capt12+capt13+capt14+capt15+capt16>=3000 &&current_time-last_time>1600)// dora ba3d la7nach
{
      stp(50);
      right(270);
  
    n=2;
    last_time=millis();
}
else if(n==2 && somme>=10000 &&current_time-last_time>800)// dora ba3d drouj
{
      stp(50);
    right(600);
   // stp(50000); 
    last_time=millis();
    n=3;
}
else if( n==3 && capt9+capt10+capt11+capt12+capt13+capt14+capt15>=5000 &&  current_time-last_time>2450)//forward 9assa 3arbi
{
    right(200);
    forwardavantmaze(350);//280
    last_time=millis();
    n=4;
}
else if(n==4&& somme <=1000 ){//barriere y9f 5s
   right(60);
    stp(4800);
    last_time=millis();
    n=5;
}

  else if(somme<=1000&&n==5&&test)//ya9ra l5tout
  {
    test=false;
  }
  else if(!test&&somme>=10000&&p==0&&n==5){
    p=1;
    test=true;
  }
  else if(!test&&somme>=10000&&p==1&&n==5){
    p=2;
    test=true;
     
  }
   else if(!test&&somme>=10000&&p==2&&n==5){
    p=3;
    test=true;
     
  }
   else if(!test&&somme>=10000&&p==3&&n==5){
    p=4;
    test=true;
     
  }
  

else if(n==5&&somme>=1500&&somme<=6000&&current_time-last_time>1600)//ycha3al e leds
{ 
  ledon(p);

  last_time=millis();
  n=6;

}





else if(n==6 && capt11+capt12+capt13+capt14+capt15>=4000&&current_time-last_time>1000)//dra tafi leds w dora al faza
{
  
  stp(50);
  forward(150);
  right_fblstou(500);
  ledoff(p);
 stp(4800);
  forward(300);
  right_fblstou(400);

 last_time=millis();
   n=7;

}

else if(n==7 && capt2+capt3+capt4+capt5+capt6>=4000 &&current_time-last_time>200)//dora before zigzag
{
  stp(50);
  forward(120);
  left_fblstou(400);
  //stp(200000);
  last_time=millis();
   n=8;

}
else if(n==8 && capt9+capt10+capt11+capt12+capt13+capt14+capt15>=5000&&current_time-last_time>300 )//first dora zigzag
{

  forward(80);
  right(500);
  //forward(120);
  left(1200);

  last_time=millis();
   n=9;

}

else if(n==9 && somme<=1000&&current_time-last_time>100)// west e zigzag
{
  stp(50);
  right_fblstou(550);

  
  last_time=millis();
   n=10;

}
else if(n==10 && somme<=1000&&current_time-last_time>300)//last dora zigzag
{
  stp(50);
  left_fblstou(450);
  last_time=millis();
   n=11;

}
else if(n==11&& somme<=1000&&current_time-last_time>300)//e5er wa9fa
{
  forward(100);
  stp(99999999);
  last_time=millis();
   n=12;

}








//patie PID


  else if(n==0){
  kp = 0.05;
  kd = 0.09;
  right_base =230;
  left_base = 230;
    PID();
    forwardPID();

  }
  
  else if(n==1){
  kp = 0.05;
  kd = 0.09;
  right_base =200;
  left_base = 200;
    PID();
    forwardPID();

  }
   else if(n==2){
  kp = 0.05;
  kd = 0.09;
  right_base =250;
  left_base = 250;
    PID();
    forwardPID();

  }
  else if(n==3){
   kp = 0.05;
  kd = 0.09;
  right_base =150;
  left_base = 150;
    PID();
    forwardPID();

  }
  else if(n==4){
 kp = 0.04;
  kd = 0.07;
  right_base =80;
  left_base = 80;
    PID();
    forwardPID();

  }
  else if(n==5){
  forwardm();
}
else if(n==6){
   kp = 0.05;
  kd = 0.09;
  right_base =250;
  left_base = 250;
    PID();
    forwardPID();


}
else if(n==7){
 kp = 0.04;
  kd = 0.08;
  right_base =200;
  left_base = 200;
    PID();
    forwardPID();


}

else if(n==8){
  kp = 0.03;
  kd = 0.06;
  right_base =130;
  left_base = 130;
    PID();
    forwardPID();

  }
  else if(n==9){
  kp = 0.03;
  kd = 0.06;
  right_base =130;
  left_base = 130;
    PID();
    forwardPID();

  }
    else if(n==10){
  kp = 0.03;
  kd = 0.06;
  right_base =130;
  left_base = 130;
    PID();
    forwardPID();

  }
      else if(n==11){
   kp = 0.05;
  kd = 0.09;
  right_base =230;
  left_base = 230;
    PID();
    forwardPID();

  }
  /*
    else if (comp==2 && somme>14000 && time-rtime>2000)
  
  {
    stp(100);
    right(700);
    comp++;
   rtime=millis();
  }
  else if(comp==3 && capt15+capt16>1500 &&time-rtime>500 )
  {
    stp(100);
    forward1(300);
    comp=4;
    rtime=millis();
    
  }
  
  else if(comp==4 && capt15+capt16+capt14>2700 && time-rtime>2000)
  {
    stp(100);
    left(250);
    stp(200);
    comp=5;
   rtime=millis();
  }
  

  else if(comp==20)
  {
      kp = 0.05;
  kd = 0.09;
  right_base =200;
  left_base = 200;
    PID_special();
    forwardPID();
  }
  */



 
}


void PID() {
  position = qtr.readLine(s);
  error = position - 6500;
  P = error;
  D = error - lasterror;
  I = error + I;
  PIDvalue = (kp * P) + (kd * D) + (ki * I);
  lasterror = error;
  right_speed = right_base + PIDvalue;
  left_speed = left_base - PIDvalue;
  left_speed = min(250, max(left_speed, 0));
  right_speed = min(250, max(right_speed, 0));
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
  left_speed = min(200, max(left_speed, 0));
  right_speed = min(200, max(right_speed, 0));
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
  left_speed = min(250, max(left_speed, 0));
  right_speed = min(250, max(right_speed, 0));
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
  right_speed = right_base + PIDvalue;
  left_speed = left_base - PIDvalue;
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
  right_speed = right_base + PIDvalue;
  left_speed = left_base - PIDvalue;
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
  right_speed = right_base + PIDvalue;
  left_speed = left_base - PIDvalue;
  left_speed = min(250, max(left_speed, 0));
  right_speed = min(250, max(right_speed, 0));
  //delay(200);
  //Serial.println(error);
}
void ledon(int p){
  switch (p){
    case 1:
    digitalWrite(ledi[0],1);
    break;
    case 2:
     digitalWrite(ledi[0],1);
      digitalWrite(ledi[1],1);
    break;
    case 3:
     digitalWrite(ledi[0],1);
      digitalWrite(ledi[1],1);
       digitalWrite(ledi[2],1);
       break;
    case 4:
       digitalWrite(ledi[0],1);
      digitalWrite(ledi[1],1);
       digitalWrite(ledi[2],1);
        digitalWrite(ledi[3],1);
    break;
  }
}
void ledoff(int p){
  switch (p){
    case 1:
    digitalWrite(ledi[0],0);
    break;
    case 2:
     digitalWrite(ledi[0],0);
      digitalWrite(ledi[1],0);
    break;
    case 3:
     digitalWrite(ledi[0],0);
      digitalWrite(ledi[1],0);
       digitalWrite(ledi[2],0);
       break;
    case 4:
       digitalWrite(ledi[0],0);
      digitalWrite(ledi[1],0);
       digitalWrite(ledi[2],0);
        digitalWrite(ledi[3],0);
    break;
  }
}
void forwardPID() {

  analogWrite(rightF, right_speed);
  analogWrite(leftF, left_speed);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
}
void forward(int x) {

  analogWrite(rightF, 180);
  analogWrite(leftF, 180);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void forwardm() {

  analogWrite(rightF, 183);
  analogWrite(leftF, 180);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  
}
void forward1(int x) {

  analogWrite(rightF, 113);
  analogWrite(leftF, 110);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void forwardf(int x) {

  analogWrite(rightF, 100);
  analogWrite(leftF, 100);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void right(int x) {

  analogWrite(rightF, 120);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void rightsafe(int x) {

  analogWrite(rightF, 100);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void right_fblstou(int x) {

  analogWrite(rightF, 120);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 120);
  delay(x);
}
void right_fblstousafe(int x) {

  analogWrite(rightF, 70);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 70);
  delay(x);
}
void left_fblstou(int x) {

  analogWrite(rightF, 0);
  analogWrite(leftF, 120);
  analogWrite(rightR, 120);
  analogWrite(leftR, 0);
  delay(x);
}
void left(int x) {

  analogWrite(rightF, 0);
  analogWrite(leftF, 120);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void leftsafe(int x) {

  analogWrite(rightF, 0);
  analogWrite(leftF, 100);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void afsaleft(int x) {

  analogWrite(rightF, 0);
  analogWrite(leftF, 80);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void afsalefts(int x) {

  analogWrite(rightF, 0);
  analogWrite(leftF, 100);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void stp(long int x) {

  analogWrite(rightF, 0);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  back(30);
  analogWrite(rightF, 0);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void back(int x) {
  analogWrite(rightF, 0);
  analogWrite(leftF, 0);
  analogWrite(rightR, 130);
  analogWrite(leftR, 130);
  delay(x);
}
void stp1(int x) {

  analogWrite(rightF, 0);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void forwardavantmaze(int x){
  analogWrite(rightF, 183);
  analogWrite(leftF,180);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);  
}
