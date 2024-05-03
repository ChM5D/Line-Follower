#include <QTRSensors.h>

const uint8_t SensorCount = 14;


uint16_t s[SensorCount];
QTRSensorsRC qtr((unsigned char[]){A8,A10,A11,A12,A13,A14,A15,A0,A1,A2,A3,A4,A5,A6}, SensorCount);
int position;
int n = 0, p = 0, m = 0;
//PID
int right_speed, left_speed;
float kp = 0.1, kd = 0.36, ki = 0, P, D, I; //vitesse 150 kp0.06 kd0.2   vitesse 255 kp0.11 kd0.38
// VITESSE 100 
//   kp = 0.05;
//   kd = 0.21;
// vitesse 180 
//   kp = 0.1;
//   kd = 0.36;
// vitesse 220 
//   kp = 0.12;
//   kd = 0.6;
// vitesse 255 
//   kp = 0.16;
//   kd = 1;
float PIDvalue, lasterror, error;
int left_base, right_base;
//motors
int leftF = 6;
int leftR = 7;
int rightF = 5;
int rightR = 4;
//time
unsigned int current_time, last_time, lunch_time;

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

void setup() {

  //Serial.begin(9600);
  int tst = 0;
  //moteurs
  pinMode(rightF, OUTPUT);
  pinMode(leftF, OUTPUT);
  pinMode(rightR, OUTPUT);
  pinMode(leftR, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(47, INPUT);
  pinMode(led, OUTPUT);

  pinMode(22, OUTPUT);
  digitalWrite(22, LOW); 

  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 80; i++) {
    qtr.calibrate();
    tst = 1;
    delay(10);
  }
  digitalWrite(LED_BUILTIN, LOW);

  int x=0;
  while(1){
  x=digitalRead(22);
   Serial.println(x);
   if(x==HIGH){
        lunch_time=millis();
      if(lunch_time-current_time>200)
        break;}
  }






  stp(200);
  forwardbg(200);
  current_time = millis();



}
int rtime;

void loop() {
    qtr.readLine(s);
  //position = qtr.readLineM(s,QTR_EMITTERS_ON,0,false);
 int current_time = millis();

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
  
  //conditions

if (n==0&& c10+c11+c12+c13>3000)
{
  digitalWrite(LED_BUILTIN, HIGH);

  
  right_fblstou(350);
  
  
  digitalWrite(LED_BUILTIN, LOW);
  n=1;
  last_time=current_time;
}
else if (n==1&&  c2+c3+c4>2000 &&current_time-last_time>300)
{
  digitalWrite(LED_BUILTIN, HIGH);
  
  
  left_fblstou(320);
  
  
  digitalWrite(LED_BUILTIN, LOW);
  n=2;
  last_time=millis();
}

else if (n==2&& c9+c10+c11+c12+c13>4000 && current_time-last_time>300)
{
  digitalWrite(LED_BUILTIN, HIGH);

  
  right_fblstou(160);
  

  digitalWrite(LED_BUILTIN, LOW);
  n=3;
  last_time=millis();
}
else if (n==3 && rr>5000 && current_time-last_time>300){

  digitalWrite(LED_BUILTIN, HIGH);

  
  left_fblstou(390);


  digitalWrite(LED_BUILTIN, LOW);
  n=4;
  last_time=millis();
}
else if (n==4 && somme>8000 && current_time-last_time>200){

  digitalWrite(LED_BUILTIN, HIGH);

  
  left_fblstou(150);
  

  digitalWrite(LED_BUILTIN, LOW);
  n=5;
  last_time=millis();
}

else if (n==5 && somme>8000 && current_time-last_time>300){

  digitalWrite(LED_BUILTIN, HIGH);

  forwardo(80);
  left_fblstou(365);

  digitalWrite(LED_BUILTIN, LOW);
  n=6;
  last_time=millis();
}

else if (n==6 && ll>5000 && current_time-last_time>300){

  digitalWrite(LED_BUILTIN, HIGH);

  right_fblstou(370);

  digitalWrite(LED_BUILTIN, LOW);
  n=7;
  last_time=millis();
}


else if (n==7 && rr>5000 && current_time-last_time>300){

  digitalWrite(LED_BUILTIN, HIGH);


  left_fblstou(320);


  digitalWrite(LED_BUILTIN, LOW);
  n=8;
  last_time=millis();
}
else if (n==8 && somme>10000 && current_time-last_time>1000){

  digitalWrite(LED_BUILTIN, HIGH);

  right_fblstou(150);
  forward(320);
  stp(5000);
  
  

  digitalWrite(LED_BUILTIN, LOW);
  n=9;
  last_time=millis();
}

else if (n==9 && rr>5000 && current_time-last_time>200){

  digitalWrite(LED_BUILTIN, HIGH);

  left_fblstou(340);
 
  digitalWrite(LED_BUILTIN, LOW);
  n=10;
  last_time=millis();
}
else if (n==10 && current_time-last_time>550){

  digitalWrite(LED_BUILTIN, HIGH);
  
  n=11;
  last_time=millis();
}

else if (n==11 && rr>5000 && current_time-last_time>100){

  digitalWrite(LED_BUILTIN, LOW);

  left_fblstou(450);

  n=12;
  last_time=millis();
}

else if (n==12 && ll<1000 && current_time-last_time>150){

  digitalWrite(LED_BUILTIN, HIGH);

  right_fblstou(150);

  digitalWrite(LED_BUILTIN, LOW);
  
  n=13;
  last_time=millis();
}




else if (n==13 && rr>4500 && current_time-last_time>500){

  digitalWrite(LED_BUILTIN, HIGH);

  forwards(150);
  left_fblstou(400);
  

  digitalWrite(LED_BUILTIN, LOW);
  
  n=14;
  last_time=millis();
}
else if (n==14 && ll>5000 && current_time-last_time>200){

  digitalWrite(LED_BUILTIN, HIGH);
  forwards(150);
  right_fblstou(270);
  stp(500000);
  

  digitalWrite(LED_BUILTIN, LOW);
  
  n=15;
  last_time=millis();
}





  //PID*****************************************************************************************************
   // VITESSE 100 
//   kp = 0.05;
//   kd = 0.21;
// vitesse 180 
//   kp = 0.1;
//   kd = 0.36;
// vitesse 220 
//   kp = 0.12;
//   kd = 0.6;
// vitesse 255 
//   kp = 0.16;
//   kd = 1;





 if(n==0){
  kp = 0.1;
  kd = 0.36;
  right_base =200;
  left_base = 200;
    PID_special();
    forwardPID();
    }




 
 else if(n==1){
   kp = 0.05;
  kd = 0.21;
  right_base =100;
  left_base = 100;
    PID();
    forwardPID();
    }
     else if(n==2){

  kp = 0.05;
  kd = 0.21;
  right_base =100;
  left_base = 100;
    PID();
    forwardPID();
    }
    else if(n==3) {
      
  kp = 0.1;
  kd = 0.36;
  right_base =180;
  left_base = 180;
    PID();
    forwardPID();
    }
  
       else if(n==4){

  kp = 0.05;
  kd = 0.21;
  right_base =100;
  left_base = 100;
    PID();
    forwardPID();
    }
    
           else if(n==5){

  kp = 0.1;
  kd = 0.36;
  right_base =150;
  left_base = 150;
    PID();
    forwardPID();
    }

               else if(n==6){

  kp = 0.1;
  kd = 0.36;
  right_base =150;
  left_base = 150;
    PID();
    forwardPID();
    }
    
                   else if(n==7){

  kp = 0.1;
  kd = 0.36;
  right_base =180;
  left_base = 180;
    PID();
    forwardPID();
    }

               else if(n==8){

  kp = 0.1;
  kd = 0.36;
  right_base =150;
  left_base = 150;
    PID();
    forwardPID();
    }

               else if(n==9){

  kp = 0.05;
  kd = 0.21;
  right_base =100;
  left_base = 100;
    PID();
    forwardPID();
    }

              else if(n==10){

  kp = 0.16;
  kd = 1;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }
                  else if(n==11){

  kp = 0.1;
  kd = 0.36;
  right_base =180;
  left_base = 180;
    PID();
    forwardPID();
    }

                  else if(n==12){

  kp = 0.03;
  kd = 0.18;
  right_base =80;
  left_base = 80;
    PID_special();
    forwardPID();
    }

             else if(n==13){

  kp = 0.03;
  kd = 0.18;
  right_base =80;
  left_base = 80;
    PID_special();
    forwardPID();
    }



             else if(n==14){

  kp = 0.03;
  kd = 0.18;
  right_base =80;
  left_base = 80;
    PID_special();
    forwardPID();
    }

   



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
  left_speed = min(255, max(left_speed, 0));
  right_speed = min(255, max(right_speed, 0));
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
void forwardPID() {

  analogWrite(rightF, right_speed);
  analogWrite(leftF, left_speed);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
}
void forwardbg(int x) {

  analogWrite(rightF, 100);
  analogWrite(leftF, 100);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void forwardo(int x) {

  analogWrite(rightF, 233);
  analogWrite(leftF, 255);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void forward(int x) {

  analogWrite(rightF, 100);
  analogWrite(leftF, 100);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void forwards(int x) {

  analogWrite(rightF, 70);
  analogWrite(leftF, 70);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void right(int x) {

  analogWrite(rightF, 150);
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

  analogWrite(rightF, 70);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 70);
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
  analogWrite(leftF, 70);
  analogWrite(rightR, 70);
  analogWrite(leftR, 0);
  delay(x);
}

void left_fblstouf(int x) {

  analogWrite(rightF, 0);
  analogWrite(leftF, 70);
  analogWrite(rightR, 100);
  analogWrite(leftR, 0);
  delay(x);
}
void left(int x) {

  analogWrite(rightF, 0);
  analogWrite(leftF, 60);
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
void stp(int x) {

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
void forwardmax(int x){
  analogWrite(rightF, 255);
  analogWrite(leftF,255);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);  
}