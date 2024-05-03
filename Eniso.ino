#include <QTRSensors.h>

const uint8_t SensorCount = 14;
int leee=52;

uint16_t s[SensorCount];
QTRSensorsRC qtr((unsigned char[]){A8,A10,A11,A12,A13,A14,A15,A0,A1,A2,A3,A4,A5,A6}, SensorCount);
int position;
int n = 0, p = 0, m = 0;
//PID
int right_speed, left_speed;
float kp = 0.098, kd = 0.07, ki = 0, P, D, I; //vitesse 150 kp0.06 kd0.2   vitesse 255 kp0.11 kd0.38
// VITESSE 100 
//   kp = 0.048;
//   kd = 0.04;
// vitesse 180 
//   kp = 0.098;
//   kd = 0.7;
// vitesse 255 
//   kp = 0.2;
//   kd = 1.3;

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
  pinMode(leee, OUTPUT);
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
  stp(2500);
  current_time = millis();
  // forward(280);
  

    

  //Serial.begin(9600);



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
/*
if (n==0&& somme >10000)
{
  digitalWrite(leee, HIGH);
  stp(50);
  right_fblstou(200);
  digitalWrite(leee, LOW);
  n=1;
  last_time=current_time;
}

else if(n==1 && somme>6500  && current_time-last_time>500 )
{
  digitalWrite(leee, HIGH);
  forward(200);
  digitalWrite(leee, LOW);
  n=2;
  last_time=millis();
}


else if(n==2 && ll>5000  && current_time-last_time>400 )
{
  digitalWrite(leee, HIGH);
  right_fblstou(470);
  digitalWrite(leee, LOW);
  n=3;
  last_time=millis();
}


else if(n==3 && rr>5000  && current_time-last_time>300 )
{
  digitalWrite(leee, HIGH);
  left_fblstou(500);
  digitalWrite(leee, LOW);
  n=4;
  last_time=millis();
}


else if(n==4 && rr>5000  && current_time-last_time>300 )
{
  digitalWrite(leee, HIGH);
  left_fblstou(500);
  digitalWrite(leee, LOW);
  n=5;
  last_time=millis();
}
else if(n==5 && ll>5000  && current_time-last_time>300 )
{
  digitalWrite(leee, HIGH);
  right_fblstou(470);
  digitalWrite(leee, LOW);
  n=6;
  last_time=millis();
}


else if(n==6 && rr>5000  && current_time-last_time>200 )
{
  digitalWrite(leee, HIGH);
  left_fblstou(430);
  digitalWrite(leee, LOW);
  n=7;
  last_time=millis();
}

else if(n==7 && ll>5000  && current_time-last_time>150 )
{
  digitalWrite(leee, HIGH);
  right_fblstou(430);
  digitalWrite(leee, LOW);
  n=8;
  last_time=millis();
}

else if(n==8 && rr>5000  && current_time-last_time>150 )
{
  digitalWrite(leee, HIGH);
  left_fblstou(520);
  digitalWrite(leee, LOW);
  n=9;
  last_time=millis();
}


else if(n==9 && ll>5500  && current_time-last_time>1600 )
{
  digitalWrite(leee, HIGH);
  right_fblstou(340);
  digitalWrite(leee, LOW);
  n=10;
  last_time=millis();
  
}


else if(n==10 && rr>5000  && current_time-last_time>200 )
{
  digitalWrite(leee, HIGH);
  left_fblstou(580);
  digitalWrite(leee, LOW);
  n=11;
  last_time=millis();
  
}

else if(n==11 && somme>7000  && current_time-last_time>100 )
{
  digitalWrite(leee, HIGH);
  forward(250);
  digitalWrite(leee, LOW);
  n=12;
  last_time=millis();
  
}


else if(n==12 && ll>6500  && current_time-last_time>550 )
{
  digitalWrite(leee, HIGH);
  right_fblstou(430);
  digitalWrite(leee, LOW);
  n=13;
  last_time=millis();
  
}



else if(n==13 && somme>8000  && current_time-last_time>300 )
{
  digitalWrite(leee, HIGH);
  forward(200);
  right_fblstou(330);
  digitalWrite(leee, LOW);
  n=14;
  last_time=millis();
  
}


else if(n==14 && somme>7000  && current_time-last_time>300 )
{
  digitalWrite(leee, HIGH);
  stp(4500);
  digitalWrite(leee, LOW);
  n=15;
  last_time=millis();
  
}

else if(n==15 && c1+c2+c3+c4+c14+c13+c12+c11<1000  && current_time-last_time>1500 )
{
  digitalWrite(leee, HIGH);
  digitalWrite(leee, LOW);
  n=16;
  last_time=millis();
  
}

else if(n==16 && rr>5000  && current_time-last_time>400 )
{
  digitalWrite(leee, HIGH);
  left_fblstou(430);
  digitalWrite(leee, LOW);
  n=17;
  last_time=millis();
  
}


else if(n==17 && ll>5000  && current_time-last_time>150 )
{
  digitalWrite(leee, HIGH);
  right_fblstou(395);
  digitalWrite(leee, LOW);
  n=18;
  last_time=millis();
  
}


else if(n==18 && rr>5000  && current_time-last_time>150 )
{
  digitalWrite(leee, HIGH);
  left_fblstou(460);
  digitalWrite(leee, LOW);
  n=19;
  last_time=millis();
  
}

else if(n==19 && somme>6500  && current_time-last_time>400 )
{
  digitalWrite(leee, HIGH);
  left_fblstou(430);
  digitalWrite(leee, LOW);
  n=20;
  last_time=millis();
  
}


else if(n==20 && ll>5000  && current_time-last_time>150 )
{
  digitalWrite(leee, HIGH);
  right_fblstou(360);
  digitalWrite(leee, LOW);
  n=21;
  last_time=millis();
  
}


else if(n==21 && ll>5500  && current_time-last_time>600 )
{
  digitalWrite(leee, HIGH);
  right(450);
  stp(5555555);
  digitalWrite(leee, LOW);
  n=22;
  last_time=millis();
  






}





  //PID*****************************************************************************************************
//kp=0.25;
//kd=0.13;
//left_base=255;
//right_base=255;
 if(n==0){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }


 else if(n==1){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }
     else if(n==2){

  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }
  else if(n==3){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID_special();
    forwardPID();
    }

    else if(n==4){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID_special();
    forwardPID();
    }

    else if(n==5){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID_special();
    forwardPID();
    }

    else if(n==6){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }

    else if(n==7){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }
  else if(n==8){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }

    else if(n==9){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }


   else if(n==10){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }
  
  else if(n==11){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID_special();
    forwardPID();
    }


  else if(n==12){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }


  else if(n==13){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }

      else if(n==14){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID_special();
    forwardPID();
    }


      else if(n==15){
  kp = 0.25;
  kd = 0.13;
  right_base =150;
  left_base = 150;
    PID_noir();
    forwardPID();
    }
     else if(n==16){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }

        else if(n==17){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }


    else if(n==18){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }

        else if(n==19){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }

    else if(n==20){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }

    else if(n==21){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }


    else if(n==22){
  kp = 0.25;
  kd = 0.13;
  right_base =255;
  left_base = 255;
    PID();
    forwardPID();
    }
*/
right_base =180;
left_base = 180;
PID();
forwardPID();




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
void forward(int x) {

  analogWrite(rightF, 104);
  analogWrite(leftF, 100);
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

  analogWrite(rightF, 150);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 150);
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
  analogWrite(leftF, 150);
  analogWrite(rightR, 150);
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