#include <QTRSensors.h>

const uint8_t SensorCount = 14;
int leee=52;

uint16_t s[SensorCount];
QTRSensorsRC qtr((unsigned char[]){A14,A13,A12,A11,A10,A8,A7,A6,A5,A4,A3,A2,A1,A0}, SensorCount);
int position;
int n = 0, p = 0, m = 0;
//PID
int right_speed, left_speed;
float kp = 0.01, kd = 0, ki = 0, P, D, I; //vitesse 150 kp0.06 kd0.2   vitesse 255 kp0.11 kd0.38
float PIDvalue, lasterror, error;
int left_base, right_base;
//motors
int leftF = 3;
int leftR = 2;
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

  Serial.begin(9600);
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
  current_time = millis();

    
  

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





 Serial.print(c1);
 Serial.print("  ");
 Serial.print(c2);
 Serial.print("  ");
 Serial.print(c3);
 Serial.print("  ");
 Serial.print(c4);
 Serial.print("  ");
 Serial.print(c5);
 Serial.print("  ");
 Serial.print(c6);
 Serial.print("  ");
 Serial.print(c7);
 Serial.print("  ");
 Serial.print(c8);
 Serial.print("  ");
 Serial.print(c9);
 Serial.print("  ");
 Serial.print(c10);
 Serial.print("  ");
 Serial.print(c11);
 Serial.print("  ");
 Serial.print(c12);
 Serial.print("  ");
 Serial.print(c13);
 Serial.print("  ");
 Serial.print(c14);
 Serial.print(":::");
 PID();
 Serial.print(position);
 Serial.println();


 

 delay(10);






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
void forwardPID() {

  analogWrite(rightF, right_speed);
  analogWrite(leftF, left_speed);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
}
void forward(int x) {

  analogWrite(rightF, 215);
  analogWrite(leftF, 255);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void right(int x) {

  analogWrite(rightF, 60);
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
void forwardavantmaze(int x){
  analogWrite(rightF, 183);
  analogWrite(leftF,180);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);  
}