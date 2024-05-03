
#include <QTRSensors.h>
#include <LiquidCrystal_I2C.h>







const uint8_t SensorCount = 16;

uint16_t s[SensorCount];
QTRSensorsRC qtr((unsigned char[]){ A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15}, SensorCount);
int position;
int n =0, p = 0, m = 0,l=1,d=-1;
int w=0;
//PID
int right_speed, left_speed;
float kp = 0.07, kd = 0.105, ki = 0, P, D, I; //vitesse 150 kp0.06 kd0.11
float PIDvalue, lasterror, error;
int left_base, right_base;
//motors
int leftF = 3;
int leftR = 2;
int rightF = 4;
int rightR = 5;
//ultrason
int front_trig=6;
int front_echo=7;
int right_trig=8;
int right_echo=9;
long fduration;
int fdistance;
long rduration;
int rdistance;
int safetyDistance;
//time
unsigned int current_time, last_time, lunch_time;
//kp = 0.07;kd = 0.15; super high speed 235 250
//kp=0.07;kd=0.13; high speed vitesse 170 185
//kp=0.06;kd=0.12; medium speed  140 155
// kp = 0.05;kd = 0.105; low speed 95 110
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
  int c15;
  int c16;
//stops
int stop1=50;
int stop2=50;
int stop3=2850;
//delays
int d0=130;
int d1=200;
int d2=230;
int d3=280;
int d4=170;
int d5=210;
int d6=150;
int d7=160;
int d8=160;
int d9=160;
int d10=210;
int d11=80;
int d12=120;
int d13=160;
int d133=140;
int d14=210;
int d15=110;
int d16=90;
int d17=160;
int d18=160;
int d19=120;
int d20=140;
int d21=110;
int d22=150;
int d23=160;
int d24=120;
int d25=120;
int d26=100;

// forwards 
int f0=50;//50
int f1=40; //20
int f2=120; //20
int f3=40; //40
int f4=40; //40
int f5=40; //20
int f6=40; //00
int f7=40; 
int f8=40; //40
int f9=50; //20
int f10=50; //00
int f11=50; //
int f12=50; //40
int f13=70; //20
int f14=50; //00
int f15=50; //back 
int b2=130;
int b3=0;
int b4=140;
/*int b5=90;
int b6=70;
int b7=70;
int b8=70;
*/
int xm=1;
int ym=0;
int r=0;
int  a ;
int last_errorm;
int last_left;
int last_right;
int high_speed_r=220;
int high_speed_l=210;
int stna=50000;
int somme=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12+c13+c14+c15+c16;
int rsomme=c1+c2+c3+c4+c5+c6+c7+c8;
int lsomme=c9+c10+c11+c12+c13+c14+c15+c16;

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//setup
void setup() {
  kp = 0.07; kd = 0.11;
  Serial.begin(9600);
  int tst = 0;
  //capt tcrt 50000
  //ultrasons
  pinMode(front_trig, OUTPUT);
  pinMode(front_echo, INPUT); 
  pinMode(right_trig, OUTPUT); 
  pinMode(right_echo, INPUT); 
  //moteurs
  pinMode(rightF, OUTPUT);
  pinMode(leftF, OUTPUT);
  pinMode(rightR, OUTPUT);
  pinMode(leftR, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(47, INPUT);
  pinMode(led, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
pinMode(22, OUTPUT);
  digitalWrite(22, LOW);  
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
  

   digitalWrite(LED_BUILTIN, LOW);
  current_time = millis();
   int x=0;
  while(1){
  x=digitalRead(22);
   Serial.println(x);
   if(x==HIGH){
        lunch_time=millis();
      if(lunch_time-current_time>200)
        break;
      }
      }
    
  

  
 //Serial.begin(9600);
 left_base=140;
  right_base=155;
  stp(400);
  
   int current_time=millis();
   int last_time=current_time;

}



//loop
void loop() {
    qtr.readLine(s);

  position = qtr.readLineM(s,QTR_EMITTERS_ON,0,false);
 current_time = millis();
 kp=0.040;kd=0.064;
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
  int c15=s[14];
  int c16=s[15];
    left_base=100;
    right_base=100;
  
  /*if((current_time -last_time) % 5 ==0){
 left_base=max(left_base--,90);
 right_base=max(right_base--,90);*
  
 }
 /*if((current_time -last_time) % 10 ==0){
 left_base-=min(left_base++,230);
 right_base=min(right_base++,245);
 if (right_base==245 ||  left_base==230)
  test1=1;}*/
 left_base=100;
  right_base=100;  
  // //autonomme
  //  if(n==50){

  //  digitalWrite(front_trig, LOW);
  //  delayMicroseconds(2);
   

  //   digitalWrite(front_trig, HIGH);
  //  delayMicroseconds(10);
  //   digitalWrite(front_trig, LOW);
  //     fduration = pulseIn(front_echo, HIGH);

  //    digitalWrite(right_trig, LOW);
  //  delayMicroseconds(2);
   
  //    digitalWrite(right_trig, HIGH);
  //  delayMicroseconds(10);
  //   digitalWrite(right_trig, LOW);
  // // Reads the echoPin, returns the sound wave travel time in microseconds
    
  //    rduration = pulseIn(right_echo, HIGH);

  //   // Calculating the distance  
  //   fdistance= fduration*0.034/2; 
  //   rdistance= rduration*0.034/2;
  //   // intialisation 
  //   if (x==1){
  //       a=rdistance;
  //       x=2;    }
    
  //     last_errorm=rdistance-a; 
       
    
  //   if(x==2){
  //   if(last_errorm>2){
  //     left_fblstou(20);
  //     stp(20);
  //   }
  //   if(last_errorm<2){
  //     right_fblstou(20);
  //     stp(20);
  //   }
  //   else  {
  //    forward(50);
  //    stp(20);
  //  }   
  //   }
  //       Serial.print("Distance: ");
  //       Serial.print(fdistance);
   
  //       Serial.print("*********Distance: ");
  //       Serial.println(rdistance);
     
  // }


  //suiveur de ligne
int somme=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12+c13+c14+c15+c16;
int msomme=c4+c5+c6+c7+c8+c9+c10+c11+c12+c13;
int rsomme=c1+c2+c3+c4+c5+c6+c7+c8;
int lsomme=c9+c10+c11+c12+c13+c14+c15+c16;
 //dourr 90 lissar loula




if(n==1 && rsomme>4500  && current_time-last_time>200){
   stp(100);
    left_fblstou(220);
    
   n=2;
   last_time=current_time;
  }
  if(n==2 && rsomme>4500  && current_time-last_time>400){
   stp(100);
    left_fblstou(290);
     
   n=3;
   last_time=current_time;
  }
   if(n==3 && somme>7000 && current_time-last_time>500){
   stp(100);
   
   n=4;
   last_time=current_time;
  }
    if(n==4 && somme<6000 && current_time-last_time>200){
   stp(100);
    n=5;
   last_time=current_time;
  }
     if(n==5 && rsomme>5000 && current_time-last_time>200){
   stp(100);
   left_fblstou(310);
   
   
   n=6;
   last_time=current_time;
  }
     if(n==6 && lsomme>5000 && current_time-last_time> 100){
   stp(100);
    forward(30);
   right_fblstouuu(230);
   n=7;
   ;
   last_time=current_time;
  }
     if(n==7 && rsomme>5000 && current_time-last_time>100){
   stp(100);
   forward(30);
   left_fblstouuu(215);
   n=10;
   last_time=current_time;
  }
  /*
   if(n==8&& somme>7000 &&current_time-last_time>200){
   stp(100);
   forward(120);
     stp(1000);
   n=9;
   last_time=current_time;
   } 
      if(n==9&& somme>8000 ){
   stp(100);
   forward(120);
   stp(1000);
   n=10;
   last_time=current_time;
   }   */
    if(n==10&&  rsomme>4500 &&current_time-last_time>1200){
   stp(100);
    left_fblstou(220);

   n=11;
   last_time=current_time;
   } 
     if(n==11&&  somme>7000 &&current_time-last_time>300){
   stp(100);
   forward(160);
    left_fblstou(250);
   n=12;
   last_time=current_time;
   } 
      if(n==12&&  lsomme>4500 &&current_time-last_time>80){
   stp(100);
    forward(80);
    right_fblstouuu(260);
   n=13;
   last_time=current_time;
   } 
        if(n==13&&  rsomme>5000 &&current_time-last_time>80){
   stp(100);
   forward(40);
    left_fblstouuu(210);
   n=14;
   last_time=current_time;
   }
          if(n==14&&  rsomme>5500 &&current_time-last_time>250){
   stp(100);
    forward(30);   
    left_fblstou(280);
   n=15;
   last_time=current_time;
   }     
            if(n==15&&  rsomme>5000 &&current_time-last_time>200){
   stp(100);
   forward(30);
    left_fblstou(290);
     last_time=current_time;
   n=16;
  
            } 
            
            if(n==16&&  somme<1700 &&current_time-last_time>500){
 stp(200);
  n=20;
            } 
   
 else if (n==20 ){
   
                        // initialize the lcd 
  // Print a message to the LCD.
lcd.init();  
  forwardo(200);
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("209");
  lcd.setCursor(2,1);
   
  lcd.print("Arduino LCM IIC 2004");
   lcd.setCursor(2,3);
  lcd.print("RONE et GABRIELLE");
  stp(9000);
  forwardo(220);
  n=17;
   last_time=current_time;
}
    else if(n==17&&  lsomme>5000 &&current_time-last_time>100){
   stp(100);
   
    right_fblstouuu(220);
   n=18;
   last_time=current_time;
   } 
   //maze
       if(n==18&&  somme<2000 &&current_time-last_time>280){
   stp(100);
   left(300);
    stp(1000);
   right_special1(700);
   right(70);
   right_special1(400);
   right(200);
   forward(70);
    forward_hey(450);
   stp(stna);
   n=19;
   last_time=current_time;
   }  


if(n==4 && n!=1 &&n!=2){
  
   kp = 0.048;kd = 0.12;
  left_base=100;
  right_base=100;
    PID();
forwardPID();
}
if(n==0){
  
   kp = 0.065;kd = 0.105;
  left_base=150;
  right_base=150;
    PID_special
    ();
forwardPID();
}

if(n==1){
  
   kp = 0.065;kd = 0.105;
  left_base=100;
  right_base=100;
    PID();
forwardPID();
}
if(n==4){
     kp = 0.065;kd = 0.105;
  left_base=160;
  right_base=160;
    PID_noir();
forwardPID();
}
if(n==5){
     kp = 0.065;kd = 0.105;
   left_base=160;
  right_base=160;
    PID();
forwardPID();
}
if(n>=6 && n<8){
     kp = 0.065;kd = 0.105;
  left_base=100;
  right_base=100;
    PID();
forwardPID();
}
if(n>=8 &&n<11){
  
   kp = 0.065;kd = 0.105;
  left_base=150;
  right_base=150;
    PID();
forwardPID();
}
if(n>=11 &&n<14){
  
   kp = 0.065;kd = 0.105;
  left_base=100;
  right_base=100;
    PID();
forwardPID();
}
if(n>=14 &&n<16 ){
  
   kp = 0.065;kd = 0.105;
  left_base=100;
  right_base=100;
    PID();
forwardPID();
}


if(n==16||n==18|| n==17 ){
  
  
   kp = 0.065;kd = 0.105;
  left_base=80;
  right_base=80;
    PID();
forwardPID();
}
if(n==19 ){
  
  
   kp = 0.065;kd = 0.105;
  left_base=80;
  right_base=80;
    PID();
forwardPID();
}
}



void PID() {
  position = qtr.readLine(s);
  error = position - 7500;
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
  error = position - 7500;
  P = error;
  D = error - lasterror;
  I = error + I;
  PIDvalue = (kp * P) + (kd * D) + (ki * I);
  lasterror = error;
  right_speed = right_base + PIDvalue;
  left_speed = left_base - PIDvalue;
  left_speed = min(160, max(left_speed, 0));
  right_speed = min(150, max(right_speed, 0));
  //delay(200);
  //Serial.println(error);
}
void PID_special() {
  bool v = false;
  position = qtr.readLine(s, QTR_EMITTERS_ON, 0, v);
  error = position - 7500;
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
  error = position - 7500;
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
  error = position - 7500;
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

  analogWrite(rightF, 132);
  analogWrite(leftF, 130);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void forwardo(int x) {

  analogWrite(rightF, 100);
  analogWrite(leftF, 112);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void forward1(int x) {

  analogWrite(rightF, 210);
  analogWrite(leftF, 200);
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
   //forward(50);
  analogWrite(rightF, 110);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 108);
  delay(x);
}
void right_fblstouu(int x) {
   //forward(50);
  analogWrite(rightF, 70);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 68);
  delay(x);
}
void right_fblstouuu(int x) {
   //forward(50);
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
   //forward(50);
  analogWrite(rightF, 0);
  analogWrite(leftF, 108);
  analogWrite(rightR, 110);
  analogWrite(leftR, 0);
  delay(x);
}
void left_fblstouu(int x) {
   //forward(50);
  analogWrite(rightF, 0);
  analogWrite(leftF, 68);
  analogWrite(rightR, 70);
  analogWrite(leftR, 0);
  delay(x);
}
void left_fblstouuu(int x) {
   //forward(50);
  analogWrite(rightF, 0);
  analogWrite(leftF, 150);
  analogWrite(rightR, 150);
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
void leftinsafe(int x) {

  analogWrite(rightF, 0);
  analogWrite(leftF, 120);
  analogWrite(rightR, 130);
  analogWrite(leftR, 0);
  delay(x);
}
void safeleft(int x) {

  analogWrite(rightF, 0);
  analogWrite(leftF, 60);
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
  analogWrite(rightR, 120);
  analogWrite(leftR, 118);
  delay(x);
}
void stp1(int x) {

  analogWrite(rightF, 0);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}
void forwardlow(int x){
  analogWrite(rightF, 80);
  analogWrite(leftF,80);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);  
}
void left_maze(int x){
  analogWrite(rightF, 60);
  analogWrite(leftF,100);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);  
}
void right_maze(int x){
  analogWrite(rightF, 100);
  analogWrite(leftF,60);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);  
}
void right_special(int x) {
   //forward(50);
  analogWrite(rightF, 210);
  analogWrite(leftF, 115);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);}
  void left_special(int x) {
   //forward(50);
  analogWrite(rightF, 70);
  analogWrite(leftF, 180);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);}
  void forward_hey(int x) {
   //forward(50);
  analogWrite(rightF, 65);
  analogWrite(leftF, 100);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);}
  void right_special1(int x) {
   //forward(50);
  analogWrite(rightF, 100);
  analogWrite(leftF, 65);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);}
