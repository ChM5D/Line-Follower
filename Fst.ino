
int c2 = A2;
int c3 = 3 ;
int c4 = 2;
int c5 = A3;
int c6 = 4;
int c7 = A0;
int c8 = 7;
int c9 = A1;
int ena = 5;
int enb = 6;
int in1 = 8;
int in2 = 9;
int in3 = 11;
int in4 = 10;

int comp=1;
int ct,lt;
float bmsr = 100;
float bmsl= 100*125/90;
float rms;
float lms;
float kp =2.4;
float kd =1.3;
int maxrms = 95;
int maxlms = 135;
int ms;
int lasterror = 0;
 void forward(int x=125) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enb,x);
    analogWrite(ena,x*0.72);
    }
void setup() {
  pinMode(c2, INPUT);
  pinMode(c3, INPUT);
  pinMode(c4, INPUT);
  pinMode(c5, INPUT);
  pinMode(c6, INPUT);
  pinMode(c7, INPUT);
  pinMode(c8, INPUT);
  pinMode(c9, INPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  analogWrite(enb,125);
  analogWrite(ena,90);


}

void loop() {
   int capt2 = digitalRead(c2);
   int capt3 = digitalRead(c3);
   int capt4 = digitalRead(c4);
   int capt5 = digitalRead(c5);
   int capt6 = digitalRead(c6);
   int capt7 = digitalRead(c7);
   int capt8 = digitalRead(c8);
   int capt9 = digitalRead(c9);
   int somme=capt2+capt3+capt4+capt5+capt6+capt7+capt8+capt9;


if (comp==1 && capt7+capt8+capt9>=2)
  {

    stops();
    delay(500);
    forward();
    delay(70);
    right();
    delay(600);
    back();
    delay(350);
    stops();
    delay(300);
    comp++;
  }
else if(comp==2&& capt8+capt9>=1){

  stops();
    delay(500);
    right();
    delay(350);
    comp++;
}
else if (comp==3&& capt2+capt3>=1)
{ 
    stops();
    delay(500);
    left();
    delay(450);
    comp++;
}
else if (comp==4&& capt2+capt3+capt4+capt5>=3)
{

    stops();
    delay(500);
    left();
    delay(560);
    comp++;
}
else if (comp==5&& capt8+capt9+capt7+capt6>=3)
{

    stops();
    delay(500);
    forward();
    delay(100);
    right();
    delay(400);
    back();
    delay(350);
    stops();
    delay(300);
    comp++;
}
else if (comp==6&& capt8+capt9+capt7>=2)
{  
    stops();
    delay(500);
    right();
    delay(450);
    comp++;
}
else if (comp==7&& capt9+capt8+capt7+capt6>=3)
{

    stops();
    delay(500);
    forward();
    delay(70);
    right();
    delay(320);
    back();
    delay(350);
    stops();
    delay(300);
    comp++;
}
else if (comp==8&& capt9+capt8+capt7+capt6>=3)
{ 
    stops();
    delay(500);
    right();
    delay(450);
    comp++;
}

else{PID();}
}
//  void forward(int x=125) {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, HIGH);
//     digitalWrite(in3, HIGH);
//     digitalWrite(in4, LOW);
//     analogWrite(enb,x);
//     analogWrite(ena,x*0.72);
//     }
     void forwardPID() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    }
 void back() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);}
  void right() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);}
    void left() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);}
      void rights() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);}
    void lefts() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);}
      void stops(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);}
void PID(){ 
  int capt2 = digitalRead(c2);
  int capt3 = digitalRead(c3);
  int capt4 = digitalRead(c4);
  int capt5 = digitalRead(c5);
  int capt6 = digitalRead(c6);
  int capt7 = digitalRead(c7);
  int capt8 = digitalRead(c8);
  int capt9 = digitalRead(c9);


 


 int error= -60*capt2 + -35*capt3 + -25*capt4 + -25*capt5 + 10*capt6+ 25*capt7+ 35*capt8 + 60*capt9;
 
 ms = kp*error +kd*(error-lasterror);
 
 rms = bmsr + ms;
 lms = bmsl - int(ms/0.72);
 lasterror = error;

 if(rms > 183){rms = 183;}

 if(lms < 0){lms = 0;}

 if(rms < 0){rms = 0;}


  analogWrite(enb,lms);
  analogWrite(ena,rms);

// if( capt9 || capt8 || capt7){right();}


// else if(capt2 || capt3 || capt4){left();}


// else {forward();}
forwardPID();
}

  
