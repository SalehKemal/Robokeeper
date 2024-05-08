#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <digitalWriteFast.h>
#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 9
#define IN2 7
#define IN1 8

#define readA bitRead(PIND,2)//faster than digitalRead()
#define readB bitRead(PIND,3)//faster than digitalRead()

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int prev_Target=0;
int prev_sent=millis();
int tn=0;

void setup() {
  Serial.begin(200000);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  //attachInterrupt(digitalPinToInterrupt(ENCA),readEncoderA,CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ENCB),readEncoderB,CHANGE);

  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  //Serial.println("target pos");
}

int T=2500;
int n=0;
int Hg=400;
int lo=0;
int target=0;
int tar_read=0;
int prev_target=0;
unsigned long previousMillis = 0;
int temp_pos=0;
int last_pos=0;
int pos=0;
int Ts=0;
//int prev_sent=micros();
void loop() {

  // set target position
  //int target = 1200;
  if (Serial.available() > 0) {
    
    tar_read = Serial.parseInt();

    if (abs(tar_read-prev_target)>200){
    target=tar_read;
    // Check if the new target is not 0
    prev_target = target; // Update the previous target
    }
    else{target=prev_target;}
  }

  else {
    target=prev_target;
  }


//unsigned long currentMillis = millis(); // Get the current time
  //target=800;
  // Toggle the output based on the elapsed time
  /*if (currentMillis - previousMillis >= T/2) {
    // If enough time has passed, toggle the output
    previousMillis = currentMillis; // Save the last time the output was toggled

    // Toggle the target value between high and low
    if (target == lo) {
      target = Hg;
    } else {
      target = lo;
    }
  }*/

 //if (Serial.available() > 0) {
    //target = Serial.parseInt(); 
    /*Serial.print(target);*/// Read target value from serial
//}

  
  // PID constants
  float kp = 9;
  float kd = 0.5;
  float ki = 0.05;

  // time difference
 

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  //int pos = 0; 
  tn=millis();
  Ts=tn-prev_sent;
  if(Ts >= .001){
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    temp_pos = posi;
    //tn=micros();
    //if(tn-prev_sent >= .01*1.0e6){
    //Serial.println(posi);
    pos=temp_pos;
    last_pos=pos;
    prev_sent=tn;
    //Serial.print("pos ");
    //Serial.print(pos);
    //Serial.print("time ");
    //Serial.print(abs(tn-prev_sent));
    //prev_sent=tn;

    //Serial.println(last_pos); 
    //pos=temp_pos;
    //prev_sent=tn;
    //Serial.println(abs(tn-prev_sent)); }
    //Serial.print(pos);
    //serial
  }
  }
  else { delay(Ts);
    pos=last_pos;}
  
  Serial.println(pos);

  // error
  //last_pos=pos;
  int e = (target -pos);

 long currT = millis();
 //float deltaT = ((float) (currT - prevT))/( 1.0e3 );
  float delatT=0.001;
  prevT = currT;
  
  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
 setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;
  //delay(2);
  
  
  
  //Serial.print(target);
  //Serial.print(" ");
//Serial.print("error: ");
//Serial.print(pos);
//Serial.print(" ");
//Serial.print("position: ");
//Serial.println();

  //Serial.println(pos);
  
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWriteFast(in1,HIGH);
    digitalWriteFast(in2,LOW);
  }
  else if(dir == -1){
    digitalWriteFast(in1,LOW);
    digitalWriteFast(in2,HIGH);
  }
  else{
    digitalWriteFast(in1,LOW);
    digitalWriteFast(in2,LOW);
  }  
}

void readEncoder(){
  //int b = digitalRead(ENCB);
  if(readA>readB){
    posi++;
  }
  else{
    posi--;
  }
}
/*void readEncoderA(){
  if(readB != readA) {
    posi ++;
  } else {
    posi --;
  }
}
  
void readEncoderB(){
    if (readA == readB) {
    posi ++;
  } else {
    posi --;
  }
}
*/