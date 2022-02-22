#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// the motor pin 1234?
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(1);

Class LFDetection
{
  public:
      int L2LF_anode; // L2-L1-R1-R2
      int L1LF_anode; // LF output for High Voltage
      int R1LF_anode;
      int R2LF_anode;
      
      int L2LF_collector;
      int L1LF_collector; // LF input for LF_data. 2 LF in black area.
      int R1LF_collector;
      int R2LF_collector;
      
      int L2LF_data;
      int L1LF_data; // LF data: 1 for black, 0 for white.
      int R1LF_data;
      int R2LF_data;
      
      
      int Turn = 0;
      
      
      float PID;
      float P, I, D;
      float kp = 1;
      float ki = 0;
      float kd = 0;
      int ref; // reference position light intensity.
      
      void LFDataRead(void);
      void TurnDetection(void);
      void IntersectionDetection(void);
      void PID(void);
      
  private:
      // unsigned long current_time;      
      // int pre_I, pre_P;
      // int prev_time;
}

void LFDetection::PID(ref, L1LF_data, R1LF_data, kp, ki, kd)
{
    P = ref - real;
    PID = P;
    /*
    I = pre_I + P * iter_time;
    D = (pre_P - P) / iter_time;
    current_time = millis();
    prev_time = current_time;
    pre_I = I;
    pre_P = P;
    PID = P * kp + I * ki + D * kd;
    */  
}

void LFDetection::LFDataRead()
{
    LLF_data = digitalRead(LLF_collector);
    RLF_data = digitalRead(RLF_collector);
}

// 90 degree turn detection
void LFDetection::TurnDetection()
{
    pass;
}

void LFDetection::IntersectionDetection();
{
    pass;
}

Class MovementControl: public: LFDetection
{
  public:
      int motL; // motor output
      int motR;
           
      int speedL;
      int speedR;
      int maxspeed;
      int ref_speed; // forward speed.
      int turnspeed;
      
      int delay_time;
      
      void SETSPEED(void);
      void TURN (void);
      void MOVE (void);
      void STOP (void);
}

void MovementControl::SETSPEED()
{



}
void MovementControl::TURN()
{
  if (Turn==0){
      pass;
  }
  
  else {
      if (Turn==1)  //LEFT 
      { 
          LeftMotor->run(BACKWARD);
          LeftMotor->setSpeed(turnspeed);
          RightMotor->run(FORWARD);
          RightMotor->setSpeed(turnspeed);
        
          while(LLF_data==0){
            LFDataRead();
          }
      } 
      else if (Turn==2){
          LeftMotor->run(FORWARD);
          LeftMotor->setSpeed(turnspeed);
          RightMotor->run(BACKWARD);
          RightMotor->setSpeed(turnspeed);
          while(RLF_data==0){
            LFDataRead();
          }
      } 
      // to fill in
      if IntersectionDetection(){
          pass;
      }
  
      LeftMotor->run(FORWARD);
      LeftMotor->setSpeed(255);
      RightMotor->run(FORWARD);
      RightMotor->setSpeed(255);
      
      Turn = 0;
  }
}

void MovementControl::MOVE()
{

  PID = PID();
  
  speedL = ref_speed + PID;
  speedR = ref_speed - PID;
  
  if (speedL > maxspeed) {
    speedL = maxspeed;
  }
  if (speedR > maxspeed) {
    speedR = maxspeed;
  }
  if (speedL < 0) {
    speedL = 0;
  }
  if (speedR < 0) {
    speedR = 0;
  }
  LeftMotor->run(FORWARD);
  LeftMotor->setSpeed(speedL);
  RightMotor->run(FORWARD);
  RightMotor->setSpeed(speedR);
  
  while(1){
    LFDataRead();
    PID();
    TurnDetection();
    TURN();
    delay(delay_time);
  }
}

void MovementControl::STOP()
{
  LeftMotor->run(FORWARD);
  LeftMotor->setSpeed(0);
  RightMotor->run(FORWARD);
  RightMotor->setSpeed(0);
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  AFMS.begin();  // create with the default frequency 1.6KHz
  LeftMotor->setSpeed(150);
  LeftMotor->run(FORWARD);
  LeftMotor->run(RELEASE);
  LeftMotor->run(BACKWARD);

  pinMode(LED_BUILTIN, OUTPUT);
  MovementControl MC;
  
  MC.motL = 1;
  MC.motR = 2;
  pinMode(MC.motL,OUTPUT);
  pinMode(MC.motR,OUTPUT);
  
  MC.LLF_anode = 3;
  MC.RLF_anode = 4;
  pinMode(MC.LLF_anode,OUTPUT);
  pinMode(MC.RLF_anode,OUTPUT);
  digitalWrite(MC.LLF_anode,HIGH);
  digitalWrite(MC.RLF_anode,HIGH);

  MC.LLF_collector = 5;
  MC.RLF_collector = 6;
  pinMode(MC.LLF_collector,INPUT);
  pinMode(MC.RLF_collector,INPUT);
  
  MC.turn_delay = 1000;
   
}

void loop() 
{
  MC.MOVE();
}

----------------------------

void setup() {
  //Motor Setup
 
}
void loop() {
 int i;
 digitalWrite(LED_BUILTIN, HIGH);
 delay(10);
 digitalWrite(LED_BUILTIN, LOW);
 Serial.print("tick ");
 LeftMotor->run(FORWARD);
 LeftMotor->setSpeed(255);
 RightMotor->run(BACKWARD);
 RightMotor->setSpeed(255);
 
 delay(2000);
 //myMotor->run(RELEASE);
 //delay(2000);
  digitalWrite(LED_BUILTIN, HIGH);
 delay(10);
 digitalWrite(LED_BUILTIN, LOW);
 LeftMotor->setSpeed(0);
 RightMotor->setSpeed(0);
 delay(2000);
 /*
 for (i=0; i<255; i++) {
   myMotor->setSpeed(i);  
   delay(1);
 }
 delay(1000);
 for (i=255; i!=0; i--) {
   myMotor->setSpeed(i);  
   delay(1);
 }
 */
}
