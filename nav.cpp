//Analogue Control
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
      
      
      float PID; // PID control feedback
      float P, I, D;
      float kp = 1;
      float ki = 0;
      float kd = 0;
      int ref; // reference position light intensity.
      
      void LFDataRead(void); // read data from our L1 & R1 LFs
      void TurnDetection(void); 
      void IntersectionDetection(void); 
      void PID(void); // PID control for straight line movement
      
  private:
      // unsigned long current_time;      
      // int pre_I, pre_P;
      // int prev_time;
}

void LFDetection::PID(ref, L1LF_data, R1LF_data, kp, ki, kd)
{
    P = ref - L1LF_data;
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
    LLF_data = analogueRead(LLF_collector);
    RLF_data = analogueRead(RLF_collector);
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
            
      int maxspeed = 255;
      int ref_speed = 150; // forward speed
      int turnspeed = 50;
      
      int delay_time = 200;
      
      void TURN (void);
      void MOVE (void);
      void STOP (void);
      
  private:
      int speedL; 
      int speedR;
}

void MovementControl::TURN() // 90 degree turn
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
      // or Turn_delay
  
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
  Serial.println("STARTO!");
  AFMS.begin();  // create with the default frequency 1.6KHz
  LeftMotor->setSpeed(150);
  LeftMotor->run(FORWARD);
  LeftMotor->run(RELEASE);
  LeftMotor->run(BACKWARD);
  RightMotor->setSpeed(150);
  RightMotor->run(FORWARD);
  RightMotor->run(RELEASE);
  RightMotor->run(BACKWARD);

  // initialize the setup 
  MovementControl MC;
  
  MC.L2LF_anode = 3
  MC.L1LF_anode = 4;
  MC.R1LF_anode = 5;
  MC.R2LF_anode = 6;

  pinMode(MC.L2LF_anode,OUTPUT);
  pinMode(MC.L1LF_anode,OUTPUT);
  pinMode(MC.R1LF_anode,OUTPUT);
  pinMode(MC.R2LF_anode,OUTPUT);
  
  digitalWrite(MC.L2LF_anode,HIGH);
  digitalWrite(MC.L1LF_anode,HIGH);
  digitalWrite(MC.R1LF_anode,HIGH);
  digitalWrite(MC.R2LF_anode,HIGH);

  MC.L2LF_collector = 7
  MC.L1LF_collector = 8;
  MC.R1LF_collector = 9;
  MC.R2LF_collector = 10;
  
  pinMode(MC.L2LF_collector,INPUT);
  pinMode(MC.L1LF_collector,INPUT);
  pinMode(MC.R1LF_collector,INPUT);
  pinMode(MC.R2LF_collector,INPUT);
  
}

void loop() 
{
  MC.MOVE();
}

