#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ArduinoQueue.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// the motor pin 1234?
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(1);

ArduinoQueue<int> L2Queue = ArduinoQueue<int>(100);
ArduinoQueue<int> R2Queue = ArduinoQueue<int>(100);

class LFDetection
{
  
  public:
      int L2LF_anode; // L2-L1-R1-R2
      int L1LF_anode = 10; // LF output for High Voltage
      int R1LF_anode = 11;
      int R2LF_anode;
      
      int L2LF_collector;
      int L1LF_collector = 8; // LF input for LF_data. 2 LF in black area.
      int R1LF_collector = 9;
      int R2LF_collector;
      
      int L2LF_data;
      int L1LF_data; // LF data: 1 for black, 0 for white.
      int R1LF_data;
      int R2LF_data;
  
      int Turn = 0;
      int detection_threshold = 100; // number of 1s in queue
      
      int delay_time = 500;
      float PID_; // PID control feedback
      float P, I, D;
      float kp = 20;
      float ki = 3;
      float kd = 0;
      int ref = 0; // reference position light intensity.
      
      void LFDataRead(void); // read data from our L1 & R1 LFs
      void TurnDetection(void); 
      void IntersectionDetection(void); 
      void PID(void); // PID control for straight line movement
      void EdgeDetection(void);
  
  
      float pre_I = 0;
      // float pre_P;
      unsigned long current_time; 
      unsigned long prev_time;
      int intersection_counter;
      int white_counter;
      
};



void LFDetection::LFDataRead()
{
    Serial.println("Reading...");
    L1LF_data = digitalRead(L1LF_collector);
    R1LF_data = digitalRead(R1LF_collector);
}

// 90 degree turn detection

void LFDetection::EdgeDetection()
{
    int Ldeq;
    int Rdeq;
    Ldeq = L2Queue.dequeue();
    Rdeq = R2Queue.dequeue();
    L2Queue.enqueue(L2LF_data);
    R2Queue.enqueue(R2LF_data);
    
    if (L2LF_data == 1){
    white_counter++;
    }
    if (Ldeq == 1){
    white_counter--;
    }
    if (R2LF_data == 1){
    white_counter++;
    }
    if (Rdeq == 1){
    white_counter--;
    }
}

void LFDetection::TurnDetection()
{
    return;
}

void LFDetection::IntersectionDetection()
{
    
    if (white_counter == detection_threshold) {
    intersection_counter++;
    }
  
    //reset L2R2LF counter
    white_counter = 0;
    for (int i=0;i<100;i++)
      {
        L2Queue.dequeue();
        R2Queue.dequeue();
    }
    for (int i=0;i<100;i++)
      {
        L2Queue.enqueue(0);
        R2Queue.enqueue(0);
    }
  
  
}




// ******************************************

class MovementControl: public LFDetection
{
  public:
            
      int maxspeed = 255;
      int ref_speed = 150; // forward speed
      int turnspeed = 50;
      
      
      void TURN (void);
      void MOVE (void);
      void STOP (void);
      void PID(void);
      
  private:
      int speedL; 
      int speedR;
};


void MovementControl::PID()
{ 

    int current_time = millis();
    //Serial.println("prev_time = " + String(prev_time));
    //Serial.println("current_time = " + String(current_time));
    //Serial.println("real_delay = " + String(current_time - prev_time));
    P = L1LF_data - R1LF_data;
    I = 0; // pre_I + P * 0.001 * (current_time - prev_time);
    PID_ = P*kp + I*ki;
    pre_I = I;
    prev_time = current_time;

  speedL = ref_speed + PID_;
  speedR = ref_speed - PID_;
  
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
    /*
    D = (pre_P - P) / iter_time;
    current_time = millis();
    prev_time = current_time;
    pre_I = I;
    pre_P = P;
    PID = P * kp + I * ki + D * kd;
    */  
}

void MovementControl::TURN() // 90 degree turn
{
  if (Turn==0){
      return;
  }
  
  else {
      if (Turn==1)  //LEFT 
      { 
          LeftMotor->run(BACKWARD);
          LeftMotor->setSpeed(turnspeed);
          RightMotor->run(FORWARD);
          RightMotor->setSpeed(turnspeed);
          delay(delay_time);
          while(L1LF_data==0){
            LFDataRead();
          }
      } 
      else if (Turn==2){
          LeftMotor->run(FORWARD);
          LeftMotor->setSpeed(turnspeed);
          RightMotor->run(BACKWARD);
          RightMotor->setSpeed(turnspeed);
          delay(delay_time);
          while(R1LF_data==0){
            LFDataRead();
          }
      } 
      // to fill in
      //if (IntersectionDetection()){
      //    return;
      //}
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
 
  
  while(1){
    LFDataRead();
    PID();
    Serial.println("PID value: "+ String(PID_));
    Serial.println("Sensor 1: " + String(L1LF_data));
    Serial.println("Sensor 2: " + String(R1LF_data));
    Serial.println("Motor speed L: " + String(speedL));
    Serial.println("Motor speed R: " + String(speedR));
    //TurnDetection();
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
  Serial.println("START!");
  AFMS.begin();  // create with the default frequency 1.6KHz
  LeftMotor->setSpeed(150);
  LeftMotor->run(FORWARD);
  LeftMotor->run(RELEASE);
  LeftMotor->run(BACKWARD);
  RightMotor->setSpeed(150);
  RightMotor->run(FORWARD);
  RightMotor->run(RELEASE);
  RightMotor->run(BACKWARD);
  
  for (int i=0;i<100;i++)
  {
    L2Queue.enqueue(0);
    R2Queue.enqueue(0);
  }

  // initialize the setup 
  
  
}

void loop() 
{
  MovementControl MC;
  
  MC.L2LF_anode = 3;
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

  MC.L2LF_collector = 7;
  MC.L1LF_collector = 8;
  MC.R1LF_collector = 9;
  MC.R2LF_collector = 10;
  
  pinMode(MC.L2LF_collector,INPUT);
  pinMode(MC.L1LF_collector,INPUT);
  pinMode(MC.R1LF_collector,INPUT);
  pinMode(MC.R2LF_collector,INPUT);

  MC.MOVE();
}
