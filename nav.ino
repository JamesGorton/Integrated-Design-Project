#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ArduinoQueue.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Voltage supply pins. From left to right L2 L1 R1 R2
// Set these pins to HIGH to supply voltage.
const int L2LF_anode;
const int L1LF_anode = 10;
const int R1LF_anode = 11;
const int R2LF_anode;

// Sensor data receive pins.
const int L2LF_collector;
const int L1LF_collector = 8;
const int R1LF_collector = 9;
const int R2LF_collector;

// Tunable Parameters.
const float kp = 20; // Proportional gain.
const float ki = 3; // Integral gain.
const float kd = 0; // Derivative gain.

const int Intersection_queue_length = 30;
const int detection_threshold = 20; // IntersectionDetection Threshold, the number of 1s in queue

const int delay_time = 500; // main loop delay.
const int max_speed = 255; // Maximum allowable motor speed.
const int ref_speed = 150; // forward speed
const int turn_speed = 50;

//  motor pins
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(1);

// Line sensor data. 0 (black) or 1 (white).
int L2LF_data;
int L1LF_data;
int R1LF_data;
int R2LF_data;

int task = 0;
int main_loop_counter = 0;

// PID parameters.
float PIDError; // PID control feedback
float P, I, D;
float pre_I = 0;
float pre_P = 0;
unsigned long current_time = 0;
unsigned long prev_time = 0;

int Turn = 0;

int intersection_counter = 0;
int white_counter = 0;

int D1_inv = 3;
int D1_ouput = 4;
int D2_inv = 5;
int D2_ouput = 6;

float D1_data;
float D2_data;

ArduinoQueue<int> L2Queue = ArduinoQueue<int>(Intersection_queue_length);
ArduinoQueue<int> R2Queue = ArduinoQueue<int>(Intersection_queue_length);

class LFDetection
{
public:
    void LFDataRead(void); // read data from our L1 & R1 LFs
    void TurnDetection(void);
    void IntersectionDetection(void);
    void EdgeDetection(void);
    void BlockDetection(void);
};


void LFDetection::LFDataRead()
{
    L1LF_data = digitalRead(L1LF_collector);
    R1LF_data = digitalRead(R1LF_collector);
    Serial.println("Sensor Data 1 2: " + String(L1LF_data) + " " + String(R1LF_data));
}

void LFDetection::EdgeDetection()
{
    int Ldeq;
    int Rdeq;
    Ldeq = L2Queue.dequeue();
    Rdeq = R2Queue.dequeue();
    L2Queue.enqueue(L1LF_data);
    R2Queue.enqueue(R1LF_data);

    if (L1LF_data == 1){
        white_counter++;
    }
    if (Ldeq == 1){
        white_counter--;
    }
    if (main_loop_counter % 5 == 0){
        Serial.println("White_Counter: " + String(white_counter));
    }

    /*
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
    */

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
    for (int i=0;i<100;i++){
        L2Queue.dequeue();
        R2Queue.dequeue();
        L2Queue.enqueue(0);
        R2Queue.enqueue(0);
    }
}

// Distance sensor version
// Distance sensor at a fixed distance has a variation at 0.2 V variation and the Distance-Voltage diagram is not linear
void LFDetection::BlockDetection()
{   
    // execute when starting block detection turn (-T/2 + T, for period T it can turn 180 degree)
    start_time = millis();
    Short = 0;
    Left_edge = 0;
    Right_edge = 0
    
    // time i.e. angle
    time_in = -1;
    time_out = -1;
    time_short = -1;
        
    // D1 data reading
    i = D1_data;
    diff = i - i_prev;
    if (diff > diff_threshold){
        if (time_in == -1){ //initialization value
            time_in = millis() - start_time;
            Left_edge = i;
                
            //starting detecting Short
        }
        else{
            time_out = millis() - start_time;
            Right_edge = i_prev;
        }
    }
    
    if (time_in != -1){
        if (time_out != -1){
            pass;
        }
        else{
            if (i<Short){
                Short = i;
                time_short = millis() - start_time;
            }
        }
    }
    
    i_prev = i;
    
    // after turning movement:
    
    robot turn - [T - (time_out+time_in)/2]
    move 
    
    
}
// ******************************************

class MovementControl: public LFDetection
{
  public:
      void TURN (void);
      void LineFollow (void);
      void STOP (void);
      void PID(void);
      void DummyMove(void);
      void FindTask(void);

  private:
      int speedL;
      int speedR;
};

void MovementControl::FindTask(){
    LFDataRead();
    EdgeDetection();

    if (intersection_counter == 0){
        task = 0;
        Serial.println("Starting Dummy Move Forward.");
    }

    if (intersection_counter == 1){
        task = 1;
        Serial.println("Starting Line Follow.");
    }

    if (false){
        task = 2;
        Serial.println("Starting Turn.");
    }

}


void MovementControl::PID()
{

    int current_time = millis();
    P = L1LF_data - R1LF_data;
    I = 0; // pre_I + P * 0.001 * (current_time - prev_time);
    PIDError = P*kp + I*ki;
    pre_I = I;
    prev_time = current_time;

  speedL = ref_speed + PIDError;
  speedR = ref_speed - PIDError;

  if (speedL > max_speed) {
    speedL = max_speed;
  }
  if (speedR > max_speed) {
    speedR = max_speed;
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
          LeftMotor->setSpeed(turn_speed);
          RightMotor->run(FORWARD);
          RightMotor->setSpeed(turn_speed);
          delay(delay_time);
          while(L1LF_data==0){
            LFDataRead();
          }
      }
      else if (Turn==2){
          LeftMotor->run(FORWARD);
          LeftMotor->setSpeed(turn_speed);
          RightMotor->run(BACKWARD);
          RightMotor->setSpeed(turn_speed);
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

void MovementControl::LineFollow()
{
    LFDataRead();
    PID();
    Serial.println("---Line following---");
    Serial.println("Sensor 1 2: " + String(L1LF_data) + " " + String(R1LF_data));
    Serial.println("PID value: "+ String(PIDError));
    Serial.println("Motor speed L R: " + String(speedL) + " "
     + String(speedR));
    TURN();
    delay(delay_time);
}

void MovementControl::DummyMove(){
    LeftMotor->run(FORWARD);
    LeftMotor->setSpeed(ref_speed);
    RightMotor->run(FORWARD);
    RightMotor->setSpeed(ref_speed);
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
  AFMS.begin();
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

  pinMode(L2LF_anode,OUTPUT);
  pinMode(L1LF_anode,OUTPUT);
  pinMode(R1LF_anode,OUTPUT);
  pinMode(R2LF_anode,OUTPUT);
  digitalWrite(L2LF_anode,HIGH);
  digitalWrite(L1LF_anode,HIGH);
  digitalWrite(R1LF_anode,HIGH);
  digitalWrite(R2LF_anode,HIGH);
  pinMode(L2LF_collector,INPUT);
  pinMode(L1LF_collector,INPUT);
  pinMode(R1LF_collector,INPUT);
  pinMode(R2LF_collector,INPUT);

  // MovementControl MC;
  // LFDetection LF;
}

void loop()
{
    Serial.println("Testing START!");
    Serial.println("------------------------");

    MovementControl MC;
    LFDetection LF;

    current_time = millis();

    MC.FindTask();

    if (task == 0){
        MC.DummyMove();
    }

    if (task == 1){
        MC.LineFollow();
    }

    if (task == 2){
        MC.TURN();
    }

    delay(delay_time);
    main_loop_counter++;
}
