#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ArduinoQueue.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Voltage supply pins. From left to right L2 L1 R1 R2
// Set these pins to HIGH to supply voltage.
/*
const int L2LF_supply;
const int L1LF_supply = 10;
const int R1LF_supply = 11;
const int R2LF_supply;
*/

// Motor shield motor pins.
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(3);

// Line sensor data receive pins.
const int L2LF_receive;
const int L1LF_receive = 8;
const int R1LF_receive = 10;
const int R2LF_receive;

// Distance sensor supply and receive pins.
const int DS_supply;
const int DS_receive;

// Tunable Parameters.
const float kp = 30; // Proportional gain.
const float ki = 0; // Integral gain.
const float kd = 0; // Derivative gain.

const int main_loop_delay_time = 100; // main loop delay.
const int printfreq = 500 / main_loop_delay_time;
const int delay_time = 100; // Misc delay.
const int max_speed = 255; // Maximum allowable motor speed.
const int ref_speed = 150; // Normal forward motor speed.
const int turn_speed = 150; // Turning speed.

const int intxn_queue_length = 5; // intersection queue length.
const int intxn_detection_threshold = 4; // IntersectionDetection Threshold, the number of 1s in intersection queue.
const int intxn_deb_time = 1000; // Intersection debounce threshold time.
int l_intxn_deb_prev; // Last Left intersection debounce start time.
int r_intxn_deb_prev; // Last Right intersection debounce start time.
int l_intxn_deb = 1; // Intersection debounce checker.
int r_intxn_deb = 1;

const int sweep_queue_length = 10; // Distance sensor sweep queue length.
const int front_queue_length = 5;
const int back_queue_length = 5;
const float dip_threshold = 4.;
float front_avg = 0;
float back_avg = 0;
ArduinoQueue<int> Q = ArduinoQueue<int>(sweep_queue_length);
ArduinoQueue<int> F = ArduinoQueue<int>(front_queue_length);
ArduinoQueue<int> B = ArduinoQueue<int>(back_queue_length);

float front_front;
float mid_front;
float back_front;

int block_found = 0;
unsigned long search_time = 0;

// ---------------------

int task = 0;
unsigned long main_loop_counter = 0;

// PID parameters.
float PIDError = 0; // PID control feedback
float P, I, D;
float pre_I = 0;
float pre_P = 0;
unsigned long current_time = 0;
unsigned long prev_time = 0;

int Turn = 2;

int left_intxn_counter = 0;
int right_intxn_counter = 0;
int left_white_counter = 0;
int right_white_counter = 0;

// Line sensor data. 0 (black) or 1 (white).
int L2LF_data;
int L1LF_data;
int R1LF_data;
int R2LF_data;

float DS_data; // Distance Sensor data.

ArduinoQueue<int> L2Queue = ArduinoQueue<int>(intxn_queue_length);
ArduinoQueue<int> R2Queue = ArduinoQueue<int>(intxn_queue_length);


class LFDetection
{
public:
    void LFDataRead(void); // Read data from line sensors.
    void DSDataRead(void); // Read data from distance sensors.
    // void TurnDetection(void);
    void EdgeDetection(void);
    void IntersectionDetection(void); // Count the number of intersections encountered.

    void BlockDetection(void);

};

void LFDetection::BlockDetection(){
  // Fill three queues
  // F----- Q ---------- B-----
  while (!F.isFull()){
    DS_data = analogRead(DS_receive);
    F.enqueue( DS_data );
    front_avg += DS_data / front_queue_length;
  }
  while (!Q.isFull()){
    DS_data = analogRead(DS_receive);
    Q.enqueue( DS_data );
  }
  while (!B.isFull()){
    DS_data = analogRead(DS_receive);
    B.enqueue( DS_data );
    back_avg += DS_data / back_queue_length;
  }
  
  while (true){
    DS_data = analogRead(DS_receive);
    front_front = F.dequeue();
    mid_front = Q.dequeue();
    back_front = B.dequeue();
    
    F.enqueue(mid_front);
    Q.enqueue(back_front);
    B.enqueue(DS_data);
    
    front_avg -= front_front / front_queue_length;
    front_avg += mid_front / front_queue_length;
    back_avg -= back_front / back_queue_length;
    back_avg += DS_data / back_queue_length;
    
    Serial.println("Front Average: " + String(front_avg));
    Serial.println("Back  Average: " + String(back_avg));
    
    if (front_avg - back_avg > dip_threshold){
      Serial.println("Block Found.");
      break;
    }
  }
}


void LFDetection::LFDataRead()
{
    L1LF_data = digitalRead(L1LF_receive);
    R1LF_data = digitalRead(R1LF_receive);

    if (main_loop_counter % printfreq == 0){
      Serial.println("Sensor Data 1 2: " + String(L1LF_data) + " " + String(R1LF_data));
    }
}

void LFDetection::DSDataRead()
{
    DS_data = digitalRead(DS_receive);
/*
    if (main_loop_counter % printfreq == 0){
      Serial.println("Distance sensor: " + String(DS_data));
    }
    */
}

void LFDetection::EdgeDetection() // Testing using L1 and R1.
{
    int Ldeq = L2Queue.dequeue();
    int Rdeq = R2Queue.dequeue();
    if (l_intxn_deb == 0){ // Debounce not finished.
        L2Queue.enqueue(0);
    }
    else{
        L2Queue.enqueue(L1LF_data); // to be changed to L2
    }
    if (r_intxn_deb == 0){ // Debounce not finished.
        R2Queue.enqueue(0);
    }
    else{
        R2Queue.enqueue(R1LF_data); // to be changed to L2
    }
    // L2Queue.enqueue(L1LF_data); // to be changed to L2
    // R2Queue.enqueue(R1LF_data); // to be changed to R2

    if (L1LF_data == 1){
        left_white_counter++;
        // Serial.println("L1 ENqueued 1");
    }
    if (Ldeq == 1){
        left_white_counter--;
        // Serial.println("L1 DEqueued 1");
    }
    if (R1LF_data == 1){
        right_white_counter++;
        // Serial.println("R1 ENqueued 1");
    }
    if (Rdeq == 1){
        right_white_counter--;
        // Serial.println("R1 DEqueued 1");
    }
    /*
    if (main_loop_counter % 10 == 0){
        Serial.println("left_White_Counter: " + String(left_white_counter));
        Serial.println("right_White_Counter: " + String(right_white_counter));
    }
    */
}

/*
void LFDetection::TurnDetection()
{
    return;
}
*/

void LFDetection::IntersectionDetection()
{
    if (current_time - l_intxn_deb_prev > intxn_deb_time){
        l_intxn_deb = 1;
    }
    if (current_time - r_intxn_deb_prev > intxn_deb_time){
        r_intxn_deb = 1;
    }
    if (left_white_counter == intxn_detection_threshold) {
        // Debounce after recorded an intersection.
        l_intxn_deb_prev = current_time;
        l_intxn_deb = 0;

        // Record the intersection.
        left_intxn_counter++;
        Serial.println("Left Intersection " + String(left_intxn_counter) + " recorded. ");

        // Reset Line follower counter.
        left_white_counter = 0;
        for (int i=0;i<intxn_queue_length;i++){
            L2Queue.dequeue();
            L2Queue.enqueue(0);
        }
    }
    if (right_white_counter == intxn_detection_threshold) {
        // Debounce after recorded an intersection.
        r_intxn_deb_prev = current_time;
        r_intxn_deb = 0;

        // Record the intersection.
        right_intxn_counter++;
        Serial.println("Right Intersection " + String(right_intxn_counter) + " recorded. ");

        // Reset Line follower counter.
        right_white_counter = 0;
        for (int i=0;i<intxn_queue_length;i++){
            R2Queue.dequeue();
            R2Queue.enqueue(0);
        }
    }
}

// ******************************************

class MovementControl: public LFDetection
{
  public:
      void FindTask(void);
      void TURN(void);
      void LineFollow(void);
      void STOP(void);
      void PID(void);
      void DummyMove(void);
      void SEARCH(void);

  private:
      int speedL;
      int speedR;
};

void MovementControl::FindTask(){
    LFDataRead();
    DSDataRead();
    EdgeDetection();
    IntersectionDetection();

    if (false){
        task = 0;
        if (main_loop_counter % printfreq == 0){
        Serial.println("Dummy Move Forward."); }// "Starting Dummy Move Forward.");
    }

    if (left_intxn_counter >= 0){
        task = 1;
        if (main_loop_counter % printfreq == 0){
        Serial.println("Line Following.");}
    }

    if (false){
        task = 2;
        if (main_loop_counter % printfreq == 0){
        Serial.println("Starting Right Turn.");}
    }

    if (false){
        task = 3;
        if (main_loop_counter % printfreq == 0){
        Serial.println("Looking for block.");}
    }
}

void MovementControl::SEARCH(){
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    delay(500);
    
    LeftMotor->run(RELEASE);
    RightMotor->run(RELEASE);
    int search_time = millis();
    while(millis() - search_time < 5000 && block_found != 1){
        DSDataRead();
        BlockDetection();
        LeftMotor->run(BACKWARD);
        LeftMotor->setSpeed(20);
    }

    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    delay(500);

    while (block_found != 1){
        DSDataRead();
        BlockDetection();
        RightMotor->run(BACKWARD);
        RightMotor->setSpeed(20);
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    delay(500);
}

void MovementControl::PID()
{

    int current_time = millis();
    P = -(L1LF_data - R1LF_data);
    I = pre_I + P * 0.001 * (current_time - prev_time);
    PIDError = P * kp + I * ki;
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
  // LeftMotor->setSpeed(0);
  RightMotor->run(FORWARD);
  // RightMotor->setSpeed(0);
  RightMotor->setSpeed(speedR);
}

void MovementControl::TURN() // 90 degree turn
{
  if (Turn==0){
      return;
  }

  else {
      if (Turn == 1)  //LEFT
      {
          LeftMotor->run(BACKWARD);
          LeftMotor->setSpeed(turn_speed);
          RightMotor->run(FORWARD);
          RightMotor->setSpeed(turn_speed);
          // delay(delay_time);
          while(R1LF_data == 0){
            LFDataRead();
          }
      }
      else if (Turn == 2){
          LeftMotor->run(FORWARD);
          LeftMotor->setSpeed(turn_speed);
          RightMotor->run(BACKWARD);
          RightMotor->setSpeed(turn_speed);
          // delay(delay_time);
          while(L1LF_data == 0){
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
    if (main_loop_counter % printfreq == 0){
        // Serial.println("Sensor 1 2: " + String(L1LF_data) + " " + String(R1LF_data));
        // Serial.println("PID value: "+ String(PIDError));
        Serial.println("Motor speed L R: " + String(speedL) + " "
         + String(speedR));
    }
    // delay(delay_time);
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
  Serial.println("Testing START!");
  AFMS.begin();

  for (int i=0;i<intxn_queue_length;i++)
  {
    L2Queue.enqueue(0);
    R2Queue.enqueue(0);
    // Serial.println(String(R2Queue.itemCount()) + " " + String(L2Queue.itemCount()));
  }
  /*
  pinMode(L2LF_supply,OUTPUT);
  pinMode(L1LF_supply,OUTPUT);
  pinMode(R1LF_supply,OUTPUT);
  pinMode(R2LF_supply,OUTPUT);
  */
  pinMode(DS_supply, OUTPUT);
  /*
  digitalWrite(L2LF_supply,HIGH);
  digitalWrite(L1LF_supply,HIGH);
  digitalWrite(R1LF_supply,HIGH);
  digitalWrite(R2LF_supply,HIGH);
  */
  pinMode(L2LF_receive,INPUT);
  pinMode(L1LF_receive,INPUT);
  pinMode(R1LF_receive,INPUT);
  pinMode(R2LF_receive,INPUT);
  pinMode(DS_receive, INPUT);

  // MovementControl MC;
  // LFDetection LF;
}

void loop()
{

    if (main_loop_counter % printfreq == 0){Serial.println("Loop: " + String(main_loop_counter) + " ------------------------");}

    MovementControl MC;
    LFDetection LF;

    // Serial.println(R2Queue.itemCount(), L2Queue.itemCount());

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

    if (task == 3){
        search_time = current_time;
        
        MC.SEARCH();
    }

    delay(main_loop_delay_time);
    main_loop_counter++;
    if (main_loop_counter % printfreq == 0){
    Serial.println(" ");}

}
