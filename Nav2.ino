#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ArduinoQueue.h>

#define trigPin 13
#define echoPin 12

// fail safe (0.5s) lag for 180 turn
// stray protection
// button for restart
// targets occupied

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

#include <Servo.h>
Servo myservo;  // create servo object to control a servo
Servo myservo2;

// Line deviate failsafe
int stray_left;
const int stray_threshold = 20;

const int servo1pin = 0;
const int servo2pin = 0;

// Motor shield motor pins.
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(3);

const int LED; // To flash at 2Hz.
// Line sensor data receive pins.
const int L2LF_receive = 10;
const int L1LF_receive = 8;
const int R1LF_receive = 11;
const int R2LF_receive = 12;

// Tunable Parameters.
const long before_search_lf_period = 2000;

const float kp = 30; // Proportional gain.
const float ki = 0; // Integral gain.
const float kd = 0; // Derivative gain.

const int main_loop_delay_time = 50; // main loop delay.
const int print_freq = 500 / main_loop_delay_time; // Print every 0.5s.
const int delay_time = 100; // Misc delay. // No use?
const int max_speed = 255; // Maximum allowable motor speed.
const int ref_speed = 200; // Normal forward motor speed.
const int turn_speed = 200; // Turning speed.

const int intxn_queue_length = 5; // intersection queue length.
const int intxn_detection_threshold = 3; // IntersectionDetection Threshold, the number of 1s in intersection queue.
const int intxn_deb_time = 1000; // Intersection debounce threshold time.
unsigned long l_intxn_deb_prev = 0; // Last Left intersection debounce start time.
unsigned long r_intxn_deb_prev = 0; // Last Right intersection debounce start time.
int l_intxn_deb = 1; // Intersection debounce checker. 1 for debounce finished.
int r_intxn_deb = 1;

// START BUTTON
const int buttonPin;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status

// TASK MANAGER
int task = 0;

int block_found = 0;
int block_approached = 0;
int block_color = 0; // 0 for blue, 1 for red.
int block_picked = 0;
int retreated_with_block = 0;
int block_placed = 0;
int block_number = 1;

int journey = 0; // 0 for go to, 1 for return

/*
   Task 0: Starting move forward
   Task 1: Line follow
   Task 2: Search turn
   Task 3: Approach
   Task 4: Pick up
   Task 5: Retreat
   Task 6: (180+search_turn) turn
   // Task 7: Search turn reverse;
*/

// Search queues. Search queue detects sudden drop only.
const int sweep_queue_length = 5; // Distance sensor sweep queue length.
const int front_queue_length = 5;
const int back_queue_length = 5;
const float dip_threshold = 10;
float front_avg = 0;
float back_avg = 0;
ArduinoQueue<int> Q = ArduinoQueue<int>(sweep_queue_length);
ArduinoQueue<int> F = ArduinoQueue<int>(front_queue_length);
ArduinoQueue<int> B = ArduinoQueue<int>(back_queue_length);
float front_front;
float mid_front;
float back_front;

// PID parameters.
float PIDError = 0; // PID control feedback
float P, I, D;
float pre_I = 0;
float pre_P = 0;
int speedL;
int speedR;
int prev_speedL;
int prev_speedR;

unsigned long main_loop_counter = 0;
unsigned long current_time = 0;
unsigned long PID_prev_time = 0; // For Pi, no use actually.
unsigned long prev_blink_time = 0;

int approach_time; // calculated time to approach and retreat from block.

int Turn = 0;

// record intersections encountered.
int left_intxn_counter = 0;
int right_intxn_counter = 0;
int left_white_counter = 0;
int right_white_counter = 0;

// Line sensor data. 0 (black) or 1 (white).
int L2LF_data;
int L1LF_data;
int R1LF_data;
int R2LF_data;

ArduinoQueue<int> L2Queue = ArduinoQueue<int>(intxn_queue_length);
ArduinoQueue<int> R2Queue = ArduinoQueue<int>(intxn_queue_length);

// DistanceIR params.
const int DS_receive =A1;
float sensorVal = 0;
float sensorVolt = 0;
float Vr = 5.0;
float sum = 0;
float k1 = 16.7647563;
float k2 = -0.80803107;
float DS_data; // Distance Sensor data.
float block_distance;

class LFDetection
{
public:
    void LFDataRead(void); // Read data from line sensors.
    void DSDataRead(void); // Read data from distance sensors.
    void EdgeDetection(void); // Detect horizontal white strips.
    void IntersectionDetection(void); // Count the number of intersections encountered.
    void BlockDetection(void);
    void Color(void);
    void Ultrasound(void);

};

void LFDetection::Color(void){
    int led = 2;
    int led2 = 3;
    // read the input on analog pin 0:
    int sensorValue = analogRead(A4);
    int sensorValue2 = analogRead(A5);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float voltage = sensorValue * (5.0 / 1023.0);
    float voltage2 = sensorValue2 * (5.0 / 1023.0);
    float difference = voltage - voltage2;
    // print out the value you read:
    Serial.println("V1: " + String(voltage));
    Serial.println("V2:" + String(voltage2));
    Serial.println("difference: " + String(difference));

    if (difference < 0.5){
      digitalWrite(led, HIGH);
      digitalWrite(led2, HIGH);
      Turn = 1; // Testing, to be deleted.
    }

    else if (difference < 1) {
      digitalWrite(led, LOW); // turn Green LED on
      digitalWrite(led2, HIGH); // Red LED off
      block_color = 0;
      Turn = 1; // left
    }
    else if (difference > 1){
      digitalWrite(led, HIGH); // Green LED off
      digitalWrite(led2, LOW); // Red LED on
      block_color = 1;
      Turn = 2;
    }
    delay(5000);
    return;
}

void LFDetection::BlockDetection(){
  // Fill three queues
  // F----- Q ---------- B-----
  while (!F.isFull()){
    DSDataRead();
    F.enqueue( DS_data );
    front_avg += DS_data / front_queue_length;
  }
  while (!Q.isFull()){
    DSDataRead();
    Q.enqueue( DS_data );
  }
  while (!B.isFull()){
    DSDataRead();
    B.enqueue( DS_data );
    back_avg += DS_data / back_queue_length;
  }

    DSDataRead();
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

    // Serial.println("Front Average: " + String(front_avg));
    // Serial.println("Back  Average: " + String(back_avg));

    if (front_avg - back_avg > dip_threshold){
      block_distance = DS_data;
      Serial.println("Block Found.");
      block_found = 1;
    }
}

void LFDetection::LFDataRead()
{
    L1LF_data = digitalRead(L1LF_receive);
    R1LF_data = digitalRead(R1LF_receive);
    L2LF_data = digitalRead(L2LF_receive);
    R2LF_data = digitalRead(R2LF_receive);
    if (R2LF_data > L2LF_data){
        stray_left++;
    }
    if (R2LF_data < L2LF_data){
        stray_left--;
    }
    if (R2LF_data = L2LF_data){
        if (stray_left < 0){stray_left++;}
        if (stray_left > 0){stray_left--;}
    }
    /*
    if (main_loop_counter % print_freq == 0){
      Serial.println("Sensor Data 1 2: " + String(L1LF_data) + " " + String(R1LF_data));
    }
    */
}

void LFDetection::DSDataRead()
{
    DS_data = digitalRead(DS_receive);
    sum=0;
    for (int i=0; i<100; i++)
    {
    sum=sum+float(analogRead(DS_receive));
    }
    sensorVal=sum/100;
    sensorVolt=sensorVal*Vr/1024;

    DS_data = pow(sensorVolt*(1/k1), 1/k2);
    if (main_loop_counter % print_freq == 0){
      Serial.println("Distance sensor: " + String(DS_data));
    }
}

void LFDetection::EdgeDetection() // Testing using L1 and R1.
{
    int Ldeq = L2Queue.dequeue();
    int Rdeq = R2Queue.dequeue();
    int L2enqueued = 0;
    int R2enqueued = 0;
    if (l_intxn_deb == 0){ // Debounce not finished.
        L2enqueued = 0;
        L2Queue.enqueue(L2enqueued);
    }
    else{
        L2Queue.enqueue(L2LF_data); // to be changed to L2

        L2enqueued = L2LF_data;
    }
    if (r_intxn_deb == 0){ // Debounce not finished.
        R2enqueued = 0;
        R2Queue.enqueue(R2enqueued);
    }
    else{
        R2Queue.enqueue(R2LF_data); // to be changed to R2
        R2enqueued = R2LF_data;
    }

    if (L2enqueued == 1){
        left_white_counter++;
        // Serial.println("L1 ENqueued 1");
    }
    if (Ldeq == 1){
        left_white_counter--;
        // Serial.println("L1 DEqueued 1");
    }
    if (R2enqueued == 1){
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

void LFDetection::Ultrasound(){
    long duration, distance;
    digitalWrite(trigPin, LOW); // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(trigPin, HIGH);
    // delayMicroseconds(1000); - Removed this line
    delayMicroseconds(10); // Added this line
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration/2) / 29.1;
    //if (distance < 4) { // This is where the LED On/Off happens
    //digitalWrite(led,HIGH); // When the Red condition is met, the Green LED should turn off
    //digitalWrite(led2,LOW);
    //}
    //else {
    //digitalWrite(led,LOW);
    //digitalWrite(led2,HIGH);
    //}
    //if (distance >= 200 || distance <= 0){
    //Serial.println(“Out of range”);
    //}
    //else {
    Serial.println(distance);
    //Serial.println(“ cm”);
    //}
    delay(500);
}

// ******************************************

class ServoMove{
public:
    void Pickup();
    void Place();
    void foo();
    void bar();
    int pos;
};
void ServoMove::foo(){
    int potpin = 0;  // analog pin used to connect the potentiometer
    int val;    // variable to read the value from the analog pin
    val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo.write(val);                  // sets the servo position according to the scaled value
    delay(15);                           // waits for the servo to get there
}
void ServoMove::bar(){
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
}
void ServoMove::Pickup(){
    block_picked = 1;
    return;
}
void ServoMove::Place(){
    block_placed = 1;
    return;
}

class MovementControl: public LFDetection, public ServoMove
{
  public:
      void FindTask(void);

      void DummyMove(void);

      void PID(void);
      void LineFollow(void);

      void Search(void);
      void Approach(void);
      void Pickup(void);
      void Retreat(void);
      void HardTurn(void);
      void LineFindTurn(void);

      void HardDelivery(void);
      void LFDelivery(void);
      void Reset(void);

      void Blink(void);
      void Stop(void);

      void Stray(void);
};

void MovementControl::Stray(void){
    if ((stray_left <= stray_threshold) && (stray_left >= (-stray_threshold))){
        return;
    }
    LeftMotor -> setSpeed(0);
    RightMotor -> setSpeed(0);
    if (stray_left > stray_threshold){
        Serial.println("STRAYED LEFT!");
        RightMotor->run(BACKWARD);
        RightMotor->setSpeed(ref_speed);
        while(L1LF_data != 1){
            L1LF_data = digitalRead(L1LF_receive);
            R1LF_data = digitalRead(R1LF_receive);
            L2LF_data = digitalRead(L2LF_receive);
            R2LF_data = digitalRead(R2LF_receive);
        }
        stray_left = 0;
    }
   
    if (stray_left < (-stray_threshold)){
        Serial.println("STRAYED RIGHT!");
        LeftMotor->run(BACKWARD);
        LeftMotor->setSpeed(ref_speed);
        while(R1LF_data != 1){
            L1LF_data = digitalRead(L1LF_receive);
            R1LF_data = digitalRead(R1LF_receive);
            L2LF_data = digitalRead(L2LF_receive);
            R2LF_data = digitalRead(R2LF_receive);
        }
        stray_left = 0;
    }
    LeftMotor -> setSpeed(0);
    RightMotor -> setSpeed(0);
    Serial.println("Stray corrected.");
    delay(500);
}

void MovementControl::HardTurn(void){
    if (Turn == 1)  //LEFT
    {
        LeftMotor->run(BACKWARD);
        LeftMotor->setSpeed(turn_speed);
        RightMotor->run(FORWARD);
        RightMotor->setSpeed(turn_speed);
        delay(2000); // turn time
    }
    if (Turn == 2) // RIGHT
    {
        LeftMotor->run(FORWARD);
        LeftMotor->setSpeed(turn_speed);
        RightMotor->run(BACKWARD);
        RightMotor->setSpeed(turn_speed);
        delay(2000);
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);

}

void MovementControl::LFDelivery(void){
    long before_delivery_lf_time = millis();
    int before_delivery_lf_period = 1000; // 1 more second before stop.
    int delivery_time = 3000;
    while (millis() - before_delivery_lf_time <= before_delivery_lf_period){
        LineFollow();
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);

    LineFindTurn();
    int current_left_intxn = left_intxn_counter;
    int current_right_intxn = right_intxn_counter;
    while(current_left_intxn == left_intxn_counter && current_right_intxn == right_intxn_counter){
        LineFollow();
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    // minor adjustments
    Place();
}

void MovementControl::HardDelivery(void){
    // This would be much better done with line follower.
    long before_delivery_lf_time = millis();
    int before_delivery_lf_period = 1000; // 1 more second before stop.
    int delivery_time = 3000;
    while (millis() - before_delivery_lf_time <= before_delivery_lf_period){
        LineFollow();
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    HardTurn();
    // move and place.
    long second_stretch_start_time = millis();
   
    LeftMotor->setSpeed(ref_speed);
    RightMotor->setSpeed(ref_speed);
    while (millis() - second_stretch_start_time < delivery_time){
        pass;
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    Place();
    block_placed = 1;
}

void MovementControl::Reset(void){
    // return move.

    block_found = 0;
    block_approached = 0;
    block_picked = 0;
    retreated_with_block = 0;
    journey = 0;
    left_intxn_counter = 1;
    right_intxn_counter = 1;
    block_number++;

    Serial.println("Ready for new run.");

}

void MovementControl::Approach(void){
    
    int k = 1000; // to be tuned.
    approach_time = k * (block_distance / ref_speed);
    approach_time = 2000; // for testing, to be deleted.
    if (block_number == 1){
        approach_time = 2000;
    }
    long approach_start_time = millis();
   
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
    LeftMotor->setSpeed(ref_speed);
    RightMotor->setSpeed(ref_speed);
    while (millis() - approach_start_time <= approach_time){        
        Blink();
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    delay(200);
    Color();
    block_approached = 1;
}

void MovementControl::Retreat(void){
    long retreat_start_time = millis();
   
    LeftMotor->run(BACKWARD);
    RightMotor->run(BACKWARD);
    LeftMotor->setSpeed(ref_speed);
    RightMotor->setSpeed(ref_speed);
    while (millis() - retreat_start_time <= approach_time){
        Blink();
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    retreated_with_block = 1;
}

void MovementControl::Pickup(void){
    // Servo move
    block_picked = 1;
}

void MovementControl::Blink(void){
  if (millis() - prev_blink_time > 500){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
    prev_blink_time = millis();
  }
}

void MovementControl::FindTask(){
    // LFDataRead();
    // EdgeDetection();
    // IntersectionDetection();

    /*
    Task 0: Starting move forward
    Task 1: Line follow
    Task 2: Search turn
    Task 3: Approach
    Task 4: Pick up
    Task 5: Retreat
    Task 6: 180 turn
    Task 7: Delivery
    Task 8: Reset
    */

    if (left_intxn_counter == 0 && right_intxn_counter == 0){
        task = 0;
        if (main_loop_counter % print_freq == 0){
        Serial.println("Dummy Move Forward."); }// "Starting Dummy Move Forward.");
    }

    if ( (1 <= right_intxn_counter <= 2 || 1 <= left_intxn_counter <= 2) && block_found == 0){
        task = 1;
        if (main_loop_counter % print_freq == 0){
        Serial.println("Line Following.");}
    }

    if ((left_intxn_counter == 3 || right_intxn_counter == 3) && block_found == 0){
        task = 2;
        //if (main_loop_counter % print_freq == 0){
        Serial.println("Searching.");}
    //}

    if (block_found == 1 && block_approached == 0
        && block_picked == 0 && retreated_with_block == 0){
        task = 3;

        Serial.println("Approaching block.");
    }

    if (block_found == 1 && block_approached == 1
        && block_picked == 0 && retreated_with_block == 0){
        task = 4;
        Serial.println("Picking up block.");
    }

    if (block_found == 1 && block_approached == 1
        && block_picked == 1 && retreated_with_block == 0 && journey == 0){
        task = 5;
        Serial.println("Retreating with block.");
    }

    if (block_found == 1 && block_approached == 1
        && block_picked == 1 && retreated_with_block == 1 && journey == 0){
        task = 6; // turn around
        Serial.println("Starting 180+.");
    }

    if (block_found == 1 && block_approached == 1
        && block_picked == 1 && retreated_with_block == 1 && journey == 1){
        task = 1; // Return line follow.
        if (main_loop_counter % print_freq == 0){
            Serial.println("Returning line follow.");
        }
    }

    if ((left_intxn_counter == 5 || right_intxn_counter == 5)){
        task = 7;
        Serial.println("Manuver to target.");
    }

    if (block_placed == 1){
        task = 8;
        Serial.println("Return and reset.");
    }


}

void MovementControl::Search(){
    if (block_number == 1){
        block_found = 1;
        Serial.println("First block.");
        return;
    }
    long before_search_lf_time = millis();
    int before_search_lf_period = 1500;
    while (millis() - before_search_lf_time < before_search_lf_period){
        LineFollow();
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    delay(500);

    LeftMotor->run(RELEASE);
    RightMotor->run(RELEASE);

    long search_start_time = millis();

    LeftMotor->run(BACKWARD);
    LeftMotor->setSpeed(200);
    while(millis() - search_start_time < 1000 && block_found != 1){
         // BlockDetection(); // No detect when angle increase.
         Blink();
    }

    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    if (block_found == 1){ // this should not execute
        Serial.println("Found at increasing angle, WRONG.");
        return;
    }


    delay(500);
    LeftMotor->run(FORWARD);
    LeftMotor->setSpeed(200);
    while (millis() - search_start_time < 2500 && block_found != 1){
        BlockDetection();
        Blink();
    }

    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);

    if (block_found == 1){
        Serial.println("Found at left half box.");
        return;
    }

    delay(500);

    RightMotor->run(BACKWARD);
    RightMotor->setSpeed(200);
    while (millis() - search_start_time < 4000 && block_found != 1){
        BlockDetection();
        Blink();
    }

    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    
    if (block_found == 1){
        Serial.println("Found at increasing angle, WRONG.");
        return;
    }
    
    delay(500);
    RightMotor->run(FORWARD);
    RightMotor->setSpeed(200);
    while (millis() - search_start_time < 5500 && block_found != 1){
        BlockDetection();
        Blink();
    }

    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);

    if (block_found == 1){
        Serial.println("Found at right half box.");
        return;
    }

    if (block_found == 0){
        Serial.println("Didn't find.");
    }
    delay(5000);

    block_found = 1; // To be deleted.

}

void MovementControl::LineFindTurn(){
  if (Turn == 1)  //LEFT
  {
      LeftMotor->run(BACKWARD);
      LeftMotor->setSpeed(turn_speed);
      RightMotor->run(FORWARD);
      RightMotor->setSpeed(turn_speed);
      delay(500); // 180 turnaround guarantee.
      while(R1LF_data != 1){
        LFDataRead();
      }
  }
  if (Turn == 2) // RIGHT
  {
          LeftMotor->run(FORWARD);
          LeftMotor->setSpeed(turn_speed);
          RightMotor->run(BACKWARD);
          RightMotor->setSpeed(turn_speed);
          delay(500); // 180 turnaround guarantee.
          while(L1LF_data != 1){
            LFDataRead();
          }
  }
  LeftMotor->setSpeed(0);
  RightMotor->setSpeed(0);

  if (journey == 1){
      journey = 0;
  }
  else if (journey == 0){
      journey = 1;
  }
  Serial.println("Turn complete. Journey flipped.");
}

void MovementControl::LineFollow(){
    LFDataRead();
    EdgeDetection();
    IntersectionDetection();
    Stray();
    PID();
    if (main_loop_counter % print_freq == 0){
        // Serial.println("Sensor 1 2: " + String(L1LF_data) + " " + String(R1LF_data));
        // Serial.println("PID value: "+ String(PIDError));
        Serial.println("Motor speed L R: " + String(speedL) + " "
         + String(speedR));
    }
    // delay(delay_time);
}

void MovementControl::PID(){ // only a simple binary controller in the end.

    P = (L1LF_data - R1LF_data);
    I = pre_I + P * 0.001 * (current_time - PID_prev_time);
    PIDError = P * kp + I * ki;
    pre_I = I;
    PID_prev_time = current_time;

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
  RightMotor->run(FORWARD);

  if (speedL != prev_speedL){
    LeftMotor->setSpeed(speedL);
    prev_speedL = speedL;
  }
  if (speedR != prev_speedR){
    RightMotor->setSpeed(speedR);
    prev_speedR = speedR;
  }
}

void MovementControl::DummyMove(){
    LFDataRead();
    EdgeDetection();
    IntersectionDetection();
    LeftMotor->run(FORWARD);
    LeftMotor->setSpeed(ref_speed);
    RightMotor->run(FORWARD);
    RightMotor->setSpeed(ref_speed);
}

void MovementControl::Stop(){
  LeftMotor->run(FORWARD);
  LeftMotor->setSpeed(0);
  RightMotor->run(FORWARD);
  RightMotor->setSpeed(0);
}

// constants won't change. They're used here to set pin numbers:



void setup()
{
  Serial.begin(9600);
  Serial.println("Testing START!");
  AFMS.begin();

  for (int i=0;i<intxn_queue_length;i++)
  {
    L2Queue.enqueue(0);
    R2Queue.enqueue(0);
  }
   
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);

   
  pinMode(LED, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(L2LF_receive,INPUT);
  pinMode(L1LF_receive,INPUT);
  pinMode(R1LF_receive,INPUT);
  pinMode(R2LF_receive,INPUT);
  pinMode(DS_receive, INPUT);

  myservo.attach(servo1pin);
  myservo2.attach(servo2pin);

}

void loop()
{
    while(true){
      buttonState = digitalRead(buttonPin);
      Serial.println(buttonState);
      if (buttonState == 1) {
            break;
      }
      delay(100);
    }

    
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    if (main_loop_counter % print_freq == 0){Serial.println("Loop: " + String(main_loop_counter) + " ------------------------");}

    MovementControl MC;
    LFDetection LF;

    current_time = millis();

    MC.FindTask();
    MC.Blink();

    switch ( task ){
        case 0:
            MC.DummyMove();
            break;
        case 1:
            MC.LineFollow();
            break;
        case 2:
            MC.Search();
            break;
        case 3:
            MC.Approach();
            break;
        case 4:
            MC.Pickup();
            break;
        case 5:
            MC.Retreat();
            break;
        case 6:
            MC.LineFindTurn();
            break;
        case 7:
            MC.HardDelivery();
            //MC.LFDelivery();
            break;
        case 8:
            MC.Reset();
            break;
    }

    delay(main_loop_delay_time);
    main_loop_counter++;
    if (main_loop_counter % print_freq == 0){
    Serial.println(" ");}

}
