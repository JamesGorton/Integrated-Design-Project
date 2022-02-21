#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *LeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(1);

void setup() {
  //Motor Setup
 Serial.begin(9600);
 Serial.println("Adafruit Motorshield v2 - DC Motor test!");
 AFMS.begin();  // create with the default frequency 1.6KHz
 LeftMotor->setSpeed(150);
 LeftMotor->run(FORWARD);
 LeftMotor->run(RELEASE);
 //Blink Setup
 pinMode(LED_BUILTIN, OUTPUT);
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
