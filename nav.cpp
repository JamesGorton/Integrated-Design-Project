Class LFDetection
{
  public:
      int LLF_anode; // LF output for High Voltage
      int RLF_anode;
      
      int LLF_collector; // LF input for LF_data. 2 LF in black area.
      int RLF_collector;
      
      int LLF_data; // LF data: 1 for black, 0 for white.
      int RLF_data;
      
      int Turn = 0;
      
      int PID;
      int P, I, D;
      int kp, ki, kd;
      int ref; // reference position light intensity.
      
      int pre_I, pre_P;
      int prev_time;
      
      void LFDataRead(void);
      void TurnDetection(void);
      void IntersectionDetection(void);
      void PID(void);
}

void LFDetection::PID(ref, real, kp, ki, kd)
{
    P = ref - real;
    I = pre_I + P * iter_time;
    D = (pre_P - P) / iter_time;
    current_time = time();
    prev_time = current_time;
    pre_I = I;
    pre_P = P;
    return P * kp + I * ki + D * kd;
}

void LFDetection::LFDataRead()
{
    LLF_data = digitalRead(LLF_collector);
    RLF_data = digitalRead(RLF_collector);
}

void LFDetection::TurnDetection()
{
    if((LLF_data==0 && RLF_data==1)==1){
      Turn = 1; // turn left
    }
    else if((RLF_data==0 && LLF_data==1)==1){
      Turn = 2; // turn right
    } 
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
     
      int turn_delay;
      
      int speedL;
      int speedR;
      int maxspeed;
      int ref_speed; // forward speed.
      
      void TURN (void);
      void MOVE (void);
      void STOP (void);
}

void MovementControl::TURN()
{
  if (Turn==0){
      pass;
  }
  
  else {
      if (Turn==1){
          analogWrite(motL,0);
          analogWrite(motR,60);
          while(LLF_data==0){
            LFDataRead();
          }
      } 
      else if (Turn==2){
          analogWrite(motL,60);
          analogWrite(motR,0);
          while(RLF_data==0){
            LFDataRead();
          }
      } 

      delay(turn_delay);
  
      analogWrite(motL,255);
      analogWrite(motR,255);
      
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
  
  analogWrite(motL,speedL);
  analogWrite(motR,speedR);
  
  while(1){
    LFDataRead();
    TurnDetection();
    TURN();
  }
}

void MovementControl::STOP()
{
  analogWrite(motL,0);
  analogWrite(motR,0);
}

void setup()
{
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
