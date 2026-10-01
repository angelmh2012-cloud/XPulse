#include <xmotionV3.h>
// P & D Variables
  #define Kp 1.35//1.5
  #define Kd 30 // 55
// Speed Limits
int LimitMin, LimitMax;
// Speed Variables
#define Speedinit 50
#define SpeedPlus 20


//#define DoorSens 2 

// Xline Pins
#define Sens A1
#define s0 A2
#define s1 A4
#define s2 A5
#define s3 4
// Motor Driver Pins
#define LPWM 10 // For XMotion V3 it will be 10 For XMotion V2 it is 3
#define LDIR 12
#define RPWM 11
#define RDIR 13
// Trimpot, Button, Dipswitch & Leds
#define TMP A3
#define MicroStart A0 // For XMotion V3 it will be A0 For XMotion V2 it is 10
#define DS1 5
#define DS2 6
#define DS3 7
#define LLED 8
#define RLED 9

// Other Variables
int TrimSpeed, ExtraSpeed, BaseSpeed, Error, LastError;
int SensNo;
int value;
int Sensed=0;
int MuxTime=200; // Delay time at MicroSeconds
float Correction, LSpeed, RSpeed;
float Old_LSpeed,Old_RSpeed;
int  ll, rr;
bool FloorColor=0;


void setup() {
  // User Inputs
  pinMode(MicroStart, INPUT_PULLUP);
  pinMode(DS1, INPUT);
  pinMode(DS2, INPUT);
  pinMode(DS3, INPUT);
  
 
 //pinMode(DoorSens, INPUT); 

  // Xline Pin IO
  pinMode(Sens, INPUT);  
  pinMode(s0,  OUTPUT);
  pinMode(s1,  OUTPUT);
  pinMode(s2,  OUTPUT);
  pinMode(s3,  OUTPUT);
  // Motor Driver Pins Output
  pinMode(RDIR, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LDIR, OUTPUT);
  pinMode(LPWM, OUTPUT);
  // Leds Output
  pinMode(LLED, OUTPUT);
  pinMode(RLED, OUTPUT);
 // Dipswitch & Button Pull-Up
  digitalWrite(DS1, HIGH); 
  digitalWrite(DS2, HIGH);
  digitalWrite(DS3, HIGH);
 
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  TrimSpeed = 20; //(Speedinit + (analogRead(TMP) / 5)); // Between 50 to 255
  ExtraSpeed = 20; //TrimSpeed + SpeedPlus;
  if (ExtraSpeed > 255) {ExtraSpeed=255; }
  BaseSpeed = 20; //TrimSpeed; 
  
  LimitMax = 40; //ExtraSpeed+40;
    if (LimitMax > 255) {LimitMax=255; }
  LimitMin = -40; //-LimitMax;
  
  Serial.begin(9600);  // start serial communication for debugging
}

void loop() {    

  // Start butonu kontrolü
  while (digitalRead(MicroStart)==0) {
    MOVE(0, 0, 1);
  }


  //while (digitalRead(DoorSens) == 1) {
   //MOVE(0, 0, 1);
 //}

  SensCall(0); 
  ll = digitalRead(Sens);
  SensCall(14); 
  rr = digitalRead(Sens);

  if(ll==0&&rr==0){FloorColor=0;}
  else if(ll==1&&rr==1){FloorColor=1;}

  while (digitalRead(DS3) == LOW) {
    MOVE(0, 0, 1);
    TestSensor();
  }  
 
  ReadSensor();
  Correction = Kp * Error + Kd * (Error - LastError); 
  LastError = Error;
  LSpeed = BaseSpeed + Correction; 
  RSpeed = BaseSpeed - Correction; 
  LSpeed = constrain(LSpeed, LimitMin, LimitMax);
  RSpeed = constrain(RSpeed, LimitMin, LimitMax);
  
  if (digitalRead(DS2) == LOW) {
    MOVE(0, 0, 1);
    Debug();
  } else {
    if (LSpeed != Old_LSpeed || RSpeed != Old_RSpeed) {
      MOVE(LSpeed, RSpeed, 1);
    }
    Old_LSpeed=LSpeed;
    Old_RSpeed=RSpeed;
  }
}

void Debug() {
  Serial.print(" Kp:"); Serial.print(Kp); 
  Serial.print(" Kd:"); Serial.print(Kd);
  Serial.print(" LimitMax:"); Serial.print(LimitMax);
  Serial.print(" TrimSpeed:"); Serial.print(TrimSpeed);
  Serial.print(" ExtraSpeed:"); Serial.print(ExtraSpeed);
  Serial.print(" BaseSpeed:"); Serial.print(BaseSpeed); Serial.print(" ");
  for (int j = 0; j <= 15; j++) {
    SensCall(j); 
  }
  Serial.print(" Error:"); Serial.print(Error);
  Serial.print(" LastError:"); Serial.print(LastError);
  Serial.print(" Correction:"); Serial.print(Correction);
  Serial.print(" LeftSpeed:"); Serial.print(LSpeed);
  Serial.print(" RightSpeed:"); Serial.println(RSpeed);
  delay(100);
}

void TestSensor() {
  Serial.print("Line Sensors:");
  for (int j = 14; j >= 0; j--) { // J is Sensor Number
    SensCall(j); Serial.print(" "); Serial.print(j); Serial.print(":"); Serial.print(digitalRead(Sens));
  }
  Serial.print("  Center Back Sensor 15:"); 
  SensCall(15); // Here 15 is middle back line sensor
  Serial.println(digitalRead(Sens));
  delay(100);
}

void SW(bool SW0, bool SW1, bool SW2, bool SW3) {
  digitalWrite(s0, SW0); digitalWrite(s1, SW1); digitalWrite(s2, SW2); digitalWrite(s3, SW3);
}

void SensCall(int i) {
  if (i == 0){ SW(0,0,0,0); }
  else if (i == 1){ SW(1,0,0,0); }
  else if (i == 2){ SW(0,1,0,0); }
  else if (i == 3){ SW(1,1,0,0); }
  else if (i == 4){ SW(0,0,1,0); }
  else if (i == 5){ SW(1,0,1,0); }
  else if (i == 6){ SW(0,1,1,0); }
  else if (i == 7){ SW(1,1,1,0); }
  else if (i == 8){ SW(0,0,0,1); }
  else if (i == 9){ SW(1,0,0,1); }
  else if (i == 10){ SW(0,1,0,1); }
  else if (i == 11){ SW(1,1,0,1); }
  else if (i == 12){ SW(0,0,1,1); }
  else if (i == 13){ SW(1,0,1,1); }
  else if (i == 14){ SW(0,1,1,1); }
  else if (i == 15){ SW(1,1,1,1); }
  delayMicroseconds(MuxTime);
}

void ReadSensor() {
  Sensed=0;
  if (FloorColor==0) {
    for (int i = 0; i < 15; i++) {
      SensCall(i); if (digitalRead(Sens) == 1) {
      Sensed=1;
        if (i == 7) {
          Error = 0; SensCall(15); if (digitalRead(Sens) == 1) {
            BaseSpeed = ExtraSpeed;
          } 
        } else if (i == 8)Error = -30;
        else if (i == 6)Error = 30;
        else if (i == 9)Error = -70;
        else if (i == 5)Error = 70;
        else if (i == 10)Error = -85;
        else if (i == 4)Error = 85;
        else if (i == 11)Error = -120;
        else if (i == 3)Error = 120;
        else if (i == 12)Error = -150;
        else if (i == 2)Error = 150;
        else if (i == 13)Error = -180;
        else if (i == 1)Error = 180;
        else if (i == 14)Error = -220;
        else if (i == 0)Error = 220;
      }
    }
    if (Sensed==0) {
      BaseSpeed=30;
    } else {  
      BaseSpeed = TrimSpeed; 
    }   
  } else {
    for (int i = 0; i < 15; i++) {
      SensCall(i); if (digitalRead(Sens) == 0) {
        if (i == 7) {
          Error = 0; SensCall(15); if (digitalRead(Sens) == 0) {
            BaseSpeed = ExtraSpeed;
          } 
        } else if (i == 8)Error = -30;
        else if (i == 6)Error = 30;
        else if (i == 9)Error = -80;
        else if (i == 5)Error = 80;
        else if (i == 10)Error = -100;
        else if (i == 4)Error = 100;
        else if (i == 11)Error = -150;
        else if (i == 3)Error = 150;
        else if (i == 12)Error = -220;
        else if (i == 2)Error = 220;
        else if (i == 13)Error = -350;
        else if (i == 1)Error = 350;
        else if (i == 14)Error = -490;
        else if (i == 0)Error = 490;
      }
    }
  }
}

void MOVE(float LeftSpeed, float RightSpeed, int timex) {
  if (LeftSpeed <= 0) {
    LeftSpeed = abs(LeftSpeed); digitalWrite(LDIR, LOW);
  } else digitalWrite(LDIR, HIGH);
  analogWrite(LPWM, LeftSpeed);
  
  if (RightSpeed <= 0) {
    RightSpeed = abs(RightSpeed); digitalWrite(RDIR, LOW);
  } else digitalWrite(RDIR, HIGH);
  analogWrite(RPWM, RightSpeed);
  delay(timex);
}