#define debug true
#include <GtGiro.h>
#include <MCP4725.h>
#include <Wire.h>
#include <GtHeartBeat.h>
#ifdef ON_ROBOT
  GtGiro giro(0x69, 2000, 30);
#else
  GtGiro giro(0x68, 2000, 30);
#endif
MCP4725 dac;

int ledFR1 = 9;
int ledFR0 = 5;
int ledFL1 = 6;
int ledFL0 = 3;
int ledBL1 = 11;
int ledBL0 = 13;
int ledBR1 = 12;
int ledBR0 = 10;

int x = 0;
int y = 0;
int s = 0;

void setup() {
  // initialize serial communication
  Serial.begin(9600);
  Serial.println("Setup");
  
  #ifdef ON_ROBOT
    dac.begin(0x60);
  #endif
  
  giro.setup();

  pinMode(ledFR1, OUTPUT);
  pinMode(ledFR0, OUTPUT);
  pinMode(ledFL1, OUTPUT);
  pinMode(ledFL0, OUTPUT);
  pinMode(ledBL1, OUTPUT);
  pinMode(ledBL0, OUTPUT);
  pinMode(ledBR1, OUTPUT);
  pinMode(ledBR0, OUTPUT);

}
long lastPrintMillis = 0;
void giroLoopCallPrint(){
  if (giro.readGiroLoopCall()){
    #ifdef ON_ROBOT
      dac.setVoltage(giro.getAdjustedHysteresisYawAsDacValue(0, 4095), false);
    #endif
    
    long currentMillis = millis();
    if (currentMillis - lastPrintMillis > 500){
        lastPrintMillis = currentMillis;
        Serial.print("Yaw\t");
        Serial.print(giro.getYaw());
        Serial.print("\tPitch\t");
        Serial.print(giro.getPitch());
        Serial.print("\tRoll\t");
        Serial.print(giro.getRoll());
        Serial.println();
     }
  }
}

void setX(int roll){
  if (roll >= 15){
    x = ((roll - 15) / 30); // x => [0,1]
  } else if (roll <= -15){
    x = ((roll + 15) / 30); // x => [-1,0]
  }
}
void setY(int pitch){
  if (pitch >= 15){
    y = ((pitch - 15) / 30); // y => [0,1]
  } else if (pitch <= -15){
    y = ((pitch + 15) / 30); // y => [-1,0]
  }
}
void setS(int spin){
  if (spin >= 15){
    y = ((spin-15)/30); // y => [0,1]
  } else if (spin <= -15){
    y = ((spin+15)/30); // y => [-1,0]
  }
}
int i = 0;
void loop() {
  giroLoopCallPrint();

  setX(giro.getRoll());
  setY(giro.getPitch());
  
  motorFLPower = x + y - s;
  motorBLPower = -x + y - s;
  motorFRPower = x - y - s;
  motorBRPower = -x - y - s;


  
  delay(20);
  
//  i++;
//  if (i > 500){
//    i = 0;
//    giro.resetAdjustedYaw();
//  }
}
