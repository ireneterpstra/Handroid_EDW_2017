#define debug true


#include <GtGiro.h>
#include <MCP4725.h>
#include <Wire.h>
#include <GtHeartBeat.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

//#define ON_ROBOT


#ifdef ON_ROBOT
  GtGiro giro(0x69, 2000, 30);
  //GtGiro giro(0x69, 1000); //fifo-reset every 1 sec
#else
  //GtGiro giro(0x68);
  GtGiro giro(0x68, 2000, 30);
#endif

MCP4725 dac;

GtHeartBeat heartBeat(13, 1000, 10);//on-board LED on port 13, 10 ms every 2 sec

void setup() {
  
  // initialize serial communication
    Serial.begin(9600);
    Serial.println("Setup");
    
    #ifdef ON_ROBOT
      dac.begin(0x60);
    #endif
    
    giro.setup();
    
    //pinMode(13, OUTPUT);
    heartBeat.setup();
}

long lastPrintMillis = 0;
void giroLoopCallPrint(){
  if (giro.readGiroLoopCall()){
    #ifdef ON_ROBOT
      //dac.setVoltage(giro.getAdjustedYawAsDacValue(0, 4095), false);
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
        Serial.print("\tStable\t");
        Serial.print(giro.isDoubleStableYaw());
        Serial.print("\tInstableCount\t");
        Serial.print(giro.getInstableFifoResetCount());
        Serial.print("\taYaw\t");
        Serial.print(giro.getAdjustedYaw());
        Serial.print("\thMode\t");
        Serial.print(giro.getHysteresisMode());
        Serial.print("\thYaw\t");
        Serial.print(giro.getAdjustedHysteresisYaw());
        Serial.print("\tdac\t");
        Serial.println(giro.getAdjustedHysteresisYawAsDacValue(0, 4095));
     }
  }
}


//long lastLedBlinkMillis = 0;
//const int ledOnMillis = 10;
//const int ledBlinkIntervalMillis = 1000;
//boolean ledOn = false;
///*
//Blinks on-board LED for 'ledOnMillis' every 'ledBlinkIntervalMillis' interval
//*/
//void heartbeatLoopCall(){
//  long currentMillis = millis();
//  
//  if (!ledOn){
//    if (currentMillis - lastLedBlinkMillis > ledBlinkIntervalMillis) {
//      lastLedBlinkMillis = currentMillis;
//      //turn led on:
//      ledOn = true;
//      digitalWrite(led, ledOn);
//    }
//  }
//  else { //led is on, note that lastLedBlinkMillis is the time the LED was turned on
//    if (currentMillis - lastLedBlinkMillis > ledOnMillis){
//      //turn led off:
//      ledOn = false;
//      digitalWrite(led, ledOn);
//    }
//  }
//}


void giroLoopCallDac(){
  //Note that readGiroLoopCall will only actually read the giro each 50 ms, otherwise returns quickly
  if (giro.readGiroLoopCall()){
    //Need to convert -180 to 180 to 0 to 4095:
    dac.setVoltage(giro.getAdjustedYawAsDacValue(0, 4095), false);
  }
}

int i = 0;
void loop() {
  giroLoopCallPrint();
  //heartbeatLoopCall();
  heartBeat.heartbeatLoopCall();
  delay(20);
  
//  i++;
//  if (i > 500){
//    i = 0;
//    giro.resetAdjustedYaw();
//  }
}


