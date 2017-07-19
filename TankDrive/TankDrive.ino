//#define debug true
//#include <GtGiro.h>
//#include <MCP4725.h>
//#include <Wire.h>
//#include <GtHeartBeat.h>
//#ifdef ON_ROBOT
//  GtGiro giro(0x69, 2000, 30);
//#else
//  GtGiro giro(0x68, 2000, 30);
//#endif
//MCP4725 dac;

int M1Pin1 = 2;    // H-bridge leg 1 (pin 2, 1A)
int M1Pin0 = 3;    // H-bridge leg 1 (pin 2, 1A)
int M2Pin1 = 4;
int M2Pin0 = 5;
int enablePinM1 = 9;    // H-bridge enable pin
int enablePinM2 = 10;    // H-bridge enable pin

int x = 0;
int y = 0;
int s = 0;

void setup() {
//  // initialize serial communication
//  Serial.begin(9600);
//  Serial.println("Setup");
//  
//  #ifdef ON_ROBOT
//    dac.begin(0x60);
//  #endif
//  
//  giro.setup();
  
  pinMode(M1Pin1, OUTPUT);
  pinMode(M1Pin0, OUTPUT);    
  pinMode(M2Pin1, OUTPUT);
  pinMode(M2Pin0, OUTPUT);   
  pinMode(enablePinM1, OUTPUT);
  pinMode(enablePinM2, OUTPUT);
 
}
//long lastPrintMillis = 0;
//void giroLoopCallPrint(){
//  if (giro.readGiroLoopCall()){
//    #ifdef ON_ROBOT
//      dac.setVoltage(giro.getAdjustedHysteresisYawAsDacValue(0, 4095), false);
//    #endif
//    
//    long currentMillis = millis();
//    if (currentMillis - lastPrintMillis > 500){
//        lastPrintMillis = currentMillis;
//        Serial.print("Yaw\t");
//        Serial.print(giro.getYaw());
//        Serial.print("\tPitch\t");
//        Serial.print(giro.getPitch());
//        Serial.print("\tRoll\t");
//        Serial.print(giro.getRoll());
//        Serial.println();
//     }
//  }
//}

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
    y = ((spin - 15) / 30); // s => [0,1]
  } else if (spin <= -15){
    y = ((spin + 15) / 30); // s => [-1,0]
  }
}

void motorWrapper(double motorPower, int motor, int pin1, int pin0){
  int power;
  if (motorPower >= 0){
    power = (motorPower) * 255;
    analogWrite(motor, power);
    //direction = 1; (forward)
      digitalWrite(pin1, LOW);   // set leg 1 of the H-bridge low
      digitalWrite(pin0, HIGH);  // set leg 2 of the H-bridge high
    
  } else{
    power = (-motorPower) * 255;
    analogWrite(motor, power);
    //direction = 0; (backward)
      digitalWrite(pin1, HIGH);   // set leg 1 of the H-bridge low
      digitalWrite(pin0, LOW);  // set leg 2 of the H-bridge high
  }
}

void loop() {
//  LM = y + s;
//  RM = -y + s;

  
  motorWrapper(.75, enablePinM1, M1Pin1, M1Pin0);
  motorWrapper(.75, enablePinM2, M2Pin1, M2Pin0);
  delay(1000);
  motorWrapper(-.75, enablePinM1, M1Pin1, M1Pin0);
  motorWrapper(-.75, enablePinM2, M2Pin1, M2Pin0);
  delay(1000);

  
//  analogWrite(enablePinM1, 200);
//  digitalWrite(M1Pin1, LOW);   // set leg 1 of the H-bridge low
//  digitalWrite(M1Pin0, HIGH);  // set leg 2 of the H-bridge high
//  analogWrite(enablePinM2, 200);
//  digitalWrite(M2Pin1, LOW);   // set leg 1 of the H-bridge low
//  digitalWrite(M2Pin0, HIGH);  // set leg 2 of the H-bridge high
//  delay(1000);
//  digitalWrite(M1Pin1, HIGH);  // set leg 1 of the H-bridge high
//  digitalWrite(M1Pin0, LOW);   // set leg 2 of the H-bridge low
//  digitalWrite(M2Pin1, HIGH);   // set leg 1 of the H-bridge low
//  digitalWrite(M2Pin0, LOW);  // set leg 2 of the H-bridge high
//  delay(1000);

}


