/*
 * Irene Terpstra
 * Team Handroid 
 * 7/21/17 
 * Provides motor wrappers for realy
 */
//int M1power = ;
int M1plus = 3;
int M1min = 4;
//int M2power = ;
int M2plus = 5;
int M2min = 6;
//int M3power = ;
int M3plus = 7;
int M3min = 8;
//int M4power = ;
int M4plus = 9;
int M4min = 10;

/*          \|/   \|/
  *          ________
  *   1   0 |         | 0  2
  *       ` |         | 
  *       ` |         |
  *        `|         | 
  *   3   0 |         | 0   4
  *       ` -----------
  * 
  * 
*/

void setup() {
  //set outputs
  pinMode(M1power, OUTPUT);
  pinMode(M1plus, OUTPUT);
  pinMode(M1min, OUTPUT);
  
  pinMode(M2power, OUTPUT);
  pinMode(M2plus, OUTPUT);
  pinMode(M2min, OUTPUT);
  
  pinMode(M3power, OUTPUT);
  pinMode(M3plus, OUTPUT);
  pinMode(M3min, OUTPUT);
  
  pinMode(M4power, OUTPUT);
  pinMode(M4plus, OUTPUT);
  pinMode(M4min, OUTPUT);
}

/* Motpr power function*/
void motorWrapper(int motorPower, int motor, int pin1, int pin0){
  int power;
  if (motorPower >= 0){
    power = map (motorPower, 0, 101, 50, 255); //Map inputed power range to 0-255 power output (in this case the range is [-100, 100])
    analogWrite(motor, power);
    //direction = forward
      digitalWrite(pin1, LOW);
      digitalWrite(pin0, HIGH);
    
  } else{
    power =  map(-motorPower, 0, 101, 50, 255); //Map inputed power range to 0-255 power output (in this case the range is [-100, 100])
    analogWrite(motor, power);
    //direction = backward
      digitalWrite(pin1, HIGH);
      digitalWrite(pin0, LOW);
  }
}


void loop() {
  
}
