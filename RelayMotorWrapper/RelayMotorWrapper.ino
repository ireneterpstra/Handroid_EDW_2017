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
int M3plus = 3;
int M3min = 4;
//int M3power = ;
int M3plus = 5;
int M3min = 6;

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

void setup() {//Setup runs once
  pinMode(Motor1Forward, OUTPUT); //Sets forwards as output
  pinMode(Motor1Backwards, OUTPUT); //Sets backwards as output
  pinMode(Motor2Forward, OUTPUT); //Sets forwards as output
  pinMode(Motor2Backwards, OUTPUT); //Sets backwards as output
}

void MoveForward () {
  digitalWrite(Motor1Backwards, LOW);
  digitalWrite(Motor2Backwards, LOW);
  // 1 millisecond 
  delay (1);
  digitalWrite(Motor1Forward, HIGH);
  digitalWrite(Motor2Forward, HIGH);
}

void MoveBackwards () {
  digitalWrite(Motor1Forward, LOW);
  digitalWrite(Motor2Forward, LOW);
  // 1 millisecond 
  delay(1);
  digitalWrite(Motor1Backwards, HIGH);
  digitalWrite(Motor2Backwards, HIGH);
}

void TurnLeft () {
  digitalWrite(Motor1Forward, LOW);
  digitalWrite(Motor2Backwards, LOW);
  // 1 millisecond 
  delay(1);
  digitalWrite(Motor1Backwards, HIGH);
  digitalWrite(Motor2Forward, HIGH);  
}

void TurnRight () {
  digitalWrite(Motor1Backwards, LOW);
  digitalWrite(Motor2Forward, LOW);
  // 1 millisecond 
  delay (1);
  digitalWrite(Motor1Forward, HIGH);
  digitalWrite(Motor2Backwards, HIGH);
}

void Stop () {
  digitalWrite(Motor1Backwards, LOW);
  digitalWrite(Motor2Forward, LOW);
  digitalWrite(Motor1Forward, LOW);
  digitalWrite(Motor2Backwards, LOW);
  // 1 millisecond 
  delay (1);
}


void loop() { //Loop runs forever//
  MoveForward();
  delay(2000);
  MoveBackwards();
  delay(2000);
  TurnRight();
  delay(2000);
  TurnLeft();
  delay(2000);
  Stop();
  delay(2000);
}
