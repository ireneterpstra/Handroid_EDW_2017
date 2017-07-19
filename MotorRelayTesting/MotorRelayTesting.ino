/*
 * Danielle Le 
 * Team Handroid 
 * 7/19/17 
 * This code acts as a motor controller for 8 relays! 
 */
#define Motor1Forward 3 //Forwards Pin3 to IN1
#define Motor1Backwards 4 //Backwards Pin4 to IN2
#define Motor2Forward 5 //Forwards Pin5 to IN3 
#define Motor2Backwards 6 //Backwards Pin6 to IN4

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
