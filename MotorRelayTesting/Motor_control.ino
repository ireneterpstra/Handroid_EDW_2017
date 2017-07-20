
/*
   Danielle Le
   Team Handroid
   7/19/17
   This code acts as a motor controller for 8 relays!
*/

#define Motor1Forward 3 //Forwards Pin3 to IN1// 
#define Motor1Backwards 4  //Backwards Pin4 to IN2//

#define Motor2Forward 5 //Forwards Pin5 to IN3// 
#define Motor2Backwards 6 //Backwards Pin6 to IN4//

#define Motor3Forward 7 // Forwards Pin7 to IN5
#define Motor3Backwards 8 //Backwards Pin8 to IN6

#define Motor4Forward 9 // Backwards Pin9 to IN7
#define Motor4Backwards 10  // Backwards Pin10 to IN8

/*          \|/   \|/
            ________
     1   0 |         | 0  2
         ` |         |
         ` |         |
          `|         |
     3   0 |         | 0   4
         ` -----------


*/

void setup() { //Setup runs once//

  pinMode(Motor1Forward, OUTPUT); //Sets forwards as output//
  pinMode(Motor1Backwards, OUTPUT); //Sets backwards as output//

  pinMode(Motor2Forward, OUTPUT); //Sets forwards as output//
  pinMode(Motor2Backwards, OUTPUT); //Sets backwards as output//

  pinMode(Motor3Forward, OUTPUT); //Sets forwards as output//
  pinMode(Motor3Backwards, OUTPUT); //Sets backwards as output//

  pinMode(Motor4Forward, OUTPUT); //Sets forwards as output//
  pinMode(Motor4Backwards, OUTPUT); //Sets backwards as output//
}

void MoveForward () {
  digitalWrite(Motor1Backwards, LOW);
  digitalWrite(Motor2Backwards, LOW);
  digitalWrite(Motor3Backwards, LOW);
  digitalWrite(Motor4Backwards, LOW);
  // 1 millisecond
  delay (1);
  digitalWrite(Motor1Forward, HIGH);
  digitalWrite(Motor2Forward, HIGH);
  digitalWrite(Motor3Forward, HIGH);
  digitalWrite(Motor4Forward, HIGH);
}

void MoveBackwards () {
  digitalWrite(Motor1Forward, LOW);
  digitalWrite(Motor2Forward, LOW);
  digitalWrite(Motor3Forward, LOW);
  digitalWrite(Motor4Forward, LOW);
  // 1 millisecond
  delay (1);
  digitalWrite(Motor1Backwards, HIGH);
  digitalWrite(Motor2Backwards, HIGH);
  digitalWrite(Motor3Backwards, HIGH);
  digitalWrite(Motor4Backwards, HIGH);
}

void TurnLeft () {
  digitalWrite(Motor1Forward, LOW);
  digitalWrite(Motor2Backwards, LOW);
  digitalWrite(Motor3Forward, LOW);
  digitalWrite(Motor4Backwards, LOW);
  // 1 millisecond
  delay (1);
  digitalWrite(Motor1Backwards, HIGH);
  digitalWrite(Motor2Forward, HIGH);
  digitalWrite(Motor3Backwards, HIGH);
  digitalWrite(Motor4Forward, HIGH);
}

void TurnRight () {
  digitalWrite(Motor1Backwards, LOW);
  digitalWrite(Motor2Forward, LOW);
  digitalWrite(Motor3Backwards, LOW);
  digitalWrite(Motor4Forward, LOW);
  // 1 millisecond
  delay (1);
  digitalWrite(Motor1Forward, HIGH);
  digitalWrite(Motor2Backwards, HIGH);
  digitalWrite(Motor3Forward, HIGH);
  digitalWrite(Motor4Backwards, HIGH);

}

void Stop () {
  digitalWrite(Motor1Backwards, LOW);
  digitalWrite(Motor2Forward, LOW);
  digitalWrite(Motor1Forward, LOW);
  digitalWrite(Motor2Backwards, LOW);
  digitalWrite(Motor3Backwards, LOW);
  digitalWrite(Motor4Forward, LOW);
  digitalWrite(Motor3Forward, LOW);
  digitalWrite(Motor4Backwards, LOW);
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

