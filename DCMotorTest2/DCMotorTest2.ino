int motor1Pin1 = 2;    // H-bridge leg 1 (pin 2, 1A)
int motor1Pin0 = 3;    // H-bridge leg 1 (pin 2, 1A)
int motor2Pin1 = 4;
int motor2Pin0 = 5;
int enablePinM1 = 9;    // H-bridge enable pin
int enablePinM2 = 10;    // H-bridge enable pin
 
void setup() {
 
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin0, OUTPUT);    
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin0, OUTPUT);   
    pinMode(enablePinM1, OUTPUT);
    pinMode(enablePinM2, OUTPUT);
 
 }

void loop() {
  analogWrite(enablePinM2, 200);
  digitalWrite(motor1Pin1, LOW);   // set leg 1 of the H-bridge low
  digitalWrite(motor1Pin0, HIGH);  // set leg 2 of the H-bridge high
  analogWrite(enablePinM1, 200);
  digitalWrite(motor2Pin1, LOW);   // set leg 1 of the H-bridge low
  digitalWrite(motor2Pin0, HIGH);  // set leg 2 of the H-bridge high
  delay(1000);
  digitalWrite(motor1Pin1, HIGH);  // set leg 1 of the H-bridge high
  digitalWrite(motor1Pin0, LOW);   // set leg 2 of the H-bridge low
  digitalWrite(motor2Pin1, HIGH);   // set leg 1 of the H-bridge low
  digitalWrite(motor2Pin0, LOW);  // set leg 2 of the H-bridge high
  delay(1000);

  }
