void setup() {
  // put your setup code here, to run once:
  pinMode(6, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  analogWrite( 6 , 153 );
  delay(1000);
  analogWrite( 6 , 0 );
  delay(1000);
}
