int ledyellow = 13;
int ledgreen = 11;
int ledwhite = 9;
int ledred = 8;
int imu_sensor = A0;
void setup() {
  // put your setup code here, to run once:

  pinMode(ledyellow, OUTPUT);
  pinMode (ledgreen, OUTPUT);
  pinMode(ledwhite, OUTPUT);
  pinMode (ledred, OUTPUT);
  pinMode (imu_sensor, INPUT);
  digitalWrite (ledyellow, LOW);
  digitalWrite (ledgreen, LOW);
  digitalWrite (ledwhite, LOW);
  digitalWrite (ledred, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  // tilting down
   if (hand tilts down){
   digitalWrite (ledyellow, HIGH); 
   digitalWrite (ledgreen, HIGH);
   digitalWrite (ledwhite, HIGH);
   digitalWrite (ledred, HIGH);
   }
   else if (hand tilts right) {
   digitalWrite (ledyellow, HIGH);
   digitalWrite (ledred, HIGH);
   digitalWrite (ledgreen, LOW);
   digitalWrite (ledwhite, LOW);
   }
   else if (hand tilts left) {
   digitalWrite (ledyellow, LOW);
   digitalWrite (ledred, LOW);
   digitalWrite (ledgreen, HIGH);
   digitalWrite (ledwhite, HIGH);
   }
   else {
   digitalWrite (ledred, LOW);
   digitalWrite (ledyellow, LOW);
   digitalWrite (ledgreen, LOW);
   digitalWrite (ledwhite, LOW);
   }
   
   
   }
