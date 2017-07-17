int ledyellow_1 = 9;
int ledwhite_1 = 6;
int ledgreen_1 = 5;
int ledred_1 = 3;
int ledyellow_2 = 12;
int ledwhite_2 = 11;
int ledgreen_2 = 10;
int ledred_2 = 13;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(FLEX_PIN, INPUT);
  
  pinMode(ledyellow_1, OUTPUT);
  pinMode (ledgreen_1, OUTPUT);
  pinMode(ledwhite_1, OUTPUT);
  pinMode (ledred_1, OUTPUT);
  pinMode(ledyellow_2, OUTPUT);
  pinMode (ledgreen_2, OUTPUT);
  pinMode(ledwhite_2, OUTPUT);
  pinMode (ledred_2, OUTPUT);
  digitalWrite (ledyellow_1, LOW);
  digitalWrite (ledgreen_1, LOW);
  digitalWrite (ledwhite_1, LOW);
  digitalWrite (ledred_1, LOW);
  digitalWrite (ledyellow_2, LOW);
  digitalWrite (ledgreen_2, LOW);
  digitalWrite (ledwhite_2, LOW);
  digitalWrite (ledred_2, LOW);
}

void loop() {
 
  // put your main code here, to run repeatedly:
  // tilting down
   if (tilting down){    
    digitalWrite (ledred_1, HIGH);
    digitalWrite (ledgreen_1, HIGH);
    digitalWrite (ledwhite_1, HIGH);
    digitalWrite (ledyellow_1, HIGH);
    digitalWrite (ledred_2, LOW);
    digitalWrite (ledgreen_2, LOW);
    digitalWrite (ledwhite_2, LOW);
    digitalWrite (ledyellow_2, LOW);
   }
   else if (tilting left) {
    digitalWrite (ledred_2, HIGH);
    digitalWrite (ledgreen_1, HIGH);
    digitalWrite (ledwhite_2, HIGH);
    digitalWrite (ledyellow_1, HIGH);
    digitalWrite (ledred_1, LOW);
    digitalWrite (ledgreen_2, LOW);
    digitalWrite (ledwhite_1, LOW);
    digitalWrite (ledyellow_2, LOW);
    
   }
   else if (tilting right) {
    digitalWrite (ledred_1, HIGH);
    digitalWrite (ledgreen_2, HIGH);
    digitalWrite (ledwhite_1, HIGH);
    digitalWrite (ledyellow_2, HIGH);
    digitalWrite (ledred_2, LOW);
    digitalWrite (ledgreen_1, LOW);
    digitalWrite (ledwhite_2, LOW);
    digitalWrite (ledyellow_1, LOW);
   }
   else if (tilting up) {
    digitalWrite (ledred_2, HIGH);
    digitalWrite (ledgreen_2, HIGH);
    digitalWrite (ledwhite_2, HIGH);
    digitalWrite (ledyellow_2, HIGH);
    digitalWrite (ledred_1, LOW);
    digitalWrite (ledgreen_1, LOW);
    digitalWrite (ledwhite_1, LOW);
    digitalWrite (ledyellow_1, LOW);
   }
   else {
    digitalWrite (ledred_1, LOW);
    digitalWrite (ledgreen_1, LOW);
    digitalWrite (ledwhite_1, LOW);
    digitalWrite (ledyellow_1, LOW);
    digitalWrite (ledred_2, LOW);
    digitalWrite (ledgreen_2, LOW);
    digitalWrite (ledwhite_2, LOW);
    digitalWrite (ledyellow_2, LOW);
   }
   
   
   
   }
