int ledyellow = 9;
int ledwhite= 6;
int ledgreen = 5;
int ledred = 3;

const int FLEX_PIN = A0; // Pin connected to voltage divider output
// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 47500.0; // Measured resistance of 3.3k resistor
const float STRAIGHT_RESISTANCE = 37300.0; // resistance when straight
const float BEND_RESISTANCE = 90000.0; // resistance at 90 deg
int minBend = -20;
int maxBend = 95;

int value; //save analog value

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(FLEX_PIN, INPUT);
  
  pinMode(ledyellow, OUTPUT);
  pinMode (ledgreen, OUTPUT);
  pinMode(ledwhite, OUTPUT);
  pinMode (ledred, OUTPUT);
//  digitalWrite (ledyellow, LOW);
//  digitalWrite (ledgreen, LOW);
//  digitalWrite (ledwhite, LOW);
//  digitalWrite (ledred, LOW);
}

void loop() {
  // Read the ADC, and calculate voltage and resistance from it
  int flexADC = analogRead(FLEX_PIN);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  Serial.print("Resistance: " + String(flexR) + " ohms\t");

  // Use the calculated resistance to estimate the sensor's bend angle:
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);
  Serial.print("Bend: " + String(angle) + " degrees");
  Serial.println();
  
  //value = map(angle, minBend, maxBend, 0, 255);//Map value 0-1023 to 0-255 (PWM)
  //analogWrite(ledPin, value);          //Send PWM value to led  
  
  // put your main code here, to run repeatedly:
  // tilting down
   if (angle >= 75){    
    digitalWrite (ledred, HIGH);
    digitalWrite (ledgreen, LOW);
    digitalWrite (ledwhite, LOW);
    digitalWrite (ledyellow, LOW);
   }
   else if (angle >= 50 & angle < 75) {
    digitalWrite (ledred, LOW);
    digitalWrite (ledgreen, HIGH);
    digitalWrite (ledwhite, LOW);
    digitalWrite (ledyellow, LOW);
   }
   else if (angle >= 0 & angle < 50) {
    digitalWrite (ledred, LOW);
    digitalWrite (ledgreen, LOW);
    digitalWrite (ledwhite, HIGH);
    digitalWrite (ledyellow, LOW);
   }
   else {
    digitalWrite (ledred, LOW);
    digitalWrite (ledgreen, LOW);
    digitalWrite (ledwhite, LOW);
    digitalWrite (ledyellow, HIGH);

   }
   
   
   }
