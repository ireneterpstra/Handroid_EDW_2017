/*
 * Irene Terpstra
 * Team Handroid 
 * 7/21/17 
 * Code for omni wheel drive with BNO055 + flex sensors 
 */
double mortorFLPower;
double mortorBLPower;
double mortorFRPower;
double mortorBRPower;

double x, y, s;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  motorFLPower = x + y - s;
  motorBLPower = -x + y - s;
  motorFRPower = x - y - s;
  motorBRPower = -x - y - s;
}
