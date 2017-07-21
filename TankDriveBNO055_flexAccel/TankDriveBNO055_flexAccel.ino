#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int FLEX_PIN1 = A0; // Pin connected to voltage divider output
const int FLEX_PIN2 = A1; // Pin connected to voltage divider output
// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 47500.0; // Measured resistance of 3.3k resistor
const float STRAIGHT_RESISTANCE = 37300.0; // resistance when straight
const float BEND_RESISTANCE = 90000.0; // resistance at 90 deg
int minBend = -20;
int maxBend = 95;
float angle1 = 0;
float angle2 = 0;

int M1Pin1 = 2;    // H-bridge leg 1 (pin 2, 1A)
int M1Pin0 = 3;    // H-bridge leg 1 (pin 2, 1A)
int M2Pin1 = 4;
int M2Pin0 = 5;
int enablePinM1 = 9;    // H-bridge enable pin
int enablePinM2 = 10;    // H-bridge enable pin

double r = 0; // roll
double p = 0; // pitch
double y = 0; // yaw / spin

double BX;
double BY;
double BZ;

int LM = 0;
int RM = 0;

int BNO055_SAMPLERATE_DELAY_MS = 100;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) Any sensor data reporting 0 should be ignored, 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

/*  Convert yaw, pitch, and roll into [-100,100] power values*/
int convertToPower(double input){
  int output = 0;
  if (input <= 45 || input >= -45){
    if (input >= 7){
      output = map(input, 0, 45, 0, 50); // ((roll - 10) / 90); // o => [0,50]
    } else if (input <= -7){
      output = map(input, -45, 0, -50, 0); // ((roll + 10) / 90); // o => [-50,0]
    }
  } else {
    if (input > 0){
      output = 50;
    } else {
      output = -50;
    }
  }
  
  return output;
}

int convertFlexPower(double input){
  int output = 0;
  if (input <= 90 && input >= 10){
    output = map(input, 5, 90, 0, 50); // ((roll - 10) / 90); // o => [0,50]
  } else if (input > 90){
    output = 50;
  }
  
  return output;
}

/* Motpr power function*/
void motorWrapper(int motorPower, int motor, int pin1, int pin0){
  int power;
  if (motorPower >= 0){
    power = map (motorPower, 0, 101, 50, 255); //(motorPower / 2) * 255; //Divide motor power by range of expected input (in this case it is [-100, 100])
    analogWrite(motor, power);
    //direction = 1; (forward)
      digitalWrite(pin1, LOW);
      digitalWrite(pin0, HIGH);
    
  } else{
    power =  map(-motorPower, 0, 101, 50, 255); //(-motorPower) * 255; //Divide motor power by range of expected input (in this case it is [-100, 100])
    analogWrite(motor, power);
    //direction = 0; (backward)
      digitalWrite(pin1, HIGH);
      digitalWrite(pin0, LOW);
  }
}

float calculateResistance(int pinout){
  float angle = 0;
  int flexADC = analogRead(pinout);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  // Use the calculated resistance to estimate the sensor's bend angle:
  angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);
  return angle;
}

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  bno.setExtCrystalUse(true);

  pinMode(FLEX_PIN1, INPUT);
  pinMode(FLEX_PIN2, INPUT);

  pinMode(M1Pin1, OUTPUT);
  pinMode(M1Pin0, OUTPUT);    
  pinMode(M2Pin1, OUTPUT);
  pinMode(M2Pin0, OUTPUT);   
  pinMode(enablePinM1, OUTPUT);
  pinMode(enablePinM2, OUTPUT);
  
}

void loop(void) {
  sensors_event_t event;
  bno.getEvent(&event);

  if(event.orientation.x <= 180){
    BX = event.orientation.x;
  } else {
    BX = event.orientation.x - 360;
  }
  BZ = event.orientation.z;
  BY = event.orientation.y;
  
  angle1 = calculateResistance(FLEX_PIN1);
  angle2 = calculateResistance(FLEX_PIN2);

  y = convertToPower(BX);
  p = convertToPower(BZ);
  r = convertToPower(BY);
  int accel = convertFlexPower(angle2);
  if (p < 0){
    accel = -accel;
  }

  if (angle1 <= 50){
    LM = accel + y;
    RM = accel - y;
  } else {
    LM = 0;
    RM = 0;
  }
  
  motorWrapper(LM, enablePinM1, M1Pin1, M1Pin0);
  motorWrapper(RM, enablePinM2, M2Pin1, M2Pin0);
  //delay(1000);


  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(event.orientation.x, 4); // yaw / spin [0,360]
//  Serial.print("\tY: ");
//  Serial.print(event.orientation.y, 4); // pitch [-180,180]
//  Serial.print("\tZ: ");
//  Serial.print(event.orientation.z, 4); // roll [-180,180]

  Serial.print("Yaw: ");
  Serial.print(BX, 4); // yaw / spin [0,360]
  Serial.print("\tPitch: ");
  Serial.print(BZ, 4); // pitch [-180,180]
  Serial.print("\tRoll: ");
  Serial.print(BY, 4); // roll [-180,180]
  displayCalStatus();
  //Serial.print("\tResistance 1: " + String(flexR1) + " ohms\t");
  Serial.print("\tBend 1: " + String(angle1));
  //Serial.print("Resistance 2: " + String(flexR2) + " ohms\t");
  Serial.print("\tBend 2: " + String(angle2));
  
  Serial.print("\tLM: " + String(LM));
  Serial.print("\tRM: " + String(RM));
  
  /* New line for  next sample */
  Serial.println("");
  
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

