/*
 * Irene Terpstra
 * Team Handroid 
 * 12/29/17 
 * Code for saving BNO055 output info 
 */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;

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

double r = 0; // roll
double p = 0; // pitch
double y = 0; // yaw / spin

double zeroBX = 0;
double tempBX = 0;
double BX;
double BY;
double BZ;

double zeroBX2 = 0;
double tempBX2 = 0;
double BX2;
double BY2;
double BZ2;


int BNO055_SAMPLERATE_DELAY_MS = 100;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, BNO055_ADDRESS_B);


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
void displaySensorDetails2(void)
{
  sensor_t sensor;
  bno2.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.println(sensor.max_value);
  Serial.print  (F("Min Value:    ")); Serial.println(sensor.min_value);
  Serial.print  (F("Resolution:   ")); Serial.println(sensor.resolution);
  Serial.println(F("------------------------------------"));
  Serial.println();
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
  bno2.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
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
  Serial.println(F("Orientation Sensor Test")); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    Serial.print(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  if(!bno2.begin())
  {
    Serial.print(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();
  displaySensorDetails2();

  bno.setExtCrystalUse(true);
  bno2.setExtCrystalUse(true);

  pinMode(FLEX_PIN1, INPUT);
  pinMode(FLEX_PIN2, INPUT);
  
}

void updateSensorData(){
  sensors_event_t event;
  bno.getEvent(&event);

  angle1 = calculateResistance(FLEX_PIN1);
  angle2 = calculateResistance(FLEX_PIN2);
  
  if (angle2 > 50){
    zeroBX = event.orientation.x;
  }
  if ((event.orientation.x - zeroBX) > 0){
    tempBX = event.orientation.x - zeroBX;
  } else {
    tempBX = 360 + (event.orientation.x - zeroBX);
  }
  
  if(tempBX <= 180){
    BX = tempBX;
  } else {
    BX = tempBX - 360;
  }
  
  BZ = event.orientation.z;
  BY = event.orientation.y;
}
void updateSensorData2(){
  sensors_event_t event;
  bno2.getEvent(&event);

  angle1 = calculateResistance(FLEX_PIN1);
  angle2 = calculateResistance(FLEX_PIN2);
  
  if (angle2 > 50){
    zeroBX2 = event.orientation.x;
  }
  if ((event.orientation.x - zeroBX2) > 0){
    tempBX2 = event.orientation.x - zeroBX2;
  } else {
    tempBX2 = 360 + (event.orientation.x - zeroBX2);
  }
  
  if(tempBX2 <= 180){
    BX2 = tempBX2;
  } else {
    BX2 = tempBX2 - 360;
  }
  
  BZ2 = event.orientation.z;
  BY2 = event.orientation.y;
}
void loop(void) {

  updateSensorData();
  updateSensorData2();
  Serial.println(String("X: ") + BX + " Y: " + BZ + " Z: " + BY + "   X2: " + BX2 + " Y2: " + BZ2 + " Z2: " + BY2);

  //Serial.print("\tBend 1: " + String(angle1));
  //Serial.print("\tBend 2: " + String(angle2));


  
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

