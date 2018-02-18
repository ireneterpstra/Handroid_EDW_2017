/*
 * Irene Terpstra
 * Team Handroid 
 * 2/12/18
 * Test code for triggering flex accel
 */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int FLEX_PIN1 = A0; // Pin connected to voltage divider output
const int FLEX_PIN2 = A1; // Pin connected to voltage divider output
// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 47000.0; // Measured resistance of 47k resistor
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

boolean recording = false;

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
  Serial.println("Orientation Sensor Test"); 
  Serial.println("");

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
  
}

void loop(void) {
  sensors_event_t event;
  bno.getEvent(&event);

  if (angle1 > 50 && angle2 > 50){
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

//  if(event.orientation.x <= 180){
//    BX = event.orientation.x;
//  } else {
//    BX = event.orientation.x - 360;
//  }
  BZ = event.orientation.z;
  BY = event.orientation.y;
  
  angle1 = calculateResistance(FLEX_PIN1);
  angle2 = calculateResistance(FLEX_PIN2);

//  if (angle1 >= 50){
//    recording = true;
//    Serial.print("Recording");
//  }

  if (angle1 > 50 && !recording){
    recording = true;
    //create new file
    Serial.print("Begin Recording");
  } else if (angle1 > 50 && recording){
    //add lines to file
    Serial.print("Recording");
  } else if (recording){
    recording = false;
    Serial.print("End Recording");
  }

//  if (recording){
//    //Serial.print("\tResistance 1: " + String(flexR1) + " ohms\t");
//    Serial.print("\tBend 1: " + String(angle1));
//    //Serial.print("Resistance 2: " + String(flexR2) + " ohms\t");
//    Serial.print("\tBend 2: " + String(angle2));
//  }

    //Serial.print("\tResistance 1: " + String(flexR1) + " ohms\t");
    //Serial.print("\tBend 1: " + String(angle1));
    //Serial.print("Resistance 2: " + String(flexR2) + " ohms\t");
    //Serial.print("\tBend 2: " + String(angle2));

  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(event.orientation.x, 4); // yaw / spin [0,360]
//  Serial.print("\tY: ");
//  Serial.print(event.orientation.y, 4); // pitch [-180,180]
//  Serial.print("\tZ: ");
//  Serial.print(event.orientation.z, 4); // roll [-180,180]

//  Serial.print("Yaw: ");
//  Serial.print(BX, 4); // yaw / spin [0,360]
//  Serial.print("\tPitch: ");
//  Serial.print(BZ, 4); // pitch [-180,180]
//  Serial.print("\tRoll: ");
//  Serial.print(BY, 4); // roll [-180,180]
//  displayCalStatus();

  
  /* New line for  next sample */
  Serial.println("");
  
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

