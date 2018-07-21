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
#include <Event.h>
#include <Timer.h>

const int chipSelect = 4;

const int FLEX_PIN1 = A0; // Pin connected to voltage divider output
const int FLEX_PIN2 = A1; // Pin connected to voltage divider output

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

Timer t;

int LEDTimer;
int sampleTimer;
int durationTimer;

int beatCounter = 0;
int LED = 8;
boolean ledOn = false;

int sampleCounter = 0;
int samplesPerBeat = 10; //TODO: find true val
int beatsPerRecording = 5; //TODO: find true val
boolean sampling = false;

boolean recording = false;

String dataFileName;
File dataFile;
int index;
int numDataLines = 0;

String header;

int fileCount = 0;

int BNO055_SAMPLERATE_DELAY_MS = 100;

//dafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BNO055 bno = Adafruit_BNO055(55, (0x28));
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, (0x29));

//---------------------------------------------------------------------------------------------------------
//  Setup
//---------------------------------------------------------------------------------------------------------
void setup(void)
{
  Serial.begin(9600);
  Serial.println(F("Orientation Sensor Test")); Serial.println();

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

  Serial.print(F("Initializing SD card..."));

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    while (1);
  }
  Serial.println(F("card initialized."));

  header = F("millis,yaw_x,pitch_z,roll_y,euler_x,euler_y,euler_z,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,quat_w,quat_x,quat_y,quat_z,quat_w2,quat_x2,quat_y2,quat_z2");

  //setNewDataFile();

  delay(500);

  /* Display some basic information on this sensor */
  displaySensorDetails();
  displaySensorDetails2();

  bno.setExtCrystalUse(true);
  bno2.setExtCrystalUse(true);

  pinMode(FLEX_PIN1, INPUT);
  pinMode(FLEX_PIN2, INPUT);
  pinMode(LED, OUTPUT);
  //t.oscillate(LED, 370, LOW);
  //t.pulse(LED, 1000, HIGH); // 10 minutes  
  
}
//---------------------------------------------------------------------------------------------------------
//  Loop
//---------------------------------------------------------------------------------------------------------

void loop(void) {
//  Serial.println(F("Loop"));
  sensors_event_t event;
  
  angle1 = calculateResistance(FLEX_PIN1);
  angle2 = calculateResistance(FLEX_PIN2);
  
  t.update();

  if (angle1 > 50 && !sampling){
    startRecording();
  } 
}

