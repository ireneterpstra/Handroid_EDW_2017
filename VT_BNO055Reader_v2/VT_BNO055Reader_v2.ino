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
const float R_DIV = 47500.0; // Measured resistance of 47k resistor
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

String dataFileName;
File dataFile;
int index;
int numDataLines = 0;

String header;

int fileCount = 0;


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

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print(F("\t"));
  if (!system)
  {
    Serial.print(F("! "));
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

String createDataString(){
  String dataString = "";

  String comma = ",";
  dataString += millis();
  dataString += comma;
  dataString += String(BX, 4);
  dataString += comma;
  dataString += String(BZ, 4);
  dataString += comma;
  dataString += String(BY, 4);
  dataString += comma;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);  
  dataString += String(euler.x(), 4);
  dataString += comma;
  dataString += String(euler.y(), 4);
  dataString += comma;
  dataString += String(euler.z(), 4);
  dataString += comma;

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  dataString += String(gyro.x(), 4);
  dataString += comma;
  dataString += String(gyro.y(), 4);
  dataString += comma;
  dataString += String(gyro.z(), 4);
  dataString += comma;

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  dataString += String(accel.x(), 4);
  dataString += comma;
  dataString += String(accel.y(), 4);
  dataString += comma;
  dataString += String(accel.z(), 4);
  dataString += comma;

  imu::Quaternion quat = bno.getQuat();
  dataString += String(quat.w(), 4);
  dataString += comma;
  dataString += String(quat.y(), 4);
  dataString += comma;
  dataString += String(quat.x(), 4);
  dataString += comma;
  dataString += String(quat.z(), 4);

  return dataString;
}
//---------------------------------------------------------------------------------------------------------
//  File Operations
//---------------------------------------------------------------------------------------------------------

void addLineToFile(String fileName, String data){
  Serial.println(F("Adding line to file "));
  File file = SD.open(fileName, FILE_WRITE);
  if(file){
    Serial.print(F("Writing to file: ")); Serial.println(fileName);
    file.println(data);
    Serial.print(F("File Size (post write) = ")); Serial.println(file.size());
    Serial.print(F("Wrote data: ")); Serial.println(data);
    file.close();
  }
  else {
    Serial.print(F("Can not open file ")); Serial.println(fileName);
//    Serial.println(F("Can not open file ") + fileName);
  }
}

void overwriteLineToFile(String fileName, String data){
  Serial.println(F("Adding line to file "));
  File file = SD.open(fileName, O_WRITE | O_CREAT | O_TRUNC);
  if(file){
    Serial.print(F("Writing to file: ")); Serial.println(fileName);
    file.println(data);
    Serial.print(F("File Size (post write) = ")); Serial.println(file.size());
    Serial.print(F("Wrote data: ")); Serial.println(data);
    file.close();
  }
  else {
    Serial.print(F("Can not open file ")); Serial.println(fileName);
//    Serial.println(F("Can not open file ") + fileName);
  }
}
/**
 * Reads the current index from the index file.
 * The index file is a file with one line that contains an ascii representation of an integer
 */
int readIndexFromFile(String fileName){
  int index = -1;
  Serial.println(F("Reading index from file "));
  File file = SD.open(fileName, FILE_READ);
  if(file){
    Serial.println(F("Reading index file"));
    String buffer = file.readStringUntil('\n');
    Serial.print(F("Index File Size = ")); Serial.println(file.size());
    //Serial.print(F("Buffer Length = ")); Serial.println(buffer.length());
    //Serial.print(F("Buffer = ")); Serial.println(buffer);

    if (buffer.length() > 0){
      index = buffer.toInt();
    }
    file.close();
  }
  else {
    Serial.print(F("Can not open index file "));Serial.print(fileName);
  }
  return index;
}

/**
 * Repaces the current index file with a new file of the same name with tyhe new index value
 */
void writeIndexToFile(String fileName, int index){
  Serial.println(F("Writing index to file "));
//  delay(100);
//  SD.remove(fileName);
//  delay(100);
//  addLineToFile(fileName, String(index));
  overwriteLineToFile(fileName, String(index));
}

/**
 * Get the next index based on the index file.
 * Increments the current index.
 * To be used to generate uniquely and sequentially named data files 
 */
int getNextFileIndex(String fileName){
  Serial.println(F("Getting index from file "));
  int index = readIndexFromFile(fileName);
  index++;
  writeIndexToFile(fileName, index);
  return index;
}

/**
 * Opens file. Prints success or fail.
 * Note that if fail, the 'file' returns 'false'
 */
File openFile(String fileName){
  Serial.println(F("Opening file "));
  File file = SD.open(fileName, FILE_WRITE);
//  delay(500);
  if(file){
    Serial.print(F("Opened file "));Serial.print(fileName); Serial.print(F(" File Size = ")); Serial.println(file.size());
    }
  else {
    Serial.print(F("Can not open file "));Serial.println(fileName);
  }
  return file;
}

String getNextDataFileName(){
  Serial.println(F("Getting next data file name "));
  int index = getNextFileIndex(F("INDEX.txt"));
  String fileName = "DATA" + String(index) + ".csv";
  return fileName;
}

/**
 * Opens a next new data file, keeping it open for writes.
 * Uses the index file to create new next name.
 * Adds header
 */
File getNewDataFile(){
  Serial.println(F("Getting new data file "));
  //1.Open new file
  dataFileName = getNextDataFileName();
  File file = openFile(dataFileName);
  //2. Add header
  file.println(header); 
  return file;
}

void closeDataFile(File file){
  Serial.print(F("Closing data file ")); Serial.print(dataFileName);Serial.print(F("File  Size = "));Serial.println(file.size());
  file.close();
}

/**
 * Creates a new cv file and assigns to this.dataFile.
 * Closes the previous if applicable
 */
void setNewDataFile(){
  if (dataFile){
    closeDataFile(dataFile);
  }
  dataFile = getNewDataFile();
}

//---------------------------------------------------------------------------------------------------------
//  IMU data
//---------------------------------------------------------------------------------------------------------

/**
 * Writes one line of data to file
 */
void writeDataCsvLine(File file){
  String comma = F(",");
  
  file.print(millis());
  file.print(comma);
  file.print(String(BX, 4));
  file.print(comma);
  file.print(String(BZ, 4));
  file.print(comma);
  file.print(String(BY, 4));
  
  file.println();
  
//  dataString += ;
//  dataString += comma;
//  dataString += String(BX, 4);
//  dataString += comma;
//  dataString += String(BZ, 4);
//  dataString += comma;
//  dataString += String(BY, 4);
//  dataString += comma;
//
//  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);  
//  dataString += String(euler.x(), 4);
//  dataString += comma;
//  dataString += String(euler.y(), 4);
//  dataString += comma;
//  dataString += String(euler.z(), 4);
//  dataString += comma;
//
//  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//  dataString += String(gyro.x(), 4);
//  dataString += comma;
//  dataString += String(gyro.y(), 4);
//  dataString += comma;
//  dataString += String(gyro.z(), 4);
//  dataString += comma;
//
//  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
//  dataString += String(accel.x(), 4);
//  dataString += comma;
//  dataString += String(accel.y(), 4);
//  dataString += comma;
//  dataString += String(accel.z(), 4);
//  dataString += comma;
//
//  imu::Quaternion quat = bno.getQuat();
//  dataString += String(quat.w(), 4);
//  dataString += comma;
//  dataString += String(quat.y(), 4);
//  dataString += comma;
//  dataString += String(quat.x(), 4);
//  dataString += comma;
//  dataString += String(quat.z(), 4);
}
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

  Serial.print(F("Initializing SD card..."));

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    while (1);
  }
  Serial.println(F("card initialized."));

  header = F("millis,yaw_x,pitch_z,roll_y,euler_x,euler_y,euler_z,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,quat_w,quat_x,quat_y,quat_z");

//  dataFileName = getNextDataFileName();
//  Serial.println(dataFileName);

//  addLineToFile(F("test1"), F("acdefg"));

//  addLineToFile(dataFileName, header);

//dataFileName = "test2.txt";
//  File dataFile = SD.open(dataFileName, FILE_WRITE);
//  if (dataFile) {
//    String header = F("millis,yaw_x,pitch_z,roll_y,euler_x,euler_y,euler_z,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,quat_w,quat_x,quat_y,quat_z");
//    dataFile.println(header);
//    Serial.println(header);
//    dataFile.close();
//  } else {
//    Serial.println("Can't open for header " + dataFileName);
//  }

  setNewDataFile();

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  bno.setExtCrystalUse(true);

  pinMode(FLEX_PIN1, INPUT);
  pinMode(FLEX_PIN2, INPUT);
  
}
//---------------------------------------------------------------------------------------------------------
//  Loop
//---------------------------------------------------------------------------------------------------------

void loop(void) {
//  Serial.println(F("Loop"));
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

//  if(event.orientation.x <= 180){
//    BX = event.orientation.x;
//  } else {
//    BX = event.orientation.x - 360;
//  }
  BZ = event.orientation.z;
  BY = event.orientation.y;

  String dataString = createDataString();

//  if (fileCount < 3){
//    File file = getNewDataFile();
//    writeDataCsvLine(file);
//    closeDataFile(file);
//    fileCount++;
//  }
  
  
//  Serial.println(dataString);

  if (numDataLines < 100){
//    addLineToFile(F("test1"), F("abcloop"));
    Serial.print(F(" L_"));Serial.print(numDataLines);
    if (dataFile){
      writeDataCsvLine(dataFile);
      numDataLines++;  
    }
    else {
      Serial.println(F("No data file in loop."));
      numDataLines++;
    }
    
//    addLineToFile(dataFileName, dataString);
    
  }
  else {
    setNewDataFile();
    numDataLines = 0;
//    writeDataCsvLine(dataFile);
//    numDataLines++;
    Serial.println(F("Created new data file "));
  }
//  delay(1000);

  /*File dataFile = SD.open(dataFileName, FILE_WRITE);
  if (dataFile) {
    Serial.print("File Size = ");
    Serial.print(dataFile.size());
    Serial.println();
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  } else{
    Serial.println("Can't open file " + dataFileName);

  }*/  
  
  //Serial.print("\tBend 1: " + String(angle1));
  //Serial.print("\tBend 2: " + String(angle2));



//  if (angle1 > 50 && !recording){
//    recording = true;
//    //create new file
//    File file = getNewDataFile();
//  } else if (angle1 > 50 && recording){
//    //add lines to file
//    writeDataCsvLine(file);
//  } else if (recording){
//    recording = false;
//    //end recording
//    closeDataFile(file);
//  }
  
//    isRecording = true;
//
//    fileName = "datalog" + index + ".txt";
//
//    File dataFile = SD.open(fileName, FILE_WRITE);
//    //dataFile.println(F("millis,yaw(X),pitch(Z),roll(Y),euler(x,y,z),gyro(x,y,z),accel(x,y,z),quat(w,x,y,z)"));
//    //Serial.println(F("millis,yaw(X),pitch(Z),roll(Y),euler(x,y,z),gyro(x,y,z),accel(x,y,z),quat(w,x,y,z)"));
//    dataFile.println(F("millis,yaw(X),pitch(Z),roll(Y)"));
//    Serial.println(F("millis,yaw(X),pitch(Z),roll(Y)"));
//    dataFile.close();
//
//    index ++
//
//    SD.remove("IndexFile.txt");
//    File indexFile = SD.open("IndexFile.txt", FILE_WRITE);
//    indexFile.println(index);
//    Serial.println(F("Index = " index));
//    indexFile.close();
//    
//  } else if (angle1 < 50 && isRecording){
//    isRecording = false;
//  }
//
//  if (isRecording){
//    
//    
//  } = SD.open("datalog1", FILE_WRITE);
//    dataFile.printl
//
//  File dataFilen(dataString);
//
//  // if the file is available, write to it:
//  if (dataFile) {
//    dataFile.close();
//    // print to the serial port too:
//    Serial.println(dataString);
//  }
//  // if the file isn't open, pop up an error:
//  else {
//    Serial.println(F("error opening datalog1.txt"));
//  }
  
 
  
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

