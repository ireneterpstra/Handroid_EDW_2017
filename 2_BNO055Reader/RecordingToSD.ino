
void startRecording(){
  //Serial.println(F("Start Recording"));
  sampling = true;
  beatCounter = 0;
  sampleCounter = 0;
  recording = true;
  //create new file
  setNewDataFile();
  int numberOfSamples = samplesPerBeat * beatsPerRecording;
  sampleTimer = t.every(100, sample, numberOfSamples);
  int sampleDuration = 100 * samplesPerBeat * beatsPerRecording;
  durationTimer = t.after(sampleDuration, stopRecording);
}

void sample(){
  //Serial.println(F("Start Sample"));
  if (dataFile){
    writeDataCsvLine(dataFile);
    Serial.print(F("."));
  }
  sampleCounter++;
  if (sampleCounter % samplesPerBeat == 0){
    startLED(500);
    beatCounter++;
  }
}

void stopRecording(){
  //Serial.println(F("Stop Recording"));
  sampling = false;
  //Serial.println();
  closeDataFile(dataFile);
  t.stop(sampleTimer);
  t.stop(durationTimer);
}

/*String createDataString(){
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
}*/

//---------------------------------------------------------------------------------------------------------
//  File Operations
//---------------------------------------------------------------------------------------------------------

void addLineToFile(String fileName, String data){
  //Serial.println(F("Adding line to file "));
  File file = SD.open(fileName, FILE_WRITE);
  if(file){
    Serial.print(F("Writing to file: ")); Serial.println(fileName);
    file.println(data);
    //Serial.print(F("File Size (post write) = ")); Serial.println(file.size());
    //Serial.print(F("Wrote data: ")); Serial.println(data);
    file.close();
  }
  else {
    Serial.print(F("Can not open file ")); Serial.println(fileName);
  }
}

void overwriteLineToFile(String fileName, String data){
  //Serial.println(F("Adding line to file "));
  File file = SD.open(fileName, O_WRITE | O_CREAT | O_TRUNC);
  if(file){
    Serial.print(F("Writing to file: ")); Serial.println(fileName);
    file.println(data);
    //Serial.print(F("File Size (post write) = ")); Serial.println(file.size());
    //Serial.print(F("Wrote data: ")); Serial.println(data);
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
  //Serial.println(F("Reading index from file "));
  File file = SD.open(fileName, FILE_READ);
  if(file){
    //Serial.println(F("Reading index file"));
    String buffer = file.readStringUntil('\n');
    //Serial.print(F("Index File Size = ")); Serial.println(file.size());
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
  //Serial.println(F("Writing index to file "));
  overwriteLineToFile(fileName, String(index));
}

/**
 * Get the next index based on the index file.
 * Increments the current index.
 * To be used to generate uniquely and sequentially named data files 
 */
int getNextFileIndex(String fileName){
  //Serial.println(F("Getting index from file "));
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
    //Serial.print(F("Opened file "));Serial.print(fileName); Serial.print(F(" File Size = ")); Serial.println(file.size());
    }
  else {
    //Serial.print(F("Can not open file "));Serial.println(fileName);
  }
  return file;
}

String getNextDataFileName(){
  //Serial.println(F("Getting next data file name "));
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
  //Serial.print(F("Closing data file ")); Serial.print(dataFileName);Serial.print(F("File  Size = "));Serial.println(file.size());
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
