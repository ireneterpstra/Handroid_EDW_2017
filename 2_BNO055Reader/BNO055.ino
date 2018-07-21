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

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print(F("\t"));
  if (!system)
  {
    Serial.print(F("! "));
  }
}


//---------------------------------------------------------------------------------------------------------
//  IMU data
//---------------------------------------------------------------------------------------------------------

/**
 * Writes one line of data to file
 */
void writeDataCsvLine(File file){
  String comma = F(",");
  updateSensorData();
  updateSensorData2();

  file.print(millis());
  file.print(comma);
  file.print(String(BX, 4));
  file.print(comma);
  file.print(String(BZ, 4));
  file.print(comma);
  file.print(String(BY, 4));

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  file.print(comma); 
  file.print(String(euler.x(), 4));
  file.print(comma);
  file.print(String(euler.y(), 4));
  file.print(comma);
  file.print(String(euler.z(), 4));

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  file.print(comma);
  file.print(String(gyro.x(), 4));
  file.print(comma);
  file.print(String(gyro.y(), 4));
  file.print(comma);
  file.print(String(gyro.z(), 4));

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  file.print(comma);
  file.print(String(accel.x(), 4));
  file.print(comma);
  file.print(String(accel.y(), 4));
  file.print(comma);
  file.print(String(accel.z(), 4));

  imu::Quaternion quat = bno.getQuat();
  file.print(comma);
  file.print(String(quat.w(), 4));
  file.print(comma);
  file.print(String(quat.y(), 4));
  file.print(comma);
  file.print(String(quat.x(), 4));
  file.print(comma);
  file.print(String(quat.z(), 4));

  imu::Quaternion quat2 = bno2.getQuat();
  file.print(comma);
  file.print(String(quat2.w(), 4));
  file.print(comma);
  file.print(String(quat2.y(), 4));
  file.print(comma);
  file.print(String(quat2.x(), 4));
  file.print(comma);
  file.print(String(quat2.z(), 4));
  
  file.println();
  Serial.print(F("X: "));
  Serial.print(BX);
  Serial.print(F(" Y: "));
  Serial.print(BZ);
  Serial.print(F(" Z: "));
  Serial.print(BY);
  Serial.print(F(" X2: "));
  Serial.print(BX2);
  Serial.print(F(" Y2: "));
  Serial.print(BZ2);
  Serial.print(F(" Z2: "));
  Serial.println(BY2);
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



