#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



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

/*  Convert yaw, pitch, and roll into [-100,100] r, p, and y values*/
int convertToPower(double input){
  int output = 0;
  if (input <= 45 || input >= -45){
    if (input >= 5){
      output = map(input, 0, 45, 0, 50); // ((roll - 10) / 90); // o => [0,50]
    } else if (input <= -5){
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
//void setPitch(double pitch){
//  if (pitch >= 5){
//    p = map(pitch, 0, 180, 0, 50); // ((pitch - 10) / 90); // p => [0,50]
//  } else if (pitch <= -5){
//    p = map(pitch, -180, 0, -50, 0); // ((pitch + 10) / 90); // p => [-50,0]
//  }
//}
//void setYaw(double yaw){
//  if (yaw >= 5){
//    y = map(yaw, 0, 180, 0, 50); // ((yaw - 10) / 90); // y => [0,1]
//  } else if (yaw <= -5){
//    y = map(yaw, -180, 0, -50, 0); // ((yaw + 10) / 90); // y => [-1,0]
//  }
//}

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
  
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4); // yaw / spin [0,360]
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4); // pitch [-180,180]
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4); // roll [-180,180]

  Serial.print("\tYaw: ");
  Serial.print(BX, 4); // yaw / spin [0,360]
  Serial.print("\tPitch: ");
  Serial.print(BY, 4); // pitch [-180,180]
  Serial.print("\tRoll: ");
  Serial.print(BZ, 4); // roll [-180,180]
  displayCalStatus();
  /* New line for  next sample */
  Serial.println("");

  y = convertToPower(BX);
  p = convertToPower(BY);
  r = convertToPower(BZ);
  
  int LM = p + y;
  int RM = p - y;
  
  motorWrapper(LM, enablePinM1, M1Pin1, M1Pin0);
  motorWrapper(RM, enablePinM2, M2Pin1, M2Pin0);
  //delay(1000);

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

