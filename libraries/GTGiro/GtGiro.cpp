#include "Arduino.h"
#include "GtGiro.h"

#ifdef debug
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTF(x, y) Serial.print(x, y)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTF(x, y)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTLNF(x, y)
#endif

GtGiro::GtGiro() {
	initialize();
}

GtGiro::GtGiro(uint8_t address)
	: mpu(address)
	{
	initialize();
}

GtGiro::GtGiro(uint8_t address, int fifoResetInterval)
	: mpu(address)
	{
	initialize();
	if (fifoResetInterval > 0){
		m_fifoResetInterval = fifoResetInterval;
		m_enablePeriodicFifoReset = true;
	}
}

/**
maxYawRateOfChange: maximum number of degrees difference between successive yaw reads.
If exceeds, assume due to i2c errors/fifo misallignment. Need to reset the FIFO
*/
GtGiro::GtGiro(uint8_t address, int fifoResetInterval, float maxYawRateOfChange)
	: mpu(address)
	{
	initialize();
	if (fifoResetInterval > 0){
		m_fifoResetInterval = fifoResetInterval;
		m_enablePeriodicFifoReset = true;
	}
	if (maxYawRateOfChange > 0){
		m_maxYawRateOfChange = maxYawRateOfChange;
		m_enableMaxYawRateOfChange = true;
	}
}

void GtGiro::initialize() {
	dmpUpdateTime = 50; //milli seconds Make sure this is consistent with the value in MPU6050_6Axis_MotionApps20_50ms.h
	dmpReady = false;
	startReadFifoCount = 0;
	lastGiroReadMillis = 0;
	m_enablePeriodicFifoReset = false;
	m_fifoResetInterval = 1000; //10 seconds
	m_lastFifoResetMillis = 0; 
	yawOffset = 0.0;
	m_yawHysteresisLowMode = true;
	m_maxYawRateOfChange = 10; //10 degrees per 50ms interval
	m_enableMaxYawRateOfChange = false;
	
	m_previousYaw = 0;
	m_stableYaw = false;
	m_previousStableYaw = false;
	m_instableFifoResetCount = 0;
}

void GtGiro::setup() {
  //join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; //400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif


  //NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  //Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  //the baud timing being too misaligned with processor ticks. You must use
  //38400 or slower in these cases, or use some kind of external separate
  //crystal solution for the UART timer.

  //initialize device
  DEBUG_PRINTLN(F("Initializing I2C devices..."));
  mpu.initialize();

  //verify connection
  DEBUG_PRINTLN(F("Testing device connections..."));
  DEBUG_PRINTLN(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  //wait for ready
  DEBUG_PRINTLN(F("\nSend any character to begin DMP programming and demo: "));

  //load and configure the DMP
  DEBUG_PRINTLN(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    //turn on the DMP, now that it's ready
    DEBUG_PRINTLN(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();

    //set our DMP Ready flag so the main loop() function knows it's okay to use it
    DEBUG_PRINTLN(F("DMP ready!"));
    dmpReady = true;

    //get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    //ERROR!
    //1 = initial memory load failed
    //2 = DMP configuration updates failed
    //(if it's going to break, usually the code will be 1)
    DEBUG_PRINTLN(F("DMP Initialization failed (code "));
    DEBUG_PRINTLN(devStatus);
    DEBUG_PRINTLN(F(")"));
  }
}
//================================================================
//===               INTERRUPT DETECTION ROUTINE                ===
//================================================================
/*
volatile bool mpuInterrupt = false;     //indicates whether MPU interrupt pin has gone high
long lastInterruptTime = 0;
int interruptDelay = 0;
void dmpDataReady() {
    mpuInterrupt = true;

    //VT: To be able to detect interrupt rate. Seems to be about 10 milli-seconds
    long currentTime = millis();
    interruptDelay = currentTime - lastInterruptTime;
    lastInterruptTime = currentTime;
}
*/

/**
 * Reads Yaw, Pitch Roll data from the DMP-FIFO into fifoBuffer.
 * Updates fifoCount
 * If FIFO doesn't yet have enough data, it waits for the FIFO to get filled.
 * If the FIFO has overflown, it resets the FIFO and returns false.
 * Make sure to call this function again before the FIFO overflows again.
 */
boolean GtGiro::read6050Fifo() {
	uint8_t mpuIntStatus = mpu.getIntStatus();

  //get current FIFO count
  fifoCount = mpu.getFIFOCount();

  //check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    //reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
    return false;
  }
  //otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02) {
    startReadFifoCount = fifoCount;
    //wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		boolean success = false;

    //Read as many packets from FIFO as possible to avoid FIFO overflow
    while (fifoCount >= packetSize) {
      //read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      //track FIFO count here in case there is > 1 packet available
      //(this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      success = true;
    }
    return success;
  }
  return false;
}

/*
* Sets yaw/pitch/roll attributes in degrees between -180 and +180
*/
void GtGiro::setYawPitchRoll() {
  m_previousYaw = m_yaw;
  m_previousStableYaw = m_stableYaw;
  //Get giro values:
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  //Compute yaw in degrees between -180 and +180:
  m_yaw = ypr[0] * 180/M_PI;
  m_pitch = ypr[1] * 180/M_PI;
  m_roll = ypr[2] * 180/M_PI;
  m_stableYaw = isYawStable();
}

/**
* Gets the value from the 9 axis
* Computes a yaw in degrees between 0 and 380
* Converts to int value between lb (e.g. 0) and ub (e.g. 4095) for the DAC.
*/
uint16_t GtGiro::getAdjustedYawAsDacValue(int lb, int ub) {
	//Need to convert -180 to 180 to lb (e.g. 0) to ub (e.g. 4095):
	int dacRange = ub - lb;
	uint16_t giroDacValue = lb + ((getAdjustedYaw() + 180.0) / 360.0) * (double)dacRange;

	return giroDacValue;
}

uint16_t GtGiro::getAdjustedHysteresisYawAsDacValue(int lb, int ub) {
	//Need to convert -200 to 200 to lb (e.g. 0) to ub (e.g. 4095):
	int dacRange = ub - lb;
	uint16_t giroDacValue = lb + ((getAdjustedHysteresisYaw() + 200.0) / 400.0) * (double)dacRange;

	return giroDacValue;
}

/*
* Should be called frequently.
* Uses millis() to ensure only reads giro based on the DMP update interval.
* dmpUpdateTime is the number of miliseconds of the DMP update interval
* Returns true if new value ready
*/

boolean GtGiro::readGiroLoopCall() {
	boolean success = false;
	long currentMillis = millis();

	if (currentMillis - lastGiroReadMillis > dmpUpdateTime) {
		//Read FIFO
		success = read6050Fifo(); //Returns false if FIFO was overflown. If so, resets FIFO.

		if (success) {
			lastGiroReadMillis = currentMillis;
			//set the yaw/pitch/roll private attributes
			setYawPitchRoll();
			
			
			if (m_enableMaxYawRateOfChange){
				//Test if yaw has become instable.
				//I.e. if yaw was stable but not anymore
				if (m_previousStableYaw && !m_stableYaw){
					//reset FIFO
					mpu.resetFIFO();
					success = false; //mark latest value as not reliable
					DEBUG_PRINTLN(F("Instable FIFO reset"));
					m_previousStableYaw = false;
					m_stableYaw = false;
					m_instableFifoResetCount++; //increment event counter (just for debugging)
				}
				
				if (!m_previousStableYaw || !m_stableYaw){
					success = false; //don't use the value unless it has been stable at least twice in a row
				}
			}

			#ifdef OUTPUT_READABLE_YAWPITCHROLL
				if (currentMillis - lastPrintMillis > 500) {
					lastPrintMillis = currentMillis;
					printYPR();
				}
			#endif
		}
		
		//Periodically reset the FIFO to recover from disruptions
		//Independent of success or not
		if (m_enablePeriodicFifoReset && (currentMillis - m_lastFifoResetMillis > m_fifoResetInterval)){
			m_lastFifoResetMillis = currentMillis;
			mpu.resetFIFO();
			DEBUG_PRINTLN(F("Periodic FIFO reset"));
		}		
	}
	return success;
}

void GtGiro::printYPR() {
	//display Euler angles in degrees
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	Serial.print("ypr\t");
	Serial.print(ypr[0] * 180.0 / M_PI);
	Serial.print("\t");
	Serial.print(ypr[1] * 180.0 / M_PI);
	Serial.print("\t");
	Serial.print(ypr[2] * 180.0 / M_PI);
	//Serial.print("\t");
	//Serial.print(interruptDelay);
	Serial.print("\tinstableCount\t");
	Serial.println(m_instableFifoResetCount);
}

double GtGiro::getYaw() {
	return m_yaw;
}

double GtGiro::getPitch() {
	return m_pitch;
}

double GtGiro::getRoll() {
	return m_roll;
}

/*
* Returns the yaw with the yawOffset applied.
* Then corrected to ensure result is within -180 and +180
*/
double GtGiro::getAdjustedYaw() {
	double adjustedYaw = getYaw() - yawOffset;
	if (adjustedYaw > 180.0) adjustedYaw -= 360.0;
	else if (adjustedYaw <= -180.0) adjustedYaw += 360.0;
	return adjustedYaw;
}
/*
* Sets the yawOffset in order to make the adjustedYaw zero at the current heading
*/
void GtGiro::resetAdjustedYaw() {
	yawOffset = m_yaw;
}

/*
* Provides a yaw with a range <-200,+200].
* This means there are duplicate values for the 'same' yaw. E.g. -190 == +170
* Uses hysteresis to switch between 'low' and 'high' mode to avoid the yaw to
* move frequently between the extreme values in the range.
*/
double GtGiro::getAdjustedHysteresisYaw() {
	double ayaw = getAdjustedYaw(); //range <-180,180]
	double hyaw = 0;

	/*
	Determine the hysteresis mode
	Mode definitely needs to jump once the yaw exceeds the ends of the boundary.
	That is:
	in low-mode: hyaw > -200 == ajaw < 160
	in high-mode: hyaw > +200 == ajaw > -160
	This means on this end, there is a range of 40 degrees where the giro can swing without a jump in mode.
	On the other side, make sure mode doesn't jump back too frequently.
	Chosen for a 250 degree range. That is:
	in low-mode if ajaw > +90
	in high-mode if ajaw < -90
	This means that the yaw has a 70 degree range where it will switch modes.
	*/
	if (m_yawHysteresisLowMode) {
		//hyaw range = <-200,+160]
		//switch to other mode if close to extremes of range: yaw < -180 || yaw > 140
		if (ayaw > 90 && ayaw < 160) m_yawHysteresisLowMode = false;
	}
	else {
		//range = <-160,+200]
		//switch to other mode if close to extremes of range: yaw < -140 || yaw > 180
		if (ayaw < -90 && ayaw > -160) m_yawHysteresisLowMode = true;
	}


	//Depending on the mode, shift the range:
	if (m_yawHysteresisLowMode) {
		//range = <-200,+160]
		if (ayaw > 160) hyaw = ayaw - 360;
		else hyaw = ayaw;
	}
	else {
		//range = <-160,+200]
		if (ayaw <= -160) hyaw = ayaw + 360;
		else hyaw = ayaw;
	}
	return hyaw;
}

boolean GtGiro::getHysteresisMode() {
	return m_yawHysteresisLowMode;
}

/*
Checks if the yaw is stable, i.e. the difference between the current yaw and the previous yaw is less then the m_maxYawRateOfChange
*/
boolean GtGiro::isYawStable(){
	boolean stable = false;
	float diff = getAbsoluteDegreesDifference(m_yaw, m_previousYaw);
	if (diff <= m_maxYawRateOfChange){
		stable = true;
	}
	return stable;
}

/*
Returns the absolute difference between two degrees, normalized between 0 and 180
Assume only relevant for differences > 1 degree and no need for a precise difference
*/
float GtGiro::getAbsoluteDegreesDifference(float degrees1, float degrees2){
	//float diff = fmod(abs(degrees1 - degrees2), 360);
	float diff = abs(degrees1 - degrees2); 
	
//	int idiff = (int) diff;
//	idiff = idiff % 360;
//	if (idiff > 180) {
//		idiff = idiff - 360;
//		idiff = abs(idiff);
//	}
	
	diff = fmod(diff, 360);
	if (diff > 180) {
		diff = diff - 360;
		diff = abs(diff);
	}
//	DEBUG_PRINT("d1=");
//	DEBUG_PRINT(degrees1);
//	DEBUG_PRINT(" d2=");
//	DEBUG_PRINT(degrees2);
//	DEBUG_PRINT(" diff=");
//	DEBUG_PRINTLN(diff);
	return diff;
	//return fmod(abs(degrees1 - degrees2), 360);
}

/*
Returns true if the yaw value has been stable twice in a row
*/
boolean GtGiro::isDoubleStableYaw(){
	return (m_previousStableYaw && m_stableYaw);
}

/*
Returns the instableFifoResetCount.
This counts the number of instable events detected.
This gives an idea if any and how many of these events happened.
*/
long GtGiro::getInstableFifoResetCount(){
	return m_instableFifoResetCount;
}