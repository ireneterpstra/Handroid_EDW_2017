#ifndef GtGiro_h
	#define GtGiro_h
	#include "Arduino.h"
	#include <stdlib.h>
	#include <math.h>

	//This library encodes 50 ms update interval for DMP:
	#include "MPU6050_6Axis_MotionApps20_50ms.h"
	
	//#define debug
	//#define OUTPUT_READABLE_YAWPITCHROLL

	class GtGiro {
		public:
			GtGiro();
			GtGiro(uint8_t address);//0x69 or 0x68
			GtGiro(uint8_t address, int fifoResetInterval);
			GtGiro(uint8_t address, int fifoResetInterval, float maxYawRateOfChange);
			void setup(); //to be called from setup()
			boolean readGiroLoopCall(); //updates the yaw/pitch/roll if time since last successful read exceeds update time
			double getYaw(); //returns current yaw (between -180 and +180) as computed by DMP.
			double getPitch();
			double getRoll();
			double getAdjustedYaw(); //returns current yaw corrected by yawOffset
			void resetAdjustedYaw(); //sets yawOffset based on current heading to ensure adjustedYaw is zero at current heading
			uint16_t getAdjustedYawAsDacValue(int lb, int ub);
			uint16_t getAdjustedHysteresisYawAsDacValue(int lb, int ub);
			double getAdjustedHysteresisYaw();
			boolean getHysteresisMode();
			boolean isDoubleStableYaw();
			long getInstableFifoResetCount();

		private:
			MPU6050 mpu;
			bool dmpReady;  		//set true if DMP init was successful
			uint8_t mpuIntStatus;   //holds actual interrupt status byte from MPU
			uint8_t devStatus;      //return status after each device operation (0 = success, !0 = error)
			uint16_t packetSize;    //expected DMP packet size (default is 42 bytes)
			uint16_t fifoCount;     //count of all bytes currently in FIFO
			uint8_t fifoBuffer[64]; //FIFO storage buffer

			//orientation/motion vars
			Quaternion q;           //[w, x, y, z]         quaternion container
			VectorFloat gravity;    //[x, y, z]            gravity vector
			float ypr[3];           //[yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

			boolean read6050Fifo(); //returns true if read was successful.

			uint16_t startReadFifoCount;
			int dmpUpdateTime; //DMP update time in milliseconds
			long lastGiroReadMillis; //last time (in millis) that the Giro was read successfully
			boolean m_enablePeriodicFifoReset;
			int m_fifoResetInterval; //time between periodic FIFO resets
			long m_lastFifoResetMillis; //last time the FIFO was reset: idea is to regularly reset the FIFO to recover from disruptions
			
			float m_maxYawRateOfChange;
			boolean m_enableMaxYawRateOfChange;
			
			void printYPR();
			void setYawPitchRoll();
			double m_yaw;
			double m_pitch;
			double m_roll;
			double yawOffset; //offset in order to reset yaw to zero at given orientation
			double m_previousYaw;
			boolean m_stableYaw; //if true, the yaw is stable and thus the previous yaw is reliable
			boolean m_previousStableYaw;
			long m_instableFifoResetCount;//for debugging only. Counts the number of instable FIFO reset counts.
			
			void initialize();

			boolean m_yawHysteresisLowMode;
			
			boolean isYawStable();
			float getAbsoluteDegreesDifference(float degrees1, float degrees2);
	};
#endif