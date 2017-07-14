#ifndef GtHeartBeat_h
	#define GtHeartBeat_h
	#include "Arduino.h"
	
	class GtHeartBeat{
		public:
			GtHeartBeat();
			GtHeartBeat(int ledPort, int blinkInterval, int ledOnTime);
			void setup(); //to be called from setup()
			boolean heartbeatLoopCall();
		private:
			void initialize();
			int m_led;
			long m_lastLedBlinkMillis;
			int m_ledOnMillis;
			int m_ledBlinkIntervalMillis;
			boolean m_ledOn;
	};
#endif