//#include "Arduino.h"
#include "GtHeartBeat.h"

GtHeartBeat::GtHeartBeat(){
	initialize();
}

GtHeartBeat::GtHeartBeat(int ledPort, int blinkInterval, int ledOnTime){
	initialize();
	m_led = ledPort;
	m_ledBlinkIntervalMillis = blinkInterval;
	m_ledOnMillis = ledOnTime;
}


void GtHeartBeat::initialize() {
	m_lastLedBlinkMillis = 0;
	m_ledOn = false;
	
	//By default 10 ms per second blink on port 13
	m_led = 13;
	m_ledOnMillis = 10;
	m_ledBlinkIntervalMillis = 1000;
}

void GtHeartBeat::setup() {
	pinMode(m_led, OUTPUT);
}
/*
Blinks on-board LED for 'ledOnMillis' every 'ledBlinkIntervalMillis' interval
Returns true when beat is triggered.
*/
boolean GtHeartBeat::heartbeatLoopCall() {
	boolean beat = false;
	long currentMillis = millis();
  
	if (!m_ledOn){
		if (currentMillis - m_lastLedBlinkMillis > m_ledBlinkIntervalMillis) {
		m_lastLedBlinkMillis = currentMillis;
		beat = true;
		// turn led on:
		m_ledOn = true;
		digitalWrite(m_led, m_ledOn);
		}
	}
	else { //led is on, note that lastLedBlinkMillis is the time the LED was turned on
		if (currentMillis - m_lastLedBlinkMillis > m_ledOnMillis){
		// turn led off:
		m_ledOn = false;
		digitalWrite(m_led, m_ledOn);
		}
	}
	return beat;
}