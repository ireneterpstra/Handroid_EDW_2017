void startLED(int onDuration){
  //Serial.print(F("LED on, Duration: "));
  //Serial.println(onDuration);
  digitalWrite(LED, HIGH); 
  LEDTimer = t.after(onDuration, stopLED);
  //Serial.print(F("Timer: "));
  //Serial.println(LEDTimer);
}

void stopLED(){
  //Serial.println(F("LED off"));
  digitalWrite(LED, LOW);
  //t.stop(LEDTimer);
}
