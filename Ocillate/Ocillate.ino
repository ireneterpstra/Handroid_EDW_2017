#include <Event.h>
#include <Timer.h>

Timer t;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  t.oscillate(LED_BUILTIN, 500, LOW);
}

// the loop function runs over and over again forever
void loop() {
  t.update();
  delay(100);
}
