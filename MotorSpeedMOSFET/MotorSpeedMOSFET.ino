/* 
 *  Danielle Le 
 *  Team Handroid 
 *  7/21/17 
 *  This example shows how to control the speed of a DC motor an LED on pin 9 using the analogWrite() function. 
 *  This example is based on the Arduino Example Fade sketch but modified to use timing instead of the delay() function
 */

// how fast the motor runs 
int turns = 0; 

//how many turns the motor makes 
int turnAmount = 1; 

//setting a number to current time (unsigned long are extended variables for number 
//storage, and store 32 bits. Unlike standard longs, they won't store negative numbers, making 
//their range very wide).  
unsigned long currentTime; 

//setting a time for infinite time
unsigned long loopTime;


void setup() {
//declaring pin 9 to an output:
  pinMode (9,OUTPUT); 
  currentTime = millis (); 
  loopTime = currentTime; 

}

void loop() {
  currentTime= millis();
  if(currentTime >= (loopTime + 20)) { 
//set the speed of pin 9:
  analogWrite (9, turns); 

// changing the turnings for the next time through the loop: 
  turns = turns + turnAmount;

// speed up or slow down the motor 
  if (turns == 0 || turns == 255) { 
    turnAmount = -turnAmount; 
     }
  if (turns == 0) {
    //5 second delay 
     delay (5000); 
  }
  //Updates the loopTime 
  loopTime = currentTime; 
  }

}
