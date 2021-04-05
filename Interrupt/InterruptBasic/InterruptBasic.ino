int pin = 2; //define interrupt pin to 2
volatile int state = LOW; // To make sure variables shared between an ISR
//the main program are updated correctly,declare them as volatile.

void setup() {
   pinMode(13, OUTPUT); //set pin 13 as output
   attachInterrupt(digitalPinToInterrupt(pin), blink, CHANGE);
   //interrupt at pin 2 blink ISR when pin to change the value
} 
void loop() { 
   digitalWrite(13, state); //pin 13 equal the state value
} 

void blink() { 
   //ISR function
   state = !state; //toggle the state when the interrupt occurs
}
