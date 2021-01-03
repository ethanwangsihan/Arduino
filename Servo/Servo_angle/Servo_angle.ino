#include <Servo.h>

Servo myservo;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(9);
}

void loop() {


  for (int pos = 0; pos <= 180; pos++)
  {
    myservo.write(pos);
    delay(100);
  }
  
}
