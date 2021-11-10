#include <SoftwareSerial.h>


#define RXPIN  2
#define TXPIN  3

SoftwareSerial m = SoftwareSerial(RXPIN, TXPIN);
int v=-1;

void setup() {
  // put your setup code here, to run once:
  m.begin(9600);
  Serial.begin(9600);
  m.println("car setup");
  Serial.println("car setup");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (m.available() > 0) {
    v = m.read();
  }

  if (v != -1)
  {
    m.write(v);
    Serial.println(v);
  }

  v = -1;

}
