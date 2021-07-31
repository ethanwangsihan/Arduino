#include <SoftwareSerial.h>
SoftwareSerial mine_craft = SoftwareSerial(10, 11);
char v;
String s;
void setup() {
  Serial.begin(9600);
  mine_craft.begin(38400);
}

void loop() {
  if (Serial.available() > 0)
  {
    v = Serial.read();
    mine_craft.write (v);
  }
  while (mine_craft.available() > 0)
  {
    s = mine_craft.readString();
    Serial.println(s);
  }
}
