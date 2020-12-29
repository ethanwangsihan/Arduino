void setup() {
  pinMode(3,OUTPUT);
}

void loop() {
  for(int i=0; i<=250; i++)
  {
    analogWrite(3,i);
    delay(5);
  }

  for(int i=250; i>=0; i--)
  {
    analogWrite(3,i);
    delay(5);
  }
}
