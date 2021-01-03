int analogReadPin=1;
int ledPin=9;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int val=analogRead(analogReadPin);
  val = map(val, 0, 1023, 0, 255);
  analogWrite(ledPin,val);
  Serial.println(val);
  delay(10);
}
