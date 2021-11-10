
int v=-1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial.println("car setup");
  Serial3.println("car setup");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial3.available() > 0) {
    v = Serial3.read();
  }

  if (v != -1)
  {
    
    Serial.println(v);
    Serial3.println(v);
  }

  v = -1;

}
