float PIE=3.14159265;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i=0; i<360; i++)
  {
    //plot("angle",i,false);
    plot("sin",sin(i*PIE/180),false);
    plot("cos",cos(i*PIE/180),true);
    delay(50);
  }
}

void plot(String label, float value, bool last)
{
  Serial.print(label);

  if (label != "")
  {
    Serial.print(":");
  }
  Serial.print(value);

  if (last==false)
  {
    Serial.print(",");
  }
  else
  {
    Serial.println();
  }
  
}
