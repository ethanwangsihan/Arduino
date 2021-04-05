#include <LedControl.h>

int DIN = 12;

int CS = 11;

int CLK = 10;

LedControl lc = LedControl(DIN, CLK, CS, 1);

void setup() {
  // put your setup code here, to run once:
  lc.shutdown(0, false); //启动时,MAX72XX处于省电模式
  lc.setIntensity(0,8);
  lc.clearDisplay(0);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  lc.setRow(0,2,B10110000); 
  delay(1000);
  
  lc.setLed(0,3,3,HIGH);

  delay(1000);

  for (int j=0; j<8; j++)
  {
    lc.setLed(0,3,j,HIGH);
  }

  delay(1000);

  for(int i=0; i<8; i++)
  {
    for (int j=0; j<8; j++)
    {
      lc.setLed(0,i,j,HIGH);
    }
    
  }

  delay(1000);

  for(int i=0; i<8; i++)
  {
    for (int j=0; j<8; j++)
    {
      lc.setLed(0,i,j,LOW);
    }
    
  }
  
}
