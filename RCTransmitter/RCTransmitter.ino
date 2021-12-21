#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>

#define VRX A1
#define VRY A2
#define BUTTON 2
int startx, starty;
int jox, joy;
int loopdelay = 1;


struct RC_Package {
  int OutX;
  int OutY;
  int Button;
};

RC_Package data;
RF24 radio(7, 8); // CE, CSN 的引脚
const byte address[6] = "54188";  //设置标识码

void setup() {
  pinMode(VRX, INPUT);
  pinMode(VRY, INPUT);
  pinMode(BUTTON, INPUT);
  Serial.begin(9600);

  //得到摇杆在中位是的数值
  startx = analogRead(VRX);
  starty = analogRead(VRY);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

}
void loop() {


  jox = analogRead(VRX);//读取x轴的数值，范围为0~1023
  joy = analogRead(VRY);//读取y轴的数值，范围为0~1023


  //x
  if (jox > startx)
  {
    data.OutX = map(jox,startx,1023,0,15);
  }
  else
  {
    data.OutX = map(jox,0,startx,-15,0);
  }

  Serial.print("outX:");
  Serial.print(data.OutX);
  
  //y
  if (joy > starty)
  {
    data.OutY = map(joy,starty,1023,0,-15);
  }
  else
  {
    data.OutY = map(joy,0,starty,15,0);
  }
  Serial.print(",outY:");
  Serial.print(data.OutY);

  
  if (digitalRead(BUTTON) == LOW)
  {
    data.Button = 0;
    Serial.print(",Button:");
    Serial.println(data.Button);
  }
  else
  {
    data.Button = 1;
    Serial.print(",Button:");
    Serial.println(data.Button);
  }

    radio.write(&data, sizeof(RC_Package));

  delay(loopdelay);
}
