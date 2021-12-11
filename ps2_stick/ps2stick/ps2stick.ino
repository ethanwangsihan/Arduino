#define VRX A1
#define VRY A2
#define SW 6
int startx,starty;
int jox, joy;
int outx, outy;
int loopdelay=10;
void setup() {
  pinMode(VRX, INPUT);
  pinMode(VRY, INPUT);
  pinMode(SW, INPUT);
  Serial.begin(9600);

  //得到摇杆在中位是的数值
  startx=analogRead(VRX); 
  starty=analogRead(VRY);


}
void loop() {

  
  jox = analogRead(VRX);//读取x轴的数值，范围为0~1023
  joy = analogRead(VRY);//读取y轴的数值，范围为0~1023

  //x
  if (jox>startx)
  {
    outx=map(jox,startx,1023,0,15);
  }
  else
  {
    outx=map(jox,0,startx,-15,0);
  }
  //y
  if (joy>starty)
  {
    outy=map(joy,starty,1024,0,-15);
  }
  else
  {
    outy=map(joy,0,starty,15,0);
  }

  Serial.print("outX:");
  Serial.print(outx);
  Serial.print(",outY:");
  Serial.println(outy);
  
  delay(loopdelay);
}
