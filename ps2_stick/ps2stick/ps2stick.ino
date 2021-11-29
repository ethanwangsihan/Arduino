#define VRX A1
#define VRY A2
#define SW 6
int resetx,resety;
int jox, joy, jos;
int outx, outy;
int loopdelay=10;
void setup() {
  pinMode(VRX, INPUT);
  pinMode(VRY, INPUT);
  pinMode(SW, INPUT);
  Serial.begin(9600);

  //得到摇杆在中位是的数值
  resetx=analogRead(VRX); 
  resety=analogRead(VRY);
  
}
void loop() {
  
  
  jox = analogRead(VRX);//读取x轴的数值，范围为0~1023
  outx=map(jox-resetx, 0,1023,0,255);
  
  joy = analogRead(VRY);//读取y轴的数值，范围为0~1023
  outy=map(joy-resety, 0,1023,0,255);
  jos = digitalRead(SW); //读取按键的数值，为0和1
  
  Serial.print("X=");
  Serial.print(outx);
  Serial.print(" |Y=");
  Serial.print(outy);
  Serial.print(" |Z=");
  Serial.println(jos);
  delay(loopdelay);
}
