
#define IN3  2
#define IN4  4
#define ENB  6
int n = 0;
int j = 40;
void setup() {
  // put your setup code here, to run once:
  pinMode(IN3,OUTPUT); //设置引脚模式
  pinMode(IN4,OUTPUT);
  pinMode(ENB,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  //卯卯的代码
  /*
  while (n < 40)
  {
  digitalWrite(IN3,LOW); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4,HIGH);
  analogWrite(ENB,n); //ENA要输出PWM信号（0-255）
  delay(90);
  n++;
  delay(700)
  while (j > 0)
  {
  digitalWrite(IN3,HIGH); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4,LOW);
  analogWrite(ENB,j); //ENA要输出PWM信号（0-255）
  delay(90);
  n--;
  }
  */
  
}
