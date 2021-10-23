
#define IN3  3
#define IN4  4
#define ENB  5
#define MINPWM 75
#define MAXPWM 255

void setup() {
  //设置串口波特率
  Serial.begin(9600);
  
  //设置引脚模式
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  //////////////////////////////
  //顺时针加速到最快MAXPWM
  //////////////////////////////
  int n = MINPWM;
  digitalWrite(IN3, LOW); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, HIGH);

  while (n < MAXPWM)
  {
    analogWrite(ENB, n); //ENA要输出PWM信号（0-255）
    Serial.print("CW:");
    Serial.println(n);
    delay(100);
    n++;
  }

  delay(500);
  
  //////////////////////////////
  //顺时针到减速到MINPWM
  //////////////////////////////
  while (n > MINPWM)
  {
    analogWrite(ENB, n); //ENA要输出PWM信号（0-255）
    Serial.print("CW:");
    Serial.println(n);
    delay(100);
    n--;
  }

  //////////////////////////////
  //完全停止
  //////////////////////////////
  digitalWrite(IN3, LOW); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, LOW);
  Serial.println("Stop");
  delay(100);

  //////////////////////////////
  //逆时针加速到最快MAXPWM
  //////////////////////////////
  digitalWrite(IN3, HIGH); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, LOW);

  while (n < MAXPWM)
  {
    analogWrite(ENB, n); //ENA要输出PWM信号(0-255)
    Serial.print("CCW:");
    Serial.println(n);
    delay(100);
    n++;
  }

  delay(500);
  
  //////////////////////////////
  //逆时针到减速到MINPWM
  //////////////////////////////
  while (n > MINPWM)
  {
    analogWrite(ENB, n); //ENA要输出PWM信号（0-255）
    Serial.print("CCW:");
    Serial.println(n);
    delay(100);
    n--;
  }

  delay(500);
  
  //////////////////////////////
  //完全停止
  //////////////////////////////
  digitalWrite(IN3, LOW); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, LOW);
  Serial.println("Stop");
  delay(100);


}
