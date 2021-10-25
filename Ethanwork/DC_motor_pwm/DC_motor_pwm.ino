
#define IN3  3
#define IN4  4
#define ENB  5
#define MINPWM 25
#define MAXPWM 65
volatile unsigned long JSQ = 0;
uint32_t timer;

void externalIntFun() {
  JSQ++;
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), externalIntFun, CHANGE);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  timer = micros();
  JSQ = 0;

}

void loop() {


  //////////////////////////////
  /////顺时针加速到最快MAXPWM/////
  //////////////////////////////
  int n = MINPWM;
  digitalWrite(IN3, LOW); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, HIGH);

  while (n < MAXPWM)
  {
    analogWrite(ENB, n); //ENA要输出PWM信号（0-255）
    plot("pwm", n, false);
    double rpm = calculateRpm();
    plot("rpm", rpm, true);
    delay(400);
    n++;
  }

  delay(400);

  //////////////////////////////
  //顺时针到减速到MINPWM
  //////////////////////////////
  while (n > MINPWM)
  {
    analogWrite(ENB, n); //ENA要输出PWM信号（0-255）
    plot("pwm", n, false);
    double rpm = calculateRpm();
    plot("rpm", rpm, true);
    delay(400);
    n--;
  }

  //////////////////////////////
  //完全停止
  //////////////////////////////
  digitalWrite(IN3, LOW); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, LOW);
  //Serial.println("Stop");
  delay(400);

  //////////////////////////////
  //逆时针加速到最快MAXPWM
  //////////////////////////////
  digitalWrite(IN3, HIGH); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, LOW);

  while (n < MAXPWM)
  {
    analogWrite(ENB, n); //ENA要输出PWM信号(0-255)
    plot("pwm", n, false);
    double rpm = calculateRpm();
    plot("rpm", rpm, true);
    delay(400);
    n++;
  }

  delay(400);

  //////////////////////////////
  //逆时针到减速到MINPWM
  //////////////////////////////
  while (n > MINPWM)
  {
    analogWrite(ENB, n); //ENA要输出PWM信号（0-255）
    plot("pwm", n, false);
    double rpm = calculateRpm();
    plot("rpm", rpm, true);
    delay(400);
    n--;
  }

  delay(400);

  //////////////////////////////
  //完全停止
  //////////////////////////////
  digitalWrite(IN3, LOW); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, LOW);
  //Serial.println("Stop");
  delay(400);


}

void plot(String label, double value, bool last)
{
  Serial.print(label);

  if (label != "")
  {
    Serial.print(":");
  }
  Serial.print(value);

  if (last == false)
  {
    Serial.print(",");
  }
  else
  {
    Serial.println();
  }

}

double calculateRpm()
{
  noInterrupts();
  uint32_t dT = micros() - timer;
  double actRpm = JSQ * 19230.77 / dT;
  JSQ = 0;
  timer = micros();
  interrupts();
  return actRpm;
}
