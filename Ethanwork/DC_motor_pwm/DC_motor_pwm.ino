
#define IN3  37
#define IN4  39
#define ENB  41

volatile unsigned long JSQ = 0; //用来收集电机霍尔传感发来的脉冲数量
uint32_t timer; //用来保存霍尔传感器计数的开始时间

double targetRpm = 4000;

double Kp = 0.001, Ki=0, Kd=0.00;
double lastError;
double ErrorIntegral;
int pwm = 0;
int loopdelay=100;

//由数字管脚2的中断触发
void externalIntFun() {
  JSQ++;
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(21), externalIntFun, CHANGE); //注册中断关联的函数externalIntFun
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  timer = micros();
  JSQ = 0;

}

void loop() {

  noInterrupts(); //停止响应中断
  uint32_t now = micros();
  uint32_t dT = now - timer; //计算霍尔传感器计数用的时间
  double actRpm = JSQ * 2727272.73 / dT; //用霍尔传感器计数除以计数时间计算每分钟的电机转速, 电机每转一圈发送26个霍尔传感器脉冲, dT的单位是微秒(百万分之一秒)
  JSQ = 0; //计算转速后, 计数器清零, 为下次计算转速做准备.
  timer = micros(); // 计时器清零
  interrupts(); //恢复中断响应

  plot("Rpm", actRpm, true);

  double error = targetRpm - actRpm;

  pwm = pwm + Pterm(error)+Dterm(error, dT);

  /*
    double error=targetRpm-actRpm;

    //计算根据P参数得到的调整Pwm
    double PWMp=Pterm(error);

    //计算根据D参数得到的调整Pwm
    double PWMd=Dterm(error,dT);

    //计算根据I参数得到的调整Pwm
    double PWMi=Dterm(error,dT);

    double PWMall=PWMp+PWMi+PWMd;

    if (PWMall>255)
    {
    PWMall=255;
    }
    else if (PWMall <0)
    {
    PWMall=0;
    }


    analogWrite(ENB, (int)PWMall); //ENA要输出PWM信号（0-255）
  */

  digitalWrite(IN3, LOW); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, HIGH);

  if (pwm > 255)
  {
    pwm = 255;
  }
  else if (pwm < 0)
  {
    pwm = 0;
  }
  analogWrite(ENB, 128);
  delay(loopdelay);

}


//在串口绘图仪上画点
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

//////////////////////////////
//计算电机当前转速
//////////////////////////////

double calculateRpm(uint32_t now)
{
  noInterrupts(); //停止响应中断
  uint32_t dT = now - timer; //计算霍尔传感器计数用的时间
  double actRpm = JSQ * 19230.77 / dT; //用霍尔传感器计数除以计数时间计算每分钟的电机转速, 电机每转一圈发送26个霍尔传感器脉冲, dT的单位是微秒(百万分之一秒)
  JSQ = 0; //计算转速后, 计数器清零, 为下次计算转速做准备.
  timer = micros(); // 计时器清零
  interrupts(); //恢复中断响应
  return actRpm;
}

//比例调节
double Pterm(double error)
{
  double PWMp = Kp * error;
  return PWMp;
}

double Dterm(double error, uint32_t dt)
{
  double dError = error - lastError;
  double errorSpeed = dError / dt;
  double PWMd = Kd * errorSpeed;
  lastError = error;
  return PWMd;
}

double Iterm(double error, uint32_t dt)
{
  ErrorIntegral = ErrorIntegral + error * dt;
  double PWMi = Ki * ErrorIntegral;
  return PWMi;
}
