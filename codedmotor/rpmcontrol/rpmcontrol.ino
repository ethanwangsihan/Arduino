#include <PID_v1.h>
//https://playground.arduino.cc/Code/PIDLibraryAdaptiveTuningsExample/
//https://playground.arduino.cc/Code/PIDLibaryBasicExample/
#include <FlexiTimer2.h>

#define IN3  26
#define IN4  28
#define ENB  6
#define MAXPWM 255

#define MINRPM 0
#define MAXRPM 4500

volatile long pulseNum = 0;

uint32_t timer;
int loopDelay = 10;
int idx = 0;


double Kp = 0.055, Ki = 0, Kd = 0.02;


enum motorMode {CW, CCW, STOP};
double TargetPoint;

void plot(String label, double value, bool last)
{
  Serial.print(label);

  if (label != "")
  {
    Serial.print(F(":"));
  }
  Serial.print(value);

  if (last == false)
  {
    Serial.print(F(","));
  }
  else
  {
    Serial.println();
  }
}

class CodedMotor
{
  private:
    int IN_1;
    int IN_2;
    int EN;

    motorMode mM;

    double Setpoint, Input, Output;

    PID myPID = PID(&Input, &Output, &Setpoint, 0, 0, 0, DIRECT);

  public:

    CodedMotor(int in1, int in2, int en, double kp, double ki, double kd) //构造函数
    {
      IN_1 = in1;
      IN_2 = in2;
      EN = en;

      myPID.SetTunings(kp, ki, kd);
      myPID.SetMode(AUTOMATIC);

      //stop motor first
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, LOW);
      mM = STOP;
      analogWrite(EN, 0);

    };

    void stop()  //停止电机
    {
      if (mM == STOP)
      {
        return;
      }

      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, LOW);
      mM = STOP;
      analogWrite(EN, 0);
      return;
    }

    outPutCW(double targetPoint, double inputPoint)
    {
      if (targetPoint > MAXRPM)
      {
        targetPoint = MAXRPM;
      }
      else if (targetPoint < MINRPM)
      {
        stop();
        return;
      }

      if (mM != CW)
      {
        mM = CW;
        digitalWrite(IN_1, LOW);
        digitalWrite(IN_2, HIGH);
      }

      Setpoint = targetPoint;
      Input = inputPoint;

      myPID.Compute();

      if (Output > MAXPWM)
      {
        Output = MAXPWM;
      }

      analogWrite(EN, Output);

    }

    outPutCCW(double targetPoint, double inputPoint)
    {
      if (targetPoint > MAXRPM)
      {
        targetPoint = MAXRPM;
      }
      else if (targetPoint < MINRPM)
      {
        stop();
        return;
      }

      if (mM != CCW)
      {
        mM = CCW;
        digitalWrite(IN_1, HIGH);
        digitalWrite(IN_2, LOW);
      }

      Setpoint = targetPoint;
      Input = inputPoint;

      myPID.Compute();

      if (Output > MAXPWM)
      {
        Output = MAXPWM;
      }

      analogWrite(EN, Output);

    }

    motorMode getMode()
    {
      return mM;
    }
};


CodedMotor cm1 = CodedMotor(IN3, IN4, ENB, Kp, Ki, Kd);

void setup() {


  Serial.begin(9600);
  //设置引脚模式
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(2, INPUT);

  attachInterrupt(digitalPinToInterrupt(20), encoderISR, CHANGE);

  timer = 0;

  FlexiTimer2::set(50, timerIntFun);         /* 设置 （每【100】毫秒执行一次 定时器中断函数）*/
  FlexiTimer2::start();                                          /* 开始  启动这个定时器中断 */

}

void loop() {

  //获取电机编码器的计数, 得到转速
  noInterrupts();
  if (timer == 0)
  {
    //for the first time
    timer = micros();
    pulseNum = 0;
    interrupts();
    return;
  }

  uint32_t dT = micros() - timer;

  double actRpm = pulseNum * 2727272.7 / dT;
  pulseNum = 0;
  timer = micros();
  interrupts();

  if (TargetPoint > 0)
  {
    cm1.outPutCW(TargetPoint, actRpm);
  }
  else if (TargetPoint < 0 )
  {
    cm1.outPutCCW( -TargetPoint, actRpm);
  }
  else if (TargetPoint == 0)
  {
    cm1.stop();
  }
/*
  Serial.print(F("Target:"));
  Serial.println(TargetPoint);
  Serial.print(F(", actRpm:"));
  Serial.println(actRpm);
  */

  /*if (cm1.getMode() == CW)
  {
    plot(F("actRpm:"), actRpm, false);
  }
  else if (cm1.getMode() == CCW)
  {
    plot(F("actRpm:"), -actRpm, false);
  }
  else if (cm1.getMode() == STOP)
  {
    plot(F("actRpm:"), 0, false);
  }
  plot(F("TargetPoint:"), TargetPoint, true);*/
  
  delay(loopDelay);
}

void encoderISR()
{
  pulseNum++;
}



void timerIntFun() {
  TargetPoint = sin(idx * 3.14159265 / 180.0) * 4900;
  idx = idx + 1;
}
