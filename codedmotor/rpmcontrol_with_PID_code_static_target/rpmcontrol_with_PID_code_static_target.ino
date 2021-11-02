#include <FlexiTimer2.h>

#define IN3  51
#define IN4  53
#define ENB  4
#define MAXPWM 255

#define MAXRPM 4900

volatile long pulseNum = 0;

uint32_t timer;
int loopDelay = 0;
int idx = 0;

double Kp = 0.008, Ki = 0.026, Kd = 0.0028;

enum motorMode {CW, CCW, STOP};
double TargetPoint;

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

class CodedMotor
{
  private:
    int IN_1;
    int IN_2;
    int EN;

    double cmKp, cmKi, cmKd;

    motorMode mM;

    double lastError;
    double errorIntegral;
    double errorLimit = 500;
    double Output;

    //比例调节
    double Pterm(double Kp, double error)
    {
      double PWMp = Kp * error;
      return PWMp;
    }

    //微分调节
    double Dterm(double Kd, double error, uint32_t dt)
    {
      double dError = error - lastError;
      double errorSpeed = dError * 1000000.0 / dt;
      double PWMd = Kd * errorSpeed;
      lastError = error;
      return PWMd;
    }

    //积分调节
    double Iterm(double Ki, double error, uint32_t dt)
    {
      errorIntegral = errorIntegral + error * dt / 1000000.0;
      double PWMi = Ki * errorIntegral;
      return PWMi;
    }

  public:

    CodedMotor(int in1, int in2, int en, double cmkp, double cmki, double cmkd) //构造函数
    {
      IN_1 = in1;
      IN_2 = in2;
      EN = en;
      cmKp = cmkp;
      cmKi = cmki;
      cmKd = cmkd;


      //stop motor first
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, LOW);
      mM = STOP;
      analogWrite(EN, 0);
      lastError = 0;
      errorIntegral = 0;
      Output = 0;

    };

    void stop()  //停止电机
    {
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, LOW);
      mM = STOP;
      analogWrite(EN, 0);
      lastError = 0;
      errorIntegral = 0;
      Output = 0;
      return;
    }

    void outPut(motorMode mm, double targetPoint, double inputPoint, uint32_t dT)
    {


      if (mm == STOP || (mm == CW && mM == CCW) || (mm == CCW && mM == CW) || targetPoint == 0)
      {
        //如果要求电机目标状态为STOP, 或者电机当前转向和电机目标转向相反,或者要求目标转速为0时,停止电机转动
        stop();
        return;
      }
      else if (mm == CW && mM != CW)
      {
        mM = CW;
        digitalWrite(IN_1, LOW);
        digitalWrite(IN_2, HIGH);
      }
      else if (mm == CCW && mM != CCW)
      {
        mM = CCW;
        digitalWrite(IN_1, HIGH);
        digitalWrite(IN_2, LOW);
      }

      double error = targetPoint - inputPoint;

      Output = Output + Pterm(cmKp, error) + Iterm(cmKi, error, dT) + Dterm(cmKd, error, dT);
      

      if (Output > MAXPWM)
      {
        Output = MAXPWM;
      }

      analogWrite(EN, Output);

    };

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

  attachInterrupt(digitalPinToInterrupt(18), encoderISR, CHANGE);

  timer = 0;
  
  FlexiTimer2::set(50, timerIntFun);         /* 设置 （每【100】毫秒执行一次 定时器中断函数）*/
  FlexiTimer2::start();                                          /* 开始  启动这个定时器中断 */

}

void loop() {

  uint32_t now = micros();

  //获取电机编码器的计数, 得到转速
  noInterrupts();
  if (timer == 0)
  {
    //for the first time
    timer = now;
    pulseNum = 0;
    interrupts();
    return;
  }

  uint32_t dT = now - timer;

  double actRpm = pulseNum * 2727272.7 / dT;
  pulseNum = 0;
  timer = now;
  interrupts();


  if (TargetPoint > 0)
  {
    cm1.outPut(CW, TargetPoint, actRpm, dT);
  }
  else if (TargetPoint < 0 )
  {
    cm1.outPut(CCW, -TargetPoint, actRpm, dT);
  }
  else if (TargetPoint == 0)
  {
    cm1.stop();
  }


  plot("TargetPoint:", TargetPoint, false);

  if (cm1.getMode() == CW)
  {
    plot("actRpm:", actRpm, true);
  }
  else if (cm1.getMode() == CCW)
  {
    plot("actRpm:", -actRpm, true);
  }
  else if (cm1.getMode() == STOP)
  {
    plot("actRpm:", 0, true);
  }

  delay(loopDelay);
}

void encoderISR()
{
  pulseNum++;
}

void timerIntFun() {
  TargetPoint = sin(idx * 3.14159265 / 180.0) * MAXRPM;
  idx = idx + 1;
}
