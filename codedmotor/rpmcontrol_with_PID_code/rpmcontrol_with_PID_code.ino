#include <FlexiTimer2.h>


#define MAXPWM 255

#define MOTOR1IN1  51
#define MOTOR1IN2  53
#define MOTOR1EN  4


#define MOTOR2IN1  47
#define MOTOR2IN2  49
#define MOTOR2EN 5

#define MOTOR3IN1  26
#define MOTOR3IN2  28
#define MOTOR3EN 6

#define MOTOR4IN1  22
#define MOTOR4IN2  24
#define MOTOR4EN 7


#define MAXRPM 4500
#define MINRPM 500

volatile long m1PulseNum = 0;
volatile long m2PulseNum = 0;
volatile long m3PulseNum = 0;
volatile long m4PulseNum = 0;

uint32_t timer;
int loopDelay = 0;
int idx = 0;

double Kp = 0.008, Ki = 0.02, Kd = 0.0028;

enum motorMode {CW, CCW, STOP};
double TargetPoint;



class CodedMotor
{
  private:
    int IN_1;
    int IN_2;
    int EN;

    double mKp, mKi, mKd;
    motorMode mM;

    double lastError;
    double errorIntegral;
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

    CodedMotor(int in1, int in2, int en, double kp, double ki, double kd) //构造函数
    {
      IN_1 = in1;
      IN_2 = in2;
      EN = en;
      mKp = kp;
      mKi = ki;
      mKd = kd;


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
      if (mM == STOP)
      {
        return;
      }

      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, LOW);
      mM = STOP;
      analogWrite(EN, 0);
      lastError = 0;
      errorIntegral = 0;
      Output = 0;
      return;
    }

    outPutCW(double targetPoint, double inputPoint, uint32_t dT)
    {


      if (targetPoint < MINRPM)
      {
        stop();
        return;
      }
      else if (mM != CW)
      {
        mM = CW;
        digitalWrite(IN_1, LOW);
        digitalWrite(IN_2, HIGH);
        lastError = 0;
        errorIntegral = 0;
        Output = 0;
      }

      if (targetPoint > MAXRPM)
      {
        targetPoint = MAXRPM;
      }

      double error = targetPoint - inputPoint;

      Output = Output + Pterm(mKp, error) + Iterm(mKi, error, dT) + Dterm(mKd, error, dT);


      if (Output > MAXPWM)
      {
        Output = MAXPWM;
      }

      analogWrite(EN, Output);

    };

    outPutCCW(double targetPoint, double inputPoint, uint32_t dT)
    {


      if (targetPoint < MINRPM)
      {
        stop();
        return;
      }
      else if (mM != CCW)
      {
        mM = CCW;
        digitalWrite(IN_1, HIGH);
        digitalWrite(IN_2, LOW);
        lastError = 0;
        errorIntegral = 0;
        Output = 0;
      }

      if (targetPoint > MAXRPM)
      {
        targetPoint = MAXRPM;
      }

      double error = targetPoint - inputPoint;

      Output = Output + Pterm(mKp, error) + Iterm(mKi, error, dT) + Dterm(mKd, error, dT);


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


CodedMotor motor1 = CodedMotor(MOTOR1IN1, MOTOR1IN2, MOTOR1EN, Kp, Ki, Kd);
CodedMotor motor2 = CodedMotor(MOTOR2IN1, MOTOR2IN2, MOTOR2EN, Kp, Ki, Kd);
CodedMotor motor3 = CodedMotor(MOTOR3IN1, MOTOR3IN2, MOTOR3EN, Kp, Ki, Kd);
CodedMotor motor4 = CodedMotor(MOTOR4IN1, MOTOR4IN2, MOTOR4EN, Kp, Ki, Kd);


void setup() {

  Serial.begin(9600);
  //设置m1引脚模式
  pinMode(MOTOR1IN1, OUTPUT);
  pinMode(MOTOR1IN2, OUTPUT);
  pinMode(MOTOR1EN, OUTPUT);
  pinMode(18, INPUT);
  attachInterrupt(digitalPinToInterrupt(18), motor1PulseNum, CHANGE);

  //设置m2引脚模式
  pinMode(MOTOR2IN1, OUTPUT);
  pinMode(MOTOR2IN2, OUTPUT);
  pinMode(MOTOR2EN, OUTPUT);
  pinMode(19, INPUT);
  attachInterrupt(digitalPinToInterrupt(19), motor2PulseNum, CHANGE);

  //设置m3引脚模式
  pinMode(MOTOR3IN1, OUTPUT);
  pinMode(MOTOR3IN2, OUTPUT);
  pinMode(MOTOR3EN, OUTPUT);
  pinMode(20, INPUT);
  attachInterrupt(digitalPinToInterrupt(20), motor3PulseNum, CHANGE);

  //设置m4引脚模式
  pinMode(MOTOR4IN1, OUTPUT);
  pinMode(MOTOR4IN2, OUTPUT);
  pinMode(MOTOR4EN, OUTPUT);
  pinMode(21, INPUT);
  attachInterrupt(digitalPinToInterrupt(21), motor4PulseNum, CHANGE);

  timer = 0;
  FlexiTimer2::set(100, timerIntFun);         /* 设置 （每【100】毫秒执行一次 定时器中断函数）*/
  FlexiTimer2::start();                                               /* 开始  启动这个定时器中断 */

}

void loop() {

  uint32_t now = micros();


  //第一次启动只做计时和计数器清零
  if (timer == 0)
  {
    //for the first time
    timer = now;
    m1PulseNum = 0;
    m2PulseNum = 0;
    m3PulseNum = 0;
    m4PulseNum = 0;
    return;
  }

  //获取电机编码器的计数, 得到转速
  noInterrupts();
  if (timer == 0)
  {
    //for the first time
    timer = now;
    m1PulseNum = 0;
    m2PulseNum = 0;
    m3PulseNum = 0;
    m4PulseNum = 0;
    interrupts();
    return;
  }

  //获取电机编码器的计数, 得到转速
  noInterrupts();

  uint32_t dT = now - timer;

  double motor1actRpm = m1PulseNum * 2727272.7 / dT;
  m1PulseNum = 0;

  double motor2actRpm = m2PulseNum * 2727272.7 / dT;
  m2PulseNum = 0;

  double motor3actRpm = m3PulseNum * 2727272.7 / dT;
  m3PulseNum = 0;

  double motor4actRpm = m4PulseNum * 2727272.7 / dT;
  m4PulseNum = 0;

  timer = now;
  interrupts();


  if (TargetPoint == 0)
  {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
  }
  else if (TargetPoint > 0)
  {
    motor1.outPutCW(TargetPoint, motor1actRpm, dT);
    motor2.outPutCW(TargetPoint, motor2actRpm, dT);
    motor3.outPutCW(TargetPoint, motor3actRpm, dT);
    motor4.outPutCW(TargetPoint, motor4actRpm, dT);
  }
  else
  {
    motor1.outPutCCW(-TargetPoint, motor1actRpm, dT);
    motor2.outPutCCW(-TargetPoint, motor2actRpm, dT);
    motor3.outPutCCW(-TargetPoint, motor3actRpm, dT);
    motor4.outPutCCW(-TargetPoint, motor4actRpm, dT);
  }


  if (motor1.getMode() == CW)
  {
    plot("Motor1Rpm:", motor1actRpm, false);
  }
  else if (motor1.getMode() == CCW)
  {
    plot("Motor1Rpm:", -motor1actRpm, false);
  }
  else if (motor1.getMode() == STOP)
  {
    plot("Motor1Rpm:", motor2actRpm, false);
  }


  if (motor2.getMode() == CW)
  {
    plot("Motor2Rpm:", motor2actRpm, false);
  }
  else if (motor2.getMode() == CCW)
  {
    plot("Motor2Rpm:", -motor2actRpm, false);
  }
  else if (motor2.getMode() == STOP)
  {
    plot("Motor2Rpm:", motor2actRpm, false);
  }


  if (motor3.getMode() == CW)
  {
    plot("Motor3Rpm:", motor3actRpm, false);
  }
  else if (motor3.getMode() == CCW)
  {
    plot("Motor3Rpm:", -motor3actRpm, false);
  }
  else if (motor3.getMode() == STOP)
  {
    plot("Motor3Rpm:", motor2actRpm, false);
  }


  if (motor4.getMode() == CW)
  {
    plot("Motor4Rpm:", motor4actRpm, false);
  }
  else if (motor4.getMode() == CCW)
  {
    plot("Motor4Rpm:", -motor4actRpm, false);
  }
  else if (motor4.getMode() == STOP)
  {
    plot("Motor4Rpm:", motor2actRpm, false);
  }

  plot("TargetPoint:", TargetPoint, true);


  delay(loopDelay);
}

void motor1PulseNum()
{
  m1PulseNum++;
}

void motor2PulseNum()
{
  m2PulseNum++;
}

void motor3PulseNum()
{
  m3PulseNum++;
}

void motor4PulseNum()
{
  m4PulseNum++;
}

void timerIntFun() {
  TargetPoint = sin(idx * 3.14159265 / 180.0) * MAXRPM + MAXRPM;
  idx = idx + 1;
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
