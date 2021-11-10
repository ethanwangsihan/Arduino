#include <PID_v1.h>

//https://playground.arduino.cc/Code/PIDLibraryAdaptiveTuningsExample/
//https://playground.arduino.cc/Code/PIDLibaryBasicExample/

#define IN3  22
#define IN4  24
#define ENB  7
#define MAXPWM 255
#define MINRPM 0
#define MAXRPM 5000

volatile long pulseNum = 0;

uint32_t timer;
int loopDelay = 100;
int idx = 0;

//double Kp = 0.055, Ki = 0.12, Kd = 0.0015; //2,3
double Kp = 0.04, Ki = 0.2, Kd = 0.001; //4

double TargetPoint = 1500;

void encoderISR()
{
  pulseNum++;
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
class CodedMotor
{
  private:
    int IN_1;
    int IN_2;
    int EN;

    double mKp, mKi, mKd;


    enum motorMode {CW, CCW, STOP};
    motorMode mM;

    double Setpoint, Input, Output;
    PID myPID = PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

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
      myPID.SetMode(AUTOMATIC);


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
      }

      if (targetPoint > MAXRPM)
      {
        targetPoint = MAXRPM;
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
      }

      if (targetPoint > MAXRPM)
      {
        targetPoint = MAXRPM;
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

  attachInterrupt(digitalPinToInterrupt(21), encoderISR, CHANGE);

  timer = 0;

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

  double actRpm = pulseNum * 2727272.73/ dT;
  pulseNum = 0;
  timer = now;
  interrupts();

  cm1.outPutCW(TargetPoint, actRpm);

  plot("TargetPoint:", TargetPoint, false);
  plot("actRpm:", actRpm, true);


  delay(loopDelay);
}
