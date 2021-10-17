#include <PID_v1.h>
//https://playground.arduino.cc/Code/PIDLibraryAdaptiveTuningsExample/
//https://playground.arduino.cc/Code/PIDLibaryBasicExample/
#include <MsTimer2.h>

#define IN3  3
#define IN4  4
#define ENB  5
#define MAXPWM 200

volatile long pulseNum = 0;

uint32_t timer;
int loopDelay = 150;
int idx = 0;

double consKp = 0.001, consKi = 0.003, consKd = 0.0005;
double aggKp = 0.005, aggKi = 0.005, aggKd = 0.001;
double Setpoint = 0, Input, Output;
double TargetPoint = 0;
enum motorMode {CW, CCW, STOP};
motorMode mM;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup() {


  Serial.begin(9600);
  //设置引脚模式
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(2, INPUT);

  attachInterrupt(digitalPinToInterrupt(2), encoderISR, CHANGE);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  mM = STOP;

  timer = 0;
  Input = 0;

  myPID.SetMode(AUTOMATIC);

  MsTimer2::set(50, timerIntFun);         /* 设置 （每【100】毫秒执行一次 定时器中断函数）*/
  MsTimer2::start();                                          /* 开始  启动这个定时器中断 */

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

  double actRpm = pulseNum * 2307692.3 / dT;
  pulseNum = 0;
  timer = micros();
  interrupts();

  if (TargetPoint > 0 && mM != CW)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(500);
    actRpm = 0;

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    mM = CW;
  }
  else if (TargetPoint < 0 && mM != CCW)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(500);
    actRpm = 0;

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    mM = CCW;
  }

  if (mM == CW)
  {
    Setpoint = TargetPoint;
  }
  else if (mM == CCW)
  {
    Setpoint = -TargetPoint;
  }


  plot("Setpoint:", Setpoint, false);
  plot("actRpm:", actRpm, true);

  Input = actRpm;

  double gap = Setpoint - Input;

  gap = abs(gap); //distance away from setpoint

  if (gap < 300)
  { //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }


  myPID.Compute();

  if (Output > MAXPWM)
  {
    Output = MAXPWM;
  }

  analogWrite(ENB, Output);

  delay(loopDelay);
}

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

void timerIntFun() {
  TargetPoint = sin(idx * 3.14159265 / 180.0) * 8000;
  idx = idx + 1;
}

class CodedMotor
{
  private:
    int IN_1;
    int IN_2;
    int EN;

    double consKp, consKi, consKd;
    double aggKp, aggKi, aggKd;
    motorMode mM = STOP;
    double Setpoint = 0, Input, Output;
    double TargetPoint = 0;
    PID myPID = PID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);

  public:
    CodedMotor(int in1, int in2, int en, double ckp, double cki, double ckd, double akp, double aki, double akd)
    {
      IN_1 = in1;
      IN_2 = in2;
      EN = en;
      consKp = ckp;
      consKi = cki;
      consKd = ckd;
      aggKp = akp;
      aggKi = aki;
      aggKd = akd;
    };

    void outPut(motorMode mm, double targetPoint, double inputPoint)
    {

      if (mm == STOP || (mm==CW && mM==CCW) || (mm==CCW && mM==CW))
      {
        digitalWrite(IN_1, LOW);
        digitalWrite(IN_2, LOW);
        mM = STOP;
        analogWrite(EN, 0);
        return;
      }
      else if (mm==CW)
      {
        mM=CW;
        digitalWrite(IN_1, LOW);
        digitalWrite(IN_2, HIGH);
      }
      else if (mm==CCW)
      {
        mM=CCW
        digitalWrite(IN_1, HIGH);
        digitalWrite(IN_2, LOW);
      }

      Setpoint = targetPoint;
      Input = inputPoint;

      double gap = Setpoint - Input;

      gap = abs(gap); //distance away from setpoint

      if (gap < 300)
      { //we're close to setpoint, use conservative tuning parameters
        myPID.SetTunings(consKp, consKi, consKd);
      }
      else
      {
        //we're far from setpoint, use aggressive tuning parameters
        myPID.SetTunings(aggKp, aggKi, aggKd);
      }

      myPID.Compute();

      if (Output > MAXPWM)
      {
        Output = MAXPWM;
      }

      analogWrite(EN, Output);
    };
};
