#include <SoftwareSerial.h>

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

double M1Kp = 0.07, M1Ki = 0.025, M1Kd = 0.01; // Motor1 PID
double M2Kp = 0.08, M2Ki = 0.018, M2Kd = 0.01; // Motor2 PID
double M3Kp = 0.05, M3Ki = 0.03, M3Kd = 0.0065; // Motor3 PID
double M4Kp = 0.07, M4Ki = 0.025, M4Kd = 0.01; // Motor4 PID
//double Kp = 0.05, Ki = 0.03, Kd = 0.0065; //3
enum motorMode {CW, CCW, STOP};

int v = -1;

SoftwareSerial m = SoftwareSerial(A0, A1);

int midRPM = 2500;
enum CARMODE {MOVEAHEAD, MOVEBACK, MOVELEFT, MOVERIGHT, MOVELEFTAHEAD, MOVERIGHTAHEAD, MOVELEFTBACK, MOVERIGHTBACK, MOVECW, MOVECCW, STOP};

CARMODE carMode = STOP;

//编码电机控制类
class CodedMotor
{
  private:
    int IN_1;
    int IN_2;
    int EN;

    double mKp, mKi, mKd;
    motorMode mM;

    double lastError;
    double lastLastError;
    double Output;

    //比例调节
    double Pterm(double Kp, double error, double lasterror)
    {
      double PWMp = Kp * (error - lasterror);
      return PWMp;
    }

    //微分调节
    double Dterm(double Kd, double error, double lasterror, double lastlasterror)
    {
      double doubleLastError = 2 * lasterror;
      double PWMd = Kd * (error - doubleLastError + lastlasterror);
      return PWMd;
    }

    //积分调节
    double Iterm(double Ki, double error)
    {
      double PWMi = Ki * error;
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
      lastLastError = 0;
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
      lastLastError = 0;
      Output = 0;
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
        lastError = 0;
        lastLastError = 0;
        Output = 0;
      }

      if (targetPoint > MAXRPM)
      {
        targetPoint = MAXRPM;
      }

      double error = targetPoint - inputPoint;

      Output = Output + Pterm(mKp, error, lastError) + Iterm(mKi, error) + Dterm(mKd, error, lastError, lastLastError);

      lastLastError = lastError;
      lastError = error;

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
        lastError = 0;
        lastLastError = 0;
        Output = 0;
      }

      if (targetPoint > MAXRPM)
      {
        targetPoint = MAXRPM;
      }

      double error = targetPoint - inputPoint;

      Output = Output + Pterm(mKp, error, lastError) + Iterm(mKi, error) + Dterm(mKd, error, lastError, lastLastError);

      lastLastError = lastError;
      lastError = error;

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



CodedMotor motor1 = CodedMotor(MOTOR1IN1, MOTOR1IN2, MOTOR1EN, M1Kp, M1Ki, M1Kd);
CodedMotor motor2 = CodedMotor(MOTOR2IN1, MOTOR2IN2, MOTOR2EN, M2Kp, M2Ki, M2Kd);
CodedMotor motor3 = CodedMotor(MOTOR3IN1, MOTOR3IN2, MOTOR3EN, M3Kp, M3Ki, M3Kd);
CodedMotor motor4 = CodedMotor(MOTOR4IN1, MOTOR4IN2, MOTOR4EN, M4Kp, M4Ki, M4Kd);



void setup() {
  m.begin(9600);
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

}

void loop() {

  //电机转速控制
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






  

  //检测蓝牙信号
  v = m.read();
  if (v != -1)
  {
    Serial.println (v);

    if (v == 'f')
    {
      carMode = MOVEAHEAD;
    }
    else if (v == 's')
    {
      carMode = STOP;
    }
    else if (v == 'b')
    {
      carMode = MOVEBACK;
    }
    else if (v == 'r')
    {
      carMode = MOVERIGHT;
    }
    else if (v == 'l')
    {
      carMode = MOVELEFT;
    }

    else if (v == 'u')
    {
      carMode = MOVECW;
    }
    else if (v == 'v')
    {
      carMode = MOVECCW;
    }
    else if (v == 'p')
    {
      if (speedPwm < maxPwm)
      {
        speedPwm = speedPwm + changePwm;
        showSpeedNum(speedPwm);
        delay(700);
      }

    }
    else if (v == 'm')
    {
      if (speedPwm > minPwm)
      {
        speedPwm = speedPwm - changePwm;
        showSpeedNum(speedPwm);
        delay(700);
      }
    }
  }

  carAction();

  v = -1;
}


void carAction()
{
  switch (carMode)
  {
    case MOVEAHEAD: {
        moveAhead(speedPwm);
        break;
      };
    case MOVEBACK: {
        moveBack(speedPwm);
        break;
      };
    case MOVELEFT: {
        moveLeft(speedPwm);
        break;
      };
    case MOVERIGHT: {
        moveRight(speedPwm);
        break;
      };

    case MOVECW: {
        moveCW(speedPwm);
        break;
      };
    case MOVECCW: {
        moveCCW(speedPwm);
        break;
      };
    case STOP: {
        stopCar();
        break;
      };
  }
}


//停车
void stopCar()
{
  Serial.println("stopCar");
  updateLed(ST);

  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, LOW);
  digitalWrite(IN4A, LOW);
  analogWrite(ENBA, 0);
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, LOW);
  analogWrite(ENAA, 0);
}

//向前
void moveAhead(int speedPwm)
{
  Serial.println("moveAhead");
  updateLed(ahead);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedPwm);
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedPwm);
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, HIGH);
  digitalWrite(IN4A, LOW);
  analogWrite(ENBA, speedPwm);
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, LOW);
  analogWrite(ENAA, speedPwm);
}

//后退
void moveBack(int speedPwm)
{
  Serial.println("moveBack");
  updateLed(back);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speedPwm);
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speedPwm);
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, LOW);
  digitalWrite(IN4A, HIGH);
  analogWrite(ENBA, speedPwm);
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, HIGH);
  analogWrite(ENAA, speedPwm);
}

//左平移
void moveLeft(int speedPwm)
{
  Serial.println("moveLeft");
  updateLed(left);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, HIGH); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speedPwm); //ENB要输出PWM信号（0-255）
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, LOW); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedPwm); //ENA要输出PWM信号（0-255）
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, HIGH); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4A, LOW);
  analogWrite(ENBA, speedPwm); //ENB要输出PWM信号（0-255）
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN2A, HIGH);
  analogWrite(ENAA, speedPwm); //ENB要输出PWM信号（0-255）
}

//右平移
void moveRight(int speedPwm)
{
  Serial.println("moveRight");
  updateLed(right);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedPwm); //ENB要输出PWM信号（0-255）
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, HIGH); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speedPwm); //ENA要输出PWM信号（0-255）
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4A, HIGH);
  analogWrite(ENBA, speedPwm); //ENB要输出PWM信号（0-255）
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, HIGH); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN2A, LOW);
  analogWrite(ENAA, speedPwm); //ENB要输出PWM信号（0-255）
}


//逆时针旋转
void moveCCW(int speedPwm)
{
  Serial.println("moveCCW");
  updateLed(CCW);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedPwm);
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speedPwm);
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, HIGH);
  digitalWrite(IN4A, LOW);
  analogWrite(ENBA, speedPwm);
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, HIGH);
  analogWrite(ENAA, speedPwm);
}

//顺时针旋转
void moveCW(int speedPwm)
{
  Serial.println("moveCW");
  updateLed(CW);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speedPwm);
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedPwm);
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, LOW);
  digitalWrite(IN4A, HIGH);
  analogWrite(ENBA, speedPwm);
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, LOW);
  analogWrite(ENAA, speedPwm);
}

void updateLed(byte Sprite[])
{
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    lc.setRow(0, i, Sprite[i]);
  }
}

void showSpeedNum(int speedPwd)
{
  int speedNum = (speedPwm - 100) / 20;
  switch (speedNum)
  {
    case 0: {
        updateLed(zero);
        break;
      };
    case 1: {
        updateLed(one);
        break;
      };
    case 2: {
        updateLed(two);
        break;
      };
    case 3: {
        updateLed(three);
        break;
      };
    case 4: {
        updateLed(four);
        break;
      };
    case 5: {
        updateLed(five);
        break;
      };

  }
}
