

#define MAXPWM 255

#define MOTOR1IN1  53
#define MOTOR1IN2  51
#define MOTOR1EN  4


#define MOTOR2IN1  49
#define MOTOR2IN2  47
#define MOTOR2EN 5

#define MOTOR3IN1  26
#define MOTOR3IN2  28
#define MOTOR3EN 6

#define MOTOR4IN1  22
#define MOTOR4IN2  24
#define MOTOR4EN 7

#define MAXRPM 4500
#define MINRPM 500
#define CURRENTRPM 1500

volatile long m1PulseNum = 0;
volatile long m2PulseNum = 0;
volatile long m3PulseNum = 0;
volatile long m4PulseNum = 0;

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

uint32_t timer;
int loopDelay = 10;


double M1Kp = 0.07, M1Ki = 0.025, M1Kd = 0.01; // Motor1 PID
double M2Kp = 0.08, M2Ki = 0.018, M2Kd = 0.01; // Motor2 PID
double M3Kp = 0.05, M3Ki = 0.025, M3Kd = 0.0065; // Motor3 PID
double M4Kp = 0.07, M4Ki = 0.025, M4Kd = 0.01; // Motor4 PID
//double Kp = 0.05, Ki = 0.03, Kd = 0.0065; //3
enum motorMode {CW, CCW, STOP};

int v = -1;

enum CARMODE {MOVEAHEAD, MOVEBACK, MOVELEFT, MOVERIGHT, MOVELEFTAHEAD, MOVERIGHTAHEAD, MOVELEFTBACK, MOVERIGHTBACK, MOVECW, MOVECCW, CARSTOP};
enum COMMAND {AHEAD, BACK, LEFT, RIGHT, LEFTAHEAD, RIGHTAHEAD, LEFTBACK, RIGHTBACK, CLOCKWISE, COUNTERCLOCKWISE, STAND, FAST, SLOW};

COMMAND carCMD;
CARMODE carMode;



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


//停车
void carStop()
{

  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  m1PulseNum = 0;
  m2PulseNum = 0;
  m3PulseNum = 0;
  m4PulseNum = 0;
  timer = 0;
}


//向前
void moveAhead(double m1actrpm, double m2actrpm, double m3actrpm, double m4actrpm)
{

  motor1.outPutCW(CURRENTRPM, m1actrpm);
  motor2.outPutCW(CURRENTRPM, m2actrpm);
  motor3.outPutCCW(CURRENTRPM, m3actrpm);
  motor4.outPutCCW(CURRENTRPM, m4actrpm);
}

//后退
void moveBack(double m1actrpm, double m2actrpm, double m3actrpm, double m4actrpm)
{

  motor1.outPutCCW(CURRENTRPM, m1actrpm);
  motor2.outPutCCW(CURRENTRPM, m2actrpm);
  motor3.outPutCW(CURRENTRPM, m3actrpm);
  motor4.outPutCW(CURRENTRPM, m4actrpm);
}

//左平移
void moveLeft(double m1actrpm, double m2actrpm, double m3actrpm, double m4actrpm)
{

  motor1.outPutCCW(CURRENTRPM, m1actrpm);
  motor2.outPutCW(CURRENTRPM, m2actrpm);
  motor3.outPutCCW(CURRENTRPM, m3actrpm);
  motor4.outPutCW(CURRENTRPM, m4actrpm);

}

//右平移
void moveRight(double m1actrpm, double m2actrpm, double m3actrpm, double m4actrpm)
{

  motor1.outPutCW(CURRENTRPM, m1actrpm);
  motor2.outPutCCW(CURRENTRPM, m2actrpm);
  motor3.outPutCW(CURRENTRPM, m3actrpm);
  motor4.outPutCCW(CURRENTRPM, m4actrpm);
}


//逆时针旋转
void moveCCW(double m1actrpm, double m2actrpm, double m3actrpm, double m4actrpm)
{
  Serial.println("moveCCW");
  motor1.outPutCW(CURRENTRPM, m1actrpm);
  motor2.outPutCW(CURRENTRPM, m2actrpm);
  motor3.outPutCW(CURRENTRPM, m3actrpm);
  motor4.outPutCW(CURRENTRPM, m4actrpm);
}

//顺时针旋转
void moveCW(double m1actrpm, double m2actrpm, double m3actrpm, double m4actrpm)
{

  Serial.println("moveCW");
  motor1.outPutCCW(CURRENTRPM, m1actrpm);
  motor2.outPutCCW(CURRENTRPM, m2actrpm);
  motor3.outPutCCW(CURRENTRPM, m3actrpm);
  motor4.outPutCCW(CURRENTRPM, m4actrpm);
}


//根据小车状态, 维持小车轮着转速和方向
void carMove(double m1actrpm, double m2actrpm, double m3actrpm, double m4actrpm)
{

  switch (carMode)
  {
    case MOVEAHEAD: {

        moveAhead( m1actrpm,  m2actrpm,  m3actrpm,  m4actrpm);
        break;
      };
    case MOVEBACK: {
        moveBack( m1actrpm,  m2actrpm,  m3actrpm,  m4actrpm);
        break;
      };
    case MOVELEFT: {
        moveLeft( m1actrpm,  m2actrpm,  m3actrpm,  m4actrpm);
        break;
      };
    case MOVERIGHT: {
        moveRight( m1actrpm,  m2actrpm,  m3actrpm,  m4actrpm);
        break;
      };

    case MOVECW: {
        moveCW( m1actrpm,  m2actrpm,  m3actrpm,  m4actrpm);
        break;
      };
    case MOVECCW: {
        moveCCW( m1actrpm,  m2actrpm,  m3actrpm,  m4actrpm);
        break;
      };
  }
}

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

  carMode = CARSTOP;

}

void loop() {
  ////////////
  //检测蓝牙信号
  ////////////
  v = Serial.read();


  if (v == 'f' ||
      v == 's' ||
      v == 'b' ||
      v == 'r' ||
      v == 'l' ||
      v == 'u' ||
      v == 'v' )
  {
    switch (v)
    {
      case 'f':
        {
          carCMD = AHEAD;
          break;
        };

      case 's':
        {
          carCMD = STAND;
          break;
        };

      case 'b':
        {
          carCMD = BACK;
          break;
        }

      case 'r':
        {
          carCMD = RIGHT;
          break;
        };

      case 'l':
        {
          carCMD = LEFT;
          break;
        };

      case 'u':
        {
          carCMD = CLOCKWISE;
          break;
        };

      case 'v':
        {
          carCMD = COUNTERCLOCKWISE;
          break;
        };

      case 'p':
        {
          carCMD = FAST;
          break;
        };

      case 'm':
        {
          carCMD = SLOW;
          break;
        };
    }



    if (carCMD != STAND)
    {

      if (carMode != CARSTOP)
      {
        //如果当前车的运动状态会发生变化, 那么需要先将车停止
        if (
          (carCMD == AHEAD && carMode != MOVEAHEAD) ||
          (carCMD == BACK && carMode != MOVEBACK) ||
          (carCMD == LEFT && carMode != MOVELEFT) ||
          (carCMD == RIGHT && carMode != MOVERIGHT) ||
          (carCMD == LEFTAHEAD && carMode != MOVELEFTAHEAD) ||
          (carCMD == RIGHTAHEAD && carMode != MOVERIGHTAHEAD) ||
          (carCMD == LEFTBACK && carMode != MOVELEFTBACK) ||
          (carCMD == RIGHTBACK && carMode != MOVERIGHTBACK) ||
          (carCMD == CLOCKWISE && carMode != MOVECW) ||
          (carCMD == COUNTERCLOCKWISE && carMode != MOVECCW)
        )
        {
          carStop();
        }
      }

      switch (carCMD)
      {
        case AHEAD:
          {
            carMode = MOVEAHEAD;
            Serial.println("MOVEAHEAD");
            break;
          };
        case BACK:
          {
            carMode = MOVEBACK;
            Serial.println("MOVEBACK");
            break;
          };
        case LEFT:
          {
            carMode = MOVELEFT;
            Serial.println("MOVELEFT");
            break;
          };
        case RIGHT:
          {
            carMode = MOVERIGHT;
            Serial.println("MOVERIGHT");
            break;
          };
        case LEFTAHEAD:
          {
            carMode = MOVELEFTAHEAD;
            Serial.println("MOVELEFTAHEAD");
            break;
          };
        case RIGHTAHEAD:
          {
            carMode = MOVERIGHTAHEAD;
            Serial.println("MOVERIGHTAHEAD");
            break;
          };
        case LEFTBACK:
          {
            carMode = MOVELEFTBACK;
            Serial.println("MOVELEFTBACK");
            break;
          };
        case RIGHTBACK:
          {
            carMode = MOVERIGHTBACK;
            Serial.println("MOVERIGHTBACK");
            break;
          };
        case CLOCKWISE:
          {
            carMode = MOVECW;
            Serial.println("MOVECW");
            break;
          };
        case COUNTERCLOCKWISE:
          {
            carMode = MOVECCW;
            Serial.println("MOVECCW");
            break;
          };
      }
    }
    else if (carMode != CARSTOP)
    {
      carStop();
      carMode = CARSTOP;
    }

  }

  if (carMode != CARSTOP)
  {
    //测量电机转速, 并维持当前运动状态

    //开始电机测速
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

/*
    plot(F("Motor1Rpm:"), motor1actRpm, false);
    plot(F("Motor2Rpm:"), motor2actRpm, false);
    plot(F("Motor3Rpm:"), motor3actRpm, false);
    plot(F("Motor4Rpm:"), motor4actRpm, false);
    plot(F("TargetPoint:"), CURRENTRPM, true);
*/

    /////////////////
    //闭环控制, 维持小车之前的运动状态
    carMove(motor1actRpm, motor2actRpm, motor3actRpm, motor4actRpm);

  }

  v = -1;

  delay(loopDelay);

}
