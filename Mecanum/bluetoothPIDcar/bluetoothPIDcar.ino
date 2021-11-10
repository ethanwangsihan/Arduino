#include <LedControl.h>
#include <PID_v1.h>

#define MAXPWM 255

#define MOTOR1IN1  51
#define MOTOR1IN2  53
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
#define CURRENTRPM 2000

LedControl lc = LedControl(A2, A3, A4, 1); //初始化点阵

byte ahead[8] = {0x18, 0x3c, 0x5a, 0x99, 0x18, 0x18, 0x18, 0x18};
byte back[8] = {0x18, 0x18, 0x18, 0x18, 0x99, 0x5a, 0x3c, 0x18};
byte left[8] = {0x10, 0x20, 0x40, 0xff, 0xff, 0x40, 0x20, 0x10};
byte right[8] = {0x08, 0x04, 0x02, 0xff, 0xff, 0x02, 0x04, 0x08};
byte right_ahead[8] = {0x1f, 0x03, 0x05, 0x09, 0x11, 0x20, 0x40, 0x80};
byte left_ahead[8] = {0xf8, 0xc0, 0xa0, 0x90, 0x88, 0x04, 0x02, 0x01};
byte right_back[8] = {0x80, 0x40, 0x20, 0x11, 0x09, 0x05, 0x03, 0x1f};
byte left_back[8] = {0x01, 0x02, 0x04, 0x88, 0x90, 0xa0, 0xc0, 0xf8};
byte CCW[8] = {0x3c, 0x42, 0x81, 0xe1, 0xc1, 0x81, 0x02, 0x3c};
byte CW[8] = {0x3c, 0x42, 0x81, 0x81, 0x87, 0x83, 0x41, 0x38};
byte ST[8] = {0x3c, 0x42, 0x81, 0xbd, 0xbd, 0x81, 0x42, 0x3c};
byte zero[8] = {0x3c, 0x42, 0x46, 0x4a, 0x52, 0x62, 0x42, 0x3c}; // 0
byte one[8] = {0x08, 0x18, 0x28, 0x08, 0x08, 0x08, 0x08, 0x3e}; // 1
byte two[8] = {0x3c, 0x42, 0x02, 0x04, 0x08, 0x10, 0x20, 0x7e}; // 2
byte three[8] = {0x3c, 0x42, 0x02, 0x1c, 0x02, 0x02, 0x42, 0x3c}; // 3
byte four[8] = {0x04, 0x0c, 0x14, 0x24, 0x44, 0x7e, 0x04, 0x04}; // 4
byte five[8] = {0x7e, 0x40, 0x7c, 0x02, 0x02, 0x02, 0x02, 0x7c}; // 5



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
int loopDelay = 100;

double M1Kp = 0.055, M1Ki = 0.12, M1Kd = 0.0015; // Motor1 PID
double M2Kp = 0.055, M2Ki = 0.12, M2Kd = 0.0015; // Motor2 PID
double M3Kp = 0.055, M3Ki = 0.12, M3Kd = 0.0015; // Motor3 PID
double M4Kp = 0.04, M4Ki = 0.2, M4Kd = 0.001; // Motor4 PID

int v = -1;

//oftwareSerial m = SoftwareSerial(A0, A1);


enum CARMODE {MOVEAHEAD, MOVEBACK, MOVELEFT, MOVERIGHT, MOVELEFTAHEAD, MOVERIGHTAHEAD, MOVELEFTBACK, MOVERIGHTBACK, MOVECW, MOVECCW, CARSTOP};
enum COMMAND {AHEAD, BACK, LEFT, RIGHT, LEFTAHEAD, RIGHTAHEAD, LEFTBACK, RIGHTBACK, CLOCKWISE, COUNTERCLOCKWISE, STAND, FAST, SLOW};


CARMODE carMode;

class CodedMotor
{
  private:
    int IN_1;
    int IN_2;
    int EN;

    enum motorMode {CW, CCW, STOP};
    motorMode mM;

    double Setpoint, Input, Output;
    PID myPID=PID(&Input, &Output, &Setpoint, 0.055, 0.12, 0.0015, DIRECT);;

  public:

    CodedMotor(int in1, int in2, int en, double kp, double ki, double kd) //构造函数
    {
      IN_1 = in1;
      IN_2 = in2;
      EN = en;

      //stop motor first
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, LOW);
      mM = STOP;
      analogWrite(EN, 0);
      myPID.SetMode(AUTOMATIC);
      myPID.SetTunings( kp, ki, kd);

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


CodedMotor motor1 = CodedMotor(MOTOR1IN1, MOTOR1IN2, MOTOR1EN, M1Kp, M1Ki, M1Kd);
CodedMotor motor2 = CodedMotor(MOTOR2IN1, MOTOR2IN2, MOTOR2EN, M2Kp, M2Ki, M2Kd);
CodedMotor motor3 = CodedMotor(MOTOR3IN1, MOTOR3IN2, MOTOR3EN, M3Kp, M3Ki, M3Kd);
CodedMotor motor4 = CodedMotor(MOTOR4IN1, MOTOR4IN2, MOTOR4EN, M4Kp, M4Ki, M4Kd);

//停车
void carStop()
{
  //Serial.println("carStop");
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

  motor1.outPutCW(CURRENTRPM, m1actrpm);
  motor2.outPutCCW(CURRENTRPM, m2actrpm);
  motor3.outPutCCW(CURRENTRPM, m3actrpm);
  motor4.outPutCW(CURRENTRPM, m4actrpm);

}

//右平移
void moveRight(double m1actrpm, double m2actrpm, double m3actrpm, double m4actrpm)
{

  motor1.outPutCCW(CURRENTRPM, m1actrpm);
  motor2.outPutCW(CURRENTRPM, m2actrpm);
  motor3.outPutCW(CURRENTRPM, m3actrpm);
  motor4.outPutCCW(CURRENTRPM, m4actrpm);
}


//逆时针旋转
void moveCCW(double m1actrpm, double m2actrpm, double m3actrpm, double m4actrpm)
{

  motor1.outPutCW(CURRENTRPM, m1actrpm);
  motor2.outPutCW(CURRENTRPM, m2actrpm);
  motor3.outPutCW(CURRENTRPM, m3actrpm);
  motor4.outPutCW(CURRENTRPM, m4actrpm);
}

//顺时针旋转
void moveCW(double m1actrpm, double m2actrpm, double m3actrpm, double m4actrpm)
{


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
        updateLed(ahead);
        moveAhead( m1actrpm,  m2actrpm,  m3actrpm,  m4actrpm);
        break;
      };
    case MOVEBACK: {
        updateLed(back);
        moveBack( m1actrpm,  m2actrpm,  m3actrpm,  m4actrpm);
        break;
      };
    case MOVELEFT: {
        updateLed(left);
        moveLeft( m1actrpm,  m2actrpm,  m3actrpm,  m4actrpm);
        break;
      };
    case MOVERIGHT: {
        updateLed(right);
        moveRight( m1actrpm,  m2actrpm,  m3actrpm,  m4actrpm);
        break;
      };

    case MOVECW: {
        updateLed(CW);
        moveCW( m1actrpm,  m2actrpm,  m3actrpm,  m4actrpm);
        break;
      };
    case MOVECCW: {
        updateLed(CCW);
        moveCCW( m1actrpm,  m2actrpm,  m3actrpm,  m4actrpm);
        break;
      };
  }
}


void updateLed(byte Sprite[])
{
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    lc.setRow(0, i, Sprite[i]);
  }
}



void setup() {

  Serial.begin(115200);
  Serial3.begin(9600);
  Serial.println("car setup");

  lc.shutdown(0, false); //启动点阵屏
  lc.setIntensity(0, 4); //调节亮度，级别从0到15
  lc.clearDisplay(0);//清除显示


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
  updateLed(ST);

}

void loop() {


  //检测蓝牙信号
  v = Serial3.read();

  if (v != -1 ) // 检查是否有从蓝牙模块接收到字符。
  {
    COMMAND carCMD;
    switch (v)// 将字符转换成控制指令变量。
    {
      case 'f':
        {

          //Serial.println("AHEAD");
          carCMD = AHEAD;
          break;
        };
      case 'b':
        {
          //Serial.println("BACK");
          carCMD = BACK;
          break;
        };

      case 'l':
        {
          //Serial.println("LEFT");
          carCMD = LEFT;
          break;
        };

      case 'r':
        {
          //Serial.println("RIGHT");
          carCMD = RIGHT;
          break;
        };
      /*
        case '':
        {
          carCMD = LEFTAHEAD;
          break;
        };

        case '':
        {
          carCMD = RIGHTAHEAD;
          break;
        };

        case '':
        {
          carCMD = LEFTBACK;
          break;
        };

        case '':
        {
          carCMD = RIGHTBACK;
          break;
        };
      */
      case 'c':
        {
          //Serial.println("CLOCKWISE");
          carCMD = CLOCKWISE;
          break;
        };

      case 'w':
        {
          //Serial.println("COUNTERCLOCKWISE");
          carCMD = COUNTERCLOCKWISE;
          break;
        };

      case 's':
        {
          //Serial.println("STAND");
          carCMD = STAND;
          break;
        };

      case '+':
        {
          //Serial.println("FAST");
          carCMD = FAST;
          break;
        };

      case '-':
        {
          //Serial.println("SLOW");
          carCMD = SLOW;
          break;
        };
    }

    if (carCMD == STAND)// 判断小车的当前状态是否已经是停止状态。
    {
      if (carMode != CARSTOP)
      {
        carStop();
        carMode = CARSTOP;
        updateLed(ST);
      }

    }
    else
    {
      if ((carCMD == AHEAD and carMode != MOVEAHEAD) or
          (carCMD == BACK and carMode != MOVEBACK) or
          (carCMD == LEFT and carMode != MOVELEFT) or
          (carCMD == RIGHT and carMode != MOVERIGHT) or
          (carCMD == LEFTAHEAD and carMode != MOVELEFTAHEAD) or
          (carCMD == RIGHTAHEAD and carMode != MOVERIGHTAHEAD) or
          (carCMD == LEFTBACK and carMode != MOVELEFTBACK) or
          (carCMD == RIGHTBACK and carMode != MOVERIGHTBACK) or
          (carCMD == CLOCKWISE and carMode != MOVECW) or
          (carCMD == COUNTERCLOCKWISE and carMode != MOVECCW))
      {

        if (carMode != CARSTOP)
        {
          carStop();
        }
      }

      switch (carCMD)
      {
        case AHEAD:
          {
            carMode = MOVEAHEAD;
            break;
          };
        case BACK:
          {
            carMode = MOVEBACK;
            break;
          };
        case LEFT:
          {
            carMode = MOVELEFT;
            break;
          };
        case RIGHT:
          {
            carMode = MOVERIGHT;
            break;
          };
        case LEFTAHEAD:
          {
            carMode = MOVELEFTAHEAD;
            break;
          };
        case RIGHTAHEAD:
          {
            carMode = MOVERIGHTAHEAD;
            break;
          };
        case LEFTBACK:
          {
            carMode = MOVELEFTBACK;
            break;
          };
        case RIGHTBACK:
          {
            carMode = MOVERIGHTBACK;
            break;
          };
        case CLOCKWISE:
          {
            carMode = MOVECW;
            break;
          };
        case COUNTERCLOCKWISE:
          {
            carMode = MOVECCW;
            break;
          };
      }
    }
  }

  if (carMode != CARSTOP)    // 判断，小车当前状态变量的值是否为停止状态。
  {
    // 测量电机转速
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
      interrupts();
      return;
    }

    //获取电机编码器的计数, 得到转速
    noInterrupts();
    uint32_t dT = now - timer;
    double motor1actRpm = m1PulseNum * 2727272.73 / dT;
    m1PulseNum = 0;

    double motor2actRpm = m2PulseNum * 2727272.73 / dT;
    m2PulseNum = 0;

    double motor3actRpm = m3PulseNum * 2727272.73 / dT;
    m3PulseNum = 0;

    double motor4actRpm = m4PulseNum * 2727272.73 / dT;
    m4PulseNum = 0;

    timer = now;
    interrupts();

    // 通过PID算法来维持小车运动状态和电机转速。
    carMove(motor1actRpm, motor2actRpm, motor3actRpm, motor4actRpm);
  }


  v = -1;

  delay(loopDelay);

}
