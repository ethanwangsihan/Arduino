#include <IRremote.h>
#include <LedControl.h>

#define IN1  2
#define IN2  4
#define ENA  6
#define ENB  9
#define IN3  7
#define IN4  8
#define ENAA  5
#define IN1A  3
#define IN2A  11
#define IN3A  12
#define IN4A  13
#define ENBA  10

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
byte zero[8] = {0x3c,0x42,0x46,0x4a,0x52,0x62,0x42,0x3c}; // 0
byte one[8] = {0x08,0x18,0x28,0x08,0x08,0x08,0x08,0x3e};// 1
byte two[8] = {0x3c,0x42,0x02,0x04,0x08,0x10,0x20,0x7e}; // 2
byte three[8] = {0x3c,0x42,0x02,0x1c,0x02,0x02,0x42,0x3c}; // 3
byte four[8] = {0x04,0x0c,0x14,0x24,0x44,0x7e,0x04,0x04}; // 4
byte five[8] = {0x7e,0x40,0x7c,0x02,0x02,0x02,0x02,0x7c}; // 5
IRrecv irrecv(A1); //初始化红外
decode_results results;

const int maxPwm = 200, minPwm = 100, changePwm = 20;
int speedPwm = 160;
enum CARMODE {MOVEAHEAD, MOVEBACK, MOVELEFT, MOVERIGHT, MOVELEFTAHEAD, MOVERIGHTAHEAD, MOVELEFTBACK, MOVERIGHTBACK, MOVECW, MOVECCW, STOP};

CARMODE carMode = STOP;


void setup() {
  Serial.begin(9600);
  //true表示省电模式，false表示正常模式
  lc.shutdown(0, false); //启动点阵屏
  lc.setIntensity(0, 4); //调节亮度，级别从0到15
  lc.clearDisplay(0);//清除显示

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3A, OUTPUT);
  pinMode(IN4A, OUTPUT);
  pinMode(ENBA, OUTPUT);
  pinMode(IN1A, OUTPUT);
  pinMode(IN2A, OUTPUT);
  pinMode(ENAA, OUTPUT);

  irrecv.enableIRIn();
}

void loop() {
  carAction();
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    if (results.value == 0xFF18E7)//前进2
    {
      carMode = MOVEAHEAD;
    }
    else if (results.value == 0xFF38C7)//停止 5
    {
      carMode = STOP;
    }
    else if (results.value == 0xFF4AB5)//后退8
    {
      carMode = MOVEBACK;
    }
    else if (results.value == 0xFF5AA5)//右6
    {
      carMode = MOVERIGHT;
    }
    else if (results.value == 0xFF10EF)//左4
    {
      carMode = MOVELEFT;
    }
    else if (results.value == 0xFF30CF)//左前1
    {
      carMode = MOVELEFTAHEAD;
    }
    else if (results.value == 0xFF7A85)//右前3
    {
      carMode = MOVERIGHTAHEAD;
    }
    else if (results.value == 0xFF42BD)//左后7
    {
      carMode = MOVELEFTBACK;
    }
    else if (results.value == 0xFF52AD)//右后9
    {
      carMode = MOVERIGHTBACK;
    }
    else if (results.value == 0xFF02FD)//顺时针转+
    {
      carMode = MOVECW;
    }
    else if (results.value == 0xFF22DD)//逆时针转-
    {
      carMode = MOVECCW;
    }
    else if (results.value == 0xFFA857)//加速+
    {
      if (speedPwm < maxPwm)
      {
        speedPwm = speedPwm + changePwm;
        showSpeedNum(speedPwm);
        delay(700);
      }

    }
    else if (results.value == 0xFFE01F)//减速-
    {
      if (speedPwm > minPwm)
      {
        speedPwm = speedPwm - changePwm;
        showSpeedNum(speedPwm);
        delay(700);
      }
    }
    carAction();
    irrecv.resume();
  }
  delay(100);
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
    case MOVELEFTAHEAD: {
        moveLeftAhead(speedPwm);
        break;
      };
    case MOVERIGHTAHEAD: {
        moveRightAhead(speedPwm);
        break;
      };
    case MOVELEFTBACK: {
        moveLeftBack(speedPwm);
        break;
      };
    case MOVERIGHTBACK: {
        moveRightBack(speedPwm);
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

//向左前移动
void moveLeftAhead(int speedPwm)
{
  Serial.println("moveLeftAhead");
  updateLed(left_ahead);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0); //ENB要输出PWM信号（0-255）
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
  digitalWrite(IN2A, LOW);
  analogWrite(ENAA, 0); //ENB要输出PWM信号（0-255）
}

//向右前移动
void moveRightAhead(int speedPwm)
{
  Serial.println("moveRightAhead");
  updateLed(right_ahead);
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
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, LOW);
  analogWrite(ENAA, speedPwm);
}

//向左后移动
void moveLeftBack(int speedPwm)
{
  Serial.println("moveLeftBack");
  updateLed(left_back);
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
  digitalWrite(IN2A, HIGH);
  analogWrite(ENAA, speedPwm);
}

//向右后移动
void moveRightBack(int speedPwm)
{
  Serial.println("moveRightBack");
  updateLed(right_back);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
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
  digitalWrite(IN2A, LOW);
  analogWrite(ENAA, 0);
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
