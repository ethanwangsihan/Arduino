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

int ahed[8][8] = {
  {0, 0, 0, 1, 1, 0, 0, 0},
  {0, 0, 1, 1, 1, 1, 0, 0},
  {0, 1, 0, 1, 1, 0, 1, 0},
  {1, 0, 0, 1, 1, 0, 0, 1},
  {0, 0, 0, 1, 1, 0, 0, 0},
  {0, 0, 0, 1, 1, 0, 0, 0},
  {0, 0, 0, 1, 1, 0, 0, 0},
  {0, 0, 0, 1, 1, 0, 0, 0}
};

/*
int back[8][8] = {
  {0, 0, 0, 1, 1, 0, 0, 0},
  {0, 0, 0, 1, 1, 0, 0, 0},
  {0, 0, 0, 1, 1, 0, 0, 0},
  {0, 0, 0, 1, 1, 0, 0, 0},
  {1, 0, 0, 1, 1, 0, 0, 1},
  {0, 1, 0, 1, 1, 0, 1, 0},
  {0, 0, 1, 1, 1, 1, 0, 0},
  {0, 0, 0, 1, 1, 0, 0, 0}
};
int left[8][8] = {
  {0, 0, 0, 1, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0, 0, 0},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {0, 1, 0, 0, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0, 0, 0},
  {0, 0, 0, 1, 0, 0, 0, 0}
};
int right[8][8] = {
  {0, 0, 0, 0, 1, 0, 0, 0},
  {0, 0, 0, 0, 0, 1, 0, 0},
  {0, 0, 0, 0, 0, 0, 1, 0},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {0, 0, 0, 0, 0, 0, 1, 0},
  {0, 0, 0, 0, 0, 1, 0, 0},
  {0, 0, 0, 0, 1, 0, 0, 0}
};
int right_ahead[8][8] = {
  {0, 0, 0, 1, 1, 1, 1, 1},
  {0, 0, 0, 0, 0, 0, 1, 1},
  {0, 0, 0, 0, 0, 1, 0, 1},
  {0, 0, 0, 0, 1, 0, 0, 1},
  {0, 0, 0, 1, 0, 0, 0, 1},
  {0, 0, 1, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0, 0, 0},
  {1, 0, 0, 0, 0, 0, 0, 0}
};
int left_ahead[8][8] = {
  {1, 1, 1, 1, 1, 0, 0, 0},
  {1, 1, 0, 0, 0, 0, 0, 0},
  {1, 0, 1, 0, 0, 0, 0, 0},
  {1, 0, 0, 1, 0, 0, 0, 0},
  {1, 0, 0, 0, 1, 0, 0, 0},
  {0, 0, 0, 0, 0, 1, 0, 0},
  {0, 0, 0, 0, 0, 0, 1, 0},
  {0, 0, 0, 0, 0, 0, 0, 1}
};
int right_back[8][8] = {
  {1, 0, 0, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0, 0, 0},
  {0, 0, 0, 1, 0, 0, 0, 1},
  {0, 0, 0, 0, 1, 0, 0, 1},
  {0, 0, 0, 0, 0, 1, 0, 1},
  {0, 0, 0, 0, 0, 0, 1, 1},
  {0, 0, 0, 1, 1, 1, 1, 1}
};
int left_back[8][8] = {
  {0, 0, 0, 0, 0, 0, 0, 1},
  {0, 0, 0, 0, 0, 0, 1, 0},
  {0, 0, 0, 0, 0, 1, 0, 0},
  {1, 0, 0, 0, 1, 0, 0, 0},
  {1, 0, 0, 1, 0, 0, 0, 0},
  {1, 0, 1, 0, 0, 0, 0, 0},
  {1, 1, 0, 0, 0, 0, 0, 0},
  {1, 1, 1, 1, 1, 0, 0, 0}
};
int CCW[8][8] = {
  {0, 0, 1, 1, 1, 1, 0, 0},
  {0, 1, 0, 0, 0, 0, 1, 0},
  {1, 0, 0, 0, 0, 0, 0, 1},
  {1, 1, 1, 0, 0, 0, 0, 1},
  {1, 1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 1},
  {0, 0, 0, 0, 0, 0, 1, 0},
  {0, 0, 1, 1, 1, 1, 0, 0}
};
int CW[8][8] = {
  {0, 0, 1, 1, 1, 1, 0, 0},
  {0, 1, 0, 0, 0, 0, 1, 0},
  {1, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 1, 1, 1},
  {1, 0, 0, 0, 0, 0, 1, 1},
  {1, 0, 0, 0, 0, 0, 0, 1},
  {0, 1, 0, 0, 0, 0, 0, 0},
  {0, 0, 1, 1, 1, 1, 0, 0}
};
int ST[8][8] = {
  {0, 0, 1, 1, 1, 1, 0, 0},
  {0, 1, 0, 0, 0, 0, 1, 0},
  {1, 0, 0, 0, 0, 0, 0, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 0, 0, 0, 0, 0, 0, 1},
  {0, 1, 0, 0, 0, 0, 1, 0},
  {0, 0, 1, 1, 1, 1, 0, 0}
};
*/

IRrecv irrecv(A1);
decode_results results;

const int maxPwm = 200, minPwm = 50, changePwm = 10;
int speedPwm = 150;
enum CARMODE {MOVEAHEAD, MOVEBACK, MOVELEFT, MOVERIGHT, MOVELEFTAHEAD, MOVERIGHTAHEAD, MOVELEFTBACK, MOVERIGHTBACK, MOVECW, MOVECCW, STOP};

CARMODE carMode = STOP;


void setup() {
  
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
  
  pinMode(A1, INPUT);

  
  irrecv.enableIRIn();
  Serial.begin(9600);
  delay(1000);
}

void loop() {
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

      }

    }
    else if (results.value == 0xFFE01F)//减速-
    {
      if (speedPwm > minPwm)
      {
        speedPwm = speedPwm - changePwm;

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
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    for (int j = 0; j <= 7; j++)
    {
      //参数1：点阵的地址(序号)
      //参数2：行数
      //参数3：列数
      //参数4：用数组表示高低电平，控制点阵屏上的led灯亮或灭
      //lc.setLed(0, i, j, ST[i][j]);
    }
  }

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
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    for (int j = 0; j <= 7; j++)
    {
      //参数1：点阵的地址(序号)
      //参数2：行数
      //参数3：列数
      //参数4：用数组表示高低电平，控制点阵屏上的led灯亮或灭
      lc.setLed(0, i, j, ahed[i][j]);
    }
  }
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
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    for (int j = 0; j <= 7; j++)
    {
      //参数1：点阵的地址(序号)
      //参数2：行数
      //参数3：列数
      //参数4：用数组表示高低电平，控制点阵屏上的led灯亮或灭
      //lc.setLed(0, i, j, back[i][j]);
    }
  }
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
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    for (int j = 0; j <= 7; j++)
    {
      //参数1：点阵的地址(序号)
      //参数2：行数
      //参数3：列数
      //参数4：用数组表示高低电平，控制点阵屏上的led灯亮或灭
      //lc.setLed(0, i, j, left[i][j]);
    }
  }
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
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    for (int j = 0; j <= 7; j++)
    {
      //参数1：点阵的地址(序号)
      //参数2：行数
      //参数3：列数
      //参数4：用数组表示高低电平，控制点阵屏上的led灯亮或灭
      //lc.setLed(0, i, j, right[i][j]);
    }
  }
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
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    for (int j = 0; j <= 7; j++)
    {
      //参数1：点阵的地址(序号)
      //参数2：行数
      //参数3：列数
      //参数4：用数组表示高低电平，控制点阵屏上的led灯亮或灭
      //lc.setLed(0, i, j, left_ahead[i][j]);
    }
  }
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
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    for (int j = 0; j <= 7; j++)
    {
      //参数1：点阵的地址(序号)
      //参数2：行数
      //参数3：列数
      //参数4：用数组表示高低电平，控制点阵屏上的led灯亮或灭
      //lc.setLed(0, i, j, right_ahead[i][j]);
    }
  }
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
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    for (int j = 0; j <= 7; j++)
    {
      //参数1：点阵的地址(序号)
      //参数2：行数
      //参数3：列数
      //参数4：用数组表示高低电平，控制点阵屏上的led灯亮或灭
      //lc.setLed(0, i, j, left_back[i][j]);
    }
  }
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
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    for (int j = 0; j <= 7; j++)
    {
      //参数1：点阵的地址(序号)
      //参数2：行数
      //参数3：列数
      //参数4：用数组表示高低电平，控制点阵屏上的led灯亮或灭
      //lc.setLed(0, i, j, right_back[i][j]);
    }
  }
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
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    for (int j = 0; j <= 7; j++)
    {
      //参数1：点阵的地址(序号)
      //参数2：行数
      //参数3：列数
      //参数4：用数组表示高低电平，控制点阵屏上的led灯亮或灭
      //lc.setLed(0, i, j, CCW[i][j]);
    }
  }
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
  lc.clearDisplay(0);//清除显示
  for (int i = 0; i <= 7; i++)
  {
    for (int j = 0; j <= 7; j++)
    {
      //参数1：点阵的地址(序号)
      //参数2：行数
      //参数3：列数
      //参数4：用数组表示高低电平，控制点阵屏上的led灯亮或灭
      //lc.setLed(0, i, j, CW[i][j]);
    }
  }
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
