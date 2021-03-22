#include <IRremote.h>
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

IRrecv irrecv(A1);
decode_results results;

int speedPwm=100;

void setup() {
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
  pinMode(A5, OUTPUT);
  irrecv.enableIRIn();
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    if (results.value == 0xFF18E7)//前进2
    {
      MoveAhead(speedPwm);
    }
    else if (results.value == 0xFF38C7)//停止 5
    {
      stopCar();
    }
    else if (results.value == 0xFF4AB5)//后退8
    {
      MoveBack(speedPwm);
    }
    else if (results.value == 0xFF5AA5)//右6
    {
      moveRight(speedPwm);
    }
    else if (results.value == 0xFF10EF)//左4
    {
      moveLeft(speedPwm);
    }
    irrecv.resume();
  }
  delay(100);
}

//停车
void stopCar()
{
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
void MoveAhead(int speedPwm)
{
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
void MoveBack(int speedPwm)
{
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
  
}

//向右前移动
void moveRightAhead(int speedPwm)
{
  
}

//向左后移动
void moveLeftBack(int speedPwm)
{
  
}

//向右后移动
void moveRightBack(int speedPwm)
{
  
}
