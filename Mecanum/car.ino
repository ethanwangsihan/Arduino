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
      //////////////////////////////
      //左下角电机代码
      //////////////////////////////
      //delay(100);
      digitalWrite(IN1, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 112); //ENB要输出PWM信号（0-255）
      /////////////////////////////
      //右下角电机代码
      /////////////////////////////
      //delay(100);
      digitalWrite(IN3, LOW); //IN1和IN2只要输出高电平信号或低电平信号就可以
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, 112); //ENA要输出PWM信号（0-255）
      Serial.println("IN3");
      //////////////////////////////
      //左上角电机代码
      //////////////////////////////
      //delay(100);
      digitalWrite(IN3A, HIGH); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN4A, LOW);
      analogWrite(ENBA, 112); //ENB要输出PWM信号（0-255）
      //////////////////////////////
      //右上角电机代码
      //////////////////////////////
      digitalWrite(IN1A, HIGH); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN2A, LOW);
      analogWrite(ENAA, 112); //ENB要输出PWM信号（0-255）
    }

    else if (results.value == 0xFF38C7)//停止 5
    {
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
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 0); //ENA要输出PWM信号（0-255）
      //////////////////////////////
      //左上角电机代码
      //////////////////////////////
      digitalWrite(IN3A, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN4A, LOW);
      analogWrite(ENBA, 0); //ENB要输出PWM信号（0-255）
      //////////////////////////////
      //右上角电机代码
      //////////////////////////////
      digitalWrite(IN1A, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN2A, LOW);
      analogWrite(ENAA, 0); //ENB要输出PWM信号（0-255）
    }

    else if (results.value == 0xFF4AB5)//后退8
    {
      //////////////////////////////
      //左下角电机代码
      //////////////////////////////
      //delay(100);
      digitalWrite(IN1, HIGH); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 112); //ENB要输出PWM信号（0-255）
      /////////////////////////////
      //右下角电机代码
      /////////////////////////////
      //delay(100);
      digitalWrite(IN3, HIGH); //IN1和IN2只要输出高电平信号或低电平信号就可以
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 112); //ENA要输出PWM信号（0-255）
      Serial.println("IN3");
      //////////////////////////////
      //左上角电机代码
      //////////////////////////////
      //delay(100);
      digitalWrite(IN3A, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN4A, HIGH);
      analogWrite(ENBA, 112); //ENB要输出PWM信号（0-255）
      //////////////////////////////
      //右上角电机代码
      //////////////////////////////
      digitalWrite(IN1A, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN2A, HIGH);
      analogWrite(ENAA, 112); //ENB要输出PWM信号（0-255）
    }
    else if (results.value == 0xFF5AA5)//右6
    {

      //////////////////////////////
      //左下角电机代码
      //////////////////////////////
      //delay(100);
      digitalWrite(IN1, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 112); //ENB要输出PWM信号（0-255）
      /////////////////////////////
      //右下角电机代码
      /////////////////////////////
      //delay(100);
      digitalWrite(IN3, HIGH); //IN1和IN2只要输出高电平信号或低电平信号就可以
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 112); //ENA要输出PWM信号（0-255）
      Serial.println("IN3");
      //////////////////////////////
      //左上角电机代码
      //////////////////////////////
      //delay(100);
      digitalWrite(IN3A, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN4A, HIGH);
      analogWrite(ENBA, 112); //ENB要输出PWM信号（0-255）
      //////////////////////////////
      //右上角电机代码
      //////////////////////////////
      digitalWrite(IN1A, HIGH); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN2A, LOW);
      analogWrite(ENAA, 112); //ENB要输出PWM信号（0-255）
    }
    else if (results.value == 0xFF10EF)//左4
    {

      //////////////////////////////
      //左下角电机代码
      //////////////////////////////
      //delay(100);
      digitalWrite(IN1, HIGH); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 112); //ENB要输出PWM信号（0-255）
      /////////////////////////////
      //右下角电机代码
      /////////////////////////////
      //delay(100);
      digitalWrite(IN3, LOW); //IN1和IN2只要输出高电平信号或低电平信号就可以
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, 112); //ENA要输出PWM信号（0-255）
      Serial.println("IN3");
      //////////////////////////////
      //左上角电机代码
      //////////////////////////////
      //delay(100);
      digitalWrite(IN3A, HIGH); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN4A, LOW);
      analogWrite(ENBA, 112); //ENB要输出PWM信号（0-255）
      //////////////////////////////
      //右上角电机代码
      //////////////////////////////
      digitalWrite(IN1A, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
      digitalWrite(IN2A, HIGH);
      analogWrite(ENAA, 112); //ENB要输出PWM信号（0-255）
    }
    irrecv.resume();
  }
  delay(100);
}
