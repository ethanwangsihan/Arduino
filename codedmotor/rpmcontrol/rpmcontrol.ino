#include <PID.h>

#define IN3  3
#define IN4  4
#define ENB  5
#define MINPWM 0
#define MAXPWM 200

volatile long pulseNum = 0;

uint32_t timer;
int loopDelay = 100;

float targetRpm = 4500;

float error = 0.0;      //当前误差
float error_last = 0.0; //上一轮误差
float error_integration = 0.0;   //积分值
float integration_last = 0.0; //上轮积分值
float KP = 0.001;
float KD = 0.0005;
float KI = 0.0002;
int pwm;
arc::PID<double> ledPid(KP,KI,KD);

void setup() {

  
  Serial.begin(9600);
  //设置引脚模式
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(2, INPUT);

  attachInterrupt(digitalPinToInterrupt(2), encoderISR, RISING);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  delay(1000);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, MINPWM);
  pwm=MINPWM;
  timer = 0;
}

void loop() {
  noInterrupts();
  if (timer==0) 
  {
    //for the first time
    timer=micros();
    pulseNum = 0;
    interrupts();
    return;
  }
  
  uint32_t dT = micros() - timer;
  
  float actRpm = pulseNum * 4615385.0 / dT;
  pulseNum = 0;
  timer = micros();
  interrupts();

  plot("actRpm:", actRpm, true);

  ledPid.setTarget(targetRpm);
  ledPid.setInput(actRpm);
  
  int dPwm=ledPid.getOutput();
  
  pwm=pwm+dPwm;

  if (pwm > MAXPWM)
  {
    pwm = MAXPWM;
  }
  else if (pwm < MINPWM)
  {
    pwm = MINPWM;
  }

  analogWrite(ENB, pwm);

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
