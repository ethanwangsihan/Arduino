int time = 10 //定义统计多少时间内的脉冲数, ms
volatile int pulse = 0; //用来保存脉冲的计数
int frequency = 1000;   //pwm Hz
int PWM_cycles = 3000; //一个采样周期内输出的PWM波数
float test_routation_rate = 60.0; //实际转速, 圈/秒
float target_routation_rate = 60.0; //目标转速
float r1=0.0;
float r2=0.0;
float error=0.0;        //当前误差
float error_last=0.0;   //上一轮误差
float integration = 0.0   //积分值
float integration_last = 0.0 //上轮积分值
float p=0.0;            //PID输出的控制变量(占空比)
float KP=0.50;
float KD=0.03;
float KI=0.50;

void setup()
{
  pinMode(2, INPUT);
  Serial.begin(9600);
  attachInterrupt(0, pulse_count, HIGH);
  pinMode(13, OUTPUT);
  
}

void pulse_count()
{
  //脉冲计数函数
  ++pulse;
}

float PID()
{
  //PID 控制器, 返回占空比
  float p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0;
  //p=KP*e + KD(e2-e1)/t+KIet

  p1=KP*error;
  p2=KD*1000*(error-error_last)/time;
  integration=integration+error*time/1000;
  p3=KI*(integration+error);
  P4=p1+p2+p3;

  //sigmold 函数方舟超调后溢出
  p5=1/(1+exp(-p4));
  return p5;
}
