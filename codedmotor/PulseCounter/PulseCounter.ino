 /* 根据霍尔传感器计算电机转速

  这里用了两个中断来处理这个任务  一个是定时器中断  一个是外部中断
  当IO口有外部中断时，计数中断次数
  当定时器时间到达，显示这个时间段内外部有多少个中断
 
 */

#include <MsTimer2.h>                                         /* 引用定时器库 */

int JSQ = 0;  /* 计数器  用于记录在单位时间内有多少个中断 */

void externalIntFun() {                            /* 外部中断函数 当IO口产生中断时执行此过程 */
  JSQ++;                                           /* 有一个中断 则 计数器加1 */
}



void timerIntFun() {                       
  Serial.println(JSQ);                                        
  JSQ = 0;                                                    
}




void setup() {
  Serial.begin(9600);                                         /* 打开并设置串口波特率 */

  attachInterrupt(digitalPinToInterrupt(2), externalIntFun, CHANGE);    /* 设置 (使用Arduino Mega 2560上面【第二个中断】, 执行【外部中断函数】,条件是当从低电平变为高电平时执行）*/

  MsTimer2::set(1000, timerIntFun);         
  MsTimer2::start();                                          
}

void loop() {
  float a = JSQ/26.0;
  plot("Rounds", a, true);
  delay(1000);
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
