 /* 根据霍尔传感器计算电机转速

  这里用了两个中断来处理这个任务  一个是定时器中断  一个是外部中断
  当IO口有外部中断时，计数中断次数
  当定时器时间到达，显示这个时间段内外部有多少个中断
 
 */

#include <MsTimer2.h>                                         /* 引用定时器库 */

int JSQ = 0;                                                  /* 计数器  用于记录在单位时间内有多少个中断 */

void externalIntFun() {                            /* 外部中断函数 当IO口产生中断时执行此过程 */
  JSQ = JSQ + 1;                                              /* 有一个中断 则 计数器加1 */
}

void timerIntFun() {                       /* 定时器中断函数 当设置时间到达后执行此过程 */
  Serial.println(JSQ);                                        /* 向串口发送 单位时间内有多少个中断 */
  JSQ = 0;                                                    /* 复位计数器 以便下次重新计数 */
}



void setup() {
  Serial.begin(9600);                                         /* 打开并设置串口波特率 */

  attachInterrupt(digitalPinToInterrupt(2), externalIntFun, RISING);    /* 设置 (使用Arduino Mega 2560上面【第二个中断】, 执行【外部中断函数】,条件是当从低电平变为高电平时执行）*/

  MsTimer2::set(1000, timerIntFun);         /* 设置 （每【100】毫秒执行一次 定时器中断函数）*/
  MsTimer2::start();                                          /* 开始  启动这个定时器中断 */
}

void loop() {

}
