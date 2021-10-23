//https://www.arduino.cn/thread-82030-1-1.html
#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN 的引脚
const byte address[6] = "20181";  //设置标识码
void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
void loop() {
  unsigned long Rtime = millis()/1000;
  radio.write(&Rtime,sizeof(Rtime));
}
