//https://www.arduino.cn/thread-82030-1-1.html
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "20181";
void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
void loop() {
  if (radio.available()) {
    unsigned long Rtime;
    radio.read(&Rtime, sizeof(Rtime));
    Serial.println(Rtime);
  }
}
