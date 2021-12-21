//https://www.arduino.cn/thread-82030-1-1.html
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "54188";
struct RC_Package {
  int OutX;
  int OutY;
  int Button;
};
RC_Package data;
int loopdelay = 1;
void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(RC_Package));
    Serial.print("outX:");
    Serial.print(data.OutX);
    Serial.print(",outY:");
    Serial.print(data.OutY);
    Serial.print(",BUTTON:");
    Serial.println(data.Button);

  }

  delay(loopdelay);

}
