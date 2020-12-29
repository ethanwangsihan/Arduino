void setup() {
  Serial.begin(9600);
}

void loop() {
  //可以用于运行时输出某个变量的值
  //Serial monitor所设置的baudrate必须和Serial.begin初始化时使用的值相同，否则会显示乱码
  Serial.print("Hello Arduino!");
  delay(2000);
  Serial.println("Hello Arduino with Return!");
  delay(2000);
}
