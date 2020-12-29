int ledPin = 13;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {

  char inputCmd[1000]; //存放从Serical Monitor中读取的输入字符串
  int idx = 0;

  while (Serial.available() > 0)
  {
    int inputChar = Serial.read();
    if (inputChar != 10)
    {
      inputCmd[idx] = inputChar;
      idx++;
    }
  }

  if (idx > 0)
  {
    inputCmd[idx] = '\0'; //字符串数组最后需要以'\0'结束
    Serial.println(inputCmd); //打印收到的命令
    
    //判断input命令
    if (strcmp(inputCmd, "on") == 0)
    {
      digitalWrite(ledPin, HIGH);
    }
    else if (strcmp(inputCmd, "off") == 0)
    {
      digitalWrite(ledPin, LOW);
    }
    else
    {
      Serial.println("Neither on nor off");
    }

  }
  else
  {
    Serial.println("No command! Will try again in 1 second!");
  }

  delay(1000);
}
