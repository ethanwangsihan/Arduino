#include<Wire.h>  //I2C库
#include<LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); //设置显示屏的地址、列、行
float vol;
void setup()
{
  Serial.begin(9600);
  pinMode(A0, INPUT);
  lcd.init(); //初始化LCD1602
  lcd.backlight(); //设置LCD背景灯亮
}
void loop()
{
  int V1 = analogRead(A0);
  vol  = V1 * 5.0 / 1023;
  Serial.println(vol);
  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print(vol);
  lcd.print("V");
  delay(1000);
}
