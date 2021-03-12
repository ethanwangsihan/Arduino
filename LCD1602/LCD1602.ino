#include <LiquidCrystal.h>

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

void setup()
{
  lcd.begin(16, 2);
  lcd.clear(); //清屏
  delay(1000); //延时1000ms
  lcd.setCursor(0, 0) ; //设置光标位置
  lcd.print("LCD-1602  ");//使屏幕显示文字
  lcd.setCursor(0, 1) ;
  lcd.print("LCD1602   ");
  delay(5000);    //延时，使显示更稳定。
}

void loop ()
{

  lcd.scrollDisplayLeft();
  delay(5000);    //延时，使显示更稳定。

  lcd.clear(); //清屏 you have to clear lcd each time you want to pring anything new, it wont work otherwise
  delay(1000); //延时1000ms
  lcd.setCursor(0, 0) ; //设置光标位置
  lcd.print("LCD-1602  ");//使屏幕显示文字
  lcd.setCursor(0, 1) ;
  lcd.print("LCD1602   ");
  delay(5000);    //延时，使显示更稳定。
}
