#include<LedControl.h> //导入库文件
//DIN、CLK、CS、点阵屏的个数
LedControl lc = LedControl(A2, A3, A4, 1); //初始化点阵

//定义8行8列的二维数组，用来控制点阵
int v[8][8] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 1, 0, 0, 0},
  {0, 0, 0, 1, 1, 0, 0, 0},
  {0, 0, 0, 0, 1, 0, 0, 0},
  {0, 0, 0, 0, 1, 0, 0, 0},
  {0, 0, 0, 0, 1, 0, 0, 0},
  {0, 0, 0, 0, 1, 0, 0, 0},
  {0, 0, 0, 1, 1, 1, 0, 0}
};

void setup() {
  //true表示省电模式，false表示正常模式
  lc.shutdown(0, false); //启动点阵屏
  lc.setIntensity(0, 4); //调节亮度，级别从0到15
  lc.clearDisplay(0);//清除显示
}

void loop() {
  for (int i = 0; i <= 7; i++)
  {
    for (int j = 0; j <= 7; j++)
    {
		//参数1：点阵的地址(序号)
        //参数2：行数
       //参数3：列数
	   //参数4：用数组表示高低电平，控制点阵屏上的led灯亮或灭
      lc.setLed(0, i, j, v[i][j]);
    }
  }
}
