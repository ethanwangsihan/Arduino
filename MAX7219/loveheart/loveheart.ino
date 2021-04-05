int DIN = 12;

int CS = 11;

int CLK = 10; //这里定义了那三个脚

unsigned char disp1[38][8] = {

  {0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x7E}, //L

  {0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C}, //O

  {0x42, 0x42, 0x42, 0x42, 0x42, 0x24, 0x18, 0x18}, //V

  {0x7E, 0x40, 0x40, 0x7E, 0x40, 0x40, 0x40, 0x7E}, //E

  {0x66, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C, 0x18}, //TX

};


void setup() {

  // put your setup code here, to run once：

  pinMode(CLK, OUTPUT);

  pinMode(CS, OUTPUT);

  pinMode(DIN, OUTPUT); //让三个脚都是输出状态



}

void loop() {

  // put your main code here, to run repeatedly：

  unsigned char i, j;

  delay(1000);

  Init_MAX7219();
  
  for (j = 0; j < 38; j++)

  {

    for (i = 1; i < 9; i++)
    {
      Write_Max7219(i, disp1[j][i - 1]);
    }

    delay(1000);

  }

}


//--------------------------------------------

//功能：向MAX7219(U3)写入字节

//入口参数：DATA

//出口参数：无

//说明：

void Write_Max7219_byte(unsigned char DATA)

{

  unsigned char i;

  digitalWrite(CS, LOW);

  for (i = 8; i >= 1; i--)

  {

    digitalWrite(CLK, LOW);

    if (DATA & 0X80)
    {
      digitalWrite(DIN, HIGH);
    }

    else
    {
      digitalWrite(DIN, LOW);
    }

    DATA <<= 1;

    digitalWrite(CLK, HIGH);

  }

}

//-------------------------------------------

//功能：向MAX7219写入数据

//入口参数：address、dat

//出口参数：无

//说明：

void Write_Max7219(unsigned char address, unsigned char dat)

{

  digitalWrite(CS, LOW);

  Write_Max7219_byte(address); //写入地址,即数码管编号

  Write_Max7219_byte(dat); //写入数据,即数码管显示数字

  digitalWrite(CS, HIGH);

}

void Init_MAX7219(void)

{

  Write_Max7219(0x09, 0x00); //译码方式：BCD码

  Write_Max7219(0x0a, 0x03); //亮度

  Write_Max7219(0x0b, 0x07); //扫描界限；4个数码管显示

  Write_Max7219(0x0c, 0x01); //掉电模式：0,普通模式：1

  Write_Max7219(0x0f, 0x00); //显示测试：1；测试结束,正常显示：0

}
