#include <Wire.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <stdlib.h>
#include <stdio.h>

#define IN1  2
#define IN2  4
#define ENA  6
#define ENB  9
#define IN3  7
#define IN4  8
#define ENAA  5
#define IN1A  3
#define IN2A  11
#define IN3A  12
#define IN4A  13
#define ENBA  10

int v = -1;

SoftwareSerial m = SoftwareSerial(A0, A1);
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

const int maxPwm = 200, minPwm = 100, changePwm = 20;
int speedPwm = 160;
enum CARMODE {MOVEAHEAD, MOVEBACK, MOVELEFT, MOVERIGHT, MOVECW, MOVECCW, STOP};

CARMODE carMode = STOP;

/////////MPU6050///////////////////
const int MPU_addr = 0x68;             // I2C address of MPU-6050
long GyZo = 0;    //陀螺仪偏移量或误差

int16_t GyZ;                 //gyroscope - force it to be a 16-bit integer
double rotZ;
double gyroZangle = 0; // Angle calculate using the gyro only
int GyRange = 0;       //gyroscope Range at the cost of Sensitivity - see setupMPU
uint32_t timer;                        //微分时间, 用来累加陀螺仪的角速度
double pi = 3.14159265;
int loopDelay = 1;


void setup() {
  m.begin(9600);
  Serial.begin(9600);

  lcd.init();                  // 初始化LCD
  lcd.backlight();             //设置LCD背景等亮
  Wire.begin();                //可能lcd.init()中已经包含了Wire.begin(), 有机会试试能不能去掉.
  setupMPU();

  int times = 2000;             //采样次数
  for (int i = 0; i < times; i++)          //得到初始的offset
  {
    recordGyro();
    GyZo += GyZ;
    delay(2);
  }

  GyZo /= times; //计算陀螺仪偏移


  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3A, OUTPUT);
  pinMode(IN4A, OUTPUT);
  pinMode(ENBA, OUTPUT);
  pinMode(IN1A, OUTPUT);
  pinMode(IN2A, OUTPUT);
  pinMode(ENAA, OUTPUT);

  carAction();
}

void loop() {

  v = m.read();

  if (v == -1)
  {

    if (carMode == MOVEAHEAD ||
        carMode == MOVEBACK ||
        carMode == MOVELEFT ||
        carMode == MOVERIGHT
       )
    {
      //对前后左右运动的小车做姿态判断和调整
      long currentTime = micros();
      double dt = (double)(currentTime - timer) / 1000000; // Calculate delta time
      timer = currentTime;
      recordGyro();
      processGyroData();
      recordAngleByGyro(dt);
      plotYawData();

      lcd.setCursor(0, 1);
      lcd.print("Y:");
      lcd.print(gyroZangle);
    }

    delay(loopDelay);
    return;
  }

  if (v == 'f')
  {
    carMode = MOVEAHEAD;
    resetYaw();
  }
  else if (v == 's')
  {
    carMode = STOP;
  }
  else if (v == 'b')
  {
    carMode = MOVEBACK;
    resetYaw();
  }
  else if (v == 'r')
  {
    carMode = MOVERIGHT;
    resetYaw();
  }
  else if (v == 'l')
  {
    carMode = MOVELEFT;
    resetYaw();
  }

  else if (v == 'u')
  {
    carMode = MOVECW;
  }
  else if (v == 'v')
  {
    carMode = MOVECCW;
  }
  else if (v == 'p')
  {
    if (speedPwm < maxPwm)
    {
      speedPwm = speedPwm + changePwm;
      delay(100);
    }

  }
  else if (v == 'm')
  {
    if (speedPwm > minPwm)
    {
      speedPwm = speedPwm - changePwm;
      delay(100);
    }
  }
  carAction();

  v = -1;
}


void carAction()
{
  switch (carMode)
  {
    case MOVEAHEAD: {
        moveAhead(speedPwm);
        break;
      };
    case MOVEBACK: {
        moveBack(speedPwm);
        break;
      };
    case MOVELEFT: {
        moveLeft(speedPwm);
        break;
      };
    case MOVERIGHT: {
        moveRight(speedPwm);
        break;
      };
    case MOVECW: {
        moveCW(speedPwm);
        break;
      };
    case MOVECCW: {
        moveCCW(speedPwm);
        break;
      };
    case STOP: {
        stopCar();
        break;
      };
  }
  updateLCDForModeAndSpeed();
}


//停车
void stopCar()
{
  //Serial.println("stopCar");
  //updateLed(ST);

  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, LOW);
  digitalWrite(IN4A, LOW);
  analogWrite(ENBA, 0);
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, LOW);
  analogWrite(ENAA, 0);
}

//向前
void moveAhead(int speedPwm)
{
  //Serial.println("moveAhead");
  //updateLed(ahead);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedPwm);
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedPwm);
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, HIGH);
  digitalWrite(IN4A, LOW);
  analogWrite(ENBA, speedPwm);
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, LOW);
  analogWrite(ENAA, speedPwm);
}

//后退
void moveBack(int speedPwm)
{
  //Serial.println("moveBack");
  //updateLed(back);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speedPwm);
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speedPwm);
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, LOW);
  digitalWrite(IN4A, HIGH);
  analogWrite(ENBA, speedPwm);
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, HIGH);
  analogWrite(ENAA, speedPwm);
}

//左平移
void moveLeft(int speedPwm)
{
  //Serial.println("moveLeft");
  //updateLed(left);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, HIGH); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speedPwm); //ENB要输出PWM信号（0-255）
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, LOW); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedPwm); //ENA要输出PWM信号（0-255）
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, HIGH); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4A, LOW);
  analogWrite(ENBA, speedPwm); //ENB要输出PWM信号（0-255）
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN2A, HIGH);
  analogWrite(ENAA, speedPwm); //ENB要输出PWM信号（0-255）
}

//右平移
void moveRight(int speedPwm)
{
  //Serial.println("moveRight");
  //updateLed(right);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedPwm); //ENB要输出PWM信号（0-255）
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, HIGH); //IN1和IN2只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speedPwm); //ENA要输出PWM信号（0-255）
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, LOW); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN4A, HIGH);
  analogWrite(ENBA, speedPwm); //ENB要输出PWM信号（0-255）
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, HIGH); //IN3和IN4只要输出高电平信号或低电平信号就可以
  digitalWrite(IN2A, LOW);
  analogWrite(ENAA, speedPwm); //ENB要输出PWM信号（0-255）
}


//逆时针旋转
void moveCCW(int speedPwm)
{
  //Serial.println("moveCCW");
  //updateLed(CCW);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedPwm);
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speedPwm);
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, HIGH);
  digitalWrite(IN4A, LOW);
  analogWrite(ENBA, speedPwm);
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, HIGH);
  analogWrite(ENAA, speedPwm);
}

//顺时针旋转
void moveCW(int speedPwm)
{
  //Serial.println("moveCW");
  //updateLed(CW);
  //////////////////////////////
  //左下角电机代码
  //////////////////////////////
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speedPwm);
  /////////////////////////////
  //右下角电机代码
  /////////////////////////////
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedPwm);
  //////////////////////////////
  //左上角电机代码
  //////////////////////////////
  digitalWrite(IN3A, LOW);
  digitalWrite(IN4A, HIGH);
  analogWrite(ENBA, speedPwm);
  //////////////////////////////
  //右上角电机代码
  //////////////////////////////
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, LOW);
  analogWrite(ENAA, speedPwm);
}


void updateLCDForModeAndSpeed()
{
  lcd.clear();
  lcd.setCursor(0, 0);               //设置显示指针
  int speedNum = (speedPwm - 100) / 20;
  char speedChar[1];
  dtostrf(speedNum, 1, 0, speedChar);

  switch (carMode)
  {
    case MOVEAHEAD: {
        lcd.print("Go Ahead ");
        lcd.print(speedChar);
        break;
      };
    case MOVEBACK: {
        lcd.print("Go Back ");
        lcd.print(speedChar);
        break;
      };
    case MOVELEFT: {
        lcd.print("Go Left ");
        lcd.print(speedChar);
        break;
      };
    case MOVERIGHT: {
        lcd.print("Go Right ");
        lcd.print(speedChar);
        break;
      };

    case MOVECW: {
        lcd.print("Go CW ");
        lcd.print(speedChar);
        break;
      };
    case MOVECCW: {
        lcd.print("Go CCW ");
        lcd.print(speedChar);
        break;
      };
    case STOP: {
        lcd.print("Stop");
        break;
      };
  }





}

void setupMPU() {
  /* *********************************************************
     This function consists of three parts:
       Sets the MPU-6050 Power Management Device reset, sleep mode,
         cycle mode, and Temperature disable bits to zero
       Sets Accelerometer sensitivity
       Sets Gyroscope Sensitivity
     Could be called again if a change in sensitivty is desired
  */

  //Local Variable
  byte c;                              //byte should be a unsigned 8-bit value
  byte buffer[1];

  /* *********************
           PWR_MGMT_1
   ********************* */
  /* Power Management Section 4.28 in Register Map Datasheet.  Since the MPU-6050
     powers up in sleep mode, we'll need to wake it.
      Register  Bit7     Bit6   Bit5   Bit4   Bit3   Bit2  Bit1  Bit0
     |   6B   |Device | Sleep | Cycle |  -  | Temp  |    CLKSEL       |
     |        | Reset |       |       |     |Disable|  (clock source) |
     -->Page 9 (in section 4) see Note: The device will come up in sleep mode upon power-up.
  */
  readFrom(0x6B, 1, buffer);
  c = buffer[1] & 0x07;                //Registry Value AND with 0b00000111 zeros bits 7,6,5,4 & 3
  writeTo(0x6B, c);                    //Then we set the bits to zero and keep the clock source bits the same
  //OR writeTo(0x6B,0);
  //Caution: this will also set the clock source to zero - which in this case is ok
  //  if you didn't want to change the clock source (example using an external clock)
  //  use a method like above.

  /* *********************
          GYRO_CONFIG
   ********************* */
  /* Gyroscope Configuration Section 4.4 in Register Map Datasheet
     full scale range: ±250, ±500, ±1000, ±2000°/sec
      Register  Bit7    Bit6    Bit5     Bit4   Bit3    Bit2  Bit1  Bit0
     |   1B   | XG_ST | YG_ST | ZG_ST |Full Scale Range|  -  |  -  |  -  |
     |        |       |       |       | Select FS_SEL  |     |     |     |
     (Setting XG_ST,TG_ST and ZG_ST to 1 will perform a self test)

     Section 4.19 Gyroscope Measurements:
      FS_SEL    Binary    Full Scale Range
     |   0   |      0  |     ±250°/sec     |
     |   1   |   1000  |     ±500°/sec     |
     |   2   |  10000  |    ±1000°/sec     |
     |   3   |  11000  |    ±2000°/sec     |
     -->Note: FS_SEL/360*60sec/min = RotationValue in RPM
        Sensitivity decreases with larger range.
  */
  switch (GyRange) {                   //GyRange set in Global Variables above
    case 0:
      writeTo(0x1B, 0x0);             //Set GYRO_CONFIG to FS_SEL ±250°/sec
      break;
    case 1:
      writeTo(0x1B, 0x8);             //Set GYRO_CONFIG to FS_SEL ±500°/sec
      break;
    case 2:
      writeTo(0x1B, 0x10);            //Set GYRO_CONFIG to FS_SEL ±1000°/sec
      break;
    case 3:
      writeTo(0x1B, 0x18);            //Set GYRO_CONFIG to FS_SEL ±2000°/sec
      break;
    default:
      writeTo(0x1B, 0x0);             //Set GYRO_CONFIG to FS_SEL ±250°/sec
  }
}

void writeTo(byte toAddress, byte value) {
  /* *********************************************************
    Generic Wire.write() registry byte routine
         toAddress - registry location value
             value - numeric byte value
  */
  Wire.beginTransmission(MPU_addr);    // This begins the I2C communication to the MPU
  Wire.write(toAddress);               // Access the register
  Wire.write(value);                   // Set the byte value to register above
  Wire.endTransmission(true);          // Close I2C communication
}

void readFrom(byte fromAddress, int num, byte results[]) {
  /* *********************************************************
     Generic Wire.read() registry byte routine
       fromAddress - registry location value
               num - number of bytes
         results[] - array to store byte values
  */
  Wire.beginTransmission(MPU_addr);    //I2C address of the MPU
  Wire.write(fromAddress);             //Starting register for Readings
  Wire.endTransmission(false);         //This resent the request and holds the MPU for exclusive communication
  Wire.requestFrom(MPU_addr, num, true); //Request num Registers bytes and releases I2C
  int i = 0;
  while (Wire.available()) {
    results[i] = Wire.read();          //Stores values in buffer arrray
    i++;
  }
}

void recordGyro() {
  /* *********************************************************
      Section 4.19 Gyroscope Measurements  Registers 0x43 - 0x48
      Note the values are broken up into High and Low bytes so we
      must store the first value and shift it 8 bits over and OR it
      with the second value to form the 16 bit value.
  */
  //Local Variable
  byte buffer[6];

  readFrom(0x43, 6, buffer);           //Request Gyro Registers (43 - 48)
  GyZ = buffer[4] << 8 | buffer[5];    //Store two bytes 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

}

void processGyroData() {
  /* *********************************************************
     Section 4.19 Gyroscope Measurements:
      FS_SEL Binary Full Scale Range  LSB Sensitity
     |   0   |   0  |     ±250°/s    | 131.0 LSB/°/s |
     |   1   |   1  |     ±500°/s    |  65.5 LSB/°/s |
     |   2   |  10  |    ±1000°/s    |  32.8 LSB/°/s |
     |   3   |  11  |    ±2000°/s    |  16.4 LSB/°/s |

     --> rotation force in direction = GyX/LSB Sensitivity
     Note that the measurement stored is a 16-bit 2's complement value
  */
  //Local Variable
  float LSB;

  switch (GyRange) {
    case 0:
      LSB = 131.0;                    //FS_SEL ±250°/s
      break;
    case 1:
      LSB = 65.5;                     //FS_SEL ±500°/s
      break;
    case 2:
      LSB = 32.8;                     //FS_SEL ±1000°/s
      break;
    case 3:
      LSB = 16.4;                     //FS_SEL ±2000°/s
      break;
    default:
      LSB = 131.0;                    //FS_SEL ±250°/s
  }
  rotZ = (double)(GyZ - GyZo) / LSB;
}

void recordAngleByGyro(double dt)
{
  //角速度累加
  gyroZangle += rotZ * dt;
}

void resetYaw()
{
  timer = micros();
  gyroZangle = 0;
}

void plot(String label, double value, bool last)
{
  Serial.print(label);

  if (label != "")
  {
    Serial.print(":");
  }
  Serial.print(value);

  if (last == false)
  {
    Serial.print(",");
  }
  else
  {
    Serial.println();
  }
}

void plotYawData()
{
  plot("yaw", gyroZangle, true);
}
