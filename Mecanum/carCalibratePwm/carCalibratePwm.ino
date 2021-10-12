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

int idlePwm = 80;
int maxPwm = 255;

int upLeftPwm = 200;
int upRightPwm = 200;
int downLefPwm = 200;
int downRightPwm = 200;

enum motor {UPLEFT, UPRIGHT, DOWNLEFT, DOWNRIGHT};
enum motorMode {CW, CCW, STOP};
enum calibrateMode {STOPMODE, UPRIGHTWHEEL, DOWNLEFTWHEEL, DOWNRIGHTWHEEL};

calibrateMode CM = STOPMODE;

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

//PID
double error = 0.0;      //当前误差
double error_last = 0.0; //上一轮误差
double error_integration = 0.0;   //积分值
double integration_last = 0.0; //上轮积分值
float KP = 5;
float KD = 2;
float KI = 0.1;


void setup() {
  m.begin(9600);
  Serial.begin(9600);

  lcd.init();                  // 初始化LCD
  lcd.backlight();             //设置LCD背景等亮
  Wire.begin();                //可能lcd.init()中已经包含了Wire.begin(), 有机会试试能不能去掉.
  setupMPU();

  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);               //设置显示指针
  lcd.print("Calibrating");

  int times = 2000;             //采样次数
  for (int i = 0; i < times; i++)          //得到初始的offset
  {
    recordGyro();
    GyZo += GyZ;
    delay(2);
  }

  GyZo /= times; //计算陀螺仪偏移
  lcd.clear();
  lcd.setCursor(0, 0);               //设置显示指针
  lcd.print("Calibrating Done");
  delay(2000);

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

  stopCar();

}

void loop() {

  v = m.read();

  if (v == -1)
  {
    if (CM == UPRIGHTWHEEL)
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

      int incPwm = PID(gyroZangle, dt);
      if (incPwm != 0)
      {
        upRightPwm -= incPwm;


        if (upRightPwm < idlePwm)
        {
          upRightPwm = idlePwm;
        }
        else if (upRightPwm > maxPwm)
        {
          upRightPwm = maxPwm;
        }

        WheelControl(UPRIGHT, upRightPwm, CW);

      }

      Serial.print("upRightPwm:");
      Serial.println(upRightPwm);

      lcd.print(" P:");
      lcd.print(upRightPwm);
      lcd.print("       ");
    }

    delay(loopDelay);
    return;
  }

  if (v == 'f')
  {
    startCalibrationForUpRight();

  }
  else if (v == 's')
  {
    stopCar();
  }

  v = -1;
}

void startCalibrationForUpRight()
{
  //calibrate the upright wheel
  //start upleft, upright at same pwm, keep downleft, downright stop
  WheelControl(UPLEFT, upLeftPwm, CCW);
  WheelControl(UPRIGHT, upRightPwm, CW);
  WheelControl(DOWNLEFT, idlePwm, CCW);
  WheelControl(DOWNRIGHT, idlePwm, CW);
  resetYaw();
  CM = UPRIGHTWHEEL;
  updateLCDForCalibrateMode();
}


//停车
void stopCar()
{
  CM = STOPMODE;
  WheelControl(UPLEFT, 0, STOP);
  WheelControl(UPRIGHT, 0, STOP);
  WheelControl(DOWNLEFT, 0, STOP);
  WheelControl(DOWNRIGHT, 0, STOP);
  updateLCDForCalibrateMode();
}

void WheelControl(motor Motor, int speedPwm, motorMode direct)
{
  switch (Motor)
  {
    case UPLEFT: //右下角电机
      {
        if (direct == CW)
        {
          digitalWrite(IN3, HIGH);
          digitalWrite(IN4, LOW);
          analogWrite(ENB, speedPwm);
        }
        else if (direct == CCW)
        {
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, HIGH);
          analogWrite(ENB, speedPwm);
        }
        else if (direct == STOP)
        {
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, LOW);
          analogWrite(ENB, 0);
        }

        break;
      }
    case DOWNLEFT: //右上角电机
      {
        if (direct == CCW)
        {
          digitalWrite(IN1A, HIGH);
          digitalWrite(IN2A, LOW);
          analogWrite(ENAA, speedPwm);
        }
        else if (direct == CW)
        {
          digitalWrite(IN1A, LOW);
          digitalWrite(IN2A, HIGH);
          analogWrite(ENAA, speedPwm);
        }
        else if (direct == STOP)
        {
          digitalWrite(IN1A, LOW);
          digitalWrite(IN2A, LOW);
          analogWrite(ENAA, 0);
        }
        break;
      }
    case UPRIGHT: //左下角电机
      {
        if (direct == CW)
        {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
          analogWrite(ENA, speedPwm);
        }
        else if (direct == CCW)
        {
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          analogWrite(ENA, speedPwm);
        }
        else if (direct == STOP)
        {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, LOW);
          analogWrite(ENA, 0);
        }
        break;
      }
    case DOWNRIGHT: //左上角电机
      {
        if (direct == CW)
        {
          digitalWrite(IN3A, HIGH);
          digitalWrite(IN4A, LOW);
          analogWrite(ENBA, speedPwm);
        }
        else if (direct == CCW)
        {
          digitalWrite(IN3A, LOW);
          digitalWrite(IN4A, HIGH);
          analogWrite(ENBA, speedPwm);
        }
        else if (direct == STOP)
        {
          digitalWrite(IN3A, LOW);
          digitalWrite(IN4A, LOW);
          analogWrite(ENBA, 0);
        }
        break;
      }
  }
}

int PID(double error, double deltaTime)
{
  //PID 控制器
  double p1 = 0.0, p2 = 0.0, p3 = 0.0, p4 = 0.0, p5 = 0.0;
  //p=KP*e + KD(e2-e1)/t+KIet
  p1 = KP * error;
  Serial.print("p1:");
  Serial.print(p1);

  p2 = KD * (error - error_last) / deltaTime;
  Serial.print(" p2:");
  Serial.print(p2);

  integration_last += error * deltaTime;
  p3 = KI * integration_last;
  Serial.print(" p3:");
  Serial.println(p3);
  error_last = error;
  return p1 + p2 + p3;
}

void updateLCDForCalibrateMode()
{
  lcd.clear();
  lcd.setCursor(0, 0);               //设置显示指针

  switch (CM)
  {
    case UPRIGHTWHEEL: {
        lcd.print("UPRIGHTWHEEL");
        break;
      };
    case DOWNLEFTWHEEL: {
        lcd.print("DOWNLEFTWHEEL");
        break;
      };
    case DOWNRIGHTWHEEL: {
        lcd.print("DOWNRIGHTWHEEL");
        break;
      case STOPMODE: {
          lcd.print("STOP");
          break;
        }
      }
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
