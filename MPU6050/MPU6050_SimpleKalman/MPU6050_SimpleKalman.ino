#include <SimpleKalmanFilter.h>
#include <Wire.h>



//const int MPU_addr = 0x69;             // I2C address of MPU-6050 当6050的AD0引脚接高电平时, MPU_addr地址为0x69
const int MPU_addr = 0x68;             // I2C address of MPU-6050 当6050的AD0引脚接低电平时, MPU_addr地址为0x69

long AcXo = 0, AcYo = 0, AcZo = 0;    //加速度计偏移量或误差
long GyXo = 0, GyYo = 0, GyZo = 0;    //陀螺仪偏移量或误差


int16_t AcXKalman, AcYKalman, AcZKalman;    //保存经过Kalman滤波之后的估计结果
double gForceX, gForceY, gForceZ;    //将结果转换成重力加速度, 其实算出来没什么用, 因为横滚俯仰是通过各个轴向数据的比例算出来的

double roll, pitch;                   //横滚, 俯仰角度
double rollOffset, pitchOffset;       //通过加速度计的偏移量计算出的初始角度偏差

int16_t Tmp;

int AcRange = 0;                       //accelerometer Range at the cost of Sensitivity - see setupMPU


int16_t GyXKalman, GyYKalman, GyZKalman;//保存经过Kalman滤波之后的估计结果

double rotX, rotY, rotZ;
double gyroXangle = 0, gyroYangle = 0, gyroZangle = 0; // Angle calculate using the gyro only

int GyRange = 0;                       //gyroscope Range at the cost of Sensitivity - see setupMPU

uint32_t timer;                        //微分时间, 用来累加陀螺仪的角速度

double pi = 3.14159265;

int loopDelay = 0;

SimpleKalmanFilter simpleKalmanFilterAccX(15, 15, 0.01);
SimpleKalmanFilter simpleKalmanFilterAccY(15, 15, 0.01);
SimpleKalmanFilter simpleKalmanFilterAccZ(15, 15, 0.01);
SimpleKalmanFilter simpleKalmanFilterGyX(15, 15, 0.01);
SimpleKalmanFilter simpleKalmanFilterGyY(15, 15, 0.01);
SimpleKalmanFilter simpleKalmanFilterGyZ(15, 15, 0.01);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("See you");
  setupMPU();
  

  for (int i = 0; i < 50; i++) //可能是为了读出初始化后产生的脏数据
  {
    recordAccel();
    recordGyro();
  }

  
  
  delay(1000);

  int times = 2000;             //采样次数
  for (int i = 0; i < times; i++)          //得到初始的offset
  {
    recordAccel();
    recordGyro();
    AcXo += AcXKalman; AcYo += AcYKalman; AcZo += AcZKalman;      //采样和
    GyXo += GyXKalman; GyYo += GyYKalman; GyZo += GyZKalman;
    delay(2);
  }

  AcXo /= times; AcYo /= times; AcZo /= times; //计算加速度计偏移
  GyXo /= times; GyYo /= times; GyZo /= times; //计算陀螺仪偏移


#ifdef RESTRICT_PITCH // Eq. 25 and 26
  rollOffset  = atan2(AcYo, AcZo) * 180 / pi;
  pitchOffset = atan2(-AcXo, sqrt(pow(AcYo, 2) + pow(AcZo, 2))) * 180 / pi;
#else // Eq. 28 and 29
  rollOffset  = atan2(AcYo, sqrt(pow(AcXo, 2) + pow(AcZo, 2))) * 180 / pi;
  pitchOffset = atan2(-AcXo, AcZo) * 180 / pi;
#endif

  timer = micros();

}

void loop() {

  long currentTime=micros();
  double dt = (double)(currentTime - timer) / 1000000; // Calculate delta time
  timer = currentTime;

  recordAccel();
  //recordTemperature();
  recordAngleByAcc();


  recordGyro();
  processGyroData();
  recordAngleByGyro(dt);

  //plotAccData();
  plotGyroData();
  //plotCompareAccGyro();

  delay(loopDelay);

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
  //readFrom(0x6B, 1, buffer);
  c = buffer[1] & 0x07;                //Registry Value AND with 0b00000111 zeros bits 7,6,5,4 & 3
  writeTo(0x6B, c);                    //Then we set the bits to zero and keep the clock source bits the same
  //writeTo(0x6B,0);
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

  /* *********************
         ACCEL_CONFIG
   ********************* */
  /* Accelerometer Configuration Section 4.5 in the Register Map datasheet
     full scale range: ±2g, ±4g, ±8g, and ±16g
     Register   Bit7    Bit6    Bit5     Bit4   Bit3    Bit2  Bit1  Bit0
     |   1C   | XA_ST | YA_ST | ZA_ST |Full Scale Range|  -  |  -  |  -  |
     |        |       |       |       | Select AFS_SEL |     |     |     |
     (Setting XA_ST,TA_ST and ZA_ST to 1 will perform a self test)

     Section 4.17
      AFS_SEL   Binary  Full Scale Range
     |   0   |      0  |     ±2g        |
     |   1   |   1000  |     ±4g        |
     |   2   |  10000  |     ±8g        |
     |   3   |  11000  |    ±16g        |
  */
  switch (AcRange) {                   //AcSensitivity set in Global Variables above
    case 0:
      writeTo(0x1C, 0x0);             //Set ACCEL_CONFIG to AFS_SEL ±2g
      break;
    case 1:
      writeTo(0x1C, 0x8);             //Set GYRO_CONFIG to AFS_SEL ±4g
      break;
    case 2:
      writeTo(0x1C, 0x10);            //Set GYRO_CONFIG to AFS_SEL ±8g
      break;
    case 3:
      writeTo(0x1C, 0x18);            //Set GYRO_CONFIG to AFS_SEL ±16g
      break;
    default:
      writeTo(0x1C, 0x0);             //Set GYRO_CONFIG to AFS_SEL ±2g
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

void recordAccel() {
  /* *********************************************************
      Section 4.17 Accelerometer Measurements Registers 3B - 40
      Note the values are broken up into High and Low bytes so we
      must store the first value and shift it 8 bits over and OR it
      with the second value to form the 16 bit value.
  */
  int16_t AcX, AcY, AcZ;                 //保存从加速度计读取的原始数据accelerometer - force it to be a 16-bit integer
  //Local Variable
  byte buffer[6];

  readFrom(0x3B, 6, buffer);
  AcX = buffer[0] << 8 | buffer[1];    //Store two bytes 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = buffer[2] << 8 | buffer[3];    //Store two bytes 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = buffer[4] << 8 | buffer[5];    //Store two bytes 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  
  AcXKalman = simpleKalmanFilterAccX.updateEstimate(AcX);    //Store two bytes 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcYKalman = simpleKalmanFilterAccY.updateEstimate(AcY);    //Store two bytes 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZKalman = simpleKalmanFilterAccZ.updateEstimate(AcZ);    //Store two bytes 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  

  

}

void processAccelData() {
  /* *********************************************************
     Section 4.17 Accelerometer Measurements

      AFS_SEL Binary Full Scale Range  LSB Sensitity
     |   0   |   0  |     ±2g        |  16384 LSB/g |
     |   1   |   1  |     ±4g        |   8192 LSB/g |
     |   2   |  10  |     ±8g        |   4096 LSB/g |
     |   3   |  11  |    ±16g        |   2048 LSB/g |

     --> g force in direction = AcX/LSB Sensitivity
  */
  //Local Variable
  float LSB;

  switch (AcRange) {
    case 0:
      LSB = 16384.0;                  //AFS_SEL ±2g
      break;
    case 1:
      LSB = 8192.0;                   //AFS_SEL ±4g
      break;
    case 2:
      LSB = 4096.0;                   //AFS_SEL ±8g
      break;
    case 3:
      LSB = 2048.0;                   //AFS_SEL ±16g
      break;
    default:
      LSB = 16384.0;                  //AFS_SEL ±2g
  }
  gForceX = (float)AcXKalman / LSB;
  gForceY = (float)AcYKalman / LSB;
  gForceZ = (float)AcZKalman / LSB;
}


void recordAngleByAcc()
{
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2(AcYKalman, AcZKalman) * 180 / pi - rollOffset;
  pitch = atan2(-AcXKalman, sqrt(pow(AcYKalman, 2) + pow(AcZKalman, 2))) * 180 / pi - pitchOffset;
#else // Eq. 28 and 29
  roll  = atan2(AcYKalman, sqrt(pow(AcXKalman, 2) + pow(AcZKalman, 2))) * 180 / pi - rollOffset;
  pitch = atan2(-AcXKalman, AcZKalman) * 180 / pi - pitchOffset;
#endif


}

void recordGyro() {
  /* *********************************************************
      Section 4.19 Gyroscope Measurements  Registers 0x43 - 0x48
      Note the values are broken up into High and Low bytes so we
      must store the first value and shift it 8 bits over and OR it
      with the second value to form the 16 bit value.
  */
  int16_t GyX, GyY, GyZ;                 //保存从陀螺仪中读取的原始数据gyroscope - force it to be a 16-bit integer
  
  //Local Variable
  byte buffer[6];

  readFrom(0x43, 6, buffer);           //Request Gyro Registers (43 - 48)
  GyX = buffer[0] << 8 | buffer[1];    //Store two bytes 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = buffer[2] << 8 | buffer[3];    //Store two bytes 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = buffer[4] << 8 | buffer[5];    //Store two bytes 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  GyXKalman=simpleKalmanFilterGyX.updateEstimate(GyX);
  GyYKalman=simpleKalmanFilterGyY.updateEstimate(GyY);
  GyZKalman=simpleKalmanFilterGyZ.updateEstimate(GyZ);

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
  rotX = (double)(GyXKalman - GyXo) / LSB;
  rotY = (double)(GyYKalman - GyYo) / LSB;
  rotZ = (double)(GyZKalman - GyZo) / LSB;
}

void recordAngleByGyro(double dt)
{
  //角速度累加得到角度
  gyroXangle += rotX * dt; // Calculate gyro angle without any filter
  gyroYangle += rotY * dt;
  gyroZangle += rotZ * dt;
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

void plotAccData()
{
  plot("AcXKalman", AcXKalman, false);
  plot("AcYKalman", AcYKalman, false);
  plot("AcZKalman", AcZKalman, true);

}


void plotGyroData()
{
  //输出通过陀螺仪测量的角速度和时间积分计算的俯仰横滚角度值
  plot("pitch", gyroYangle, false);
  plot("roll", gyroXangle, false);
  plot("yaw", gyroZangle, true);
}


void plotCompareAccGyro()
{
  plot("Acc_pitch", pitch, false);
  plot("Gyro_pitch", gyroYangle, false);

  plot("Acc_roll", roll, false);
  plot("Gyro_roll", gyroXangle, true);

}
