/***************************************************************************
  This is a library example for the HMC5883 magnentometer/compass

  Designed specifically to work with the Adafruit HMC5883 Breakout
  http://www.adafruit.com/products/1746
 
  *** You will also need to install the Adafruit_Sensor library! ***

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries with some heading example from
  Love Electronics (loveelectronics.co.uk)
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 GNU General Public License as
 published by the Free Software Foundation.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

float PAI=3.14159265;
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

float offsetX,offsetY,offsetZ;  //磁场偏移

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(5000);
}

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  calibrateMag();
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);

  
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x-=offsetX); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y-=offsetY); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z-=offsetZ); Serial.print("  ");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = -1.034;
  heading += declinationAngle;

  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    

    /*
  // Check for wrap due to addition of declination.
  if(heading > 2*PAI)
    heading -= 2*PAI;
    */
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/PAI; 
  
  Serial.print("Heading (radians/degrees): "); Serial.print(heading);Serial.print("/");Serial.println(headingDegrees);
  
  delay(1000);
}

//JW:校正HMC5883地磁传感器
void calibrateMag()
{
  int CalThreshold=2;
  int x,y,z; //三轴数据
  int xMax, xMin, yMax, yMin, zMax, zMin;
  //初始化
  getRawData(&x,&y,&z);  
  xMax=xMin=x;
  yMax=yMin=y;
  zMax=zMin=z;
  offsetX = offsetY = offsetZ = 0; //JW:磁场的校正值
 
  Serial.println("Starting Calibration......");
  Serial.println("Please turn your device around in 20 seconds");
 
  for(int i=0;i<200;i++)
  {
    getRawData(&x,&y,&z);
    // 计算最大值与最小值
    // 计算传感器绕X,Y,Z轴旋转时的磁场强度最大值和最小值
    if (x > xMax)
      xMax = x;
    if (x < xMin )
      xMin = x;
    if(y > yMax )
      yMax = y;
    if(y < yMin )
      yMin = y;
    if(z > zMax )
      zMax = z;
    if(z < zMin )
      zMin = z;
 
    delay(100);
 
    if(i%10 == 0)
    {
      Serial.print(xMax);
      Serial.print(" ");
      Serial.println(xMin);

      Serial.print(yMax);
      Serial.print(" ");
      Serial.println(yMin);

      Serial.print(zMax);
      Serial.print(" ");
      Serial.println(zMin);
    }
  }
  //计算修正量
  if(abs(xMax - xMin) > CalThreshold )
    offsetX = (xMax + xMin)/2;
  if(abs(yMax - yMin) > CalThreshold )
    offsetY = (yMax + yMin)/2;
  if(abs(zMax - zMin) > CalThreshold )
    offsetZ = (zMax +zMin)/2;
 
  Serial.print("offsetX:");
  Serial.print("");
  Serial.print(offsetX);
  Serial.print(" offsetY:");
  Serial.print("");
  Serial.print(offsetY);
  Serial.print(" offsetZ:");
  Serial.print("");
  Serial.println(offsetZ);
}

//JW:读取HMC5883地磁数据
void getRawData(int* x ,int* y,int* z)
{
    /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
  *x=event.magnetic.x;
  *y=event.magnetic.y;
  *z=event.magnetic.z;
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(*x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(*y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(*z); Serial.print("  ");Serial.println("uT");
}
