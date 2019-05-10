//libs
#include <Wire.h>

#define MPU 0x53

//PID
float pidK[3];//P I D
pidK[0]=1;//P
pidK[1]=0.1;//I
pidK[2]=10;//D
int max=400;
float pidMem, pidPrev, pidSP;//memory for integral, prev for derivation, sp->set point
//GyroAccell
float accX,accY,accZ;
float gyroX,gyroY,gyroZ;


void setup()
{
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68 <-address
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);
  digitalWrite(12,HIGH);
}

void loop()
{
  Wire.begin();

  readGyro();


}
