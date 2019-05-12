//based off howtomechatronis (mpu6050) and Joop Brokking (YMFC-3D)

//
// pin2 -> throttle;
// pin3 -> roll;
//
//
//
//
//
//libs
#include <Wire.h>

#define MPU 0x68
#define DELTA 20 //delta time between cycles 20ms
//PID
float pidK[3]={1, 0.1, 10};//P I D

int max=400;
float pidMem, pidPrev, pidSP, tempError, pidOut;//memory for integral, prev for derivation, sp->set point
volatile int RXIn, RXInT;
int escOutL, escOutR;
boolean lastState;
//GyroAccell
float accX,accY,accZ, accXAng,accYAng,accZAng;
float gyroX,gyroY,gyroZ, gyroXAng,gyroYAng,gyroZAng;
float accXErr, accYErr, accZErr, gyroXErr, gyroYErr, gyroZErr;
float roll, pitch, yaw;
float throttle;

unsigned long timerRoll, timerThrottle;
unsigned long begTime, initTime;


void setup()
{
  digitalWrite(12,HIGH);
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68 <-address
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  //reset pid values
  pidMem=0;
  pidPrev=0;

  setupIMU();
  digitalWrite(12,LOW);

}
void loop(){}
void cycle() //20 ms cycle => 50hz
{

  readGyro(); //read data from mpu
  normInput(); //normalize inputs
  calcPID(); //calculate pid values
  actuate(); //output
}

void readGyro()
{
  Wire.begin();
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  accX = (Wire.read() << 8 | Wire.read())/4096; //p.29 datasheet
  accY = (Wire.read() << 8 | Wire.read())/4096;
  accZ = (Wire.read() << 8 | Wire.read())/4096;

  accXAng= (atan(accY/ sqrt(pow(accX,2)+pow(accZ,2))) *180/PI)+ accXErr;
  accYAng= (atan(-1*accX/sqrt(pow(accY,2)+pow(accZ,2))) *180/PI)+ accYErr;

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  gyroX = (Wire.read() << 8 | Wire.read())/32.8; //p.29 datasheet
  gyroY = (Wire.read() << 8 | Wire.read())/32.8;
  gyroZ = (Wire.read() << 8 | Wire.read())/32.8;

  gyroX+= gyroXErr;
  gyroY+= gyroYErr;
  gyroZ+= gyroZErr;

  gyroXAng+=gyroX* DELTA/1000.0; //roll
  gyroYAng+=gyroY* DELTA/1000.0; //pitch
  gyroZAng+=gyroZ* DELTA/1000.0; //yaw

  roll= 0.96* gyroXAng + 0.04* accXAng;
  pitch= 0.96* gyroYAng + 0.04* accYAng;
  yaw=gyroZAng;

}

void normInput()
{
  pidSP=0;
  if (RXIn>1510) pidSP=RXIn-1510;
  else if (RXIn<1490) pidSP=RXIn-1490;
}

void calcPID()
{
  tempError=RXIn-pidSP;
  pidMem+=pidK[1]*tempError;
  if(pidMem>max) pidMem=max;
  else if(pidMem<(-1*max)) pidMem= (-1*max);
  pidOut=pidK[0]*tempError+pidMem+pidK[2]*(tempError-pidPrev);
  if(pidOut>max)pidOut=max;
  else if(pidOut<(-1*max))pidOut=(-1*max);

  pidPrev=tempError;

}

void actuate()
{
  throttle=timerThrottle;
  escOutL=throttle+pidOut;
  escOutR=throttle-pidOut;

  if (escOutL>2000) escOutL=2000;
  if (escOutR>2000) escOutR=2000;
  if (escOutL<1000) escOutL=1000;
  if (escOutR<1000) escOutR=1000;

}
void setupIMU()
{
}

ISR(PCINT2_vect)//interrupt when change in pin d0-d7 (input)
{

  if(PINB & B00000100)//pin 2 is on
  {
    begTime=micros();
    cycle();
    if (micros()-begTime>1000)Serial.println("TOO SLOW");

  }



}
