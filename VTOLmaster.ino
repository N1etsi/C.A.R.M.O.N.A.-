////////////////////////////////////////////////////
/*
  FLIGHT CONTROLLER v x.xx
  Hardware
  ->Brains-STM32F103 chip
  ->Gyro-MPU6050
  ->Barometer-BMP280
  ->SD Reader-generic
  ->TOF Sensor- GY-VL53LO
  ->Servos- 9g generic
  ->RX (PPM) - FlySky x6b (8 channels with ppm)

  Modes
  ->0 Idle - Do nothing/ Ignore Inputs/ Not Armed
  ->1 Passive - TX has all the controll, all manual
  ->2 Hover - Auto hover
  ->3 Flight - Plane mode, SAS

  CHANNELS (x6b)
  CH1- ROLL
  CH2- PITCH
  CH3- THROTTLE
  CH4- YAW
  CH5- SWA (2 positions)- Safe Switch - (Un)Armed
  CH6- SWB (2 positions)- Set Up/ Armed
  CH7- SWC (3 positions)- Flight Mode Select (1~3)
  CH8- SWD (2 positions)- Stability/Acro Mode (?)



  NAAM - 2018
*/
/////////////////////////////////////////////////////
//libs
#include <wire.h>
//declare global suff
uint8_t FlM=0; //Flight Mode
//0-P 1-I 2-D
float PIDroll[3];
float PIDpitch[3];
float PIDyaw[3];

uint32_t pidMaxroll=100;
uint32_t pidMaxyaw=100;
uint32_t pidMaxpitch=100;






void main()
{
  //setup

  //loop
  while(1) //refresh limited by esc speed (50hz/20ms)
  {
    //read sensors/ state
    //calculate stuff
    //write stuff

  }
}
/////////////////////////////////////////////////////////////
/*
NOTES
Data Types (atomic)
  -bool
  -int8_t/uint8_t
  -int16_t/uint16_t
  -int32_t/uint32_t
  -float
  (non-Atomic)
  -float
  -double
  -long double


*/
