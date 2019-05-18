#include <Wire.h>
#include <Servo.h>


//global variable
#define PI 3.14
#define DELTA 20 //miliscnds per cycle
Servo leftEsc, rightEsc; //controlled by the same left pin 6, right pin 7
Servo leftServ, rightServ;
Servo rudSer, elevSer;

//pid
float pRoll = 1.0;
float iRoll = 0.05;
float dRoll = 15.0;
int maxRoll = 400;

float pPitch = 1.0;
float iPitch = 0.05;
float dPitch = 15.0;
int maxPitch = 400;

float pYaw = 1.0;
float iYaw = 0.05;
float dYaw = 15.0;
int maxYaw = 400;

float rollImem, rollDlast, rollSP, rollOut;
float pitchImem, pitchDlast, pitchSP, pitchOut;
float yawImem, yawDlast, yawSP, yawOut;

float tempError;

//mpu6050 stuff
#define MPU 0x68
int16_t accX, accY, accZ, gyrX, gyrY, gyrZ;
float accXAng, accYAng, gyroXAng, gyroYAng, gyroZAng, totAngle[2];
float roll, pitch, yaw;

//RX stuff
bool prevStch1, prevStch2, prevStch3, prevStch4;
unsigned long timerCH1, timerCH2, timerCH3, timerCH4;
volatile int rxIn1, rxIn2, rxIn3, rxIn4;
float rollAdjust, pitchAdjust;
int throttle;

//controll stuff
unsigned long looptimer;
int mode=1; //hard code to run on hover mode
int modeT=1750; //hard code servo pwm on hover

//esc and servo stuff
              //left motor,right motor, left serv, right servo
unsigned long timer1, timer2, timer3, timer4;
int escL, escR, servL, servR;


void setup()
{

  TWBR = 12;  //sets i2c clock to 400khz
  DDRD |= B11110000;   // ports d0 to d7 4-7 -> output, 2escs and 2 servos
  DDRB |= B00110000;  // ports d8 to d13 8 - 11 -> input, 4ch
  digitalWrite(12,1);
  //MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);

  //ESC INIT
  for (int cal_int = 0; cal_int < 1250 ; cal_int ++)//Wait 5 seconds before continuing
  {
    PORTD |= B11000000;                                                     //Set digital port  6 and 7 high.
    delayMicroseconds(950);                                                //Wait 1000us.
    PORTD &= B00111111;                                                     //Set digital port 6 and 7 low.
    delayMicroseconds(3000);
  }

  PCICR |= (1 << PCIE0);                                  //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);   //ch1    pin8                //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);   //ch2    pin9                //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);   //ch3    pin10               //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);   //ch4    pin11



//  setupMPU();

  //setupFlight
  pitch=accYAng;
  roll=accXAng;
  rollImem=0;
  rollDlast=0;
  pitchImem=0;
  pitchDlast=0;
  yawImem=0;
  yawDlast=0;


  looptimer=micros();
}
void loop()
{

  //Accelarometer
  getAcc();
  //Gyro
  getGyr();

  calcPID();

  calcOutput();

  //output
  writeOutEscServ(); //deals with the loop timer

}
float degrad(float deg) //degree to rad
{
  return deg*3.14/180.0;
}
ISR(PCINT0_vect)
{
  unsigned long ctTime=micros();
  //CH1
  if( prevStch1==0 && PINB & B00000001) //pin 8 from 0 to 1 (RE)
  {
    prevStch1=1;
    timerCH1=ctTime;
  }
  else if (prevStch1==1 && !(PINB & B00000001))//pin 8 FE
  {
    prevStch1=0;
    rxIn1= (ctTime-timerCH1)/1000;
  }

  //CH2
  if ( prevStch2==0 && PINB & B00000010) // pin 9 RE
  {
    prevStch2=1;
    timerCH2=ctTime;
  }
  else if(prevStch2==1 && PINB & !(B00000010)) //pin 9 FE
  {
    prevStch2=0;
    rxIn2 = (ctTime-timerCH2)/1000;
  }

  //ch3
  if ( prevStch3==0 && PINB & B00000100) //pin 10 RE
  {
    prevStch3=1;
    timerCH3=ctTime;
  }
  else if(prevStch3==1 && PINB & !(B00000100)) // pin 10 FE
  {
    prevStch3=0;
    rxIn3 = (ctTime-timerCH3)/1000;
  }

  //CH4
  if ( prevStch4==0 && PINB & B00001000) // pin 11 RE
  {
    prevStch4=1;
    timerCH4=ctTime;
  }
  else if(prevStch4==1 && PINB & !(B0000100)) // pin 11 FE
  {
    prevStch4=0;
    rxIn4 = (ctTime-timerCH4)/1000;
  }

}
void getAcc()
{
  //Accel
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true); //request 6 blocks

  accX=Wire.read()<<8|Wire.read()/4096;
  accY=Wire.read()<<8|Wire.read()/4096;
  accZ=Wire.read()<<8|Wire.read()/4096;
  //Euler Transf.
  accXAng= (atan(accY/ sqrt(pow(accX,2)+pow(accZ,2))) *180/PI);//+ accXErr;
  accYAng= (atan(-1*accX/sqrt(pow(accY,2)+pow(accZ,2))) *180/PI);//+ accYErr;
}
void getGyr()
{

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43(Gyro X first 8 bits)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); //Get 6 (8bit)registers

  gyrX = (Wire.read() << 8 | Wire.read())/32.8; //p.29 datasheet
  gyrY = (Wire.read() << 8 | Wire.read())/32.8;
  gyrZ = (Wire.read() << 8 | Wire.read())/32.8;

//  gyrX+= gyrXErr;
  //gyrY+= gyrYErr;
  //gyrZ+= gyrZErr;

  gyroXAng+=gyrX* DELTA/1000.0; //roll
  gyroYAng+=gyrY* DELTA/1000.0; //pitch
  gyroZAng+=gyrZ* DELTA/1000.0; //yaw

  roll= 0.96* gyroXAng + 0.04* accXAng;
  pitch= 0.96* gyroYAng + 0.04* accYAng;
  yaw=gyroZAng;

  rollAdjust= roll*15; //why 15?
  pitchAdjust= pitch*15;
}

void calcPID()
{
  rollSP=0;
  if(rxIn1>1505) rollSP=rxIn1-1505;
  else if(rxIn1<1495) rollSP=rxIn1-1495;
  rollSP-=rollAdjust;
  rollSP/=3.0;

  pitchSP=0;
  if(rxIn2>1505) pitchSP=rxIn2-1505;
  else if(rxIn2<1495) pitchSP=rxIn2-1495;
  pitchSP-=pitchAdjust;
  pitchSP/=3.0;

  yawSP=0;
  if(rxIn3>1050)
  {
    if(rxIn4>1505) yawSP=rxIn4-1505;
    else if(rxIn4<1495) yawSP=rxIn4-1495;
    yawSP/=3.0;
  }
  //"calculate PID()"
  //ROLL PID
  tempError = roll- rollSP;
  rollImem += iRoll * tempError;
  if (rollImem>maxRoll) rollImem=maxRoll;
  else if (rollImem<(-1*maxRoll)) rollImem = -1*maxRoll;

  rollOut= pRoll* tempError + rollImem + dRoll* (tempError-rollDlast);
  if (rollOut>maxRoll) rollOut=maxRoll;
  else if (rollOut<(-1*maxRoll)) rollOut = -1*maxRoll;

  rollDlast=tempError;

  //PITCH PID
  tempError= pitch - pitchSP;
  pitchImem+= iPitch* tempError;
  if (pitchImem>maxPitch) pitchImem=maxPitch;
  else if (pitchImem<(-1*maxPitch)) pitchImem = -1*maxPitch;

  pitchOut= pPitch* tempError + pitchImem + dPitch* (tempError- pitchDlast);
  if (pitchOut>maxPitch) pitchOut=maxPitch;
  else if (pitchOut<(-1*maxPitch)) pitchOut = -1*maxPitch;

  pitchDlast=tempError;

  //YAW PID
  tempError= yaw - yawSP;
  yawImem+= yawImem* tempError;
  if(yawImem>maxYaw) yawImem=maxYaw;
  else if(yawImem<(-1*maxYaw)) yawImem = -1*maxYaw;

  yawOut= pYaw* tempError + yawImem + dYaw* (tempError-yawDlast);
  if(yawOut>maxYaw) yawOut=maxYaw;
  else if(yawOut<(-1*maxYaw)) yawOut = -1*maxYaw;

  yawDlast= tempError;

}
void calcOutput()
{
  throttle=rxIn3;

  escL = throttle + mode*rollOut;//+ (1-mode)*yawPID
  escR = throttle - mode*rollOut; //- (1-mode)*yawPID
  servL = modeT + mode*pitchOut + mode*yawOut; // + (1-mode)*rollPID
  servR = modeT + mode*pitchOut - mode*yawOut; // - (1-mode)*rollPID

  if(escL<1050) escL=1050;
  if(escR<1050) escR=1050;
  if(servL<1050) servL=1050;
  if(servR<1050) servR=1050;

  if(escL>2000) escL=2000;
  if(escR>2000) escR=2000;
  if(servL>2000) servL=2000;
  if(servR>2000) servR=2000;
  //milisecnds to micros
  escL*=1000;
  escR*=1000;
  servL*=1000;
  servR*=1000;
}
void writeOutEscServ()
{
  unsigned long escTimer;
  //DEGUB PRONT ALL TIMERS
  Serial.println( "INPUT\n\tCH1: %d\tCH2: %d\tCH3: %d\tCH4: %d", rxIn1, rxIn2, rxIn3, rxIn4);
  Serial.println("OUTPUT\n\tESCL: %d\tESCR: %d\tServL: %d\tServR: %d", escL, escR, servL, servR);
  Serial.println("LOOP TIMER: %d", looptimer);
  Serial.println();
  while (looptimer+DELTA*1000 > micros());
  looptimer=micros();

  PORTD |= B11110000; //starts pulse for every pwm
  timer1=escL+looptimer;
  timer2=escR+looptimer;
  timer3=servL+looptimer;
  timer4=servR+looptimer;
  //do nothing for 1ms , add smth???

  while(PORTD >=16)
  {
    escTimer=micros();
    if(timer1<= escTimer) PORTD &= B11101111; //time's up, FE pwm
    if(timer2<= escTimer) PORTD &= B11011111; //time's up, FE pwm
    if(timer3<= escTimer) PORTD &= B10111111; //time's up, FE pwm
    if(timer4<= escTimer) PORTD &= B01111111; //time's up, FE pwm
  }

}

/////
/*
modeT= 1100 if plane, 1800 hover
mode=0 if plane , 1 if hover
*/
/////

/*
run control loop at 100hz? so we only receive a signal every other loop
even worth to time the loop? yes=> timing the esc's

PWM

get->calc->out

note: add battery checker on the 1ms dead time of pwm out? +beep if low?
missing some calcPID stuff
*/
