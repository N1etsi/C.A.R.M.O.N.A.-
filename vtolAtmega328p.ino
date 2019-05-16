#include <Wire.h>
#include <Servo.h>
/*
run control loop at 100hz? so we only receive a signal every other loop
even worth to time the loop? yes=> timing the esc's

PWM
*/
//global variable
Servo leftEsc, rightEsc; //controlled by the same
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

//mpu6050 stuff
int16_t accX, accY, accZ, gyrX, gyrY, gyrZ;

//RX stuff
bool prevStch1, prevStch2, prevStch3, prevStch4;
unsigned long timerCH1, timerCH2, timerCH3, timerCH4;
volatile int rxIn1, rxIn2, rxIn3, rxIn4;

//controll stuff
unsigned long looptimer;



void setup()
{
  Wire.begin();
  TWBR = 12;  //sets i2c clock to 400khz
  DDRD |= B11110000    // ports d0 to d7 4-7 -> output, 2escs and 2 servos
  DDRB |= B00110000    // ports d8 to d13 8 - 11 -> input, 4ch

  digitalWrite(12,1);

  looptimer=micros();

  for (cal_int = 0; cal_int < 1250 ; cal_int ++)//Wait 5 seconds before continuing
  {
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);
  }

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);   //ch1    pin8                              //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);  //ch2     pin9                             //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2); //ch3      pin10                             //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3); //ch4      pin11

  setupMPU();


}
void loop()
{
/////
/*
escL = throttle + mode*rollPID + (1-mode)*yawPID
escR = throttle - mode*rollPID - (1-mode)*yawPID
serL = modeT + mode*pitchPID + mode*yawPID + (1-mode)*rollPID
serR = modeT + mode*pitchPID - mode*yawPID - (1-mode)*rollPID

modeT= 1100 if plane, 1800 hover
mode=0 if plane , 1 if hover
*/
/////

loop_timer=micros();
}

float degrad(float deg) //degree to rad
{
  return deg*3.14/180.0
}

ISR(PCINT0_vect)
{
  //CH1
  if( prevStch1==0 && PINB & B00000001) //pin 8 from 0 to 1 (RE)
  {
    prevStch1=1;
    timerCH1=micros();
  }
  else if (prevStch1==1 && !(PINB & B00000001))//pin 8 FE
  {
    prevStch1=0;
    rxIn1= micros()-timerCH1;
  }

  //CH2
  if ( prevStch2==0 && PINB & B00000010) // pin 9 RE
  {
    prevStch2=1;
    timerCH2=micros();
  }
  else if(prevStch2==1 && PINB & !(B00000010)) //pin 9 FE
  {
    prevStch2=0;
    rxIn2 = micros()-timerCH2;
  }
  //ch3
  if ( prevStch3==0 && PINB & B00000100) //pin 10 RE
  {
    prevStch3=1;
    timerCH3=micros();
  }
  else if(prevStch3==1 && PINB & !(B00000100)) // pin 10 FE
  {
    prevStch3=0;
    rxIn3 = micros()-timerCH3;
  }
  //CH4
  if ( prevStch4==0 && PINB & B00001000) // pin 11 RE
  {
    prevStch4=1;
    timerCH4=micros();
  }
  else if(prevStch4==1 && PINB & !(B0000100)) // pin 11 FE
  {
    prevStch4=0;
    rxIn4 = micros()-timerCH4;
  }


}
