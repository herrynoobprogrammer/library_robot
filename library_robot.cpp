#include "library_robot.h"

void setup(){
  Serial.begin(115200);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);

  pinMode(sPin[0], INPUT);
  pinMode(sPin[1], INPUT);
  pinMode(sPin[2], INPUT);
  pinMode(sPin[3], INPUT);
  pinMode(sPin[4], INPUT);
  pinMode(sPin[5], INPUT);

  initlibraryrobot();
}
void readsensor()
{
  uint8_t readPin[6];
  readPin[0] = digitalRead(sPin[0]);
  readPin[1] = digitalRead(sPin[1]);
  readPin[2] = digitalRead(sPin[2]);
  readPin[3] = digitalRead(sPin[3]);
  readPin[4] = digitalRead(sPin[4]);
  readPin[5] = digitalRead(sPin[5]);

  Serial.print(readPin[0]);
  Serial.print(readPin[1]);
  Serial.print(readPin[2]);
  Serial.print(readPin[3]);
  Serial.print(readPin[4]); 
  Serial.print(readPin[5]); 
  Serial.println();
}

void motorjalankanan(int8_t speed, String arah){
    if (arah == "maju")
    {
        analogWrite(pwm1, speed);
        analogWrite(pwm2, 0);
    }
    if (arah == "mundur")
    {
        analogWrite(pwm1, 0);
        analogWrite(pwm2, speed);
    }
}
void motorjalankiri(int8_t speed, String arah){
    if (arah == "maju")
    {
        analogWrite(pwm3, speed);
        analogWrite(pwm4, 0);
    }
     if (arah == "mundur")
    {
        analogWrite(pwm3, 0);
        analogWrite(pwm4, speed);
    }
}

void testmotorkanan(int8_t speed, String arah)
{
    motorjalankanan(speed, arah);
}
void testmotorkiri(int8_t speed, String arah)
{
    motorjalankiri(speed, arah);
}
void testkeduamotor(int8_t speed_kiri,int8_t speed_kanan, String arah_kiri, String arah_kanan)
{
    motorjalankanan(speed_kanan, arah_kanan);
    motorjalankiri(speed_kiri, arah_kiri);
}
void majutimer(int8_t speed, int delay_)
{
    testkeduamotor(speed,speed, "maju", "maju");
    delay(delay_);
    testkeduamotor(0,0, "maju", "maju");
}
void belokiri(int8_t speed)
{
    motorjalankanan(speed, "maju");
    motorjalankiri(speed, "mundur");
}
void belokanan(int8_t speed)
{
    motorjalankanan(speed, "mundur");
    motorjalankiri(speed, "maju");
}
void belokirimaju(int8_t speed)
{
    motorjalankanan(speed, "maju");
    motorjalankiri(speed-(speed/2), "maju");
}
void belokananmaju(int8_t speed)
{
    motorjalankanan(speed-(speed/2), "maju");
    motorjalankiri(speed, "maju");
}
//Akhir dari library