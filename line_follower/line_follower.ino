#include "library_robot.h"

uint8_t readPin[6];
int b;
float errt1, errt0 = 0, Interr = 0;

void initlibraryrobot() {
  // put your setup code here, to run once:
  testkeduamotor(0,0,"maju","maju");
}
void loop() {
  // put your main code here, to run repeatedly:
  delay(5000);
  linefollow();
}

int samplingsensor()
{
  int ska = 0, ski = 0, error=0;
  //kiri
  readPin[0] = digitalRead(sPin[0]);
  readPin[1] = digitalRead(sPin[1]);
  readPin[2] = digitalRead(sPin[2]);
  //kanan
  readPin[3] = digitalRead(sPin[3]);
  readPin[4] = digitalRead(sPin[4]);
  readPin[5] = digitalRead(sPin[5]);
  //error
  ska = readPin[5]*2 + readPin[4]*4 + readPin[3]*8;
  ski = readPin[0]*2 + readPin[1]*4 + readPin[2]*8;

  error = ska-ski;
  //Serial.println(error);
  return error;
}

float linefollow()
{
  testkeduamotor(0,0,"maju","maju");
  float KP = 0.03, KI = 0.0, KD = 0.00, speed = 45;
  //float KP = 0.02, KI = 0.003, KD = 0.00, speed = 45;
  //Test PID
  while(true)
  {
   errt1 = samplingsensor();
  if(errt1!=0)
    Interr += errt1;
  else
    Interr = 0;
  
  float p = errt1*KP;
  float d = (errt1-errt0) * KD;
  float i = Interr * KI;
  float out = p+i+d;

  errt0 = errt1;
  Serial.println(errt1);

  if(errt1 > 2)
  {
    belokananmaju(45+out);
  }
  else if(errt1<-2)
  {
    belokirimaju(45+(out*-1));
  }
  else
  {
    testkeduamotor(speed,speed,"maju","maju");
  }
 }
}
