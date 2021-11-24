#ifndef LIBRARYROBOT_H_
#define LIBRARYROBOT_H_

#include <Arduino.h>
// pin output motor
#define pwm1 5
#define pwm2 6
#define pwm3 9
#define pwm4 10
//pin sensor
#define s0 A0
#define s1 A1
#define s2 A2
#define s3 A3
#define s4 A4
#define s5 A5
const uint8_t sPin[6] = {s0, s1, s2, s3, s4, s5};
#define initlibraryrobot initlibraryrobot

void remotesetup();
void initlibraryrobot();
void manualcontrol();
void autocontrol();
void readsensor();

void testmotorkanan(int8_t speed, String arah);
void testmotorkiri(int8_t speed, String arah);
void testkeduamotor(int8_t speed_kiri,int8_t speed_kanan, String arah_kiri, String arah_kanan);
void belokiri(int8_t speed);
void belokanan(int8_t speed);
void belokirimaju(int8_t speed);
void belokananmaju(int8_t speed);
void majutimer(int8_t speed, int delay_);

#endif