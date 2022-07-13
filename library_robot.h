#ifndef LIBRARYROBOT_H_
#define LIBRARYROBOT_H_

#include <Arduino.h>
// pin output motor
#define pwm1 5
#define pwm2 6
#define pwm3 9
#define pwm4 10
//pin sensor
#define s0 A6
#define s1 A1
#define s2 A0
#define s3 A3
#define s4 A2
#define s5 A7
const uint8_t sPin[6] = {s0, s1, s2, s3, s4, s5};
#define initlibraryrobot initlibraryrobot

#define ff true
#define bb false

//led 
#define buzzer 8
#define led1 12
#define led2 13

void remotesetup();
void initlibraryrobot();
void manualcontrol();
void autocontrol();
void readsensor();
void lampusensor();
void checkComplete();

void motorjalankanan(int8_t speed);
void motorjalankiri(int8_t speed);
void testmotorkanan(int8_t speed);
void testmotorkiri(int8_t speed);
void testkeduamotor(int8_t speed_kiri, int8_t speed_kanan);
void belokiri(int8_t speed);
void belokanan(int8_t speed);
void belokirimaju(int8_t speed);
void belokananmaju(int8_t speed);
void majutimer(int8_t speed, int delay_);
void linefindkanan(int8_t speed, int sensor);
void linefindkiri(int8_t speed, int sensor);
void lf_crossfind(int8_t speed);
void lf_delay(int8_t speed, int delay_);
void linefollower(int8_t speed);
void lf_crsfindskip(int8_t speed, bool arah, int skip);
void buzzeron(int jumlah);
void lf_crossfindArah(int8_t speed, bool arah);

int detectcross();
float sampling();
int satusensor(int pin);
void kirimdata(int data);
void ifReadcond(int data);

#endif