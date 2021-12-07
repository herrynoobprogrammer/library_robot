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
#define buzzer 8
const int sPin[6] = {s0, s1, s2, s3, s4, s5};
#define initlibraryrobot initlibraryrobot

void remotesetup();
void initlibraryrobot();
void manualcontrol();
void autocontrol();
void readsensor();

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
void buzzeron(int8_t loop_);
void test_sensor();

int detectcross();
int sampling();
int sampling_new();
int satusensor(int pin);

#endif