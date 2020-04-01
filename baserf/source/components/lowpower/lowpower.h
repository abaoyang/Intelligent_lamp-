/***********************************************************************************
  Filename:     lowpower.h

  Description:  Basic RF library header file

***********************************************************************************/
#ifndef LOW_POWER_H
#define LOW_POWER_H

#include <ioCC2530.h>

typedef unsigned char uchar;
typedef unsigned int  uint;
typedef unsigned long ulong;

#define LED1 P1_0            //P1.0口控制LED1
#define LED2 P1_1            //P1.1口控制LED2
#define BUZZER P1_1
#define VOICEIN P0_1
#define BUTTONIN P2_0

void DelayMS(uint msec);
void InitLed(void);
void SysPowerMode(uchar mode);
void InitSleepTimer(void);
void Set_ST_Period(uint sec); 

#endif

