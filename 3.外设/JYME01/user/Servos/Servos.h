#ifndef __SERVOS_H
#define __SERVOS_H
#include "main.h"
#include "tim.h" 

void servos_Init(void);
void drive_servos1(int16_t angle_servos);
void drive_servos2(int16_t angle_servos);
void drive_servos3(int16_t angle_servos);
void drive_servos4(int16_t angle_servos);

void drive_servos_jiazhua1(int16_t angle_servos);
void drive_servos_jiazhua2(int16_t angle_servos);
#endif

