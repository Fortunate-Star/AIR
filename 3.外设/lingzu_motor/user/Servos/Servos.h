#ifndef __SERVOS_H
#define __SERVOS_H
#include "main.h"
#include "tim.h" 
void servos_Init(void);
void drive_servos(int16_t angle_servos);

#endif

