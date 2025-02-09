#ifndef __ENCODER_H
#define __ENCODER_H
#include "main.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void Encoder_Init(TIM_HandleTypeDef *htim);
int Read_speed(TIM_HandleTypeDef *htim);
void Read(void);
#endif
