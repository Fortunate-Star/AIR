#include "encoder.h"
uint32_t sys_tick;
int speed1;
void Encoder_Init(TIM_HandleTypeDef *htim)
{
	HAL_TIM_Encoder_Start(htim,TIM_CHANNEL_ALL);//这里把PWM通道全部打开了，原因未知
}


int Read_speed(TIM_HandleTypeDef *htim)
{
	int temp;
	temp = (short)__HAL_TIM_GetCounter(htim);
	__HAL_TIM_SetCounter(htim,0);
	return temp;
}

//滴答定时器
void Read(void)
{
	if(uwTick-sys_tick<10)
	return;
	sys_tick = uwTick;
	speed1 = Read_speed(&htim2);
}
