#include "encoder.h"
uint32_t sys_tick;
int speed1;
void Encoder_Init(TIM_HandleTypeDef *htim)
{
	HAL_TIM_Encoder_Start(htim,TIM_CHANNEL_ALL);//�����PWMͨ��ȫ�����ˣ�ԭ��δ֪
}


int Read_speed(TIM_HandleTypeDef *htim)
{
	int temp;
	temp = (short)__HAL_TIM_GetCounter(htim);
	__HAL_TIM_SetCounter(htim,0);
	return temp;
}

//�δ�ʱ��
void Read(void)
{
	if(uwTick-sys_tick<10)
	return;
	sys_tick = uwTick;
	speed1 = Read_speed(&htim2);
}
