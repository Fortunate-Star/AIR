#include "motor.h"

int abs(int p)
{
	if(p>0) return p;
	if(p<0) return -p;
	return p;
}

void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	wheel_stop();
}



void left_wheel_head(void)
{
	AIN2=1;		AIN1=0;
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,abs(6500));
}	


void left_wheel_back(void)
{
	AIN2=0;		AIN1=1;
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,abs(6500));
}	

void left_wheel_stop(void)
{
	AIN2=1;		AIN1=1;
}

void right_wheel_head(void)
{		
	BIN2=1;		BIN1=0;
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,abs(6500));
}

void right_wheel_back(void)
{
	BIN2=0;		BIN1=1;
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,abs(6500));
}

void right_wheel_stop(void)
{
	BIN2=1;		BIN1=1;
}

void go_head(void)
{
	left_wheel_head();
	right_wheel_head();
}

void go_back(void)
{
	left_wheel_back();
	right_wheel_back();
}

void go_left(void)
{
	left_wheel_stop();
	right_wheel_head();
}

void go_right(void)
{
	left_wheel_head();
	right_wheel_stop();
}

void turn_left(void)
{
	left_wheel_back();
	right_wheel_head();
}

void turn_right(void)
{
	left_wheel_head();
	right_wheel_back();
}

void wheel_stop(void)
{
	AIN2=1;		AIN1=1;
	BIN2=1;		BIN1=1;
}

