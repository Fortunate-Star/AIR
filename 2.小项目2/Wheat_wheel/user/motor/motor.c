#include "motor.h"

int abs(int p)
{
	if(p>0) return p;
	if(p<0) return -p;
	return p;
}

void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//左下
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);//右上
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);//右下
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//左上
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,abs(5000));
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,abs(5000));
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,abs(5000));
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,abs(5000));
	go_stop();

	
}

void go_stop(void)
{
	AIN1=0; AIN2=0; BIN1=0; BIN2=0; CIN1=0; CIN2=0; DIN1=0; DIN2=0;

}

void go_head(void)
{
	left_up_go;	right_up_go;	left_down_go;	right_dowm_go;
}

void go_back(void)
{
	left_up_back;	 right_up_back;	left_down_back;	right_dowm_back;
}

void stop_left(void)
{
	left_up_back;	left_down_back;	right_up_go;	right_dowm_go;
}

void stop_right(void)
{
	left_up_go;	left_down_go;	right_up_back;	right_dowm_back;
}

void go_left(void)
{
	left_up_back;		left_down_go;		right_up_go;		right_dowm_back
}

void go_right(void)
{
	left_up_go;	left_down_back;	right_up_back;	right_dowm_go;
}
