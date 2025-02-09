#include "Servos.h"

/*�������
1.���ȿ�ʼHAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
2.���������жϣ�
��ɫ�����
PSC��84-1
ARR��20000-1
arp��ENABLE
��Χ��500-2700
27
*/
void servos_Init(void)
{
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);	
	
}
	
void drive_servos1(int16_t angle_servos)
{
	htim5.Instance->CCR1=angle_servos;
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,angle_servos);
}

void drive_servos2(int16_t angle_servos)
{
	htim5.Instance->CCR2=angle_servos;
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,angle_servos);
}

void drive_servos3(int16_t angle_servos)
{
	htim5.Instance->CCR3=angle_servos;
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,angle_servos);
}

void drive_servos4(int16_t angle_servos)
{
	htim5.Instance->CCR1=angle_servos;
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,angle_servos);
}


void drive_servos_jiazhua1(int16_t angle_servos)
{
		htim4.Instance->CCR3=angle_servos;
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,angle_servos);
}
void drive_servos_jiazhua2(int16_t angle_servos)
{
			htim4.Instance->CCR4=angle_servos;
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,angle_servos);
}
//�󣺿� 700��800 ��1200��500
//�ң��� 1100��1600 ��1600��1200

