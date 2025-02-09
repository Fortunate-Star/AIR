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
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	
}
	
void drive_servos(int16_t angle_servos)
{
	
	htim2.Instance->CCR1=angle_servos;
	htim2.Instance->CCR2=angle_servos;
	htim2.Instance->CCR3=angle_servos;
	htim2.Instance->CCR4=angle_servos;
	
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,angle_servos);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,angle_servos);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,angle_servos);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,angle_servos);
		
}

