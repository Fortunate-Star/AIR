#include "Servos.h"

/*�������
1.���ȿ�ʼHAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
2.���������жϣ�
��ɫ�����
PSC��180-1
ARR��2000
arp��ENABLE
��Χ��200-1200
*/
void drive_servos(int16_t angle_servos)
{
	
			htim2.Instance->CCR1=angle_servos;
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,angle_servos);
		
}

