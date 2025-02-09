#include "Servos.h"

/*舵机控制
1.首先开始HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
2.进行死区判断，
粉色舵机：
PSC：180-1
ARR：2000
arp：ENABLE
范围：200-1200
*/
void drive_servos(int16_t angle_servos)
{
	
			htim2.Instance->CCR1=angle_servos;
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,angle_servos);
		
}

