#include "JDY_31.h"
#include "motor.h"

extern uint8_t rx_data[2];
extern uint8_t rx_num;
//该函数要放在mian函数里面，还要放在.it后缀的中断函数里！！！
void JDY_31_Init(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	 HAL_UART_Receive_IT(huart,pData,Size);
}

void JDY_31_contorl(void)//要加在.it后缀的中断函数里
{
	switch(rx_num)
	{
		case 0x00:go_stop();
			break;
		case 0x01:go_head();
			break ;
		case 0x02:go_back();
			break ;
		case 0x03:go_left();
			break ;
		case 0x04:go_right();
			break ;
		case 0x05:stop_left();
			break ;
		case 0x06:stop_right();
			break ;
	}
}

