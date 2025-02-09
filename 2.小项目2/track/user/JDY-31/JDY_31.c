#include "JDY_31.h"

extern uint8_t rx_data[2];
extern uint8_t rx_num;
//�ú���Ҫ����mian�������棬��Ҫ����.it��׺���жϺ��������
void JDY_31_Init(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	 HAL_UART_Receive_IT(huart,pData,Size);
}

void JDY_31_contorl(void)//Ҫ����.it��׺���жϺ�����
{
	switch(rx_num)
	{
		case 0x00:wheel_stop();
			break;
		case 0x01:go_head();
			break ;
		case 0x02:go_back();
			break ;
		case 0x03:turn_left();
			break ;
		case 0x04:turn_right();
			break ;
		case 0x05:go_left();
			break ;
		case 0x06:go_right();
			break ;
	}
}

