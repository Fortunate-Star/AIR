#include "JYME01.h"
uint8_t usart_txbuf[10]={0X01,0X03,0X00,0XD5,0X00,0X02,0XD5,0XF3};
uint8_t rxbuf[10];
float combined_value;
void JYME01_Init(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_DMA(huart,rxbuf,9);
}


void JYME01_Process(UART_HandleTypeDef *huart)
{
	for(uint8_t i=0;i<9;i++)
		{
		HAL_UART_Transmit_DMA(huart,&usart_txbuf[i],9);
		
		}
	 HAL_UART_Receive_DMA(huart,rxbuf,9);
	
}

void JYME01_RxCpltCallback(void)
{
	if (rxbuf[0] == NULL)
    {
        return;
    }
  	if ( rxbuf[0] != 0X01||rxbuf[1] != 0X03|| rxbuf[2] != 0X04||rxbuf[3]!=0X00)
	  {
	    	return;
	  }
		combined_value = (float)((rxbuf[3] << 24)|(rxbuf[4] << 16)|(rxbuf[5] << 8)|(rxbuf[6]))/0x40000*360.0;
		
}