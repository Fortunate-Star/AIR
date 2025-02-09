#include "main.h"
											/*此函数为uart的DMA空闲中断函数，即使发送一个数据，也会进行中断函数的处理*/

//初始化
void uart_DMA_Init(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)//就是在初始化时提前接收一次数据
{
	HAL_UARTEx_ReceiveToIdle_DMA(huart,pData,Size);
}

//发送函数
void uart_DMA_transimit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
	HAL_UART_Transmit_DMA(huart,pData,Size);
	HAL_Delay(1000);
}

//中断回调函数重定义，在该函数内可以写，发生中断后要执行的任务
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{

//}
