#include "main.h"
											/*�˺���Ϊuart��DMA�����жϺ�������ʹ����һ�����ݣ�Ҳ������жϺ����Ĵ���*/

//��ʼ��
void uart_DMA_Init(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)//�����ڳ�ʼ��ʱ��ǰ����һ������
{
	HAL_UARTEx_ReceiveToIdle_DMA(huart,pData,Size);
}

//���ͺ���
void uart_DMA_transimit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
	HAL_UART_Transmit_DMA(huart,pData,Size);
	HAL_Delay(1000);
}

//�жϻص������ض��壬�ڸú����ڿ���д�������жϺ�Ҫִ�е�����
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{

//}
