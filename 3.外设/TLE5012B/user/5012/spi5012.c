#include "spi5012.h"
#include "stdarg.h"
#include "stdio.h"
 
extern SPI_HandleTypeDef hspi4;
//extern UART_HandleTypeDef huart2;
 
//float tle5012(int ID)
//{
//	float data,ang_v,xiaoshu;
//	uint16_t ang_reg_v;
//	uint16_t data_v;
//	switch(ID)
//	{
//		case 0:
//			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
//			ang_reg_v = 0x8021;
//			HAL_SPI_Transmit_DMA(&hspi4, (uint8_t *)(&ang_reg_v),1);
//			HAL_SPI_Receive_DMA(&hspi4, (uint8_t *)(&data_v), 1);
//			data_v = data_v & 0x7fff;
//			ang_v = data_v / (0x7fff / 360.0);
//			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
//			xiaoshu=((ang_v * 1000) - (int)(ang_v)*1000)/1000;
//			HAL_Delay(50);
//			data = ((int)ang_v)+xiaoshu;

//		case 1:
//			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
//			ang_reg_v = 0x8021;
//			HAL_SPI_Transmit_DMA(&hspi4, (uint8_t *)(&ang_reg_v),1);
//			HAL_SPI_Receive_DMA(&hspi4, (uint8_t *)(&data_v), 1);
//			data_v = data_v & 0x7fff;
//			ang_v = data_v / (0x7fff / 360.0);
//			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
//			xiaoshu=((ang_v * 1000) - (int)(ang_v)*1000)/1000;
//			HAL_Delay(50);
//			data = ((int)ang_v)+xiaoshu;		

//		case 2:
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
//			ang_reg_v = 0x8021;
//			HAL_SPI_Transmit_DMA(&hspi4, (uint8_t *)(&ang_reg_v),1);
//			HAL_SPI_Receive_DMA(&hspi4, (uint8_t *)(&data_v), 1);
//			data_v = data_v & 0x7fff;
//			ang_v = data_v / (0x7fff / 360.0);
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
//			xiaoshu=((ang_v * 1000) - (int)(ang_v)*1000)/1000;
//			HAL_Delay(50);
//			data = ((int)ang_v)+xiaoshu;	
////		case 3:
////			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
////			ang_reg_v = 0x8021;
////			HAL_SPI_Transmit_DMA(&hspi4, (uint8_t *)(&ang_reg_v),1);
////			HAL_SPI_Receive_DMA(&hspi4, (uint8_t *)(&data_v), 1);
////			data_v = data_v & 0x7fff;
////			float ang_v = data_v / (0x7fff / 360.0);
////			float xiaoshu;
////			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
////			xiaoshu=((ang_v * 1000) - (int)(ang_v)*1000)/1000;
////			HAL_Delay(50);
////			data = ((int)ang_v)+xiaoshu;			
//}
//	return data;
//   
//}
// 
 
uint16_t SPI4_ReadWriteHalfword(uint16_t data)
{
	   uint16_t receivedData = 0;

    // 发送数据并接收返回值
    if (HAL_SPI_TransmitReceive(&hspi4, (uint8_t *)&data, (uint8_t *)&receivedData, 1, 1000) != HAL_OK)
    {
        // 错误处理
        return 0; // 或者进行其他错误处理
    }

    return receivedData;
}


uint16_t ReadRegister(int ID)
{
	uint16_t data;
	switch(ID)
	{	
		case 0:
						SPI4_CS1_0;
						SPI4_ReadWriteHalfword(0x8020);
						data =32768 -  (SPI4_ReadWriteHalfword(0xff)&0x7fff);
						SPI4_CS1_1;
						break;
		case 1:
						SPI4_CS2_0;
						SPI4_ReadWriteHalfword(0x8020);
						data = 32768 - (SPI4_ReadWriteHalfword(0xff)&0x7fff);
						SPI4_CS2_1;
						break;
		case 2:
						SPI4_CS3_0;
						SPI4_ReadWriteHalfword(0x8020);
						data = 32768 - (SPI4_ReadWriteHalfword(0xff)&0x7fff);
						SPI4_CS3_1;
						break;
		case 3:
						SPI4_CS4_0;
						SPI4_ReadWriteHalfword(0x8020);
						GPIOE->MODER &= 0xFFFFCFFF;
						data = 32768 - (SPI4_ReadWriteHalfword(0xff)&0x7fff);
						GPIOE->MODER |= 0x00002000;
						SPI4_CS4_1;
						break;
	}
	return data;
}