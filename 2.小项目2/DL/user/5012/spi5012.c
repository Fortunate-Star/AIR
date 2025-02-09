#include "spi5012.h"
#include "stdarg.h"
#include "stdio.h"
 
extern SPI_HandleTypeDef hspi4;
//extern UART_HandleTypeDef huart2;
 

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