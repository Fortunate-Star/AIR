#include "main.h"
 
#include "stdarg.h"
#include "stdio.h"
 
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart4;
 
void USARTx_Printf (char *fmt_pa, ...);
 
void tle5012(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
 
    uint16_t ang_reg_v = 0x8021, data_v;
    HAL_SPI_Transmit(&hspi4, (uint8_t *)(&ang_reg_v), 1, 0xff);
    HAL_SPI_Receive(&hspi4, (uint8_t *)(&data_v), 1, 0xff);
 
    data_v = data_v & 0x7fff;
    float ang_v = data_v / (0x7fff / 360.0);
 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
 
    USARTx_Printf("angle: %d.%03d\n", (int)ang_v, (int)((ang_v * 1000) - (int)(ang_v)*1000));
    HAL_Delay(500);
}
 
void USARTx_Printf (char *fmt_pa, ...)
{
    static char buffer_v[50];
    va_list arg_p;//定义指针arg_ptr
    va_start(arg_p, fmt_pa);//把arg_ptr定位到fmt开始的的位置
    uint8_t len_v = vsnprintf(buffer_v, 50, fmt_pa, arg_p);
    va_end(arg_p);
    while (HAL_UART_Transmit_IT(&huart4, (uint8_t *)buffer_v, len_v) != HAL_OK)
        ;
}



//#include "Encoder.h"

//uint16_t SPI4_ReadWriteHalfword(uint16_t data)
//{
//	uint8_t cnt=0;	
//	while((SPI4->SR&1<<1)==0)
//	{
//		cnt++;
//		if(cnt>200)return 0;
//	}
//	
//	SPI4->DR = data;
//  cnt=0;	
//	
//	while((SPI4->SR&1<<0)==0)
//	{
//		cnt++;
//		if(cnt>200)return 0;
//	}
//	return SPI4->DR;
//}


//uint16_t ReadRegister(int ID)
//{
//	uint16_t data;
//	switch(ID)
//	{	
//		case 0:
//						SPI4_CS1_0;
//						SPI4_ReadWriteHalfword(0x8020);
//						GPIOE->MODER &= 0xFFFFCFFF;
//						data =32768 -  (SPI4_ReadWriteHalfword(0xff)&0x7fff);
//						GPIOE->MODER |= 0x00002000;
//						SPI4_CS1_1;
//						break;
//		case 1:
//						SPI4_CS2_0;
//						SPI4_ReadWriteHalfword(0x8020);
//						GPIOE->MODER &= 0xFFFFCFFF;
//						data = 32768 - (SPI4_ReadWriteHalfword(0xff)&0x7fff);
//						GPIOE->MODER |= 0x00002000;
//						SPI4_CS2_1;
//						break;
//		case 2:
//						SPI4_CS3_0;
//						SPI4_ReadWriteHalfword(0x8020);
//						GPIOE->MODER &= 0xFFFFCFFF;
//						data = 32768 - (SPI4_ReadWriteHalfword(0xff)&0x7fff);
//						GPIOE->MODER |= 0x00002000;
//						SPI4_CS3_1;
//						break;
//		case 3:
//						SPI4_CS4_0;
//						SPI4_ReadWriteHalfword(0x8020);
//						GPIOE->MODER &= 0xFFFFCFFF;
//						data = 32768 - (SPI4_ReadWriteHalfword(0xff)&0x7fff);
//						GPIOE->MODER |= 0x00002000;
//						SPI4_CS4_1;
//						break;
//	}
//	return data;
//}
//uint16_t Read_M1_Encoder(void)
//{
//	uint16_t data;
//	SPI4_CS1_0; 
//	SPI4_ReadWriteHalfword(0x8020);
//	GPIOE->MODER &= 0xFFFFCFFF;
//	data = (SPI4_ReadWriteHalfword(0xff)&0x7fff);
//	GPIOE->MODER |= 0x00002000;
//	SPI4_CS1_1;
//	
//	return data;
//}

