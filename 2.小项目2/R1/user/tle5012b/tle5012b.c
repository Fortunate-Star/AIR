#include "main.h"
#include "tle5012b.h"
#include "stdarg.h"
#include "stdio.h"
 
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart2;
 
float tle5012(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
 
    uint16_t ang_reg_v = 0x8021, data_v;
    HAL_SPI_Transmit_DMA(&hspi4, (uint8_t *)(&ang_reg_v),1);
    HAL_SPI_Receive_DMA(&hspi4, (uint8_t *)(&data_v), 1);
    data_v = data_v & 0x7fff;
    float ang_v = data_v / (0x7fff / 360.0);
		float xiaoshu;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		xiaoshu=((ang_v * 1000) - (int)(ang_v)*1000)/1000;
    HAL_Delay(50);
	return ((int)ang_v)+xiaoshu;
}

