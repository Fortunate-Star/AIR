#include "main.h"
#include "spi5012.h"
#include "stdarg.h"
#include "stdio.h"
 
extern SPI_HandleTypeDef hspi4;
//extern UART_HandleTypeDef huart2;
 
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
 
 
//void USARTx_Printf (char *fmt_pa, ...);
// 
//void tle5012(void)
//{
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
// 
//    uint16_t ang_reg_v = 0x8021, data_v;
//    HAL_SPI_Transmit(&hspi4, (uint8_t *)(&ang_reg_v), 1, 0xff);
//    HAL_SPI_Receive(&hspi4, (uint8_t *)(&data_v), 1, 0xff);
// 
//    data_v = data_v & 0x7fff;
//    float ang_v = data_v / (0x7fff / 360.0);
// 
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
// 
//    USARTx_Printf("angle: %d.%03d\n", (int)ang_v, (int)((ang_v * 1000) - (int)(ang_v)*1000));
//    HAL_Delay(500);
//}
// 
//void USARTx_Printf (char *fmt_pa, ...)
//{
//    static char buffer_v[50];
//    va_list arg_p;//定义指针arg_ptr
//    va_start(arg_p, fmt_pa);//把arg_ptr定位到fmt开始的的位置
//    uint8_t len_v = vsnprintf(buffer_v, 50, fmt_pa, arg_p);
//    va_end(arg_p);
//    while (HAL_UART_Transmit_IT(&huart2, (uint8_t *)buffer_v, len_v) != HAL_OK)
//        ;
//}
