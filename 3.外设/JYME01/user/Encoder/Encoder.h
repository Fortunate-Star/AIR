#ifndef _ENCODER_H
#define _ENCODER_H
#include "main.h"

#define SPI4_CS1_0  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)
#define SPI4_CS1_1  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET)

#define SPI4_CS2_0  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET)
#define SPI4_CS2_1  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET)

#define SPI4_CS3_0  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET)
#define SPI4_CS3_1  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET)

#define SPI4_CS4_0  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET)
#define SPI4_CS4_1  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET)

#define SPI4_CS0_0  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET)
#define SPI4_CS0_1  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET)
void tle5012(void);
//uint16_t SPI4_ReadWriteHalfword(uint16_t data);
//uint16_t ReadRegister(int ID);
//uint16_t Read_M1_Encoder(void);

#endif 

