#ifndef __JDY31_H
#define __JDY31_H
#include "main.h"
#include "usart.h"

void JDY_31_Init(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void JDY_31_contorl(void);
#endif
