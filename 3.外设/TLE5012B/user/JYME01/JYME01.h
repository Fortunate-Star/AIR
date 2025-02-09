#ifndef _JYME01__H
#define _JYME01__H
#include "main.h"

void JYME01_Init(UART_HandleTypeDef *huart);

void JYME01_Process(UART_HandleTypeDef *huart);

void JYME01_RxCpltCallback(void);
#endif
