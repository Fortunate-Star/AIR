#ifndef _UART_DMA_H
#define _UART_DMA_H
void uart_DMA_Init(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void uart_DMA_transimit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);

#endif

