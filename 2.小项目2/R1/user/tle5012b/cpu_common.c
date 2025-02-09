#include "cpu_common.h"

static UART_HandleTypeDef *huart = &huart4;

int fputc(int ch, FILE *f){
	HAL_UART_Transmit( huart, (uint8_t *)&ch, sizeof(ch), 0xff );
	return ch;
}
