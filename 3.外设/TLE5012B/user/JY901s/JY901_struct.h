#ifndef __JY901_STRUCT_H
#define __JY901_STRUCT_H
#include "main.h"
#define RXBUFFER_LEN 33		//接收3类数据，一共33位

typedef struct
{
	float angle[3];
}Angle;

typedef struct
{
	float a[3];
}Acc;

typedef struct
{
	float w[3];
}SGyro;

typedef struct User_USART
{
		uint8_t Rx_flag;											
		uint8_t Rx_len;												
		uint8_t frame_head;					//帧头
		uint8_t RxBuffer[RXBUFFER_LEN];		//接收缓冲
		Angle angle;						//角度
		Acc acc;							//加速度
		SGyro w;							//角速度
}User_USART;

#endif
