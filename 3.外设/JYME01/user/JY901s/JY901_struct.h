#ifndef __JY901_STRUCT_H
#define __JY901_STRUCT_H
#include "main.h"
#define RXBUFFER_LEN 33		//����3�����ݣ�һ��33λ

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
		uint8_t frame_head;					//֡ͷ
		uint8_t RxBuffer[RXBUFFER_LEN];		//���ջ���
		Angle angle;						//�Ƕ�
		Acc acc;							//���ٶ�
		SGyro w;							//���ٶ�
}User_USART;

#endif
