#include "Vofa_firewater.h"
#include <stdarg.h>
#include "main.h"
#include <string.h>
#include <stdio.h>
#define TX_BUF_LEN  256     /* ���ͻ�����������������Ҫ���е��� */
extern UART_HandleTypeDef huart2;
uint8_t TxBuf[TX_BUF_LEN];  /* ���ͻ�����                       */
void MyPrintf(const char *__format, ...)
{
  va_list ap;
  va_start(ap, __format);
  
  /* ��շ��ͻ����� */
  memset(TxBuf, 0x0, TX_BUF_LEN);
  
  /* ��䷢�ͻ����� */
  vsnprintf((char*)TxBuf, TX_BUF_LEN, (const char *)__format, ap);
  va_end(ap);
  int len = strlen((const char*)TxBuf);
  
  /* �����ڷ������� */
  HAL_UART_Transmit(&huart2, (uint8_t*)&TxBuf, len, 0xFFFF);
}

void firewater_task(void)
{
	float a=1;
	MyPrintf("%.2f\n",a);
}


