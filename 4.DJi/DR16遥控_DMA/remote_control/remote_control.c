													/*//�ú�������Դ�A��DBUS*/
#include "remote_control.h"
#include <stdio.h>
#include <math.h>

uint8_t buffer[18]={0};//ң�ط�����ֵ
RC_Ctl_t RC_Ctl;//��������ֵ�������㣬ת���ýṹ��
extern UART_HandleTypeDef huart1;

void remote_control_Init(void)
{
	HAL_UART_Receive_DMA(&huart1,buffer,18);
	
}

//ң���������񣬷���while�в��Ͻ���
void remote_control_task(void)
{
	HAL_UART_Receive_DMA(&huart1,buffer,18);//�����жϽ���

}


//�жϽ��պ�������
void remote_control_callback(void)
{
		RC_Ctl.ch0 = (buffer[0] | buffer[1] << 8) & 0x07FF; 
		RC_Ctl.ch1 = ((buffer[1] >> 3) | (buffer[2] << 5)) & 0x07FF;
		RC_Ctl.ch2 = ((buffer[2] >> 6) | (buffer[3] << 2) |(buffer[4] << 10)) & 0x07FF;
		RC_Ctl.ch3 = ((buffer[4] >> 1) | (buffer[5]<<7)) & 0x07FF;
		RC_Ctl.s1  = ((buffer[5] >> 4) & 0x000C) >> 2;
		RC_Ctl.s2  = ((buffer[5] >> 4) & 0x0003);
		RC_Ctl.ch4 =  buffer[16] | (buffer[17]<<8);
//	while(remote_control_security());
}



