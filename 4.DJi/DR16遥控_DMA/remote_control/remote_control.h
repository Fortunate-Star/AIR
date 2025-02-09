#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "main.h" 

#define proportion_a  0.01515151
#define proportion_b -15.51515151

typedef struct
{
  uint16_t ch0;//�� ���� 
  uint16_t ch1;//�� ���� 
  uint16_t ch2;//�� ���� 
  uint16_t ch3;//�� ���� 
  uint16_t ch4;//������ ��->�� 1684->364
  uint8_t s1;//�� ������ 132
  uint8_t s2;//�� ������ 132
	
  uint8_t last_s1;
  uint8_t last_ch4;
 
}RC_Ctl_t;

//�ú�������Դ�A��DBUS
void remote_control_Init(void);

//ң�������񣬷���while�в��Ͻ���
void remote_control_task(void);

//�жϽ��պ�������
void remote_control_callback(void);


#endif

