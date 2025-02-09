#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "main.h" 

#define proportion_a  0.01515151
#define proportion_b -15.51515151
#define remote_tle5012 12.41212121212121

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

typedef struct
{
  int16_t ch0;//�� ���� 
  int16_t ch1;//�� ���� 
  int16_t ch2;//�� ���� 
  int16_t ch3;//�� ���� 
  int16_t ch4;//������ ��->�� -660->660
  uint8_t s1;//�� ������ 132
  uint8_t s2;//�� ������ 132
	
 
}RC_Ct2_t;

//�ú�������Դ�A��DBUS
void remote_control_Init(void);

//����ң����ԭʼ���ݣ���ΧΪ364-16384���м�ֵΪ1024
void Get_remote_data(void);
	
//��ң�����������ݵ��м�ֵ��Ϊ0
void Get_remote_data_deal(void);

//ң�������񣬷���while�в��Ͻ���
void remote_control_task(void);

//�жϽ��պ�������
void remote_control_callback(void);

// ���� V_a, V_b, V_c, V_d �ĺ���
void calculateVelocities(float V_x, float V_y, float w, float *V_a, float *V_b, float *V_c, float *V_d) ;


#endif

