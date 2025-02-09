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

//ң������ȫ��⣬��ֹң����δ��ʱͨ�絼�³����ܷ�
uint8_t remote_control_security(void);

//ң�������񣬷���while�в��Ͻ���
void remote_control_task(void);

//�ú�����ң������Χ��364-1684��Ϊ-10 - 10
float remote_control_limit(uint16_t remote_num);


//�жϽ��պ�������
void remote_control_callback(void);

// ���� V_a, V_b, V_c, V_d �ĺ���
void calculateVelocities(float V_x, float V_y, float w, float *V_a, float *V_b, float *V_c, float *V_d) ;

//Ŀ���ٶ����ƺ����������л���λ  ������ٶ�ֵ  ת�����ң����ֵ��-10 - 10�� speed_turn�����ֵ
float speed_turn_limit(float speed , float remote , float i);
#endif

