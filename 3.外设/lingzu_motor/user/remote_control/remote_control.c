													/*//�ú�������Դ�A��DBUS*/
#include "remote_control.h"
#include <stdio.h>
#include <math.h>
// ���峣�� A, B �� L����Щ�Ƕ�ֵ
#define A 45.0 // ���磬�Ƕ� A ��ֵ����λ���ȣ�
#define B 45.0 // ���磬�Ƕ� B ��ֵ����λ���ȣ�
#define L 1.0  // ���磬���� L ��ֵ
#define M_PI 3.1415926

float V_x, V_y, w;
float V_A = 0, V_B = 0, V_C = 0, V_D = 0;

uint8_t buffer[18]={0};//ң�ط�����ֵ
RC_Ctl_t RC_Ctl;//��������ֵ�������㣬ת���ýṹ��
extern UART_HandleTypeDef huart1;
extern float speed_turn[6];
extern float speed_6020;
//��ʼ��
void remote_control_Init(void)
{
	HAL_UART_Receive_DMA(&huart1,buffer,18);
	
}

//ң������ȫ��⣬��ֹң����δ��ʱͨ�絼�³����ܷ�
uint8_t remote_control_security(void)
{
	if(RC_Ctl.ch0 == 0 || RC_Ctl.ch1 == 0 || RC_Ctl.ch2 == 0 || RC_Ctl.ch3 == 0) return 1;
	else return 0;
}


//ң���������񣬷���while�в��Ͻ���
void remote_control_task(void)
{
	HAL_UART_Receive_IT(&huart1,buffer,18);//�����жϽ���
	V_x =  speed_turn_limit(V_x,remote_control_limit(RC_Ctl.ch2),5);
	V_y = -speed_turn_limit(V_y,remote_control_limit(RC_Ctl.ch3),5);
	 w	=  speed_turn_limit(w,remote_control_limit(RC_Ctl.ch0),3);
	calculateVelocities(V_x,V_y,w,&V_A,&V_B,&V_C,&V_D);
	speed_turn[0] = V_A; speed_turn[1] = V_D; speed_turn[2] = V_C; speed_turn[3] = V_B;
	switch(RC_Ctl.s2)
	{
		case 1:speed_6020 = 0.5; break;
		case 3:speed_6020 = 0; break;
		case 2:speed_6020 = 1; break;
	}
	
	

}

//�ú�����ң������Χ��364-1684��Ϊ-10 - 10
float remote_control_limit(uint16_t remote_num)
{	float z = remote_num * proportion_a + proportion_b;
	if(z > -0.005 && z < 0.005) z = 0;
	
	return z;
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


// ���� V_a, V_b, V_c, V_d �ĺ�����wΪ���ٶ�
void calculateVelocities(float V_x, float V_y, float w, float *V_a, float *V_b, float *V_c, float *V_d) 
	{
    // �Ƕ� A �� B ת��Ϊ����
    float radA = A * M_PI / 180.0;
    float radB = B * M_PI / 180.0;

    // ���� cos(A) �� cos(B)
    float cosA = cos(radA);
    float cosB = cos(radB);

    // ���� V_a, V_b, V_c, V_d
    *V_a = V_x * cosB - V_y * cosA + L * w;
    *V_b = -V_x * cosB - V_y * cosA + L * w;
    *V_c = -V_x * cosB + V_y * cosA + L * w;
    *V_d = V_x * cosB + V_y * cosA + L * w;
}

//Ŀ���ٶ����ƺ����������л���λ  ������ٶ�ֵ  ת�����ң����ֵ��-10 - 10�� speed_turn�����ֵ
float speed_turn_limit(float speed , float remote , float i)
{	
	speed = 0.1*i*remote;
	if(speed > -0.005 && speed < 0.005) speed = 0;
	if(speed < -(i+0.2)) speed=0;
	return speed;
}

