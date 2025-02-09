													/*//该函数仅针对大疆A板DBUS*/
#include "remote_control.h"
#include <stdio.h>
#include <math.h>
// 定义常量 A, B 和 L，这些是定值
#define A 45.0 // 例如，角度 A 的值（单位：度）
#define B 45.0 // 例如，角度 B 的值（单位：度）
#define L 1.0  // 例如，常数 L 的值
#define M_PI 3.1415926

float V_x, V_y, w;
int16_t Angle_x,Angle_y,Angle_w;
float V_A = 0, V_B = 0, V_C = 0, V_D = 0;

uint8_t buffer[18]={0};//遥控反馈的值
RC_Ctl_t RC_Ctl;//将反馈的值进行运算，转给该结构体
RC_Ct2_t RC_REMOT;//将反馈的数据进行数值更改

extern UART_HandleTypeDef huart1;
extern float angle_turn[4];


//初始化
void remote_control_Init(void)
{
	HAL_UART_Receive_DMA(&huart1,buffer,18);
}

//遥控器主任务，放在while中不断接收
void remote_control_task(void)
{
	HAL_UART_Receive_DMA(&huart1,buffer,18);//开启中断接收
	Angle_x = RC_REMOT.ch2 * remote_tle5012 * 2;
	angle_turn[0] = 6000 + Angle_x ; angle_turn[1] = 10000 + Angle_x;
	angle_turn[2] = 1600 + Angle_x; angle_turn[3] = 8000 + Angle_x ;

}
//中断接收函数任务
void remote_control_callback(void)
{
	
	Get_remote_data();
	Get_remote_data_deal();

}
//接收遥控器原始数据，范围为364-16384，中间值为1024
void Get_remote_data(void)
{
		RC_Ctl.ch0 = (buffer[0] | buffer[1] << 8) & 0x07FF; 
		RC_Ctl.ch1 = ((buffer[1] >> 3) | (buffer[2] << 5)) & 0x07FF;
		RC_Ctl.ch2 = ((buffer[2] >> 6) | (buffer[3] << 2) |(buffer[4] << 10)) & 0x07FF;
		RC_Ctl.ch3 = ((buffer[4] >> 1) | (buffer[5]<<7)) & 0x07FF;
	  RC_Ctl.ch4 =  buffer[16] | (buffer[17]<<8);
		RC_Ctl.s1  = ((buffer[5] >> 4) & 0x000C) >> 2;
		RC_Ctl.s2  = ((buffer[5] >> 4) & 0x0003);
}

//将遥控器反馈数据的中间值改为0
void Get_remote_data_deal(void)
{
	RC_REMOT.ch0 = (int16_t)(RC_Ctl.ch0) - 1024;
	RC_REMOT.ch1 = (int16_t)(RC_Ctl.ch1) - 1024;
	RC_REMOT.ch2 = (int16_t)(RC_Ctl.ch2) - 1024;
  RC_REMOT.ch3 = (int16_t)(RC_Ctl.ch3) - 1024;
  RC_REMOT.ch4 = (int16_t)(RC_Ctl.ch4) - 1024;
	RC_REMOT.s1  = RC_Ctl.s1;
	RC_REMOT.s2  = RC_Ctl.s2;
}



// 计算 V_a, V_b, V_c, V_d 的函数，w为角速度
void calculateVelocities(float V_x, float V_y, float w, float *V_a, float *V_b, float *V_c, float *V_d) 
	{
    // 角度 A 和 B 转换为弧度
    float radA = A * M_PI / 180.0;
    float radB = B * M_PI / 180.0;

    // 计算 cos(A) 和 cos(B)
    float cosA = cos(radA);
    float cosB = cos(radB);

    // 计算 V_a, V_b, V_c, V_d
    *V_a = V_x * cosB - V_y * cosA + L * w;
    *V_b = -V_x * cosB - V_y * cosA + L * w;
    *V_c = -V_x * cosB + V_y * cosA + L * w;
    *V_d = V_x * cosB + V_y * cosA + L * w;
}

