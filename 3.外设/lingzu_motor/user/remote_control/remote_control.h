#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "main.h" 

#define proportion_a  0.01515151
#define proportion_b -15.51515151

typedef struct
{
  uint16_t ch0;//右 左右 
  uint16_t ch1;//右 上下 
  uint16_t ch2;//左 左右 
  uint16_t ch3;//左 上下 
  uint16_t ch4;//左侧滚轴 下->上 1684->364
  uint8_t s1;//左 上中下 132
  uint8_t s2;//右 上中下 132
	
  uint8_t last_s1;
  uint8_t last_ch4;
 
}RC_Ctl_t;

//该函数仅针对大疆A板DBUS
void remote_control_Init(void);

//遥控器安全检测，防止遥控器未及时通电导致程序跑飞
uint8_t remote_control_security(void);

//遥控器任务，放在while中不断接收
void remote_control_task(void);

//该函数把遥控器范围从364-1684改为-10 - 10
float remote_control_limit(uint16_t remote_num);


//中断接收函数任务
void remote_control_callback(void);

// 计算 V_a, V_b, V_c, V_d 的函数
void calculateVelocities(float V_x, float V_y, float w, float *V_a, float *V_b, float *V_c, float *V_d) ;

//目标速度限制函数，用于切换挡位  输出的速度值  转换后的遥控器值（-10 - 10） speed_turn的最大值
float speed_turn_limit(float speed , float remote , float i);
#endif

