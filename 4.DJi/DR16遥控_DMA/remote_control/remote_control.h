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

//遥控器任务，放在while中不断接收
void remote_control_task(void);

//中断接收函数任务
void remote_control_callback(void);


#endif

