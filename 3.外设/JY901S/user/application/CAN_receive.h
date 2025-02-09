/**
  ****************************(C) COPYRIGHT 2019 DJI****************************

  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
	
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN   hcan1      //CAN串口1
#define GIMBAL_CAN    hcan2      //CAN串口2

/* CAN send and receive ID */ 
// CAN 发送和接收结构体 ID
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
	  CAN_GIMBAL_ALL_ID = 0x1FF,
		CAN_3508_M5_ID = 0x205,
		CAN_3508_M6_ID = 0x206,
	  CAN_3508_M7_ID = 0x207,
  	CAN_3508_M8_ID = 0x208,
	
		CAN_GIMBAL_ID= 0x2FF,
		CAN_6020_M1_ID = 0x209,
	CAN_6020_M2_ID = 0x20A,
} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;    			  //转子的机械角度  0-8198     
    int16_t  speed_rpm;			//转子的转速 ,转每分钟
    int16_t  given_current;	//实际转矩电流16384
    uint8_t  temperate;			//电机温度
    int16_t  last_ecd;			//上一次的转子机械角度
} motor_measure_t;

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN2_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw:    (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch:  (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot:  (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev:    (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);


/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
 const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_6020_motor_measure_point(uint8_t i);

//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

#endif






