/**
  ****************************(C) COPYRIGHT 2019 DJI****************************

  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
	
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN   hcan1      //CAN����1
#define GIMBAL_CAN    hcan2      //CAN����2

/* CAN send and receive ID */ 
// CAN ���ͺͽ��սṹ�� ID
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
    uint16_t ecd;    			  //ת�ӵĻ�е�Ƕ�  0-8198     
    int16_t  speed_rpm;			//ת�ӵ�ת�� ,תÿ����
    int16_t  given_current;	//ʵ��ת�ص���16384
    uint8_t  temperate;			//����¶�
    int16_t  last_ecd;			//��һ�ε�ת�ӻ�е�Ƕ�
} motor_measure_t;

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN2_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw:    (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch:  (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot:  (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev:    (0x208) ������������Ƶ���
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);


/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
 const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_6020_motor_measure_point(uint8_t i);

//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

#endif






