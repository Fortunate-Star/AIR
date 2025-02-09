#ifndef __MOTOR_H
#define __MOTOR_H

#include "CAN_receive.h"
#include "pid.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "lib.h"
#include "spi5012.h"
#include <math.h>
#define abs(x) ((x)>0? (x):(-(x)))
#define M3508_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define G3508_SPEED_PID_KP 2000.0f			//3508����������
#define G3508_SPEED_PID_KI 0.0f
#define G3508_SPEED_PID_KD 0.0f

#define G3508_POSITION_FAST_PID_KP 0.0001f 
#define G3508_POSITION_FAST_PID_KI 0.0f
#define G3508_POSITION_FAST_PID_KD 0.01f

#define G3508_SPEED_DL_PID_KP 13000.0f				//10000									//3508��������
#define G3508_SPEED_DL_PID_KI 0.0f
#define G3508_SPEED_DL_PID_KD 2.0f
#define G3508_SPEED_PID_MAX_OUT 10000.0f
#define G3508_SPEED_PID_MAX_IOUT 2000.0f

#define G3508_POSITION_DL_PID_KP 0.005f 			//0.005
#define G3508_POSITION_DL_PID_KI 0.0f
#define G3508_POSITION_DL_PID_KD 0.01f
#define G3508_POSITION_PID_MAX_OUT 2.0f
#define G3508_POSITION_PID_MAX_IOUT 0.0f


#define G2006_SPEED_PID_KP 20000.0f				
#define G2006_SPEED_PID_KI 0
#define G2006_SPEED_PID_KD 0
#define G2006_SPEED_PID_MAX_OUT 30000.0f
#define G2006_SPEED_PID_MAX_IOUT 6000.0f

#define G2006_POSITION_PID_KP 0.01f 
#define G2006_POSITION_PID_KI 0
#define G2006_POSITION_PID_KD 0.005f
#define G2006_POSITION_PID_MAX_OUT 10.0f
#define G2006_POSITION_PID_MAX_IOUT 10.0f

#define G6020_SPEED_PID_KP 9500.0f      
#define G6020_SPEED_PID_KI 0.0f
#define G6020_SPEED_PID_KD 5000.0f
#define G6020_SPEED_PID_MAX_OUT 30000.0f
#define G6020_SPEED_PID_MAX_IOUT 6000.0f

#define G6020_POSITION_PID_KP 0.01f   
#define G6020_POSITION_PID_KI 0
#define G6020_POSITION_PID_KD 0.005
#define G6020_POSITION_PID_MAX_OUT 1.0f
#define G6020_POSITION_PID_MAX_IOUT 1.0f

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.0007314830295443713f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR


typedef struct
{
  const motor_measure_t *chassis_motor_measure;
	PidTypeDef motor_angle_pid;
  PidTypeDef motor_speed_pid;
  fp32 speed;
  fp32 speed_set;
	fp32 motor_angle;
  fp32 motor_angle_set;
	fp32 set_ramp_angle;
  int16_t give_current;
	fp32 min_angle_limit;		
	fp32 mid_angle_limit;
	fp32 max_angle_limit;
	
} Chassis_Motor_t;


typedef enum
{
	CHASSIS_ZERO_FORCE,                  //��ʼ��ģʽ
	CHASSIS_NO_MOVE,                     //���̱��ֲ���                      
	CHASSIS_MOVE,				  								//ǰ������
	CHASSIS_OPSFOLLOW,  			  //opsѭ��		
	CHASSIC_SLOW_MOVE,         //�����ƶ�
	CHASSIC_ABS_MOVE,         //�����ƶ�
	CHASSIC_FAST_SPEED,
	CHASSIC_SLOW_SPEED,
} chassis_behaviour_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Chassis_PID_t;



typedef struct
{
	Chassis_Motor_t motor_chassis[8];          //���̵������
	Chassis_Motor_t motor_chassis2[8];          //���̵������
	Chassis_Motor_t motor_2006;
	Chassis_Motor_t motor_6020[2];



	fp32 vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
	fp32 vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
	fp32 wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s

	fp32 vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
	fp32 vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
	fp32 wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s

	
	fp32 chassis_yaw_set;

	fp32 vx_max_speed;  //ǰ����������ٶ� ��λm/s
	fp32 vx_min_speed;  //ǰ��������С�ٶ� ��λm/s
	fp32 vy_max_speed;  //���ҷ�������ٶ� ��λm/s
	fp32 vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s
	
	fp32 chassis_yaw;   //�����ǵ�yaw�Ƕ�
  fp32 chassis_pitch; //�����ǵ�pitch�Ƕ�
  fp32 chassis_roll;  //�����ǵ�roll�Ƕ�


	const fp32 * chassis_w_offset;
} chassis_move_t;




extern void chassis_init(chassis_move_t *chassis_move_init);								//���̳�ʼ������Ҫ��pid��ʼ��
extern void chassis_feedback_update(chassis_move_t *chassis_move_update);		//�������ݸ���
extern void chassis_mode_set(chassis_move_t *chassis_move_mode);						//����ģʽѡ��ͨ��ң�����Ŀ���
extern void chassis_control_loop(chassis_move_t *chassis_move_control_loop);//����PID�����Լ��˶��ֽ�		
extern void chassis_UpdateMotorAngleSum(Chassis_Motor_t *chassis_Motor);		//����ֵ�Ӻͺ���
float RAMP_float( float final, float now, float ramp );//ÿ������ramp��һ���Ƕ�ֵ
float Angle_Limit (float angle ,float max);
void sigang_Init(chassis_move_t *chassis_move_control_loop);
void chassis_wheel_angle(float vx_set, float vy_set, float wz_set, float wheel_angle[4]);
void chassis_wheel_speed(float vx_set, float vy_set, float wz_set, float wheel_speed[4]);

#endif

