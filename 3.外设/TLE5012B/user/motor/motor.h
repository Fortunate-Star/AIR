#ifndef __MOTOR_H
#define __MOTOR_H

#include "CAN_receive.h"
#include "pid.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "lib.h"
#define abs(x) ((x)>0? (x):(-(x)))

#define M3508_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f  


#define G3508_SPEED_FAST_PID_KP 400.0f			//3508����������
#define G3508_SPEED_FAST_PID_KI 0.0f
#define G3508_SPEED_FAST_PID_KD 0.0f

#define G3508_POSITION_FAST_PID_KP 0.0001f 
#define G3508_POSITION_FAST_PID_KI 0.0f
#define G3508_POSITION_FAST_PID_KD 0.01f

#define G3508_SPEED_SLOW_PID_KP 2000.0f				//10000									//3508��������
#define G3508_SPEED_SLOW_PID_KI 0.0f
#define G3508_SPEED_SLOW_PID_KD 0.0f
#define G3508_SPEED_PID_MAX_OUT 10000.0f
#define G3508_SPEED_PID_MAX_IOUT 2000.0f

#define G3508_POSITION_SLOW_PID_KP 0.005f 			//0.005
#define G3508_POSITION_SLOW_PID_KI 0.0f
#define G3508_POSITION_SLOW_PID_KD 0.01f
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

#define CHASSIS_ANGLE_PID_KP   2.0f   //10
#define CHASSIS_ANGLE_PID_KI   0.0f    //0
#define CHASSIS_ANGLE_PID_KD  1.0f    //0
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  9  //6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.1f

#define CHASSIS_CONTROL_TIME 0.002//����������Ƽ��
//���̵������ٶ�
#define MAX_WHEEL_SPEED 2.0f
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 3.00f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.00f
#define CHASSIS_WZ_SET_SCALE 0.0f
//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define CHASSIS_WZ_RC_SEN 0.005f     //��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���

#define CHASSIS_VX_RC_SEN 0.0065f  //ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.0065f  //ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.2f
#define MOTOR_DISTANCE_TO_CENTER 0.08f

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
void go_head_back(float speed);
void go_left_right(float speed);
void sigang_Init(chassis_move_t *chassis_move_control_loop);


#endif

