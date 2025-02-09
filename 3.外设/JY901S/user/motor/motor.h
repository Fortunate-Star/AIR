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


#define G3508_SPEED_FAST_PID_KP 400.0f			//3508不带减速器
#define G3508_SPEED_FAST_PID_KI 0.0f
#define G3508_SPEED_FAST_PID_KD 0.0f

#define G3508_POSITION_FAST_PID_KP 0.0001f 
#define G3508_POSITION_FAST_PID_KI 0.0f
#define G3508_POSITION_FAST_PID_KD 0.01f

#define G3508_SPEED_SLOW_PID_KP 2000.0f				//10000									//3508带减速器
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

#define CHASSIS_CONTROL_TIME 0.002//底盘任务控制间隔
//底盘电机最大速度
#define MAX_WHEEL_SPEED 2.0f
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 3.00f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.00f
#define CHASSIS_WZ_SET_SCALE 0.0f
//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define CHASSIS_WZ_RC_SEN 0.005f     //不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例

#define CHASSIS_VX_RC_SEN 0.0065f  //遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.0065f  //遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
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
	CHASSIS_ZERO_FORCE,                  //初始化模式
	CHASSIS_NO_MOVE,                     //底盘保持不动                      
	CHASSIS_MOVE,				  								//前进后退
	CHASSIS_OPSFOLLOW,  			  //ops循迹		
	CHASSIC_SLOW_MOVE,         //缓慢移动
	CHASSIC_ABS_MOVE,         //缓慢移动
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
	Chassis_Motor_t motor_chassis[8];          //底盘电机数据
	Chassis_Motor_t motor_2006;
	Chassis_Motor_t motor_6020[2];



	fp32 vx;                         //底盘速度 前进方向 前为正，单位 m/s
	fp32 vy;                         //底盘速度 左右方向 左为正  单位 m/s
	fp32 wz;                         //底盘旋转角速度，逆时针为正 单位 rad/s

	fp32 vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
	fp32 vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
	fp32 wz_set;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s

	
	fp32 chassis_yaw_set;

	fp32 vx_max_speed;  //前进方向最大速度 单位m/s
	fp32 vx_min_speed;  //前进方向最小速度 单位m/s
	fp32 vy_max_speed;  //左右方向最大速度 单位m/s
	fp32 vy_min_speed;  //左右方向最小速度 单位m/s
	
	fp32 chassis_yaw;   //陀螺仪的yaw角度
  fp32 chassis_pitch; //陀螺仪的pitch角度
  fp32 chassis_roll;  //陀螺仪的roll角度


	const fp32 * chassis_w_offset;
} chassis_move_t;




extern void chassis_init(chassis_move_t *chassis_move_init);								//底盘初始化，主要是pid初始化
extern void chassis_feedback_update(chassis_move_t *chassis_move_update);		//底盘数据更新
extern void chassis_mode_set(chassis_move_t *chassis_move_mode);						//底盘模式选择，通过遥控器的开关
extern void chassis_control_loop(chassis_move_t *chassis_move_control_loop);//底盘PID计算以及运动分解		
extern void chassis_UpdateMotorAngleSum(Chassis_Motor_t *chassis_Motor);		//编码值加和函数
float RAMP_float( float final, float now, float ramp );//每次增加ramp的一个角度值
void go_head_back(float speed);
void go_left_right(float speed);
void sigang_Init(chassis_move_t *chassis_move_control_loop);


#endif

