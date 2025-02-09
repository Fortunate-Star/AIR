#include "motor.h"
int16_t angle_out_3508=0;
float angle_turn[4]={0};
fp32 chassis_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 chassis_angle_set[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 chassis_speed_set[4] = {0.0f, 0.0f, 0.0f, 0.0f};
 void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
	//底盘速度环pid值
		const fp32 motor_speed_pid[3] = {G3508_SPEED_PID_KP,G3508_SPEED_PID_KI,	G3508_SPEED_PID_KD};
		const fp32 motor_speed_DL_pid[3] = {G3508_SPEED_DL_PID_KP,G3508_SPEED_DL_PID_KI,	G3508_SPEED_DL_PID_KD};
		const fp32 motor_angle_DL_pid[3] = {G3508_POSITION_DL_PID_KP,G3508_POSITION_DL_PID_KI,	G3508_POSITION_DL_PID_KD};
	
		
//		const fp32 motor_2006_speed_pid[3] = {G2006_SPEED_PID_KP, G2006_SPEED_PID_KI, G2006_SPEED_PID_KD};
//		const fp32 motor_2006_angle_pid[3] = {G2006_POSITION_PID_KP, G2006_POSITION_PID_KI, G2006_POSITION_PID_KD};
//		
//		const fp32 motor_6020_speed_pid[3] = {G6020_SPEED_PID_KP, G6020_SPEED_PID_KI, G6020_SPEED_PID_KD};
//		const fp32 motor_6020_angle_pid[3] = {G6020_POSITION_PID_KP, G6020_POSITION_PID_KI, G6020_POSITION_PID_KD};
																					
																													/*3508初始化*/
//舵电机初始化
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis2[i].chassis_motor_measure = get_Chassis_Motor2_Measure_Point(i);
        PID_Init(&chassis_move_init->motor_chassis2[i].motor_speed_pid, PID_POSITION, motor_speed_DL_pid, G3508_SPEED_PID_MAX_OUT, G3508_SPEED_PID_MAX_IOUT);
				PID_Init(&chassis_move_init->motor_chassis2[i].motor_angle_pid, PID_POSITION, motor_angle_DL_pid, G3508_POSITION_PID_MAX_OUT, G3508_POSITION_PID_MAX_IOUT);
    }
		
//轮电机初始化
		 for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        PID_Init(&chassis_move_init->motor_chassis[i].motor_speed_pid, PID_POSITION, motor_speed_pid, G3508_SPEED_PID_MAX_OUT, G3508_SPEED_PID_MAX_IOUT);
    }
		
    //用一阶滤波代替斜波函数生成
//    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
//    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

//初始姿态初始化
void sigang_Init(chassis_move_t *chassis_move_control_loop)
{
	//			左前								左后								右前										右后
	angle_turn[0] = 5500; angle_turn[1] = 9900; angle_turn[2] = 1800; angle_turn[3] = 8200;
	chassis_feedback_update(chassis_move_control_loop);
	chassis_control_loop(chassis_move_control_loop);
	
}

 void chassis_feedback_update(chassis_move_t *chassis_move_update)//更新底盘数据反馈
{
    if (chassis_move_update == NULL)
    {
        return;
    }
		
																									/*3508数据更新*/
  
    for (int i = 0; i < 4; i++)
    {
        //更新舵电机速度
        chassis_move_update->motor_chassis2[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN* chassis_move_update->motor_chassis2[i].chassis_motor_measure->speed_rpm;
				chassis_angle[i] = ReadRegister(i); 
		}

		
		 for (int i = 0; i < 4; i++)
    {
        //更新轮电机速度
      chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN* chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
		}

		
}


#define WHEEL_PERIMETER 			 260.75f	//车轮周长 (轮子直径 * PI 再转换成mm)
#define M3508_RATIO 	 				 6				//电机减速比
#define W_ANGLE 							0.707107// 根号2 /2
int8_t dirt[4] = { 1, -1, 1, 1};//默认轮子转向
void chassis_wheel_speed(float vx_set, float vy_set, float wz_set, float wheel_speed[4])
{
	  fp32 wheel_rpm_ratio;
	
    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * M3508_RATIO * 1000;
	
		//目标速度的值
		wheel_speed[0] = dirt[0] * sqrt(	pow(vy_set - wz_set* W_ANGLE ,2)+	pow(vx_set + wz_set* W_ANGLE ,2)) * wheel_rpm_ratio ;
    wheel_speed[1] = dirt[1] * sqrt(	pow(vy_set + wz_set* W_ANGLE ,2)+	pow(vx_set + wz_set* W_ANGLE ,2)) * wheel_rpm_ratio	;
    wheel_speed[2] = dirt[2] * sqrt(	pow(vy_set + wz_set* W_ANGLE ,2)+	pow(vx_set - wz_set* W_ANGLE ,2))	* wheel_rpm_ratio;
    wheel_speed[3] = dirt[3] * sqrt(	pow(vy_set - wz_set* W_ANGLE ,2)+	pow(vx_set - wz_set* W_ANGLE ,2)) * wheel_rpm_ratio;
		
}

fp64 atan_angle[4];
float angle_turn_change[4]={14300,1600,9800,0};//车轮朝向前时编码器值

void chassis_wheel_angle(float vx_set, float vy_set, float wz_set, float wheel_angle[4])
{
		//目标角度计算
    if(!(vx_set == 0 && vy_set == 0 && wz_set == 0))//防止除数为零，atan2(0, 0)，是一个不确定的值，通常会抛出一个错误或返回无穷大等不合理的结果。
    {
			//由于atan2算出来的结果是弧度，需转换成角度 计算公式为 弧度 * 180.f / PI 最终得到角度值 (0.707107f == 根号2 /2)
      atan_angle[0] = atan2((vy_set - wz_set  * W_ANGLE),(vx_set + wz_set  * W_ANGLE)) * 180.0f / PI;		
      atan_angle[1] = atan2((vy_set - wz_set  * W_ANGLE),(vx_set - wz_set  * W_ANGLE)) * 180.0f / PI;
      atan_angle[2] = atan2((vy_set + wz_set  * W_ANGLE),(vx_set + wz_set  * W_ANGLE)) * 180.0f / PI;
      atan_angle[3] = atan2((vy_set + wz_set  * W_ANGLE),(vx_set - wz_set  * W_ANGLE)) * 180.0f / PI;	
    }  
		
		// 将一圈360°转换成编码值的一圈0-32767 -> 角度 * 32767 / 360 最终转换为需要转动的角度对应的编码值，再加上偏置角度,最终得到目标编码值
		//目标角度的编码值
		wheel_angle[0] = Angle_Limit(angle_turn_change[0] + (fp32)(atan_angle[0] * 91.02f), 32768.f);
		wheel_angle[1] = Angle_Limit(angle_turn_change[1]+ (fp32)(atan_angle[1] * 91.02f), 32768.f);
		wheel_angle[2] = Angle_Limit(angle_turn_change[2] + (fp32)(atan_angle[2] * 91.02f), 32768.f);
		wheel_angle[3] = Angle_Limit(angle_turn_change[3] + (fp32)(atan_angle[3] * 91.02f), 32768.f);
		
		//优弧 劣弧 驱动电机转向判断
	 if( abs( chassis_angle[0] - wheel_angle[0] ) > 8192 )
	 {	
			dirt[0] = 1;
			wheel_angle[0] = Angle_Limit( wheel_angle[0] - 16384, 32768 );
	 }
	 else
		 dirt[0] = -1;
		
   if( abs( chassis_angle[1] - wheel_angle[1] ) > 8192 )
	 {	
			dirt[1] = -1;
			wheel_angle[1] = Angle_Limit( wheel_angle[1] - 16384, 32768 );
	 }
	 else
		 dirt[1] = 1;

   if( abs( chassis_angle[2] - wheel_angle[2] ) > 8192 )
	 {	
			dirt[2] = -1;
			wheel_angle[2] = Angle_Limit( wheel_angle[2] - 16384, 32768 );
	 }
	 else
		 dirt[2] = 1;

   if( abs( chassis_angle[3] - wheel_angle[3] ) > 8192 )
	 {	
			dirt[3] = -1;
			wheel_angle[3] = Angle_Limit( wheel_angle[3] - 16384, 32768 );
	 }
	 else
		 dirt[3] = 1; 
}

float Angle_Limit (float angle ,float max)
{
		if(angle > max)
			angle -= max;
		if(angle < 0)
			angle += max; 
		return angle;
}

                                             /*底盘PID计算以及运动分解*/

float speed_turn[4]={0};
int per_angle_turn = 300;
 void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{		
	
	chassis_wheel_angle(chassis_move_control_loop->vx_set,chassis_move_control_loop->vy_set,chassis_move_control_loop->wz_set,chassis_angle_set);
	chassis_wheel_speed(chassis_move_control_loop->vx_set,chassis_move_control_loop->vy_set,chassis_move_control_loop->wz_set,chassis_speed_set);
		

	//舵电机控制
		for(int i=0; i<4; i++)
	{
//		chassis_move_control_loop->motor_chassis[i].motor_angle_set=
//		RAMP_float(angle_turn[i],chassis_move_control_loop->motor_chassis[i].motor_angle_set, per_angle_turn);
//		
		angle_out_3508 = -PID_Calc_Ecd(&chassis_move_control_loop->motor_chassis2[i].motor_angle_pid,
		chassis_angle[i],chassis_angle_set[i],32768);
		
		chassis_move_control_loop->motor_chassis2[i].give_current = 
		PID_Calc(&chassis_move_control_loop->motor_chassis2[i].motor_speed_pid,
		chassis_move_control_loop->motor_chassis2[i].speed, angle_out_3508);	
	}	
	
		//轮电机控制
		for(int i=0; i<4; i++)
	{
		Angle_Limit(chassis_speed_set[i],4);
		chassis_move_control_loop->motor_chassis[i].give_current = 
		PID_Calc(&chassis_move_control_loop->motor_chassis[i].motor_speed_pid,
		chassis_move_control_loop->motor_chassis[i].speed, chassis_speed_set[i]);	
	}	

}


void chassis_UpdateMotorAngleSum(Chassis_Motor_t *chassis_Motor)//编码值加和函数
{		 
		//临界值判断法
		if (abs(chassis_Motor->chassis_motor_measure->ecd - chassis_Motor->chassis_motor_measure->last_ecd) > 4095)//转过半圈
		{		
				//本次测量角度小于上次测量角度且过了半圈,则说明本次过了零点
				if (chassis_Motor->chassis_motor_measure->ecd < chassis_Motor->chassis_motor_measure->last_ecd)//过半圈且过零点
				{
						//已经转到下一圈,则累计转过 8191(一圈) - 上次 + 本次
						chassis_Motor->motor_angle += 8191 -  chassis_Motor->chassis_motor_measure->last_ecd + chassis_Motor->chassis_motor_measure->ecd;
				}
				else
				{
						//超过了一圈
						chassis_Motor->motor_angle -= 8191 - chassis_Motor->chassis_motor_measure->ecd + chassis_Motor->chassis_motor_measure->last_ecd;
				}
		}
		else      
		{
				//未过临界值,累加上转过的角度差
				chassis_Motor->motor_angle += chassis_Motor->chassis_motor_measure->ecd  -  chassis_Motor->chassis_motor_measure->last_ecd;
		}
		
}

									//目标值				当前值			转动角度
float RAMP_float( float final, float now, float ramp )//每次增加ramp的一个角度值
{
	  float buffer = 0;
	
	
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}


