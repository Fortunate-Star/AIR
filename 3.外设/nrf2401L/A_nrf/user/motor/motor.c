#include "motor.h"

 void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
	//�����ٶȻ�pidֵ
		const fp32 motor_speed_fast_pid[3] = {G3508_SPEED_FAST_PID_KP,G3508_SPEED_FAST_PID_KI,	G3508_SPEED_FAST_PID_KD};
		const fp32 motor_angle_fast_pid[3] = {G3508_POSITION_FAST_PID_KP,G3508_POSITION_FAST_PID_KI,	G3508_POSITION_FAST_PID_KD};
		const fp32 motor_speed_slow_pid[3] = {G3508_SPEED_SLOW_PID_KP,G3508_SPEED_SLOW_PID_KI,	G3508_SPEED_SLOW_PID_KD};
		const fp32 motor_angle_slow_pid[3] = {G3508_POSITION_SLOW_PID_KP,G3508_POSITION_SLOW_PID_KI,	G3508_POSITION_SLOW_PID_KD};

		
		const fp32 motor_2006_speed_pid[3] = {G2006_SPEED_PID_KP, G2006_SPEED_PID_KI, G2006_SPEED_PID_KD};
		const fp32 motor_2006_angle_pid[3] = {G2006_POSITION_PID_KP, G2006_POSITION_PID_KI, G2006_POSITION_PID_KD};
		
		const fp32 motor_6020_speed_pid[3] = {G6020_SPEED_PID_KP, G6020_SPEED_PID_KI, G6020_SPEED_PID_KD};
		const fp32 motor_6020_angle_pid[3] = {G6020_POSITION_PID_KP, G6020_POSITION_PID_KI, G6020_POSITION_PID_KD};
	
//		PID_Init(&(chassis_move_init->motor_2006.motor_speed_pid), PID_POSITION, motor_2006_speed_pid, G2006_SPEED_PID_MAX_OUT, G2006_SPEED_PID_MAX_IOUT);
//		PID_Init(&(chassis_move_init->motor_2006.motor_angle_pid), PID_POSITION, motor_2006_angle_pid, G2006_POSITION_PID_MAX_OUT, G2006_POSITION_PID_MAX_IOUT);
																				
																														/*6020��ʼ��*/
//		chassis_move_init->motor_6020.chassis_motor_measure = get_6020_motor_measure_point();
//		PID_Init(&(chassis_move_init->motor_6020.motor_speed_pid), PID_POSITION, motor_6020_speed_pid, G6020_SPEED_PID_MAX_OUT, G6020_SPEED_PID_MAX_IOUT);
//		PID_Init(&(chassis_move_init->motor_6020.motor_angle_pid), PID_POSITION, motor_6020_angle_pid, G6020_POSITION_PID_MAX_OUT, G6020_POSITION_PID_MAX_IOUT);
//////	

																													/*3508��ʼ��*/
		uint8_t i;
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        PID_Init(&chassis_move_init->motor_chassis[i].motor_speed_pid, PID_POSITION, motor_speed_slow_pid, G3508_SPEED_PID_MAX_OUT, G3508_SPEED_PID_MAX_IOUT);
				PID_Init(&chassis_move_init->motor_chassis[i].motor_angle_pid, PID_POSITION, motor_angle_slow_pid, G3508_POSITION_PID_MAX_OUT, G3508_POSITION_PID_MAX_IOUT);
    }
		
    //��һ���˲�����б����������
//    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
//    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);


    //����һ������
    chassis_feedback_update(chassis_move_init);
		
}


 void chassis_feedback_update(chassis_move_t *chassis_move_update)//���µ������ݷ���
{
    if (chassis_move_update == NULL)
    {
        return;
    }
		
																									/*3508���ݸ���*/
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //���µ���ٶ�
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN* chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
			chassis_move_update->motor_chassis[i].motor_angle = chassis_move_update->motor_chassis[i].chassis_motor_measure->ecd;
					chassis_UpdateMotorAngleSum(&(chassis_move_update->motor_chassis[i]));
		}
																									 /*6020���ݸ���*/
//			chassis_move_update->motor_6020.speed = chassis_move_update->motor_6020.chassis_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
//			chassis_move_update->motor_6020.motor_angle = chassis_move_update->motor_6020.chassis_motor_measure->ecd;

		
		
}

//void chassis_mode_set(chassis_move_t *chassis_move_mode)
//	
//{
//    if (chassis_move_mode == NULL)
//    {
//        return;
//    }
//		static int init_time;
//		if(chassis_move_mode->chassis_mode == CHASSIS_NO_MOVE)
//		{
//			init_time++;
//			if(init_time<1000) return;
//			else init_time=0;
//		}
////		chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE ;     //���Կ���
//		chassis_move_mode->chassis_mode = CHASSIS_MOVE;	//�ֶ�ģʽ
//}



                                             /*����PID�����Լ��˶��ֽ�*/
int16_t angle_turn=0;
float speed_turn=0;
int16_t angle_out_6020=0;
int16_t angle_out_3508=0;
int16_t angle_6020goal=0;
 void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{

	
	
//																							/*6020����*/
//	chassis_move_control_loop->motor_6020.motor_angle_set=RAMP_float(angle_turn,
//	chassis_move_control_loop->motor_6020.motor_angle_set, 300);
//	
//	angle_out_6020 = PID_Calc_Ecd(&chassis_move_control_loop->motor_6020.motor_angle_pid,
//		chassis_move_control_loop->motor_6020.motor_angle,
//		chassis_move_control_loop->motor_6020.motor_angle_set,8192);
//	
//	chassis_move_control_loop->motor_6020.give_current =
//		PID_Calc(&chassis_move_control_loop->motor_6020.motor_speed_pid,
//		chassis_move_control_loop->motor_6020.speed, angle_out_6020);
	
	
																					/*3508����*/
	for(int i=0; i<4; i++)
	{
	
//	chassis_move_control_loop->motor_chassis[i].motor_angle_set=
//	RAMP_float(angle_turn,
//	chassis_move_control_loop->motor_chassis[i].motor_angle_set, 3);
	
	
//	angle_out_3508=PID_Calc(&chassis_move_control_loop->motor_chassis[i].motor_angle_pid,
//	chassis_move_control_loop->motor_chassis[i].motor_angle,
//	chassis_move_control_loop->motor_chassis[i].motor_angle_set);
	
	chassis_move_control_loop->motor_chassis[i].give_current = 
	PID_Calc(&chassis_move_control_loop->motor_chassis[i].motor_speed_pid,
	chassis_move_control_loop->motor_chassis[i].speed, speed_turn );
	}
}


void chassis_UpdateMotorAngleSum(Chassis_Motor_t *chassis_Motor)//����ֵ�Ӻͺ���
{		 
		//�ٽ�ֵ�жϷ�
		if (abs(chassis_Motor->chassis_motor_measure->ecd - chassis_Motor->chassis_motor_measure->last_ecd) > 4095)//ת����Ȧ
		{		
				//���β����Ƕ�С���ϴβ����Ƕ��ҹ��˰�Ȧ,��˵�����ι������
				if (chassis_Motor->chassis_motor_measure->ecd < chassis_Motor->chassis_motor_measure->last_ecd)//����Ȧ�ҹ����
				{
						//�Ѿ�ת����һȦ,���ۼ�ת�� 8191(һȦ) - �ϴ� + ����
						chassis_Motor->motor_angle += 8191 -  chassis_Motor->chassis_motor_measure->last_ecd + chassis_Motor->chassis_motor_measure->ecd;
				}
				else
				{
						//������һȦ
						chassis_Motor->motor_angle -= 8191 - chassis_Motor->chassis_motor_measure->ecd + chassis_Motor->chassis_motor_measure->last_ecd;
				}
		}
		else      
		{
				//δ���ٽ�ֵ,�ۼ���ת���ĽǶȲ�
				chassis_Motor->motor_angle += chassis_Motor->chassis_motor_measure->ecd  -  chassis_Motor->chassis_motor_measure->last_ecd;
		}
		
}

									//Ŀ��ֵ				��ǰֵ			ת���Ƕ�
float RAMP_float( float final, float now, float ramp )//ÿ������ramp��һ���Ƕ�ֵ
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

