#include "chassis.h"
#include "lib.h"

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
#define fabs(x)     (((x)>0)?(x):-(x))	
		
		//3////4//
		//2////1//
PidTypeDef yaw_pid;
uint16_t speed_now=0;
chassis_move_t chassis_move;//�����˶�����
/***********************************  ���̳�ʼ������  ************************************/
void chassis_init(chassis_move_t *chassis_move_init);//���̳�ʼ������Ҫ��pid��ʼ��
void chassis_feedback_update(chassis_move_t *chassis_move_update);//�������ݸ���
void chassis_mode_set(chassis_move_t *chassis_move_mode);//����ģʽѡ��ͨ��ң�����Ŀ���
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);//����PID�����Լ��˶��ֽ�		
void chassis_UpdateMotorAngleSum(Chassis_Motor_t *chassis_Motor);

void chassic_task(void *pvParameters)
{
    
      
		chassis_init(&chassis_move);
    while (1)
    {
			chassis_mode_set(&chassis_move);
			chassis_feedback_update(&chassis_move);//�������ݸ���
//			chassis_set_contorl(&chassis_move);
			chassis_control_loop(&chassis_move);//���̿���PID����
				
			CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
												chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
			
			
		}
		
		
}
 void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
	//�����ٶȻ�pidֵ
		const fp32 motor_speed_pid[3] = {G3508_SPEED_PID_KP,G3508_SPEED_PID_KI,	G3508_SPEED_PID_KD};
		const fp32 motor_angle_pid[3] = {G3508_POSITION_PID_KP,G3508_POSITION_PID_KI,	G3508_POSITION_PID_KD};
		
//		const fp32 motor_2006_speed_pid[3] = {G2006_SPEED_PID_KP, G2006_SPEED_PID_KI, G2006_SPEED_PID_KD};
//		const fp32 motor_2006_angle_pid[3] = {G2006_POSITION_PID_KP, G2006_POSITION_PID_KI, G2006_POSITION_PID_KD};
//		const fp32 motor_6020_speed_pid[3] = {G6020_SPEED_PID_KP, G6020_SPEED_PID_KI, G6020_SPEED_PID_KD};
//		const fp32 motor_6020_angle_pid[3] = {G6020_POSITION_PID_KP, G6020_POSITION_PID_KI, G6020_POSITION_PID_KD};

//		const fp32 chassis_yaw_pid[3] = {CHASSIS_ANGLE_PID_KP, CHASSIS_ANGLE_PID_KI, CHASSIS_ANGLE_PID_KD};

//		const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
//		const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
//		
		uint8_t i;

//		chassis_move_init->motor_2006.chassis_motor_measure = get_trigger_motor_measure_point();
//		chassis_move_init->motor_6020.chassis_motor_measure = get_yaw_gimbal_motor_measure_point();
//		PID_Init(&(chassis_move_init->motor_2006.motor_speed_pid), PID_POSITION, motor_2006_speed_pid, G2006_SPEED_PID_MAX_OUT, G2006_SPEED_PID_MAX_IOUT);
//		PID_Init(&(chassis_move_init->motor_2006.motor_angle_pid), PID_POSITION, motor_2006_angle_pid, G2006_POSITION_PID_MAX_OUT, G2006_POSITION_PID_MAX_IOUT);
//	
//		PID_Init(&(chassis_move_init->motor_6020.motor_speed_pid), PID_POSITION, motor_6020_speed_pid, G6020_SPEED_PID_MAX_OUT, G6020_SPEED_PID_MAX_IOUT);
//		PID_Init(&(chassis_move_init->motor_6020.motor_angle_pid), PID_POSITION, motor_6020_angle_pid, G6020_POSITION_PID_MAX_OUT, G6020_POSITION_PID_MAX_IOUT);
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        PID_Init(&chassis_move_init->motor_chassis[i].motor_speed_pid, PID_POSITION, motor_speed_pid, G3508_SPEED_PID_MAX_OUT, G3508_SPEED_PID_MAX_IOUT);
				PID_Init(&chassis_move_init->motor_chassis[i].motor_angle_pid, PID_POSITION, motor_angle_pid, G3508_POSITION_PID_MAX_OUT, G3508_POSITION_PID_MAX_IOUT);
    }
		
//		PID_Init(&chassis_move_init->yaw_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
//		
//	
//		chassis_move_init->motor_2006.motor_angle_set=0;
//		
//		chassis_move_init->motor_6020.motor_angle_set=0;
//    //��һ���˲�����б����������
//    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
//    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

//    //��� ��С�ٶ�
//    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
//    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

//    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
//    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //����һ������
    chassis_feedback_update(chassis_move_init);
		

//		chassis_move_init->chassis_yaw_set = chassis_move_init->chassis_yaw;
}
int eror_2006=6770;//6730
int eror_6020=6150;
void chassis_feedback_update(chassis_move_t *chassis_move_update)//���µ������ݷ���
{
    if (chassis_move_update == NULL)
    {
        return;
    }
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //���µ���ٶ�
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
				chassis_UpdateMotorAngleSum(&(chassis_move_update->motor_chassis[i]));
		}
		
		{chassis_move_update->motor_2006.speed  = chassis_move_update->motor_2006.chassis_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
			if(chassis_move_update->motor_2006.chassis_motor_measure->ecd>=eror_2006&&chassis_move_update->motor_2006.chassis_motor_measure->ecd<8192)	
			chassis_move_update->motor_2006.motor_angle  = chassis_move_update->motor_2006.chassis_motor_measure->ecd -eror_2006;
			else 
			chassis_move_update->motor_2006.motor_angle  = 8191-eror_2006+chassis_move_update->motor_2006.chassis_motor_measure->ecd ;
		}
		{	chassis_move_update->motor_6020.speed  = chassis_move_update->motor_6020.chassis_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
//			if(chassis_move_update->motor_6020.chassis_motor_measure->ecd>=eror_6020&&chassis_move_update->motor_6020.chassis_motor_measure->ecd<8192)	
//			chassis_move_update->motor_6020.motor_angle  = chassis_move_update->motor_6020.chassis_motor_measure->ecd -eror_6020;
//			else 
//			chassis_move_update->motor_6020.motor_angle  = 8191-eror_6020+chassis_move_update->motor_6020.chassis_motor_measure->ecd ;
//			chassis_move_update->motor_6020.motor_angle  = chassis_move_update->motor_6020.chassis_motor_measure->ecd;
		}
		chassis_UpdateMotorAngleSum(&chassis_move_update->motor_6020);
		//���µ���ǰ���ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
		
		chassis_move_update->chassis_yaw = 0;
    chassis_move_update->chassis_pitch = 0;
    chassis_move_update->chassis_roll = 0;
		
		
}
void chassis_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
		static int init_time;
		if(chassis_move_mode->chassis_mode == CHASSIS_NO_MOVE)
		{
			init_time++;
			if(init_time<1000) return;
			else init_time=0;
		}
//		chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE ;     //���Կ���
		chassis_move_mode->chassis_mode = CHASSIS_MOVE;	//�ֶ�ģʽ
}
fp32 Vx=0.3f,Vy=0.3f,Vz=3.0f;

//float speed=10;
//static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
//{
//	wheel_speed[0] = (	vx_set   + ( CHASSIS_WZ_SET_SCALE - 1.0f)  * wz_set);
//	wheel_speed[1] = ( -vx_set   + ( CHASSIS_WZ_SET_SCALE - 1.0f)  * wz_set);
//	wheel_speed[2] = ( -vx_set  + ( CHASSIS_WZ_SET_SCALE - 1.0f)  * wz_set);
//	wheel_speed[3] = (	vx_set  + ( CHASSIS_WZ_SET_SCALE - 1.0f)  * wz_set);
//}
int speed=0,Z=0;
fp32 Z_fbs;
long int dant=50000;
int angle_turn,angle_mode=0;
long int motor1_angle_turn=-8000,motor2_angle_turn=6000;
int step_T=0,step_TT=0;
int mode_BMQ=0;
int XJ_STEP=0;
float limit(float x,float min,float max)
{
	x = (x>max) ? max : ((x<min) ? min : x);
	return x;
}
void chassis_control_loop(chassis_move_t *chassis_move_control_loop)//����PID�����Լ��˶��ֽ�
{
		static fp32 angle_out;
		fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
		angle_out=PID_Calc(&chassis_move_control_loop->motor_chassis[0].motor_angle_pid, chassis_move_control_loop->motor_chassis[0].motor_angle, chassis_move_control_loop->motor_chassis[0].motor_angle_set);
		chassis_move_control_loop->motor_chassis[0].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[0].motor_speed_pid, chassis_move_control_loop->motor_chassis[0].speed, angle_out);
	
		
		
		
		
		
//		if(XJ_STEP==1)
//		{
//				
//		
//		}
//		else if(XJ_STEP==2)
//		{

//		}
//		else Z=0;
//		chassis_move_control_loop->vx_set+=speed;
//		chassis_move_control_loop->wz_set+=Z;

//				
//    
//		
//		chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
//																			chassis_move_control_loop->vy_set,
//																			chassis_move_control_loop->wz_set, 
//																			wheel_speed);

//		if(fabs(angle_turn-chassis_move_control_loop->motor_6020.motor_angle)<300||fabs(angle_turn-chassis_move_control_loop->motor_6020.motor_angle)>7800)
//		angle_mode=1;
//		else angle_mode=0;
//		 
//		if(step_T==1)
//		{
//			//400,8000
//			step_TT=1;
//			chassis_move_control_loop->motor_chassis[3].motor_angle=0;
//			chassis_move_control_loop->motor_chassis[3].motor_angle_set=0;
//			chassis_move_control_loop->motor_chassis[2].motor_angle=0;
//			chassis_move_control_loop->motor_chassis[2].motor_angle_set=0;
//			
//			step_T=2;mode_BMQ=1;
//		}
//    else if(step_T==2)	
//		{	
//		  chassis_move_control_loop->motor_2006.motor_angle_set=400;
//			
//			if(	chassis_move_control_loop->motor_chassis[2].motor_angle-motor2_angle_turn<=0)
//		     chassis_move_control_loop->motor_chassis[2].motor_angle_set+=4;

//			else if((chassis_move_control_loop->motor_chassis[2].motor_angle-motor2_angle_turn>=100))

//			  step_T=3;
//		}
//	  else if(step_T==3)	
//		{	
//		  chassis_move_control_loop->motor_2006.motor_angle_set=0;
//			step_TT=0;
//		  speed=10;
//		}
//		 else if(step_T==4)	
//		{	
//		  chassis_move_control_loop->motor_2006.motor_angle_set=0;
//			speed=0;
//		}
		
//			chassis_move_control_loop->motor_6020.motor_angle_set=angle_turn;
		
		
//			chassis_move_control_loop->motor_6020.motor_angle_set=RAMP_float(angle_turn,chassis_move_control_loop->motor_6020.motor_angle_set, 3);
//		
//			angle_out=PID_Calc_Ecd(&chassis_move_control_loop->motor_2006.motor_angle_pid,chassis_move_control_loop->motor_2006.motor_angle,chassis_move_control_loop->motor_2006.motor_angle_set,8192);
//			chassis_move_control_loop->motor_2006.give_current = PID_Calc(&chassis_move_control_loop->motor_2006.motor_speed_pid, chassis_move_control_loop->motor_2006.speed, angle_out);
//			angle_out=PID_Calc_Ecd(&chassis_move_control_loop->motor_6020.motor_angle_pid,chassis_move_control_loop->motor_6020.motor_angle,chassis_move_control_loop->motor_6020.motor_angle_set,8192);
//			angle_out=PID_Calc(&chassis_move_control_loop->motor_6020.motor_angle_pid, chassis_move_control_loop->motor_6020.motor_angle, chassis_move_control_loop->motor_6020.motor_angle_set);
//			angle_out=PID_Calc_Ecd(&chassis_move_control_loop->motor_6020.motor_angle_pid,chassis_move_control_loop->motor_6020.motor_angle,chassis_move_control_loop->motor_6020.motor_angle_set,8192);
//			chassis_move_control_loop->motor_6020.give_current = PID_Calc(&chassis_move_control_loop->motor_6020.motor_speed_pid, chassis_move_control_loop->motor_6020.speed, angle_out);

//		 if(step_TT==0)
//		 {
//			chassis_move_control_loop->motor_chassis[0].motor_angle_set=wheel_speed[0];
//			chassis_move_control_loop->motor_chassis[1].motor_angle_set=wheel_speed[1];
//			chassis_move_control_loop->motor_chassis[2].motor_angle_set=wheel_speed[2];
//			chassis_move_control_loop->motor_chassis[3].motor_angle_set=wheel_speed[3];
//		 }
//		if(mode_BMQ==0)
//		{
//			angle_out=PID_Calc(&chassis_move_control_loop->motor_chassis[0].motor_angle_pid, chassis_move_control_loop->motor_chassis[0].motor_angle, chassis_move_control_loop->motor_chassis[0].motor_angle_set);
//			chassis_move_control_loop->motor_chassis[0].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[0].motor_speed_pid, chassis_move_control_loop->motor_chassis[0].speed, angle_out);
//			angle_out=PID_Calc(&chassis_move_control_loop->motor_chassis[3].motor_angle_pid, chassis_move_control_loop->motor_chassis[3].motor_angle, chassis_move_control_loop->motor_chassis[3].motor_angle_set);
//			chassis_move_control_loop->motor_chassis[3].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[3].motor_speed_pid, chassis_move_control_loop->motor_chassis[3].speed, angle_out);
//			angle_out=PID_Calc(&chassis_move_control_loop->motor_chassis[1].motor_angle_pid, chassis_move_control_loop->motor_chassis[1].motor_angle, chassis_move_control_loop->motor_chassis[1].motor_angle_set);
//			chassis_move_control_loop->motor_chassis[1].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[1].motor_speed_pid, chassis_move_control_loop->motor_chassis[1].speed, angle_out);
//			angle_out=PID_Calc(&chassis_move_control_loop->motor_chassis[2].motor_angle_pid, chassis_move_control_loop->motor_chassis[2].motor_angle, chassis_move_control_loop->motor_chassis[2].motor_angle_set);
//			chassis_move_control_loop->motor_chassis[2].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[2].motor_speed_pid, chassis_move_control_loop->motor_chassis[2].speed, angle_out);
//		}	
//		else if(mode_BMQ==1)//2�ű�����
//		{
//			angle_out=PID_Calc(&chassis_move_control_loop->motor_chassis[2].motor_angle_pid, chassis_move_control_loop->motor_chassis[2].motor_angle, chassis_move_control_loop->motor_chassis[2].motor_angle_set);
//			chassis_move_control_loop->motor_chassis[0].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[0].motor_speed_pid, chassis_move_control_loop->motor_chassis[0].speed, -angle_out);
//			chassis_move_control_loop->motor_chassis[3].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[3].motor_speed_pid, chassis_move_control_loop->motor_chassis[3].speed, -angle_out);
//			chassis_move_control_loop->motor_chassis[1].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[1].motor_speed_pid, chassis_move_control_loop->motor_chassis[1].speed, angle_out);
//			chassis_move_control_loop->motor_chassis[2].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[2].motor_speed_pid, chassis_move_control_loop->motor_chassis[2].speed, angle_out);
//		}
//			else if(mode_BMQ==2)//3�ű�����
//		{
//			angle_out=PID_Calc(&chassis_move_control_loop->motor_chassis[3].motor_angle_pid, chassis_move_control_loop->motor_chassis[3].motor_angle, chassis_move_control_loop->motor_chassis[3].motor_angle_set);
//			chassis_move_control_loop->motor_chassis[0].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[0].motor_speed_pid, chassis_move_control_loop->motor_chassis[0].speed, angle_out);
//			chassis_move_control_loop->motor_chassis[3].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[3].motor_speed_pid, chassis_move_control_loop->motor_chassis[3].speed, angle_out);
//			chassis_move_control_loop->motor_chassis[1].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[1].motor_speed_pid, chassis_move_control_loop->motor_chassis[1].speed, -angle_out);
//			chassis_move_control_loop->motor_chassis[2].give_current=PID_Calc(&chassis_move_control_loop->motor_chassis[2].motor_speed_pid, chassis_move_control_loop->motor_chassis[2].speed, -angle_out);
//		
//		}
		
}
fp32 V_X,V_Y,V_Z;
static void chassis_UpdateMotorAngleSum(Chassis_Motor_t *chassis_Motor)//����ֵ�Ӻͺ���
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
float RAMP_float( float final, float now, float ramp )
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
