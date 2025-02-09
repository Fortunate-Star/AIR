#ifndef __LINGZU_H
#define __LINGZU_H
#include "main.h"
#include "can.h"
//���Ʋ�����ֵ����������
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -44.0f
#define V_MAX 44.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -17.0f
#define T_MAX 17.0f
#define MAX_P 720
#define MIN_P -720
//����CANID����
#define Master_CAN_ID 0x00                      //����ID     
//��������궨��
#define Communication_Type_GetID 0x00           //��ȡ�豸��ID��64λMCUΨһ��ʶ��
#define Communication_Type_MotionControl 0x01 	//�������������Ϳ���ָ��
#define Communication_Type_MotorRequest 0x02	//���������������������״̬
#define Communication_Type_MotorEnable 0x03	    //���ʹ������
#define Communication_Type_MotorStop 0x04	    //���ֹͣ����
#define Communication_Type_SetPosZero 0x06	    //���õ����е��λ
#define Communication_Type_CanID 0x07	        //���ĵ�ǰ���CAN_ID
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11	//��ȡ��������
#define Communication_Type_SetSingleParameter 0x12	//�趨��������
#define Communication_Type_ErrorFeedback 0x15	    //���Ϸ���֡
//������ȡ�궨��
#define Run_mode 0x7005	
#define Iq_Ref   0x7006
#define Spd_Ref  0x700A
#define Limit_Torque 0x700B
#define Cur_Kp 0x7010
#define Cur_Ki 0x7011
#define Cur_Filt_Gain 0x7014
#define Loc_Ref 0x7016
#define Limit_Spd 0x7017
#define Limit_Cur 0x7018

#define Gain_Angle 720/32767.0
#define Bias_Angle 0x8000
#define Gain_Speed 30/32767.0
#define Bias_Speed 0x8000
#define Gain_Torque 12/32767.0
#define Bias_Torque 0x8000
#define Temp_Gain   0.1

#define Motor_Error 0x00
#define Motor_OK 0X01

enum CONTROL_MODE   //����ģʽ����
{
    Motion_mode = 0,//�˿�ģʽ  
    Position_mode,  //λ��ģʽ
    Speed_mode,     //�ٶ�ģʽ  
    Current_mode    //����ģʽ
};
enum ERROR_TAG      //����ش�����
{
    OK                 = 0,//�޹���
    BAT_LOW_ERR        = 1,//Ƿѹ����
    OVER_CURRENT_ERR   = 2,//����
    OVER_TEMP_ERR      = 3,//����
    MAGNETIC_ERR       = 4,//�ű������
    HALL_ERR_ERR       = 5,//HALL�������
    NO_CALIBRATION_ERR = 6//δ�궨
};

typedef struct{           //С�׵���ṹ��
		uint8_t CAN_ID;       //CAN ID
    uint8_t MCU_ID;       //MCUΨһ��ʶ��[��8λ����64λ]
	float Angle;          //�ش��Ƕ�
	float Speed;          //�ش��ٶ�
	float Torque;         //�ش�����
	float Temp;			  //�ش��¶�
	
	uint16_t set_current;
	uint16_t set_speed;
	uint16_t set_position;
	
	uint8_t error_code;
	
	float Angle_Bias;
	
}LINGZU_Motor;
extern LINGZU_Motor mi_motor[4];//Ԥ�ȶ����ĸ�С�׵��

extern void chack_cybergear(uint8_t ID);
extern void start_cybergear(LINGZU_Motor *Motor);
extern void stop_cybergear(LINGZU_Motor *Motor, uint8_t clear_error);
extern void set_mode_cybergear(LINGZU_Motor *Motor, uint8_t Mode);
extern void set_current_cybergear(LINGZU_Motor *Motor, float Current);
extern void set_zeropos_cybergear(LINGZU_Motor *Motor);
extern void set_CANID_cybergear(LINGZU_Motor *Motor, uint8_t CAN_ID);
extern void init_cybergear(LINGZU_Motor *Motor, uint8_t Can_Id, uint8_t mode);
extern void motor_controlmode(LINGZU_Motor *Motor,float torque, float MechPosition, float speed, float kp, float kd);

																							/*����ڲ���ʽת������*/
static void Set_Motor_Parameter(LINGZU_Motor *Motor,uint16_t Index,float Value,char Value_type);//д��������
static uint32_t Get_Motor_ID(uint32_t CAN_ID_Frame);//��ȡ����ظ�֡��չID�еĵ��CANID
static void Motor_Data_Handler(LINGZU_Motor *Motor,uint8_t DataFrame[8],uint32_t IDFrame);//����ظ�֡���ݴ�����
static uint8_t* Float_to_Byte(float f);// ������ת4�ֽں���
static float uint16_to_float(uint16_t x,float x_min,float x_max,int bits);//С�׵������16λ����ת����
static int float_to_uint(float x, float x_min, float x_max, int bits);//С�׵�����͸���ת16λ����
#endif
