#ifndef _LINGZU_DRIVER_H_
#define _LINGZU_DRIVER_H_
#include "main.h"
 
typedef enum {
    RunMode_idx = 0x7005,
    IqRef_idx = 0x7006,
    SpdRef_idx = 0x700A,
    LimitTorque_idx = 0x700B,
    CurKp_idx = 0x7010,
    CurKi_idx = 0x7011,
    CurFiltGain_idx = 0x7014,
    LocRef_idx = 0x7016,
    LimitSpd_idx = 0x7017,
    LimitCur_idx = 0x7018,
    MechPos_idx = 0x7019,
    IqFilt_idx = 0x701A,
    MechVel_idx = 0x701B,
    Vbus_idx = 0x701C,
    Rotation_idx = 0x701D,
    LocKp_idx = 0x701E,
    SpdKp_idx = 0x7020,
    SpdKi_idx = 0x7020
} ControlTable;
 
typedef enum {
    RunMode_Typ = 's', 
    IqRef_Typ = 'f',
    SpdRef_Typ = 'f',
    LimitTorque_Typ = 'f',
    CurKp_Typ = 'f',
    CurKi_Typ = 'f',
    CurFiltGain_Typ = 'f',
    LocRef_Typ = 'f',
    LimitSpd_Typ = 'f',
    LimitCur_Typ = 'f',
    MechPos_Typ = 'f',
    IqFilt_Typ = 'f',
    MechVel_Typ = 'f',
    Vbus_Typ = 'f',
    Rotation_Typ = 's',
    LocKp_Typ = 'f',
    SpdKp_Typ = 'f',
    SpdKi_Typ = 'f'
} ControlTableType;
 
//���Ʋ�����ֵ����������
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f
#define MAX_P 720
#define MIN_P -720
 
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
#define Communication_Type_ErrorFeedback 0x15
 
enum CONTROL_MODE   //����ģʽ����
{
    Motion_mode = 0,//�˿�ģʽ
    Position_mode,  //λ��ģʽ
    Velcity_mode,     //λ��ģʽ
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
 
#endif
