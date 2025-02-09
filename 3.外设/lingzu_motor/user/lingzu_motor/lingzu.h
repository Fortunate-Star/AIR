#ifndef __LINGZU_H
#define __LINGZU_H
#include "main.h"
#include "can.h"
//控制参数最值，谨慎更改
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
//主机CANID设置
#define Master_CAN_ID 0x00                      //主机ID     
//控制命令宏定义
#define Communication_Type_GetID 0x00           //获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl 0x01 	//用来向主机发送控制指令
#define Communication_Type_MotorRequest 0x02	//用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03	    //电机使能运行
#define Communication_Type_MotorStop 0x04	    //电机停止运行
#define Communication_Type_SetPosZero 0x06	    //设置电机机械零位
#define Communication_Type_CanID 0x07	        //更改当前电机CAN_ID
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11	//读取单个参数
#define Communication_Type_SetSingleParameter 0x12	//设定单个参数
#define Communication_Type_ErrorFeedback 0x15	    //故障反馈帧
//参数读取宏定义
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

enum CONTROL_MODE   //控制模式定义
{
    Motion_mode = 0,//运控模式  
    Position_mode,  //位置模式
    Speed_mode,     //速度模式  
    Current_mode    //电流模式
};
enum ERROR_TAG      //错误回传对照
{
    OK                 = 0,//无故障
    BAT_LOW_ERR        = 1,//欠压故障
    OVER_CURRENT_ERR   = 2,//过流
    OVER_TEMP_ERR      = 3,//过温
    MAGNETIC_ERR       = 4,//磁编码故障
    HALL_ERR_ERR       = 5,//HALL编码故障
    NO_CALIBRATION_ERR = 6//未标定
};

typedef struct{           //小米电机结构体
		uint8_t CAN_ID;       //CAN ID
    uint8_t MCU_ID;       //MCU唯一标识符[后8位，共64位]
	float Angle;          //回传角度
	float Speed;          //回传速度
	float Torque;         //回传力矩
	float Temp;			  //回传温度
	
	uint16_t set_current;
	uint16_t set_speed;
	uint16_t set_position;
	
	uint8_t error_code;
	
	float Angle_Bias;
	
}LINGZU_Motor;
extern LINGZU_Motor mi_motor[4];//预先定义四个小米电机

extern void chack_cybergear(uint8_t ID);
extern void start_cybergear(LINGZU_Motor *Motor);
extern void stop_cybergear(LINGZU_Motor *Motor, uint8_t clear_error);
extern void set_mode_cybergear(LINGZU_Motor *Motor, uint8_t Mode);
extern void set_current_cybergear(LINGZU_Motor *Motor, float Current);
extern void set_zeropos_cybergear(LINGZU_Motor *Motor);
extern void set_CANID_cybergear(LINGZU_Motor *Motor, uint8_t CAN_ID);
extern void init_cybergear(LINGZU_Motor *Motor, uint8_t Can_Id, uint8_t mode);
extern void motor_controlmode(LINGZU_Motor *Motor,float torque, float MechPosition, float speed, float kp, float kd);

																							/*电机内部格式转换函数*/
static void Set_Motor_Parameter(LINGZU_Motor *Motor,uint16_t Index,float Value,char Value_type);//写入电机参数
static uint32_t Get_Motor_ID(uint32_t CAN_ID_Frame);//提取电机回复帧扩展ID中的电机CANID
static void Motor_Data_Handler(LINGZU_Motor *Motor,uint8_t DataFrame[8],uint32_t IDFrame);//电机回复帧数据处理函数
static uint8_t* Float_to_Byte(float f);// 浮点数转4字节函数
static float uint16_to_float(uint16_t x,float x_min,float x_max,int bits);//小米电机回文16位数据转浮点
static int float_to_uint(float x, float x_min, float x_max, int bits);//小米电机发送浮点转16位数据
#endif
