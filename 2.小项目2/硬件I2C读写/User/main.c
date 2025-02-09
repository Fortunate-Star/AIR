#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "MPU60502.h"


typedef struct 
{
	int16_t AX;
	int16_t AY;
	int16_t AZ;
	int16_t GX;
	int16_t GY;
	int16_t GZ;
}MPUTypeDef;

MPUTypeDef mpu1_data,mpu2_data;
uint8_t MPUID1,	MPUID2;


int main(void)
{
	OLED_Init();		//OLED初始化
	MPU6050_Init();		//MPU6050初始化(两个mpu6050一起初始化)
	MPUID1 = MPU6050_GetID(MPU_ID1);				//获取获取第一个MPU6050的ID号
	MPUID2 = MPU6050_GetID(MPU_ID2);				//获取获取第二个MPU6050的ID号
	

	while(1)
	{
	MPU6050_GetData(MPU_ID1,&(mpu1_data.AX), &(mpu1_data.AY), &(mpu1_data.AZ), &(mpu1_data.GX), &(mpu1_data.GY), &(mpu1_data.GZ));		//获取MPU6050的数据		
	MPU6050_GetData(MPU_ID2,&(mpu2_data.AX), &(mpu2_data.AY), &(mpu2_data.AZ), &(mpu2_data.GX), &(mpu2_data.GY), &(mpu2_data.GZ));		//获取MPU6050的数据		
	}	
}
