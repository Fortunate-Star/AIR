#ifndef __MPU6050_H
#define __MPU6050_H

#include"stdint.h"
typedef enum
{
//	MPU_ID1 = 0xD0,
//	MPU_ID2 = 0x30,
	HC_SR04 = 0xAE
}ID_typedef;
void MPU6050_WriteReg(ID_typedef MPU6050_ADDRESS, uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t MPU6050_ADDRESS,uint8_t RegAddress);

void MPU6050_Init(void);

uint8_t MPU6050_GetID(ID_typedef MPU6050_ADDRESS);
void MPU6050_GetData(ID_typedef MPU6050_ADDRESS,int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif
