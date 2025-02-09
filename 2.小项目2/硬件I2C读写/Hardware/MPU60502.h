#ifndef __MPU60502_H
#define __MPU60502_H

void MPU60502_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU60502_ReadReg(uint8_t RegAddress);

void MPU60502_Init(void);
uint8_t MPU60502_GetID(void);
void MPU60502_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif
