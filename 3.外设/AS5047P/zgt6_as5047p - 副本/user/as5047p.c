#include "as5047p.h"
#define AS5407P_CS_L HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
#define AS5407P_CS_H HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
extern SPI_HandleTypeDef hspi1;
// ������ż����
uint16_t Parity_bit_Calculate(uint16_t data_2_cal)
{
	uint16_t parity_bit_value=0;
	while(data_2_cal != 0)
	{
		parity_bit_value ^= data_2_cal; 
		data_2_cal >>=1;
	}
	return (parity_bit_value & 0x1); 
}
// SPI���Ͷ�ȡ����
uint16_t SPI_ReadWrite_OneByte(uint16_t _txdata)
{

	AS5407P_CS_L	//cs����
	uint16_t rxdata;
	if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)&_txdata,(uint8_t *)&rxdata,1,1000) !=HAL_OK)
		rxdata=0;
  AS5407P_CS_H		//cs����
	return rxdata;
}
uint16_t AS5047_read(uint16_t add)
{
	uint16_t data;
	add |= 0x4000;	//��ָ�� bit14 ��1
	if(Parity_bit_Calculate(add)==1) add=add|0x8000; //���ǰ15λ 1�ĸ���λż������Bit15 ��1
	SPI_ReadWrite_OneByte(add);		//����һ��ָ����ܶ��ص�����
		data=SPI_ReadWrite_OneByte(NOP|0x4000); //����һ����ָ���ȡ��һ��ָ��ص����ݡ�
	data &=0x3fff;
	return data;
}
