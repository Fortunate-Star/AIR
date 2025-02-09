#include "as5047p.h"
extern SPI_HandleTypeDef hspi4;

#include "AS5047P.h"



//��datasheet�е�˵���Ĳ�����У�����SPIΪ˫ȫ��������2�ֽ�ģʽ��ʱ�Ӽ��Եͣ���λ2�����ʹ��
//CSNΪSPIƬѡʹ���źţ��͵�ƽʹ�ܣ�����Ϊ���ģʽ�����԰��Լ���Ҫ�޸����źš�

AS5047_TypeDef Devs[]=
{
{&hspi4,GPIOF,GPIO_PIN_1},
{&hspi4,GPIOF,GPIO_PIN_1}
};



#define assert_idx(__devidx)   if(__devidx<0 && __devidx >sizeof(Devs)/sizeof(AS5047_TypeDef) ) error_inf_loop();
#define CS_1(__devidx) HAL_GPIO_WritePin(Devs[__devidx].CSNport,Devs[__devidx].CSNpin,GPIO_PIN_SET)
#define CS_0(__devidx) HAL_GPIO_WritePin(Devs[__devidx].CSNport,Devs[__devidx].CSNpin,GPIO_PIN_RESET)

void error_inf_loop()
{
  while(1);
}


uint16_t Parity_bit_Calculate(uint16_t data_2_cal)
{
	uint16_t parity_bit_value=0;
	//�Ե�λ��ʼͳ����ż
	while(data_2_cal != 0)
	{
		//(((��^��)^��)^��)^�١������ʽ1����ݺܲ͢�ͬ�����^��������Ϊ1����ʾ��һ��1�����ʽ1���Ի��ɣ�((1^��)^��)^�١������ʽ2
		parity_bit_value ^= data_2_cal; 
		data_2_cal >>=1;
	}
	return (parity_bit_value & 0x1);
}


uint16_t SPI_ReadWrite_OneByte(int devidx,uint16_t _txdata)
{
  assert_idx(devidx);
  CS_0(devidx);
	uint16_t rxdata;
	//waring���ù�,����ȥ���ǵ�ַ���ݲ��ᱻ�ضϣ�HAL��������ã���16�ֽڵ����ݸ�ʽ��16�ֽڵķ��ͳ�ȥ
	if(HAL_SPI_TransmitReceive(Devs[devidx].hspin,(uint8_t *)&_txdata,(uint8_t *)&rxdata,1,100) !=HAL_OK)
		rxdata=0;
  CS_1(devidx);
	return rxdata;
}
uint16_t AS5047_WriteData(int devidx,uint16_t addr,uint16_t data)
{
	//���͵�ַָ��
	// & 0x3fff �õ� 13:0λ data���� ������żУ��λ����
	if(Parity_bit_Calculate(addr & 0x3fff ) == 1 ) 
		addr |= 0x8000;  //��15bit ��1  żУ��
	CS_0(devidx);  
	SPI_ReadWrite_OneByte(devidx,addr);
	CS_1(devidx);
	
	//��������ָ��
	if(Parity_bit_Calculate(data &0x3fff) ==1)
		data |=0x8000;
	
	uint16_t feedback;
	CS_0(devidx);
	feedback = SPI_ReadWrite_OneByte(devidx,data);
	CS_1(devidx);
	
	return feedback;
}
uint16_t AS5047_ReadData(int devidx,uint16_t addr)
{
	uint16_t data;
	if(Parity_bit_Calculate(addr) ==0 ) 
		addr |=0x8000; //1000 0000 0000 0000
	addr |= AS5047_COMMAND_READ; //0100 0000 0000 0000
  SPI_ReadWrite_OneByte(devidx,addr); 
	data=SPI_ReadWrite_OneByte(devidx,NOP_AS5047P_VOL_REG_ADD);  //ANGLECOM_AS5047P_VOL_REG_ADD=11 1111 1111
	data &=0x3fff;
  
//	SPI_ReadWrite_OneByte(devidx,addr);
//	data = SPI_ReadWrite_OneByte(devidx,addr);
	//�˴���������żУ���ж��Ƿ���յ���ȷ���ݣ�����Ҳ���Բ�����ֱ��ȥ��15��14bit
	data &= 0x3fff; 
	return data;
}








void AS5047_Init(void)
{
  //����ABIģʽ������ֱ�1024.
  for(int i=0;i<sizeof(Devs)/sizeof(AS5047_TypeDef);i++)
  {
    AS5047_WriteData(i,SETTINGS1_AS5047P_nVOL_REG_ADD,5); //0000 0101
    AS5047_WriteData(i,SETTINGS2_AS5047P_nVOL_REG_ADD,0);
  }
  

}
void AS5047_SetZeroPosition(int devidx)
{
	uint16_t DIAAGC=AS5047_ReadData(devidx,DIAAGC_AS5047P_VOL_REG_ADD);
	//��ȡ��ǰ�Ƕ�
	uint16_t ANGLEUNC=AS5047_ReadData(devidx,ANGLEUNC_AS5047P_VOL_REG_ADD);
	//												ANGLEUNC��13:0 14����Ч���֣�����6��ȡ��8λ��
	AS5047_WriteData(devidx,ZPOSM_AS5047P_nVOL_REG_ADD,(ANGLEUNC >>6) & 0x00ff);
	//																							�õ���6λ & 11 1111
	AS5047_WriteData(devidx,ZPOSL_AS5047P_nVOL_REG_ADD, ANGLEUNC  & 0x003f);
}

void AS5047_Diagnostics(void)
{
	;
}



uint16_t AS5047D_Get_MAGCORDIC_Value(int devidx)
{
	unsigned int CORDICMAG = AS5047_ReadData(devidx,MAG_AS5047P_VOL_REG_ADD);
	return CORDICMAG;
}

unsigned int AS5047P_Get_AGC_Value(int devidx)
{
	unsigned int DIAAGC = AS5047_ReadData(devidx,DIAAGC_AS5047P_VOL_REG_ADD);
	return (unsigned char)((DIAAGC >> 8) & 0x00FF);
}


uint16_t AS5047_Get_ZeroPosition(int devidx)
{
	uint16_t ZPOSM=AS5047_ReadData(devidx,ZPOSM_AS5047P_nVOL_REG_ADD);
	uint16_t ZPOSL=AS5047_ReadData(devidx,ZPOSL_AS5047P_nVOL_REG_ADD);
	//���߰�λ�͵�6λƴ����
	return ( ZPOSM<<6 ) & (ZPOSL & 0x003f ) ;
}

uint16_t AS5047_Get_ERRFL(int devidx)
{
	return AS5047_ReadData(devidx,ERRFL_AS5047P_VOL_REG_ADD);
}


