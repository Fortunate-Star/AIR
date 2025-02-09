#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(CAN_HandleTypeDef* hcan)
{
	CAN_FilterTypeDef		CAN_FilterConfigStructure;
	if (hcan == &hcan2)
	{
	    CAN_FilterConfigStructure.FilterBank = 14;//***********************!!
	}
	if (hcan == &hcan1)
	{
		CAN_FilterConfigStructure.FilterBank = 0;
	}
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;

	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
	CAN_FilterConfigStructure.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(hcan, &CAN_FilterConfigStructure); 

}
