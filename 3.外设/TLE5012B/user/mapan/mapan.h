#ifndef __MAPAN_H
#define __MAPAN_H
#include "main.h"
typedef  struct
{
	float x;
	float y;
	float last_x;
	float last_y;
	float sum_x;
	float sum_y;
	

} RC_ctrl_t;	
void Data_send(void);
void DATE_Y_TO_MCU(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#endif

