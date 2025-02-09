#include "mapan.h"
uint8_t rx_data[10];
extern UART_HandleTypeDef huart8;
void Data_send(void)
{
//	uint8_t i;
//	HAL_StatusTypeDef status;
//	 for (i = 0; i < 8; i++)
//    {
//        // 使用 HAL_USART_Transmit 发送数据 
//        status = HAL_UART_Transmit(&huart8, &usart_txbuf[i], 1, 1000);  // 发送1个字节
//			rx_data[i] = usart_txbuf[i];
//		}		
//        if (status != HAL_OK)
//        {
//            // 处理错误，比如打印错误信息，或者进入错误处理函数
//            Error_Handler();  // 确保您有 Error_Handler() 来处理错误
//        }

//        // 如果你想阻塞，直到发送完成，可以检查传输完成标志
//        while (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_TC) == RESET)
//        {
//            // 等待 TC 标志位被置位，表示数据发送完毕
//        }
//    }
}

void DATE_Y_TO_MCU(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
  	if ( sbus_buf[0] != 0X01||sbus_buf[1] != 0X03|| sbus_buf[2] != 0X04||sbus_buf[3]!=0X00)
	  {
	    	return;
	  }
		rc_ctrl->last_y=rc_ctrl->y;
		rc_ctrl->y=(float)((sbus_buf[3]<<24)|(sbus_buf[4]<<16)|(sbus_buf[5]<<8)|sbus_buf[6])/0x40000*360.0;
	if(rc_ctrl->y>360||rc_ctrl->y<0)
		{	
			rc_ctrl->y=rc_ctrl->last_y;
			return;
		}
}