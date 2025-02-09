#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"
#include "sys.h"

#define AIN1 PBout(0)//右上
#define AIN2 PBout(1)

#define BIN1 PAout(12)//右下
#define BIN2 PAout(7)

#define CIN1 PBout(12)//左上
#define CIN2 PBout(13)

#define DIN1 PBout(14)//左下
#define DIN2 PBout(15)

//左上
#define left_up_go 			CIN1=1;CIN2=0;
#define left_up_back 	 CIN1=0;CIN2=1;
//左下
#define	left_down_go		DIN1=1;DIN2=0;
#define	left_down_back	DIN1=0;DIN2=1;
//右上
#define	right_up_go			AIN1=1;AIN2=0;
#define	right_up_back		AIN1=0;AIN2=1;

//右下
#define	right_dowm_go		BIN1=1;BIN2=0;
#define	right_dowm_back	BIN1=0;BIN2=1;

int abs(int p);
extern TIM_HandleTypeDef htim1;
void Motor_Init(void);
int abs(int p);
void go_stop(void);
void go_head(void);
void go_back(void);
void stop_left(void);
void stop_right(void);
void go_left(void);
void go_right(void);
#endif
