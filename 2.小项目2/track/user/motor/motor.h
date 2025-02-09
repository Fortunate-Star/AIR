#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"
#include "sys.h"
#define AIN1 PBout(13)
#define AIN2 PBout(12)
#define BIN1 PBout(14)
#define BIN2 PBout(15)

int abs(int p);
extern TIM_HandleTypeDef htim1;
void Motor_Init(void);
int abs(int p);
void left_wheel_head(void);
void left_wheel_back(void);
void left_wheel_stop(void);
void right_wheel_head(void);
void right_wheel_back(void);
void right_wheel_stop(void);
void go_head(void);
void go_back(void);
void go_left(void);
void go_right(void);
void turn_left(void);
void turn_right(void);
void wheel_stop(void);
#endif
