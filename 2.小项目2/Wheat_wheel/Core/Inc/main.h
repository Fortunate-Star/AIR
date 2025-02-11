/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IIC.h"
#include "oled.h"
#include "motor.h"
#include "JDY_31.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BIN2_Pin GPIO_PIN_7
#define BIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOB
#define AN2_Pin GPIO_PIN_1
#define AN2_GPIO_Port GPIOB
#define CIN1_Pin GPIO_PIN_12
#define CIN1_GPIO_Port GPIOB
#define CIN2_Pin GPIO_PIN_13
#define CIN2_GPIO_Port GPIOB
#define DIN1_Pin GPIO_PIN_14
#define DIN1_GPIO_Port GPIOB
#define DIN2_Pin GPIO_PIN_15
#define DIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_12
#define BIN1_GPIO_Port GPIOA
#define MPU_INT_Pin GPIO_PIN_15
#define MPU_INT_GPIO_Port GPIOA
#define mpu_sda_Pin GPIO_PIN_3
#define mpu_sda_GPIO_Port GPIOB
#define mpu_scl_Pin GPIO_PIN_4
#define mpu_scl_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
