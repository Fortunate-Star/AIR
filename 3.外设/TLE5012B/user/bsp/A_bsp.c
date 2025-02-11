/**
 * @file drv_bsp.c
 * @author yssickjgd (1345578933@qq.com)
 * @brief A��İ弶֧�ְ�
 * @version 0.1
 * @date 2022-08-02
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "A_bsp.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief ��ʼ��ȫ���弶֧�ְ�����
 *
 * @param Status ����״̬�İ�λ��
 */
void BSP_Init(uint32_t Status, float IMU_Heater_Rate, float Buzzer_Rate)
{
    BSP_DC24_LU(static_cast<Enum_BSP_DC24_Status>((Status & BSP_DC24_LU_ON) == 0 ? BSP_DC24_Status_DISABLED : BSP_DC24_Status_ENABLED));
    BSP_DC24_LD(static_cast<Enum_BSP_DC24_Status>((Status & BSP_DC24_LD_ON) == 0 ? BSP_DC24_Status_DISABLED : BSP_DC24_Status_ENABLED));
    BSP_DC24_RU(static_cast<Enum_BSP_DC24_Status>((Status & BSP_DC24_RU_ON) == 0 ? BSP_DC24_Status_DISABLED : BSP_DC24_Status_ENABLED));
    BSP_DC24_RD(static_cast<Enum_BSP_DC24_Status>((Status & BSP_DC24_RD_ON) == 0 ? BSP_DC24_Status_DISABLED : BSP_DC24_Status_ENABLED));

    BSP_LED_R(static_cast<Enum_BSP_LED_Status>((Status & BSP_LED_R_ON) == 0 ? BSP_LED_Status_DISABLED : BSP_LED_Status_ENABLED));
    BSP_LED_G(static_cast<Enum_BSP_LED_Status>((Status & BSP_LED_G_ON) == 0 ? BSP_LED_Status_DISABLED : BSP_LED_Status_ENABLED));

    BSP_LED_1(static_cast<Enum_BSP_LED_Status>((Status & BSP_LED_1_ON) == 0 ? BSP_LED_Status_DISABLED : BSP_LED_Status_ENABLED));
    BSP_LED_2(static_cast<Enum_BSP_LED_Status>((Status & BSP_LED_2_ON) == 0 ? BSP_LED_Status_DISABLED : BSP_LED_Status_ENABLED));
    BSP_LED_3(static_cast<Enum_BSP_LED_Status>((Status & BSP_LED_3_ON) == 0 ? BSP_LED_Status_DISABLED : BSP_LED_Status_ENABLED));
    BSP_LED_4(static_cast<Enum_BSP_LED_Status>((Status & BSP_LED_4_ON) == 0 ? BSP_LED_Status_DISABLED : BSP_LED_Status_ENABLED));
    BSP_LED_5(static_cast<Enum_BSP_LED_Status>((Status & BSP_LED_5_ON) == 0 ? BSP_LED_Status_DISABLED : BSP_LED_Status_ENABLED));
    BSP_LED_6(static_cast<Enum_BSP_LED_Status>((Status & BSP_LED_6_ON) == 0 ? BSP_LED_Status_DISABLED : BSP_LED_Status_ENABLED));
    BSP_LED_7(static_cast<Enum_BSP_LED_Status>((Status & BSP_LED_7_ON) == 0 ? BSP_LED_Status_DISABLED : BSP_LED_Status_ENABLED));
    BSP_LED_8(static_cast<Enum_BSP_LED_Status>((Status & BSP_LED_8_ON) == 0 ? BSP_LED_Status_DISABLED : BSP_LED_Status_ENABLED));
}

/**
 * @brief ���Ͻ�DC24
 *
 * @param Status ״̬
 */
void BSP_DC24_LU(Enum_BSP_DC24_Status Status)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief ���½�DC24
 *
 * @param Status ״̬
 */
void BSP_DC24_LD(Enum_BSP_DC24_Status Status)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief ���Ͻ�DC24
 *
 * @param Status ״̬
 */
void BSP_DC24_RU(Enum_BSP_DC24_Status Status)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief ���½�DC24
 *
 * @param Status ״̬
 */
void BSP_DC24_RD(Enum_BSP_DC24_Status Status)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief ��ɫLED
 *
 * @param Status ״̬
 */
void BSP_LED_R(Enum_BSP_LED_Status Status)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief ��ɫLED
 *
 * @param Status ״̬
 */
void BSP_LED_G(Enum_BSP_LED_Status Status)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief 1��LED
 *
 * @param Status ״̬
 */
void BSP_LED_1(Enum_BSP_LED_Status Status)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief 2��LED
 *
 * @param Status ״̬
 */
void BSP_LED_2(Enum_BSP_LED_Status Status)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief 3��LED
 *
 * @param Status ״̬
 */
void BSP_LED_3(Enum_BSP_LED_Status Status)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief 4��LED
 *
 * @param Status ״̬
 */
void BSP_LED_4(Enum_BSP_LED_Status Status)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief 5��LED
 *
 * @param Status ״̬
 */
void BSP_LED_5(Enum_BSP_LED_Status Status)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief 6��LED
 *
 * @param Status ״̬
 */
void BSP_LED_6(Enum_BSP_LED_Status Status)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief 7��LED
 *
 * @param Status ״̬
 */
void BSP_LED_7(Enum_BSP_LED_Status Status)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, static_cast<GPIO_PinState>(Status));
}

/**
 * @brief 8��LED
 *
 * @param Status ״̬
 */
void BSP_LED_8(Enum_BSP_LED_Status Status)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, static_cast<GPIO_PinState>(Status));
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
