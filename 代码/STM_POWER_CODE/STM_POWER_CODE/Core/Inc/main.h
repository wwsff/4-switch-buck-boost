/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define CH1_VIN_Pin GPIO_PIN_0
#define CH1_VIN_GPIO_Port GPIOC
#define CH1_VOUT_Pin GPIO_PIN_1
#define CH1_VOUT_GPIO_Port GPIOC
#define CH1_IOUT_Pin GPIO_PIN_2
#define CH1_IOUT_GPIO_Port GPIOC
#define CH1_TL_Pin GPIO_PIN_2
#define CH1_TL_GPIO_Port GPIOA
#define CH1_TMOS_Pin GPIO_PIN_3
#define CH1_TMOS_GPIO_Port GPIOC
#define CH1_IL_Pin GPIO_PIN_0
#define CH1_IL_GPIO_Port GPIOA
#define CH2_IL_Pin GPIO_PIN_1
#define CH2_IL_GPIO_Port GPIOA
#define CH1_PWR_EN_Pin GPIO_PIN_3
#define CH1_PWR_EN_GPIO_Port GPIOA
#define CH2_IOUT_Pin GPIO_PIN_4
#define CH2_IOUT_GPIO_Port GPIOA
#define CH2_TMOS_Pin GPIO_PIN_5
#define CH2_TMOS_GPIO_Port GPIOA
#define CH2_TL_Pin GPIO_PIN_6
#define CH2_TL_GPIO_Port GPIOA
#define CH2_VIN_Pin GPIO_PIN_4
#define CH2_VIN_GPIO_Port GPIOC
#define CH2_VOUT_Pin GPIO_PIN_5
#define CH2_VOUT_GPIO_Port GPIOC
#define CH1_IIN_Pin GPIO_PIN_0
#define CH1_IIN_GPIO_Port GPIOB
#define CH2_IIN_Pin GPIO_PIN_1
#define CH2_IIN_GPIO_Port GPIOB
#define CH2_PWR_EN_Pin GPIO_PIN_2
#define CH2_PWR_EN_GPIO_Port GPIOB
#define CH1_ALERT_N_Pin GPIO_PIN_10
#define CH1_ALERT_N_GPIO_Port GPIOB
#define CH2_ALERT_N_Pin GPIO_PIN_11
#define CH2_ALERT_N_GPIO_Port GPIOB
#define CH2_IN_H_Pin GPIO_PIN_12
#define CH2_IN_H_GPIO_Port GPIOB
#define CH2_IN_L_Pin GPIO_PIN_13
#define CH2_IN_L_GPIO_Port GPIOB
#define CH2_OUT_H_Pin GPIO_PIN_14
#define CH2_OUT_H_GPIO_Port GPIOB
#define CH2_OUT_L_Pin GPIO_PIN_15
#define CH2_OUT_L_GPIO_Port GPIOB
#define LED_RUN_Pin GPIO_PIN_8
#define LED_RUN_GPIO_Port GPIOC
#define LED_CAN_Pin GPIO_PIN_9
#define LED_CAN_GPIO_Port GPIOC
#define CH1_IN_H_Pin GPIO_PIN_8
#define CH1_IN_H_GPIO_Port GPIOA
#define CH1_IN_L_Pin GPIO_PIN_9
#define CH1_IN_L_GPIO_Port GPIOA
#define CH1_OUT_H_Pin GPIO_PIN_10
#define CH1_OUT_H_GPIO_Port GPIOA
#define CH1_OUT_L_Pin GPIO_PIN_11
#define CH1_OUT_L_GPIO_Port GPIOA
#define CH1_OC_N_Pin GPIO_PIN_12
#define CH1_OC_N_GPIO_Port GPIOA
#define CH2_OC_N_Pin GPIO_PIN_15
#define CH2_OC_N_GPIO_Port GPIOA
#define LED_CH1_Pin GPIO_PIN_12
#define LED_CH1_GPIO_Port GPIOC
#define LED_FAULT_Pin GPIO_PIN_2
#define LED_FAULT_GPIO_Port GPIOD
#define LED_CH2_Pin GPIO_PIN_7
#define LED_CH2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
