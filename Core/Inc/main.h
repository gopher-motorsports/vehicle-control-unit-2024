/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOC
#define MCU_STATUS_LED_Pin GPIO_PIN_1
#define MCU_STATUS_LED_GPIO_Port GPIOC
#define GSENSE_LED_Pin GPIO_PIN_2
#define GSENSE_LED_GPIO_Port GPIOC
#define STATUS_B_Pin GPIO_PIN_3
#define STATUS_B_GPIO_Port GPIOC
#define STATUS_R_Pin GPIO_PIN_0
#define STATUS_R_GPIO_Port GPIOA
#define PUMP_PWM_Pin GPIO_PIN_1
#define PUMP_PWM_GPIO_Port GPIOA
#define STATUS_G_Pin GPIO_PIN_2
#define STATUS_G_GPIO_Port GPIOA
#define CURR_FAULT_3V3_Pin GPIO_PIN_4
#define CURR_FAULT_3V3_GPIO_Port GPIOA
#define DRS_PWM_Pin GPIO_PIN_5
#define DRS_PWM_GPIO_Port GPIOA
#define CURR_FAULT_5V5_Pin GPIO_PIN_6
#define CURR_FAULT_5V5_GPIO_Port GPIOA
#define PUMP_PRESS_Pin GPIO_PIN_7
#define PUMP_PRESS_GPIO_Port GPIOA
#define BRK_Pin GPIO_PIN_4
#define BRK_GPIO_Port GPIOC
#define TS_SNS_Pin GPIO_PIN_5
#define TS_SNS_GPIO_Port GPIOC
#define APPS2_Pin GPIO_PIN_0
#define APPS2_GPIO_Port GPIOB
#define APPS1_Pin GPIO_PIN_1
#define APPS1_GPIO_Port GPIOB
#define RTD_BUTTON_Pin GPIO_PIN_10
#define RTD_BUTTON_GPIO_Port GPIOB
#define CANRX_Pin GPIO_PIN_12
#define CANRX_GPIO_Port GPIOB
#define CANTX_Pin GPIO_PIN_13
#define CANTX_GPIO_Port GPIOB
#define RAD_FAN_Pin GPIO_PIN_14
#define RAD_FAN_GPIO_Port GPIOB
#define BSPD_TS_SNS_FAULT_Pin GPIO_PIN_15
#define BSPD_TS_SNS_FAULT_GPIO_Port GPIOB
#define BSPD_BRK_FAULT_Pin GPIO_PIN_6
#define BSPD_BRK_FAULT_GPIO_Port GPIOC
#define BSPD_TS_BRK_FAULT_Pin GPIO_PIN_7
#define BSPD_TS_BRK_FAULT_GPIO_Port GPIOC
#define BRK_LT_Pin GPIO_PIN_8
#define BRK_LT_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define HARDFAULT_LED_Pin GPIO_PIN_15
#define HARDFAULT_LED_GPIO_Port GPIOA
#define MCU_AUX_2_Pin GPIO_PIN_12
#define MCU_AUX_2_GPIO_Port GPIOC
#define MCU_AUX_1_Pin GPIO_PIN_2
#define MCU_AUX_1_GPIO_Port GPIOD
#define AUX_GPIO_2_Pin GPIO_PIN_4
#define AUX_GPIO_2_GPIO_Port GPIOB
#define AUX_GPIO_1_Pin GPIO_PIN_5
#define AUX_GPIO_1_GPIO_Port GPIOB
#define USART_TX_Pin GPIO_PIN_6
#define USART_TX_GPIO_Port GPIOB
#define USART_RX_Pin GPIO_PIN_7
#define USART_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
