/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define MCU_Flow1_Pin GPIO_PIN_13
#define MCU_Flow1_GPIO_Port GPIOC
#define MCU_Flow2_Pin GPIO_PIN_14
#define MCU_Flow2_GPIO_Port GPIOC
#define MCU_Flow3_Pin GPIO_PIN_15
#define MCU_Flow3_GPIO_Port GPIOC
#define MCU_LED_BANK_EN_Pin GPIO_PIN_4
#define MCU_LED_BANK_EN_GPIO_Port GPIOA
#define MCU_Pump3_HS_LR_Pin GPIO_PIN_5
#define MCU_Pump3_HS_LR_GPIO_Port GPIOA
#define MCU_Pump3_LS_LR_Pin GPIO_PIN_6
#define MCU_Pump3_LS_LR_GPIO_Port GPIOA
#define MCU_Pump3_HS_RL_Pin GPIO_PIN_7
#define MCU_Pump3_HS_RL_GPIO_Port GPIOA
#define MCU_Pump3_LS_RL_Pin GPIO_PIN_0
#define MCU_Pump3_LS_RL_GPIO_Port GPIOB
#define MCU_Pump2_HS_LR_Pin GPIO_PIN_1
#define MCU_Pump2_HS_LR_GPIO_Port GPIOB
#define MCU_Pump2_LS_LR_Pin GPIO_PIN_2
#define MCU_Pump2_LS_LR_GPIO_Port GPIOB
#define MCU_Pump2_HS_RL_Pin GPIO_PIN_10
#define MCU_Pump2_HS_RL_GPIO_Port GPIOB
#define MCU_Pump2_LS_RL_Pin GPIO_PIN_11
#define MCU_Pump2_LS_RL_GPIO_Port GPIOB
#define MCU_Pump1_LS_RL_Pin GPIO_PIN_12
#define MCU_Pump1_LS_RL_GPIO_Port GPIOB
#define MCU_Pump1_HS_RL_Pin GPIO_PIN_13
#define MCU_Pump1_HS_RL_GPIO_Port GPIOB
#define MCU_Pump1_LS_LR_Pin GPIO_PIN_14
#define MCU_Pump1_LS_LR_GPIO_Port GPIOB
#define MCU_Pump1_HS_LR_Pin GPIO_PIN_15
#define MCU_Pump1_HS_LR_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOB
#define MCU_SM_PUL__Pin GPIO_PIN_6
#define MCU_SM_PUL__GPIO_Port GPIOB
#define MCU_SM_DIR__Pin GPIO_PIN_7
#define MCU_SM_DIR__GPIO_Port GPIOB
#define MCU_SM_EN__Pin GPIO_PIN_3
#define MCU_SM_EN__GPIO_Port GPIOH
#define MCU_VM_EN_Pin GPIO_PIN_8
#define MCU_VM_EN_GPIO_Port GPIOB
#define MCU_AM_EN_Pin GPIO_PIN_9
#define MCU_AM_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern uint8_t LED1_flag; 
extern uint8_t LED2_flag; 
extern uint8_t Motor1_flag; 
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
