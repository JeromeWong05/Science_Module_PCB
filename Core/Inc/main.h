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
#define VAL3_Pin GPIO_PIN_13
#define VAL3_GPIO_Port GPIOC
#define VAL2_Pin GPIO_PIN_14
#define VAL2_GPIO_Port GPIOC
#define VAL1_Pin GPIO_PIN_15
#define VAL1_GPIO_Port GPIOC
#define SM_EN_Pin GPIO_PIN_0
#define SM_EN_GPIO_Port GPIOA
#define SM_DIR_Pin GPIO_PIN_1
#define SM_DIR_GPIO_Port GPIOA
#define SM_PUL_Pin GPIO_PIN_2
#define SM_PUL_GPIO_Port GPIOA
#define AM_EN_Pin GPIO_PIN_3
#define AM_EN_GPIO_Port GPIOA
#define VM_EN_Pin GPIO_PIN_4
#define VM_EN_GPIO_Port GPIOA
#define P1_LS_RL_Pin GPIO_PIN_5
#define P1_LS_RL_GPIO_Port GPIOA
#define P1_LS_LR_Pin GPIO_PIN_6
#define P1_LS_LR_GPIO_Port GPIOA
#define P1_HS_LR_Pin GPIO_PIN_7
#define P1_HS_LR_GPIO_Port GPIOA
#define P1_HS_RL_Pin GPIO_PIN_0
#define P1_HS_RL_GPIO_Port GPIOB
#define P2_HS_RL_Pin GPIO_PIN_1
#define P2_HS_RL_GPIO_Port GPIOB
#define P2_HS_LR_Pin GPIO_PIN_2
#define P2_HS_LR_GPIO_Port GPIOB
#define P2_LS_LR_Pin GPIO_PIN_10
#define P2_LS_LR_GPIO_Port GPIOB
#define P2_LS_RL_Pin GPIO_PIN_11
#define P2_LS_RL_GPIO_Port GPIOB
#define P3_LS_RL_Pin GPIO_PIN_12
#define P3_LS_RL_GPIO_Port GPIOB
#define P3_LS_LR_Pin GPIO_PIN_13
#define P3_LS_LR_GPIO_Port GPIOB
#define P3_HS_LR_Pin GPIO_PIN_14
#define P3_HS_LR_GPIO_Port GPIOB
#define P3_HS_RL_Pin GPIO_PIN_15
#define P3_HS_RL_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOB
#define VAL6_Pin GPIO_PIN_4
#define VAL6_GPIO_Port GPIOB
#define VAL5_Pin GPIO_PIN_5
#define VAL5_GPIO_Port GPIOB
#define VAL4_Pin GPIO_PIN_6
#define VAL4_GPIO_Port GPIOB
#define FLOW3_Pin GPIO_PIN_7
#define FLOW3_GPIO_Port GPIOB
#define FLOW2_Pin GPIO_PIN_8
#define FLOW2_GPIO_Port GPIOB
#define FLOW1_Pin GPIO_PIN_9
#define FLOW1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern uint8_t LED1_flag; 
extern uint8_t LED2_flag; 
extern uint8_t Motor1_flag; 
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
