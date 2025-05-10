/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint32_t pulse_duration; 
  double flow_rate; 
} flow_struct; 

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_PUMPS  3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
uint8_t LED1 = 0; 
uint8_t LED2 = 0; 
uint8_t Timer6_flag = 0; 
uint32_t tim6_val = 0; 
uint32_t tim6_overflow = 0; 
uint32_t flow_start[3] = {0};

Pump_struct pump1, pump2, pump3; 
flow_struct flow[3]; 
valve_struct valve[6]; 


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
// System functions
int _write(int,char*,int);
uint32_t Get_timer6_us(void);
static void DWT_DelayInit(void);
static inline void Delay_us(uint32_t);
// Pump
void PumpCtrl(void);
void PumpGPIO(uint8_t);
void BootstrapCharge(uint8_t, uint8_t);
void Pumpoff(uint8_t);
// Valve 
void ValveCtrl(void);
void ValveGPIO(uint8_t, uint8_t);
// Flow sensor 
void Readflow(void);
// Misc 
void Update_LED(void);
void Timer6_test(void);





/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  DWT_DelayInit();
  HAL_Delay(1000);
  printf("> ");

  // begin timer6 ISR
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    Update_LED();
    PumpCtrl();
    ValveCtrl();
    Timer6_test();



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 119;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VAL3_Pin|VAL2_Pin|VAL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SM_EN_Pin|SM_DIR_Pin|SM_PUL_Pin|AM_EN_Pin
                          |VM_EN_Pin|P1_LS_RL_Pin|P1_LS_LR_Pin|P1_HS_LR_Pin
                          |LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, P1_HS_RL_Pin|P2_HS_RL_Pin|P2_HS_LR_Pin|P2_LS_RL_Pin
                          |P3_LS_RL_Pin|P3_LS_LR_Pin|P3_HS_LR_Pin|P3_HS_RL_Pin
                          |LED2_Pin|VAL6_Pin|VAL5_Pin|VAL4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VAL3_Pin VAL2_Pin VAL1_Pin */
  GPIO_InitStruct.Pin = VAL3_Pin|VAL2_Pin|VAL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SM_EN_Pin SM_DIR_Pin SM_PUL_Pin AM_EN_Pin
                           VM_EN_Pin P1_LS_RL_Pin P1_LS_LR_Pin P1_HS_LR_Pin
                           LED1_Pin */
  GPIO_InitStruct.Pin = SM_EN_Pin|SM_DIR_Pin|SM_PUL_Pin|AM_EN_Pin
                          |VM_EN_Pin|P1_LS_RL_Pin|P1_LS_LR_Pin|P1_HS_LR_Pin
                          |LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : P1_HS_RL_Pin P2_HS_RL_Pin P2_HS_LR_Pin P2_LS_RL_Pin
                           P3_LS_RL_Pin P3_LS_LR_Pin P3_HS_LR_Pin P3_HS_RL_Pin
                           LED2_Pin VAL6_Pin VAL5_Pin VAL4_Pin */
  GPIO_InitStruct.Pin = P1_HS_RL_Pin|P2_HS_RL_Pin|P2_HS_LR_Pin|P2_LS_RL_Pin
                          |P3_LS_RL_Pin|P3_LS_LR_Pin|P3_HS_LR_Pin|P3_HS_RL_Pin
                          |LED2_Pin|VAL6_Pin|VAL5_Pin|VAL4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : P2_LS_LR_Pin */
  GPIO_InitStruct.Pin = P2_LS_LR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(P2_LS_LR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FLOW3_Pin FLOW2_Pin FLOW1_Pin */
  GPIO_InitStruct.Pin = FLOW3_Pin|FLOW2_Pin|FLOW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/************************ SYSTEM FUNCTIONS (ISR) ************************/
int _write(int file, char *ptr, int len) 
{
  CDC_Transmit_FS((uint8_t*)ptr, len);
  return len;
}

static void DWT_DelayInit(void)
{
    CoreDebug->DEMCR   |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL         |= DWT_CTRL_CYCCNTENA_Msk;
}

// Delay in microseconds
static inline void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq()/1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

// INTERRUPT CALLBACKS
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) //check if timer6 IT flag
  {
    tim6_overflow++;
  }
}

// GPIO EXTI CALLBACKS 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Rising / Falling edge 
  if (GPIO_Pin == FLOW1_Pin)
  {
    if (HAL_GPIO_ReadPin(GPIOB, FLOW1_Pin)) 
    {
      flow_start[0] = Get_timer6_us();
    }
    else 
    {
      flow[0].pulse_duration = Get_timer6_us() - flow_start[0];
    }
  }
  if (GPIO_Pin == FLOW2_Pin)
  {
    if (HAL_GPIO_ReadPin(GPIOB, FLOW2_Pin)) 
    {
      flow_start[1] = Get_timer6_us();
    }
    else 
    {
      flow[1].pulse_duration = Get_timer6_us() - flow_start[1];
    }
  }
  if (GPIO_Pin == FLOW3_Pin)
  {
    if (HAL_GPIO_ReadPin(GPIOB, FLOW3_Pin)) 
    {
      flow_start[2] = Get_timer6_us();
    }
    else 
    {
      flow[2].pulse_duration = Get_timer6_us() - flow_start[2];
    }
  }
}

/************************ HELPER FUNCTIONS ************************/

void Update_LED(void)
{
  if (LED1) HAL_GPIO_WritePin(GPIOA, LED1_Pin, 1);
  else HAL_GPIO_WritePin(GPIOA, LED1_Pin, 0);
  if (LED2) HAL_GPIO_WritePin(GPIOB, LED2_Pin, 1);
  else HAL_GPIO_WritePin(GPIOB, LED2_Pin, 0);
}

void Timer6_test(void)
{
  if (Timer6_flag)
  {
    tim6_val = Get_timer6_us();
    while(Get_timer6_us() - tim6_val < 10e6);
    printf("10 seconds reached\r\n");
    LED2 = 1; 
    Timer6_flag = 0; 
  }
}

uint32_t Get_timer6_us(void)
{
  int temp = __HAL_TIM_GET_COUNTER(&htim6);
  return tim6_overflow * 65536 + temp;
}

void PumpCtrl(void)
{
  if (pump1.status)
  {
    if (Get_timer6_us() - pump1.start_us < pump1.duration_us)
    {
      PumpGPIO(3);
      LED2 = 1; 
    }
    else 
    {
      LED2 = 0; 
      pump1.status = 0; 
      pump1.start_us = 0; 
      pump1.duration_us = 0; 
      Pumpoff(3);
      printf("Pump 1 done!\r\n");
    }
  }

  if (pump2.status)
  {
    if (Get_timer6_us() - pump2.start_us < pump2.duration_us)
    {
      PumpGPIO(3);
      LED2 = 1; 
    }
    else 
    {
      LED2 = 0;  
      pump2.status = 0; 
      pump2.start_us = 0; 
      pump2.duration_us = 0; 
      Pumpoff(3);
      printf("Pump 2 done!\r\n");
    }
  }
  
  if (pump3.status)
  {
    if (Get_timer6_us() - pump3.start_us < pump3.duration_us)
    {
      Readflow();
      PumpGPIO(3);
      LED2 = 1; 
    }
    else 
    {
      LED2 = 0; 
      pump3.status = 0; 
      // pump3.start_us = 0; 
      pump3.duration_us = 0; 
      Pumpoff(3);
      // printf("Pump 3 done!\r\n");
      printf("Start time: %ld, End time: %ld, Difference: %ld\r\n", pump3.start_us, Get_timer6_us(), Get_timer6_us()-pump3.start_us);
    }
  }
}

void PumpGPIO(uint8_t pumpnum)
{
  switch(pumpnum)
  {
    case 1: 
        // Precharging bootstrap capacitor 
        Pumpoff(1);
        BootstrapCharge(1,pump1.dir); 
        Delay_us(1);
        Pumpoff(1);
        if (pump1.dir) // 1 is LR
        {
          HAL_GPIO_WritePin(GPIOA, P1_LS_LR_Pin, 1);
          HAL_GPIO_WritePin(GPIOA, P1_HS_LR_Pin, 1);
        }
        else 
        {
          HAL_GPIO_WritePin(GPIOA, P1_LS_RL_Pin, 1);
          HAL_GPIO_WritePin(GPIOB, P1_HS_RL_Pin, 1);
        }
      break; 

    case 2: 
      // Precharging bootstrap capacitor 
      Pumpoff(2);
      BootstrapCharge(2,pump2.dir); 
      Delay_us(1);
      Pumpoff(2);
      if (pump2.dir) // 1 is LR
      {
        HAL_GPIO_WritePin(GPIOB, P2_LS_LR_Pin, 1);
        HAL_GPIO_WritePin(GPIOB, P2_HS_LR_Pin, 1);
      }
      else 
      {
        HAL_GPIO_WritePin(GPIOB, P2_LS_RL_Pin, 1);
        HAL_GPIO_WritePin(GPIOB, P2_HS_RL_Pin, 1);
      }
      break; 

    case 3: 
      // Precharging bootstrap capacitor 
      Pumpoff(3);
      BootstrapCharge(3,pump3.dir); 
      Delay_us(1);
      Pumpoff(3);

      if (pump3.dir) // 1 is LR
      {
        HAL_GPIO_WritePin(GPIOB, P3_LS_LR_Pin, 1);
        HAL_GPIO_WritePin(GPIOB, P3_HS_LR_Pin, 1);
      }
      else 
      {
        HAL_GPIO_WritePin(GPIOB, P3_LS_RL_Pin, 1);
        HAL_GPIO_WritePin(GPIOB, P3_HS_RL_Pin, 1);
      }
      break; 
  }
}

void BootstrapCharge(uint8_t pumpnum, uint8_t dir)
{
  switch(pumpnum)
  {
    case 1: 
      break;
    case 2: 
      break; 
    case 3:
      if (dir) HAL_GPIO_WritePin(GPIOB, P3_LS_LR_Pin, 1);
      else HAL_GPIO_WritePin(GPIOB, P3_LS_RL_Pin, 1);
      break;
  }
}

void Pumpoff(uint8_t pumpnum)
{
  switch(pumpnum)
  {
    case 1: 
      HAL_GPIO_WritePin(GPIOA, P1_LS_RL_Pin, 0);
      HAL_GPIO_WritePin(GPIOA, P1_LS_LR_Pin, 0);
      HAL_GPIO_WritePin(GPIOA, P1_HS_LR_Pin, 0);
      HAL_GPIO_WritePin(GPIOB, P1_HS_RL_Pin, 0);
      break; 

    case 2: 
      HAL_GPIO_WritePin(GPIOB, P2_LS_RL_Pin, 0);
      HAL_GPIO_WritePin(GPIOB, P2_LS_LR_Pin, 0);
      HAL_GPIO_WritePin(GPIOB, P2_HS_LR_Pin, 0);
      HAL_GPIO_WritePin(GPIOB, P2_HS_RL_Pin, 0);
      break; 

    case 3: 
      HAL_GPIO_WritePin(GPIOB, P3_LS_RL_Pin, 0);
      HAL_GPIO_WritePin(GPIOB, P3_LS_LR_Pin, 0);
      HAL_GPIO_WritePin(GPIOB, P3_HS_LR_Pin, 0);
      HAL_GPIO_WritePin(GPIOB, P3_HS_RL_Pin, 0);
      break; 
  }
}

void Readflow(void)
{
  // pulse vs flow rate eq: flow_rate = (9*pulse + 800) / (640)
  for (uint8_t i = 0; i < 3; i++)
  {
    flow[i].flow_rate = (9 * flow[i].pulse_duration + 800) / 640; 
    
  }
  printf("1:%.3f 2:%.3f 3:%.3f       \r\n", flow[0].flow_rate, flow[1].flow_rate, flow[2].flow_rate);
}

void ValveCtrl(void)
{
  for (uint8_t i = 0; i < 6; i++)
  {
    if (valve[i].status)
    {
      if (Get_timer6_us()-valve[i].start_us < valve[i].duration_us)
      {
        ValveGPIO(i+1,1);
        LED1 = 1;
      }
      else
      {
        LED1 = 0;
        valve[i].status = 0; 
        valve[i].start_us = 0; 
        valve[i].duration_us = 0; 
        ValveGPIO(i+1,0);
        printf("Valve %d stopped\r\n", i+1);
      }
    }
  }
}

void ValveGPIO(uint8_t valvenum, uint8_t state)
{
  switch(valvenum){
    case 1: 
      if (state)  HAL_GPIO_WritePin(GPIOC, VAL1_Pin,1);
      else        HAL_GPIO_WritePin(GPIOC, VAL1_Pin,0);
      break; 
    case 2: 
      if (state)  HAL_GPIO_WritePin(GPIOC, VAL2_Pin,1);
      else        HAL_GPIO_WritePin(GPIOC, VAL2_Pin,0);
      break;
    case 3: 
      if (state)  HAL_GPIO_WritePin(GPIOC, VAL3_Pin,1);
      else        HAL_GPIO_WritePin(GPIOC, VAL3_Pin,0);
      break; 
    case 4: 
      if (state)  HAL_GPIO_WritePin(GPIOB, VAL4_Pin,1);
      else        HAL_GPIO_WritePin(GPIOB, VAL4_Pin,0);
      break;  
    case 5: 
      if (state)  HAL_GPIO_WritePin(GPIOB, VAL5_Pin,1);
      else        HAL_GPIO_WritePin(GPIOB, VAL5_Pin,0);
      break; 
    case 6: 
      if (state)  HAL_GPIO_WritePin(GPIOB, VAL6_Pin,1);
      else        HAL_GPIO_WritePin(GPIOB, VAL6_Pin,0);
      break; 
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
