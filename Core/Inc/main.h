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
#define CTRL_Loop_Freq 100
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void delay_us(uint32_t us);
void DWT_Init(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define S1_LIM_Pin GPIO_PIN_13
#define S1_LIM_GPIO_Port GPIOC
#define S1_LIM_EXTI_IRQn EXTI15_10_IRQn
#define S1_ENA_Pin GPIO_PIN_2
#define S1_ENA_GPIO_Port GPIOC
#define S2_ENA_Pin GPIO_PIN_3
#define S2_ENA_GPIO_Port GPIOC
#define E2_A_Pin GPIO_PIN_0
#define E2_A_GPIO_Port GPIOA
#define E2_B_Pin GPIO_PIN_1
#define E2_B_GPIO_Port GPIOA
#define B2_PWM_Pin GPIO_PIN_4
#define B2_PWM_GPIO_Port GPIOA
#define S4_ENA_Pin GPIO_PIN_5
#define S4_ENA_GPIO_Port GPIOA
#define B1_PWM_Pin GPIO_PIN_6
#define B1_PWM_GPIO_Port GPIOA
#define S1_PULSES_Pin GPIO_PIN_7
#define S1_PULSES_GPIO_Port GPIOA
#define B4_DIR_Pin GPIO_PIN_5
#define B4_DIR_GPIO_Port GPIOC
#define B3_PWM_Pin GPIO_PIN_0
#define B3_PWM_GPIO_Port GPIOB
#define B4_PWM_Pin GPIO_PIN_1
#define B4_PWM_GPIO_Port GPIOB
#define S4_PULSES_Pin GPIO_PIN_2
#define S4_PULSES_GPIO_Port GPIOB
#define S3_DIR_Pin GPIO_PIN_12
#define S3_DIR_GPIO_Port GPIOB
#define B2_DIR_Pin GPIO_PIN_13
#define B2_DIR_GPIO_Port GPIOB
#define S2_PULSES_Pin GPIO_PIN_14
#define S2_PULSES_GPIO_Port GPIOB
#define B1_DIR_Pin GPIO_PIN_15
#define B1_DIR_GPIO_Port GPIOB
#define E4_B_Pin GPIO_PIN_6
#define E4_B_GPIO_Port GPIOC
#define E4_A_Pin GPIO_PIN_7
#define E4_A_GPIO_Port GPIOC
#define S4_DIR_Pin GPIO_PIN_8
#define S4_DIR_GPIO_Port GPIOC
#define B3_DIR_Pin GPIO_PIN_9
#define B3_DIR_GPIO_Port GPIOC
#define E1_A_Pin GPIO_PIN_8
#define E1_A_GPIO_Port GPIOA
#define E1_B_Pin GPIO_PIN_9
#define E1_B_GPIO_Port GPIOA
#define S2_DIR_Pin GPIO_PIN_10
#define S2_DIR_GPIO_Port GPIOA
#define E3_B_Pin GPIO_PIN_11
#define E3_B_GPIO_Port GPIOA
#define E3_A_Pin GPIO_PIN_12
#define E3_A_GPIO_Port GPIOA
#define BPill_RX_Pin GPIO_PIN_15
#define BPill_RX_GPIO_Port GPIOA
#define S4_LIM_Pin GPIO_PIN_10
#define S4_LIM_GPIO_Port GPIOC
#define S4_LIM_EXTI_IRQn EXTI15_10_IRQn
#define S3_LIM_Pin GPIO_PIN_11
#define S3_LIM_GPIO_Port GPIOC
#define S3_LIM_EXTI_IRQn EXTI15_10_IRQn
#define ROS_TX_Pin GPIO_PIN_12
#define ROS_TX_GPIO_Port GPIOC
#define ROS_RX_Pin GPIO_PIN_2
#define ROS_RX_GPIO_Port GPIOD
#define BPill_TX_Pin GPIO_PIN_3
#define BPill_TX_GPIO_Port GPIOB
#define S3_PULSES_Pin GPIO_PIN_4
#define S3_PULSES_GPIO_Port GPIOB
#define ESTOP_Pin GPIO_PIN_5
#define ESTOP_GPIO_Port GPIOB
#define ESTOP_EXTI_IRQn EXTI9_5_IRQn
#define S3_ENA_Pin GPIO_PIN_6
#define S3_ENA_GPIO_Port GPIOB
#define S2_LIM_Pin GPIO_PIN_7
#define S2_LIM_GPIO_Port GPIOB
#define S2_LIM_EXTI_IRQn EXTI9_5_IRQn
#define S1_DIR_Pin GPIO_PIN_8
#define S1_DIR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
