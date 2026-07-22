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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_ON_Pin GPIO_PIN_1
#define MOTOR_ON_GPIO_Port GPIOF
#define MOTOR_START_Pin GPIO_PIN_2
#define MOTOR_START_GPIO_Port GPIOF
#define ESTOP_btn_Pin GPIO_PIN_7
#define ESTOP_btn_GPIO_Port GPIOF
#define ESTOP_btn_EXTI_IRQn EXTI9_5_IRQn
#define STOP_btn_Pin GPIO_PIN_9
#define STOP_btn_GPIO_Port GPIOF
#define STOP_btn_EXTI_IRQn EXTI9_5_IRQn
#define STOP_Pin GPIO_PIN_10
#define STOP_GPIO_Port GPIOF
#define LAMP_GREEN_Pin GPIO_PIN_0
#define LAMP_GREEN_GPIO_Port GPIOC
#define W610_CS_Pin GPIO_PIN_0
#define W610_CS_GPIO_Port GPIOA
#define LAMP_RED_Pin GPIO_PIN_3
#define LAMP_RED_GPIO_Port GPIOA
#define FRAM_CS_Pin GPIO_PIN_4
#define FRAM_CS_GPIO_Port GPIOA
#define MOTOR_START_btn_Pin GPIO_PIN_1
#define MOTOR_START_btn_GPIO_Port GPIOG
#define MOTOR_START_btn_EXTI_IRQn EXTI1_IRQn
#define W610_INT_Pin GPIO_PIN_10
#define W610_INT_GPIO_Port GPIOE
#define W610_INT_EXTI_IRQn EXTI15_10_IRQn
#define rs4852_Pin GPIO_PIN_13
#define rs4852_GPIO_Port GPIOD
#define rs485_Pin GPIO_PIN_12
#define rs485_GPIO_Port GPIOC
#define W610_RST_Pin GPIO_PIN_0
#define W610_RST_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
