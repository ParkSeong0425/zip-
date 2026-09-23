/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "run.h"
#include "cli.h"
#include "net.h"
#include "fram.h"
#include "save.h"
#include "hmi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Event_Task */
osThreadId_t Event_TaskHandle;
const osThreadAttr_t Event_Task_attributes = {
  .name = "Event_Task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for Motor_Task */
osThreadId_t Motor_TaskHandle;
const osThreadAttr_t Motor_Task_attributes = {
  .name = "Motor_Task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};
/* Definitions for Comm_Task */
osThreadId_t Comm_TaskHandle;
const osThreadAttr_t Comm_Task_attributes = {
  .name = "Comm_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for MotorQueue */
osMessageQueueId_t MotorQueueHandle;
const osMessageQueueAttr_t MotorQueue_attributes = {
  .name = "MotorQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Event_Task(void *argument);
void Start_Motor_Task(void *argument);
void Start_Comm_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MotorQueue */
  MotorQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &MotorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  MotorQueueHandle = osMessageQueueNew(4, sizeof(MotorCommand), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Event_Task */
  Event_TaskHandle = osThreadNew(Start_Event_Task, NULL, &Event_Task_attributes);

  /* creation of Motor_Task */
  Motor_TaskHandle = osThreadNew(Start_Motor_Task, NULL, &Motor_Task_attributes);

  /* creation of Comm_Task */
  Comm_TaskHandle = osThreadNew(Start_Comm_Task, NULL, &Comm_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_Event_Task */
/**
  * @brief  Function implementing the Event_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Event_Task */
void Start_Event_Task(void *argument)
{
  /* USER CODE BEGIN Start_Event_Task */
  for(;;)
  { // HMI_Run();
    osDelay(500);
  }
  /* USER CODE END Start_Event_Task */
}

/* USER CODE BEGIN Header_Start_Motor_Task */
/**
* @brief Function implementing the Motor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Motor_Task */
void Start_Motor_Task(void *argument)
{
  /* USER CODE BEGIN Start_Motor_Task */
    Motor_Run();

  /* USER CODE END Start_Motor_Task */
}

/* USER CODE BEGIN Header_Start_Comm_Task */
/**
* @brief Function implementing the Comm_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Comm_Task */
void Start_Comm_Task(void *argument)
{
  /* USER CODE BEGIN Start_Comm_Task */

	Save_Init();
	NET_Init();
    for (;;)
    {
        CLI_Run();
        NET_Run();

        osDelay(1);
    }

  /* USER CODE END Start_Comm_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

