/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
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
#include "net.h"
#include "rfid.h"
#include "can.h"
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
/* Definitions for MOTOR_Task */
osThreadId_t MOTOR_TaskHandle;
const osThreadAttr_t MOTOR_Task_attributes = {
  .name = "MOTOR_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for NET_Task */
osThreadId_t NET_TaskHandle;
const osThreadAttr_t NET_Task_attributes = {
  .name = "NET_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for RFID */
osThreadId_t RFIDHandle;
const osThreadAttr_t RFID_attributes = {
  .name = "RFID",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MOTOR_Queue */
osMessageQueueId_t MOTOR_QueueHandle;
const osMessageQueueAttr_t MOTOR_Queue_attributes = {
  .name = "MOTOR_Queue"
};
/* Definitions for TCPQueue */
osMessageQueueId_t TCPQueueHandle;
const osMessageQueueAttr_t TCPQueue_attributes = {
  .name = "TCPQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMOTOR_Task(void *argument);
void StartNET_Task(void *argument);
void StartRFID_Task(void *argument);

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
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MOTOR_Queue */
  MOTOR_QueueHandle = osMessageQueueNew (64, sizeof(uint8_t), &MOTOR_Queue_attributes);

  /* creation of TCPQueue */
  TCPQueueHandle = osMessageQueueNew (64, sizeof(uint8_t), &TCPQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	CAN1_Start();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MOTOR_Task */
  MOTOR_TaskHandle = osThreadNew(StartMOTOR_Task, NULL, &MOTOR_Task_attributes);

  /* creation of NET_Task */
  NET_TaskHandle = osThreadNew(StartNET_Task, NULL, &NET_Task_attributes);

  /* creation of RFID */
  RFIDHandle = osThreadNew(StartRFID_Task, NULL, &RFID_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMOTOR_Task */
/**
  * @brief  Function implementing the MOTOR_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMOTOR_Task */
void StartMOTOR_Task(void *argument)
{
  /* USER CODE BEGIN StartMOTOR_Task */
  MOTOR_TaskRun(argument);
  /* USER CODE END StartMOTOR_Task */
}
/* USER CODE BEGIN Header_StartNET_Task */
/**
* @brief Function implementing the NET_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNET_Task */
void StartNET_Task(void *argument)
{
  /* USER CODE BEGIN StartNET_Task */
  /* Infinite loop */
	TM_TaskRun(argument);
  /* USER CODE END StartNET_Task */
}

/* USER CODE BEGIN Header_StartRFID_Task */
/**
* @brief Function implementing the RFID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRFID_Task */
void StartRFID_Task(void *argument)
{
  /* USER CODE BEGIN StartRFID_Task */
  /* Infinite loop */
	 Rfid_Run();
  /* USER CODE END StartRFID_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

