/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

typedef struct
{
    uint32_t Id;
    uint8_t  Data[8];
    uint8_t  Len;
} CanFrame;

extern osMessageQueueId_t CanQueue;

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

void CAN1_Start(void);
void CAN1_Tx_Data(uint32_t id, uint8_t *data, uint8_t len);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

