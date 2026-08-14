/*
 * rfid.h  -  수신 보드 (STM32F4, CMSIS-RTOS v2)
 */

#ifndef INC_RFID_H_
#define INC_RFID_H_

#include "main.h"
#include "cmsis_os.h"

extern volatile int card_ok;

void Rfid_Run(void);
void Rfid_Request(void);
#endif
