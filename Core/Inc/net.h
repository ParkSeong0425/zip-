/*
 * net.h
 *
 *  Created on: Aug 3, 2026
 *      Author: HWNOT
 */
#ifndef INC_NET_H_
#define INC_NET_H_

#include "main.h"

/* freertos.c의 StartUART_Task()가 호출한다.
   W6100을 올리고 TCP 명령을 받아 모터를 돌린다 */
void TM_TaskRun(void *arg);
void MotorTaskRun(void *arg);
void print(const char *s);
#endif

