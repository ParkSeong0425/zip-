/*
 * net.h
 *
 *  Created on: Aug 3, 2026
 *      Author: HWNOT
 */
#ifndef INC_NET_H_
#define INC_NET_H_

#include "main.h"

typedef enum {
	STATE_W = 'W', STATE_R = 'R', STATE_I = 'I', STATE_P = 'P'
} State;

typedef enum {
	STEP_STOP, STEP_XY, STEP_TILT, STEP_TURN, STEP_BACK, STEP_CENTER
} Step;

typedef struct {
	int x, y, rpm;            /* X/Y 목표 mm 와 속도 */
	int rot, center, rot_rpm; /* 틸트 목표와 정면과 속도 */
	int dir;                  /* -1 입고 +1 출고 */
	int wait, rx_wait;
	int lastx, lasty;
} Job;


/* freertos.c의 StartUART_Task()가 호출한다.
   W6100을 올리고 TCP 명령을 받아 모터를 돌린다 */
void reply(const char *s);
void print(const char *s);
void TM_TaskRun(void *arg);
void MotorTaskRun(void *arg);
void print(const char *s);
void net_cmd(char *s);

extern volatile uint8_t rfid_check;

#endif

