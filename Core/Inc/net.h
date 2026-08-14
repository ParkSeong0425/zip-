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

/* READY 에서 0도 정렬을 끝낸 뒤 XY 로 간다 */
typedef enum {
	STEP_STOP,
	STEP_READY,
	STEP_XY,
	STEP_TILT,
	STEP_TURN,
	STEP_BACK,
	STEP_CENTER
} Step;

/* Move.mode 값. 숫자는 바꾸지 말 것.
   IN/OUT 의 -1/+1 은 틸트 각도 부호로 쓰고,
   RIGHT 이상은 수동 회전 판정에 쓴다 */
typedef enum {
	MODE_HOME = 0,
	MODE_IN = -1,
	MODE_OUT = 1,
	MODE_X = 2,
	MODE_Y = 3,
	MODE_RIGHT = 4,
	MODE_LEFT = 5,
	MODE_CENTER = 6
} Mode;

typedef struct {
	int x_mm, y_mm, speed;       /* X/Y 목표 mm 와 속도 */
	int tilt, tilt_speed;        /* 틸트 목표 펄스와 속도 */
	int mode;                    /* 어떤 명령인지. Mode 참고 */
	int tilt_delay, back_delay;  /* 틸트 전 지연, 복귀 전 지연 */
	int prev_x, prev_y;          /* 응답에 실을 열과 단 */
} Move;

/* freertos.c의 StartUART_Task()가 호출한다.
   W6100을 올리고 TCP 명령을 받아 모터를 돌린다 */
void reply(const char *s);
void print(const char *s);
void TM_TaskRun(void *arg);
void MotorTaskRun(void *arg);
void net_cmd(char *s);

extern volatile uint8_t rfid_check;

#endif
