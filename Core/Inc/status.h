/*
 * status.h : 기계 작업 상태
 */
#ifndef INC_STATUS_H_
#define INC_STATUS_H_

#include "run.h"

#define WAIT   'W'      /* 대기 */
#define RUN    'R'      /* 운전 */
#define HOME   'I'      /* 원점복귀 */
#define PAUSE  'P'      /* 일시정지 */

extern char status_1;   /* 렉1 상태 */
extern char status_2;   /* 렉2 상태 */

/* 만재 : 0 = 빈칸, 1 = 만재 */
extern int full_1_1;    /* 렉1 1층 */
extern int full_1_2;
extern int full_1_3;
extern int full_1_4;

extern int full_2_1;    /* 렉2 1층 */
extern int full_2_2;
extern int full_2_3;
extern int full_2_4;

void Status_Start(int rack);
void Status_End(int command, int rack);

#endif
