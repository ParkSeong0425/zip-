/*
 * t_motor.h
 *
 *  Created on: Sep 11, 2026
 *      Author: HWNOT
 */
#ifndef INC_T_MOTOR_H_
#define INC_T_MOTOR_H_

#include "main.h"

/* TL57-R : UART2 RS485 */
#define TM_BAUD          115200
#define TM_MAX_RPM       3000

/* 각도 변환 : 1000 pulse/rev, 웜기어 20:1 */
#define TM_PULSE           1000
#define TM_GEAR            20
#define TM_START_RPM       0
#define TM_ACC_MS          500
#define TM_DEC_MS          500

/* HOME */
#define TM_HOME_MODE     24
#define TM_HOME_RPM      100
#define TM_CREEP_RPM     50

/* PA_004 Run state */
#define TM_DONE          (1 << 0)
#define TM_HOMED         (1 << 1)
#define TM_RUN           (1 << 2)
#define TM_FAULT         (1 << 3)
#define TM_ENABLE        (1 << 4)

/* 기본 기능 */
int TM_Init(void);
int TM_Move(int32_t deg, int percent);
int TM_C(int percent);
int TM_R(int32_t deg, int percent);
int TM_L(int32_t deg, int percent);
int TM_Done(void);
int TM_Home(void);
int TM_Stop(void);

/* 상태 읽기 */
void TM_State(uint16_t *state);
int TM_ID_COMM(uint16_t *state);
int TM_Pos(int32_t *deg);

/* 디버깅용 */
int TM_Read(uint16_t reg, uint16_t *val);

#endif /* INC_T_MOTOR_H_ */
