/*
 * motor.h
 *
 *  Created on: Jul 27, 2026
 *      Author: HWNOT
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

typedef struct {
	UART_HandleTypeDef *uart;
	GPIO_TypeDef *port;
	uint16_t pin;
} Motor;

extern Motor motorX;
extern Motor motorY;

/* 모두 1=성공 0=실패 */
int motor_init(Motor *m);
int motor_move(Motor *m, int rpm, int target);
int motor_stop(Motor *m);
int motor_pos(Motor *m, int *out);
int motor_zero(Motor *m);
int motor_home_on(Motor *m);
int motor_estop(Motor *m, int on);

#endif
