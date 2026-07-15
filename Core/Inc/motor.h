#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

typedef struct {
	UART_HandleTypeDef *uart;
	GPIO_TypeDef *port;
	uint16_t pin;
	int off;
} Motor;

// extern Motor motorX;   /* X axis unused */
extern Motor motorY;

// DI 기능 레지스터
#define R_DI1   0x0302
#define R_DI1L  0x0303
#define R_DI3   0x0306
#define R_DI3L  0x0307

// DI9 = H03_18 / H03_19
#define R_DI9   0x0312
#define R_DI9L  0x0313

#define JOG_SPEED 0x0604
#define JOG_RPM 100

void print(const char *s);
void motor_sync(Motor *m, int last);

int motor_check(Motor *m);
int motor_init(Motor *m, int dir);
int motor_pos(Motor *m, int *out);
int motor_move(Motor *m, int rpm, int pos);
int motor_wait(Motor *mx, int xtarget, Motor *my, int ytarget, int timeout);
int motor_begin(Motor *m, int v);
int motor_di5(Motor *m, int v);
int motor_stop(Motor *m);
int motor_write(Motor *m, uint16_t reg, uint16_t val);

// mode: 0 = 정방향 홈서치, 1 = 역방향 홈서치
int motor_home(Motor *m, int mode);

#endif
