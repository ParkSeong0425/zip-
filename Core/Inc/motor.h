#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

typedef struct {
    UART_HandleTypeDef *uart;
    GPIO_TypeDef *port;
    uint16_t pin;
    int off;
} Motor;

extern Motor motorX;
extern Motor motorY;

#define JOG_SPEED  0x0604
#define JOG_RPM    100

void print(const char *s);
void motor_sync(Motor *m, int last);

int motor_check(Motor *m);
int motor_init(Motor *m, int dir);
int motor_pos(Motor *m, int *out);
int motor_move(Motor *m, int rpm, int pos);
int motor_wait(Motor *mx,int xt,Motor *my,int yt,int timeout);
int motor_speed_mode(Motor *m);
int motor_speed(Motor *m,int rpm);
int motor_pos_mode(Motor *m);
int motor_home_on(Motor *m);
int motor_zero(Motor *m);
int motor_begin(Motor *m, int v);
int motor_di5(Motor *m, int v);
int motor_stop(Motor *m);
int motor_write(Motor *m, uint16_t reg, uint16_t val);

#endif
