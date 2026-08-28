/*
 * fram.h
 *
 * 렉 한 칸 = X 위치 x_n 개 + Y 위치 y_n 개 + 틸트 4 개 (L, R, C, rpm).
 * 값은 전부 int 4바이트. X/Y 는 mm, 틸트는 엔코더 각.
 */
#ifndef FRAM_H
#define FRAM_H

#include "main.h"

typedef enum {
	IN_X = 1, IN_Y, OUT_X, OUT_Y
} Pos;

void fram_read(uint16_t addr, void *buf, uint16_t len);
void fram_write(uint16_t addr, void *buf, uint16_t len);

int fram_load(void);
int net_load(int *v);

int cfg_save(int rack, int inx, int iny, int outx, int outy);
void cfg_load(int *rack, int *inx, int *iny, int *outx, int *outy);

int pos_count(int axis);
int pos_save(int rack, int axis, int *v, int n);
int pos_load(int rack, int axis, int no, int *v);

int rot_save(int rack, int l, int r, int c, int rpm);
int rot_load(int rack, int *l, int *r, int *c, int *rpm);
void net_save(int *v);
#endif
