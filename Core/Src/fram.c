/*
 * fram.c
 *
 * MB85RS64 8KB.
 * 0 ~ 7   : 설정 (magic, 렉 수, X 칸 수, Y 칸 수)
 * 8 ~     : 렉 1 부터 차례로. 렉 하나 = X 칸 + Y 칸 + 틸트 4 개.
 */
#include "fram.h"

extern SPI_HandleTypeDef hspi3;

#define WREN      0x06
#define WRITE     0x02
#define READ      0x03

#define MAGIC     0x4652
#define CFG_ADDR  0
#define DATA_ADDR 8
#define FRAM_SIZE 8192
#define ROT_N     4      /* L, R, C, rpm */

#define CS_LOW()  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET)

static int rack_n, x_n, y_n;

void fram_read(uint16_t addr, void *buf, uint16_t len) {
	uint8_t cmd[3] = { READ, addr >> 8, addr };
	CS_LOW();
	HAL_SPI_Transmit(&hspi3, cmd, 3, 100);
	HAL_SPI_Receive(&hspi3, buf, len, 100);
	CS_HIGH();
}

void fram_write(uint16_t addr, void *buf, uint16_t len) {
	uint8_t wren = WREN;
	uint8_t cmd[3] = { WRITE, addr >> 8, addr };
	CS_LOW();
	HAL_SPI_Transmit(&hspi3, &wren, 1, 100);
	CS_HIGH();
	CS_LOW();
	HAL_SPI_Transmit(&hspi3, cmd, 3, 100);
	HAL_SPI_Transmit(&hspi3, buf, len, 100);
	CS_HIGH();
}

/* 렉 한 칸이 차지하는 바이트 */
static int rack_size(int nx, int ny) {
	return (nx + ny + ROT_N) * 4;
}

/* axis 1=X, 2=Y, 3=틸트 */
static uint16_t addr(int rack, int axis, int no) {
	uint16_t a = DATA_ADDR + (rack - 1) * rack_size(x_n, y_n);
	if (axis == 1)
		return a + (no - 1) * 4;
	if (axis == 2)
		return a + (x_n + no - 1) * 4;
	return a + (x_n + y_n + no - 1) * 4;
}

/* 부팅 때 한 번. 01SAVE 를 한 적이 없으면 0 */
int fram_load(void) {
	uint16_t v[4];
	fram_read(CFG_ADDR, v, sizeof(v));
	if (v[0] != MAGIC || v[1] < 1 || v[2] < 1 || v[3] < 1
			|| DATA_ADDR + v[1] * rack_size(v[2], v[3]) > FRAM_SIZE)
		return 0;
	rack_n = v[1];
	x_n = v[2];
	y_n = v[3];
	return 1;
}

/* 01SAVE. 렉 수와 X/Y 칸 수를 정한다. 틸트 값은 0 으로 지운다 */
int cfg_save(int rack, int nx, int ny) {
	uint16_t v[4] = { MAGIC, rack, nx, ny };
	int zero[ROT_N] = { 0 };
	int i;
	if (rack < 1 || nx < 1 || ny < 1
			|| DATA_ADDR + rack * rack_size(nx, ny) > FRAM_SIZE)
		return 0;
	fram_write(CFG_ADDR, v, sizeof(v));
	rack_n = rack;
	x_n = nx;
	y_n = ny;
	for (i = 1; i <= rack_n; i++)
		fram_write(addr(i, 3, 1), zero, sizeof(zero));
	return 1;
}

void cfg_load(int *rack, int *nx, int *ny) {
	*rack = rack_n;
	*nx = x_n;
	*ny = y_n;
}

int pos_count(int axis) {
	if (axis == 1)
		return x_n;
	if (axis == 2)
		return y_n;
	return 0;
}

/*
 * n=1 이면 v[0] 을 1번 칸에만 넣는다.
 * n=2 면 v[0], v[1] 간격으로 마지막 칸까지 자동으로 채운다.
 */
int pos_save(int rack, int axis, int *v, int n) {
	int count = pos_count(axis);
	int gap, p, i;
	if (rack < 1 || rack > rack_n || count < 1 || n < 1)
		return 0;
	if (n < 2) {
		fram_write(addr(rack, axis, 1), &v[0], 4);
		return 1;
	}
	gap = v[1] - v[0];
	for (i = 0; i < count; i++) {
		p = v[0] + gap * i;
		fram_write(addr(rack, axis, i + 1), &p, 4);
	}
	return 1;
}

int pos_load(int rack, int axis, int no, int *v) {
	if (rack < 1 || rack > rack_n || no < 1 || no > pos_count(axis))
		return 0;
	fram_read(addr(rack, axis, no), v, 4);
	return 1;
}

int rot_save(int rack, int l, int r, int c, int rpm) {
	int v[ROT_N] = { l, r, c, rpm };
	if (rack < 1 || rack > rack_n)
		return 0;
	fram_write(addr(rack, 3, 1), v, sizeof(v));
	return 1;
}

int rot_load(int rack, int *l, int *r, int *c, int *rpm) {
	int v[ROT_N];
	if (rack < 1 || rack > rack_n)
		return 0;
	fram_read(addr(rack, 3, 1), v, sizeof(v));
	*l = v[0];
	*r = v[1];
	*c = v[2];
	*rpm = v[3];
	return 1;
}
