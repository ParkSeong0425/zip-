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

#define MAGIC     0x4653
#define CFG_ADDR  0
/*
 *  주소 0~1   MAGIC
	주소 2~3   rack
	주소 4~5   inx
	주소 6~7   iny
	주소 8~9   outx
	주소 10~11 outy
	주소 12~   위치 데이터 라서 ADDR은 12개
 */
#define DATA_ADDR 12
#define NET_ADDR  4096   /* TCP 설정. 위치 데이터와 안 겹치는 자리 */
#define NET_MAGIC 0x4E54
#define NET_N     6
#define FRAM_SIZE 8192
#define POS_N     8
#define AXIS_N    4      /* 입고 X,Y / 출고 X,Y */
#define ROT_N     4      /* L, R, C, rpm */
#define DATA_N    (POS_N * AXIS_N + ROT_N)

#define CS_LOW()  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET)

static int rack_n, inx_n, iny_n, outx_n, outy_n;

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

/* 렉 하나가 사용하는 바이트 */
static int rack_size(void) {
    return DATA_N * 4;
}

/* 렉, 축, 번호의 FRAM 주소 */
static uint16_t addr(int rack, int axis, int no) {
	uint16_t a = DATA_ADDR + (rack - 1) * rack_size();

	if (axis <= AXIS_N)
		return a + ((axis - 1) * POS_N + no - 1) * 4;

	return a + (AXIS_N * POS_N + no - 1) * 4;
}

/* 부팅할 때 저장된 설정을 불러온다 */
int fram_load(void) {
	uint16_t v[6];

	fram_read(CFG_ADDR, v, sizeof(v));

	if (v[0] != MAGIC || v[1] < 1
			|| v[2] < 2 || v[2] > 8
			|| v[3] < 2 || v[3] > 8
			|| v[4] < 2 || v[4] > 8
			|| v[5] < 2 || v[5] > 8)
		return 0;

	rack_n = v[1];
	inx_n = v[2];
	iny_n = v[3];
	outx_n = v[4];
	outy_n = v[5];

	return 1;
}

/* 01FS 설정 저장. 값이 바뀌면 위치값을 초기화한다 */
int cfg_save(int rack, int inx, int iny, int outx, int outy) {
	uint16_t v[6] = { MAGIC, rack, inx, iny, outx, outy };
	int zero[DATA_N] = { 0 };
	int changed = rack != rack_n
			|| inx != inx_n || iny != iny_n
			|| outx != outx_n || outy != outy_n;
	int i;

	if (rack < 1
			|| inx < 2 || inx > 8
			|| iny < 2 || iny > 8
			|| outx < 2 || outx > 8
			|| outy < 2 || outy > 8)
		return 0;

	fram_write(CFG_ADDR, v, sizeof(v));

	rack_n = rack;
	inx_n = inx;
	iny_n = iny;
	outx_n = outx;
	outy_n = outy;

	if (changed)
		for (i = 1; i <= rack_n; i++)
			fram_write(addr(i, IN_X, 1), zero, sizeof(zero));

	return 1;
}

/* 현재 설정값을 반환한다 */
void cfg_load(int *rack, int *inx, int *iny, int *outx, int *outy) {
	*rack = rack_n;
	*inx = inx_n;
	*iny = iny_n;
	*outx = outx_n;
	*outy = outy_n;
}

/* 축에 설정된 위치 개수를 반환한다 */
int pos_count(int axis) {
	switch (axis) {
	case IN_X:
		return inx_n;
	case IN_Y:
		return iny_n;
	case OUT_X:
		return outx_n;
	case OUT_Y:
		return outy_n;
	default:
		return 0;
	}
}
/* 해당 축의 모든 위치값을 저장한다 */
int pos_save(int rack, int axis, int *v, int n) {
    if (rack < 1 || rack > rack_n
            || axis < 1 || axis > AXIS_N || n != pos_count(axis))
        return 0;

    fram_write(addr(rack, axis, 1), v, sizeof(int) * n);
    return 1;
}

/* 저장된 위치값 하나를 읽는다 */
int pos_load(int rack, int axis, int no, int *v) {
	if (rack < 1 || rack > rack_n
			|| no < 1 || no > pos_count(axis))
		return 0;

	fram_read(addr(rack, axis, no), v, sizeof(int));
	return 1;
}

int rot_save(int rack, int l, int r, int c, int rpm) {
	int v[ROT_N] = { l, r, c, rpm };
	if (rack < 1 || rack > rack_n)
		return 0;
	fram_write(addr(rack, 5, 1), v, sizeof(v));
	return 1;
}
/* 저장된 L, R, C, rpm을 읽는다 */
int rot_load(int rack, int *l, int *r, int *c, int *rpm) {
	int v[ROT_N];
	if (rack < 1 || rack > rack_n)
		return 0;
	fram_read(addr(rack, 5, 1), v, sizeof(v));
	*l = v[0];
	*r = v[1];
	*c = v[2];
	*rpm = v[3];
	return 1;
}

/* TCP 설정을 읽는다. 저장된 게 없으면 0 */
int net_load(int *v) {
	uint16_t magic;

	fram_read(NET_ADDR, &magic, sizeof(magic));
	if (magic != NET_MAGIC)
		return 0;

	fram_read(NET_ADDR + 2, v, sizeof(int) * NET_N);
	return 1;
}

/* TCP 설정을 저장한다 */
void net_save(int *v) {
	uint16_t magic = NET_MAGIC;

	fram_write(NET_ADDR, &magic, sizeof(magic));
	fram_write(NET_ADDR + 2, v, sizeof(int) * NET_N);
}
