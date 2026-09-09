/*
 * fram.c
 *
 * MB85RS64 8KB
 *
 * FS     : 장비 구조
 * FP     : 위치 / tilt
 * OFFSET : FP 6~8
 * FI     : Ethernet / TCP 설정
 */
#include "fram.h"

extern SPI_HandleTypeDef hspi3;

#define WREN      0x06
#define WRITE     0x02
#define READ      0x03

/* FS 영역 */
#define FS_MAGIC  0x4653
#define FS_ADDR   0

/*
 * 0~1   FS_MAGIC
 * 2~3   rack
 * 4~5   inx
 * 6~7   iny
 * 8~9   outx
 * 10~11 outy
 * 12~   위치 데이터
 */
#define DATA_ADDR 12

/* FI 영역 */
#define NET_ADDR       4096
#define NET_MAGIC_OLD  0x4E54   /* 기존 FI 1~6 */
#define NET_MAGIC      0x4E55   /* 새 FI 1~7 */
#define NET_OLD_N      6
#define NET_N          7

/* FP 6~8 OFFSET 영역. 기존 위치 주소를 건드리지 않는다 */
#define OFFSET_ADDR    4200
#define OFFSET_MAGIC   0x4F46
#define OFFSET_N       3

#define FRAM_SIZE 8192
#define POS_N     8
#define AXIS_N    4      /* IN_X, IN_Y, OUT_X, OUT_Y */
#define ROT_N     4      /* L, R, C, rpm */
#define DATA_N    (POS_N * AXIS_N + ROT_N)

#define CS_LOW()  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET)

static int rack_n, inx_n, iny_n, outx_n, outy_n;

void fram_read(uint16_t addr, void *buf, uint16_t len)
{
	uint8_t cmd[3] = { READ, addr >> 8, addr };

	CS_LOW();
	HAL_SPI_Transmit(&hspi3, cmd, 3, 100);
	HAL_SPI_Receive(&hspi3, buf, len, 100);
	CS_HIGH();
}

void fram_write(uint16_t addr, void *buf, uint16_t len)
{
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

/* 렉 하나가 사용하는 기존 FP 위치 바이트 */
static int rack_size(void)
{
	return DATA_N * 4;
}

/* 기존 FP 1~5 주소 */
static uint16_t addr(int rack, int axis, int no)
{
	uint16_t a = DATA_ADDR + (rack - 1) * rack_size();

	if (axis <= AXIS_N)
		return a + ((axis - 1) * POS_N + no - 1) * 4;

	return a + (AXIS_N * POS_N + no - 1) * 4;
}

/* FP 6~8은 렉마다 magic + int 3개를 따로 사용한다 */
static uint16_t offset_addr(int rack)
{
	return OFFSET_ADDR + (rack - 1) * (2 + OFFSET_N * 4);
}

/* ========================= FS ========================= */
/* 부팅할 때 FRAM에 저장된 FS를 읽는다 */
int fram_load(void)
{
	uint16_t v[6];

	fram_read(FS_ADDR, v, sizeof(v));

	if (v[0] != FS_MAGIC || v[1] < 1
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

/* FS 저장. 구조가 바뀌면 기존 FP 1~5 위치값을 초기화한다 */
int fs_save(int rack, int inx, int iny, int outx, int outy)
{
	uint16_t v[6] = { FS_MAGIC, rack, inx, iny, outx, outy };
	int zero[DATA_N] = { 0 };
	int changed;
	int i;

	if (rack < 1
			|| inx < 2 || inx > 8
			|| iny < 2 || iny > 8
			|| outx < 2 || outx > 8
			|| outy < 2 || outy > 8)
		return 0;

	changed = rack != rack_n
			|| inx != inx_n || iny != iny_n
			|| outx != outx_n || outy != outy_n;

	fram_write(FS_ADDR, v, sizeof(v));

	rack_n = rack;
	inx_n = inx;
	iny_n = iny;
	outx_n = outx;
	outy_n = outy;

	if (changed) {
		for (i = 1; i <= rack_n; i++)
			fram_write(addr(i, IN_X, 1), zero, sizeof(zero));
	}

	return 1;
}

/* 현재 FS 값을 반환한다 */
void fs_load(int *rack, int *inx, int *iny, int *outx, int *outy)
{
	*rack = rack_n;
	*inx = inx_n;
	*iny = iny_n;
	*outx = outx_n;
	*outy = outy_n;
}

/* 기존 이름을 쓰는 파일이 있어도 바로 깨지지 않게 둔다 */
int cfg_save(int rack, int inx, int iny, int outx, int outy)
{
	return fs_save(rack, inx, iny, outx, outy);
}

void cfg_load(int *rack, int *inx, int *iny, int *outx, int *outy)
{
	fs_load(rack, inx, iny, outx, outy);
}

/* ========================= FP 1~5 ========================= */
int pos_count(int axis)
{
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
int pos_save(int rack, int axis, int *v, int n)
{
	if (rack < 1 || rack > rack_n
			|| axis < 1 || axis > AXIS_N
			|| n != pos_count(axis))
		return 0;

	fram_write(addr(rack, axis, 1), v, sizeof(int) * n);
	return 1;
}

/* 저장된 위치값 하나를 읽는다 */
int pos_load(int rack, int axis, int no, int *v)
{
	if (rack < 1 || rack > rack_n
			|| no < 1 || no > pos_count(axis))
		return 0;

	fram_read(addr(rack, axis, no), v, sizeof(int));
	return 1;
}

/* 기존 FP3 tilt 데이터 저장 */
int rot_save(int rack, int l, int r, int c, int rpm)
{
	int v[ROT_N] = { l, r, c, rpm };

	if (rack < 1 || rack > rack_n)
		return 0;

	fram_write(addr(rack, 5, 1), v, sizeof(v));
	return 1;
}

/* 저장된 L, R, C, rpm을 읽는다 */
int rot_load(int rack, int *l, int *r, int *c, int *rpm)
{
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

/* ========================= FP 6~8 OFFSET ========================= */
/* no: 6=Y offset, 7=tilt offset, 8=X offset */
int offset_save(int rack, int no, int value)
{
	uint16_t base;
	uint16_t magic;
	int zero[OFFSET_N] = { 0 };

	if (rack < 1 || rack > rack_n || no < 6 || no > 8)
		return 0;

	base = offset_addr(rack);
	fram_read(base, &magic, sizeof(magic));

	/* 이 렉의 OFFSET을 처음 쓰면 나머지 OFFSET도 0으로 만든다 */
	if (magic != OFFSET_MAGIC) {
		magic = OFFSET_MAGIC;
		fram_write(base, &magic, sizeof(magic));
		fram_write(base + 2, zero, sizeof(zero));
	}

	fram_write(base + 2 + (no - 6) * 4, &value, sizeof(value));
	return 1;
}

/* 저장되지 않은 OFFSET은 기본값 0으로 읽는다 */
int offset_load(int rack, int no, int *value)
{
	uint16_t base;
	uint16_t magic;

	if (rack < 1 || rack > rack_n || no < 6 || no > 8)
		return 0;

	base = offset_addr(rack);
	fram_read(base, &magic, sizeof(magic));

	if (magic != OFFSET_MAGIC) {
		*value = 0;
		return 1;
	}

	fram_read(base + 2 + (no - 6) * 4, value, sizeof(int));
	return 1;
}

/* ========================= FI ========================= */
/* FI 설정을 읽는다. 기존 6개 형식이면 7번 timeout만 20000으로 붙인다 */
int net_load(int *v)
{
	uint16_t magic;

	fram_read(NET_ADDR, &magic, sizeof(magic));

	if (magic == NET_MAGIC) {
		fram_read(NET_ADDR + 2, v, sizeof(int) * NET_N);
		return 1;
	}

	if (magic == NET_MAGIC_OLD) {
		fram_read(NET_ADDR + 2, v, sizeof(int) * NET_OLD_N);
		v[6] = 20000;
		return 1;
	}

	return 0;
}

/* FI 1~7 저장 */
void net_save(int *v)
{
	uint16_t magic = NET_MAGIC;

	fram_write(NET_ADDR, &magic, sizeof(magic));
	fram_write(NET_ADDR + 2, v, sizeof(int) * NET_N);
}
