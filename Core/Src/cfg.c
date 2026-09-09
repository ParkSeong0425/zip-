/*
 * cfg.c
 *
 * FS : 장비 구조 설정
 * FP : 위치 / OFFSET 설정
 * FI : Ethernet / TCP 설정
 * FD : FI 기본값 복구
 */
#include "cfg.h"
#include "net.h"
#include "fram.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define NET_N   7   /* SIP SN GW DIP PORT MODE RX_TIMEOUT */
#define VALUE_N 10  /* 01FP 한 줄의 최대 숫자 개수 */

/* fram.h 정리 전에도 사용할 수 있도록 새 FRAM 함수만 선언 */
int fs_save(int rack, int inx, int iny, int outx, int outy);
void fs_load(int *rack, int *inx, int *iny, int *outx, int *outy);
int offset_save(int rack, int no, int value);
int offset_load(int rack, int no, int *value);

static int net_value[NET_N];

static const int net_start[NET_N] = {
	(172 << 24) | (20 << 16) | (0 << 8) | 101,
	(255 << 24) | (255 << 16) | (255 << 8) | 0,
	(0 << 24) | (0 << 16) | (0 << 8) | 0,
	(172 << 24) | (20 << 16) | (0 << 8) | 100,
	2500,
	0,
	20000
};

void ack(const char *s)
{
	char b[64];
	snprintf(b, sizeof(b), "01%c%s", ACK, s);
	reply(b);
}

void nak(const char *s)
{
	char b[64];
	snprintf(b, sizeof(b), "01%c%s", NAK, s);
	reply(b);
}

/* 밑줄로 나눈 숫자를 최대 n개 읽는다 */
static int split(char *s, int *v, int n)
{
	int count = 0;

	while (*s && count < n) {
		v[count++] = atoi(s);
		while (*s && *s != '_') s++;
		if (*s) s++;
	}

	return count;
}

/* ========================= FS ========================= */
/* 01FS_1_rack_inx_iny_outx_outy */
static void FS(int rack, int inx, int iny, int outx, int outy)
{
	fs_save(rack, inx, iny, outx, outy);
}

/* 01FS_1 조회 */
static void FS_show(void)
{
	char b[64];
	int rack, inx, iny, outx, outy;

	fs_load(&rack, &inx, &iny, &outx, &outy);

	snprintf(b, sizeof(b), "01FS_1_%d_%d_%d_%d_%d",
			rack, inx, iny, outx, outy);
	reply(b);
}

/* ========================= FP ========================= */
/*
 * 1 IN_X
 * 2 IN_Y
 * 3 TILT 기존값
 * 4 OUT_X
 * 5 OUT_Y
 * 6 Y(lift) offset
 * 7 tilt offset
 * 8 X(travel) offset
 */
static int FP(int rack, int no, int *v, int count)
{
	switch (no) {
	case 1:
		return pos_save(rack, IN_X, v, count);

	case 2:
		return pos_save(rack, IN_Y, v, count);

	case 3:
		/* 기존 FP3 동작 유지: 두 번째 데이터가 tilt R 값 */
		if (count < 2)
			return 0;
		return rot_save(rack, 0, v[1], 0, 0);

	case 4:
		return pos_save(rack, OUT_X, v, count);

	case 5:
		return pos_save(rack, OUT_Y, v, count);

	case 6:
	case 7:
	case 8:
		if (count != 1)
			return 0;
		return offset_save(rack, no, v[0]);

	default:
		return 0;
	}
}

/* ========================= FI ========================= */
/* 1 SIP 2 SN 3 GW 4 DIP 5 PORT 6 MODE 7 RX_TIMEOUT */
static void FI(int no, int value)
{
	net_value[no - 1] = value;
	net_save(net_value);
}

/* ========================= FD ========================= */
/* 현재 FD는 기존과 동일하게 FI만 기본값으로 복구한다 */
static void FD(void)
{
	memcpy(net_value, net_start, sizeof(net_value));
	net_save(net_value);
}

/* 저장된 FI 설정을 불러온다. 없으면 기본값을 쓴다 */
void cfg_init(void)
{
	if (!net_load(net_value)) {
		memcpy(net_value, net_start, sizeof(net_value));
		net_save(net_value);
	}
}

/* FI 설정값 하나를 반환한다 */
int cfg_net(int no)
{
	if (no < 1 || no > NET_N)
		return 0;

	return net_value[no - 1];
}

/* FI 현재 설정값을 1~7까지 보낸다 */
void cfg_show(void)
{
	char b[40];
	int i, v;

	for (i = 0; i < 4; i++) {
		v = net_value[i];
		snprintf(b, sizeof(b), "01FI_1_%d_%d.%d.%d.%d", i + 1,
				(v >> 24) & 255, (v >> 16) & 255,
				(v >> 8) & 255, v & 255);
		reply(b);
	}

	snprintf(b, sizeof(b), "01FI_1_5_%d", net_value[4]);
	reply(b);

	snprintf(b, sizeof(b), "01FI_1_6_%d", net_value[5]);
	reply(b);

	snprintf(b, sizeof(b), "01FI_1_7_%d", net_value[6]);
	reply(b);
}

/* FS / FP / FI / FD 프로토콜을 숫자로 나눈 뒤 각 함수에 넘긴다 */
void cfg_cmd(char *s)
{
	int rack, inx, iny, outx, outy;
	int v[VALUE_N], count;
	int no, value;
	int a, b, c, d;

	/* FS 조회 */
	if (!strcmp(s, "01FS_1")) {
		FS_show();
		return;
	}

	/* FS 저장 */
	if (!strncmp(s, "01FS_", 5)) {
		if (sscanf(s, "01FS_1_%d_%d_%d_%d_%d",
				&rack, &inx, &iny, &outx, &outy) != 5
				|| rack < 1
				|| inx < 2 || inx > 8
				|| iny < 2 || iny > 8
				|| outx < 2 || outx > 8
				|| outy < 2 || outy > 8) {
			nak("bad_data");
			return;
		}

		FS(rack, inx, iny, outx, outy);
		ack(s + 2);
		return;
	}

	/* FP 저장 */
	if (!strncmp(s, "01FP_", 5)) {
		count = split(s + 5, v, VALUE_N);

		if (count < 3) {
			nak("bad_data");
			return;
		}

		if (!FP(v[0], v[1], v + 2, count - 2)) {
			nak("fram_data");
			return;
		}

		ack(s + 2);
		return;
	}

	/* FI IP 설정 1~4 */
	if (!strncmp(s, "01FI_", 5)
			&& sscanf(s, "01FI_1_%d_%d.%d.%d.%d",
					&no, &a, &b, &c, &d) == 5) {

		if (no < 1 || no > 4
				|| a < 0 || a > 255
				|| b < 0 || b > 255
				|| c < 0 || c > 255
				|| d < 0 || d > 255) {
			nak("bad_data");
			return;
		}

		value = (a << 24) | (b << 16) | (c << 8) | d;
		FI(no, value);
		ack(s + 2);
		return;
	}

	/* FI PORT / MODE / RX TIMEOUT */
	if (!strncmp(s, "01FI_", 5)
			&& sscanf(s, "01FI_1_%d_%d", &no, &value) == 2) {

		if (no < 5 || no > 7) {
			nak("bad_data");
			return;
		}

		FI(no, value);
		ack(s + 2);
		return;
	}

	/* FD */
	if (!strcmp(s, "01FD_1")) {
		FD();
		ack(s + 2);
		cfg_show();
		return;
	}

	nak("bad_cmd");
}
