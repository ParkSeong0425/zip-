/*
 * cfg.c
 *
 * 01FP 위치 저장, 01FI TCP 설정, 01FD 기본값.
 * IP 는 int 하나에 네 칸을 담는다. 172.20.0.101 -> 0xAC140033
 */
#include "cfg.h"
#include "net.h"
#include "fram.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define NET_N   6   /* SIP SN GW DIP PORT MODE */
#define VALUE_N 10  /* 01FP 한 줄의 최대 숫자 개수 */

static int net_value[NET_N];

static const int net_start[NET_N] = {
	(172 << 24) | (20 << 16) | (0 << 8) | 101,
	(255 << 24) | (255 << 16) | (255 << 8) | 0,
	(0 << 24) | (0 << 16) | (0 << 8) | 0,
	(172 << 24) | (20 << 16) | (0 << 8) | 100,
	2500,
	0
};

void ack(const char *s) {
	char b[64];
	snprintf(b, sizeof(b), "01%c%s", ACK, s);
	reply(b);
}

void nak(const char *s) {
	char b[64];
	snprintf(b, sizeof(b), "01%c%s", NAK, s);
	reply(b);
}

/* 밑줄로 나눈 숫자를 최대 n 개 읽는다 */
static int split(char *s, int *v, int n) {
	int count = 0;

	while (*s && count < n) {
		v[count++] = atoi(s);
		while (*s && *s != '_') s++;
		if (*s) s++;
	}
	return count;
}

/* 세 번째 칸의 시작 위치. 01FI_1_1_172.20.0.51 의 172 자리 */
static char *third(char *s) {
	int bar = 0;

	for (; *s; s++)
		if (*s == '_' && ++bar == 3)
			return s + 1;
	return 0;
}

/* 저장된 설정을 불러온다. 없으면 기본값을 쓴다 */
void cfg_init(void) {
	if (!net_load(net_value)) {
		memcpy(net_value, net_start, sizeof(net_value));
		net_save(net_value);
	}
}

/* 1 SIP 2 SN 3 GW 4 DIP 5 PORT 6 MODE */
int cfg_net(int no) {
	if (no < 1 || no > NET_N)
		return 0;
	return net_value[no - 1];
}

/* 01FP_1_1_230_950_0_0 : 렉 1 입고 X 위치 */
static void pos_cmd(char *s) {
	int v[VALUE_N], count, axis;

	count = split(s + 5, v, VALUE_N);
	if (count < 3) { nak("bad_data"); return; }

	if (v[1] == 3) {
		if (count < 4 || !rot_save(v[0], 0, v[3], 0, 0)) {
			nak("fram_data"); return;
		}
	} else {
		axis = v[1] == 1 ? IN_X : v[1] == 2 ? IN_Y
				: v[1] == 4 ? OUT_X : v[1] == 5 ? OUT_Y : 0;
		if (!axis || !pos_save(v[0], axis, v + 2, count - 2)) {
			nak("fram_data"); return;
		}
	}
	ack(s + 2);
}

/* 01FI_1_1_172.20.0.51 : 자기 IP. 5 는 포트, 6 은 모드 */
static void net_cfg_cmd(char *s) {
	int rack, no, a, b, c, d;
	char *p = third(s);

	if (!p || sscanf(s + 5, "%d_%d", &rack, &no) != 2
			|| rack != 1 || no < 1 || no > NET_N) {
		nak("bad_data"); return;
	}

	if (no <= 4) {
		if (sscanf(p, "%d.%d.%d.%d", &a, &b, &c, &d) != 4
				|| a > 255 || b > 255 || c > 255 || d > 255) {
			nak("bad_data"); return;
		}
		net_value[no - 1] = (a << 24) | (b << 16) | (c << 8) | d;
	} else {
		net_value[no - 1] = atoi(p);
	}

	net_save(net_value);
	ack(s + 2);
}

/* 지금 설정값을 여섯 줄로 보낸다 */
void cfg_show(void) {
	char b[40];
	int i, v;

	for (i = 0; i < 4; i++) {
		v = net_value[i];
		snprintf(b, sizeof(b), "01FI_1_%d_%d.%d.%d.%d", i + 1,
				(v >> 24) & 255, (v >> 16) & 255, (v >> 8) & 255, v & 255);
		reply(b);
	}
	snprintf(b, sizeof(b), "01FI_1_5_%d", net_value[4]); reply(b);
	snprintf(b, sizeof(b), "01FI_1_6_%d", net_value[5]); reply(b);
}

/* 01FP, 01FI, 01FD 를 처리한다 */
void cfg_cmd(char *s) {
	if (!strncmp(s, "01FP_", 5)) {
		pos_cmd(s);
	} else if (!strncmp(s, "01FI_", 5)) {
		net_cfg_cmd(s);
	} else if (!strcmp(s, "01FD_1")) {
		memcpy(net_value, net_start, sizeof(net_value));
		net_save(net_value);
		ack(s + 2);
		cfg_show();
	} else {
		nak("bad_cmd");
	}
}
