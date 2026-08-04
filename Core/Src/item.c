/*
 * item.c
 *
 *  Created on: Jul 21, 2026
 *      Author: kotec
 */
#include "item.h"
#include "net.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_X    8
#define MAX_Y    8

#define FIRST    'a'
#define LAST     'g'
#define CODE_N   (LAST - FIRST + 1)

#define MAX_PER  10     /* �� ĭ�� �� �ִ� ���� */
#define QUEUE_N  16     /* �̸� ������ �� �ִ� ���� */

/* ===== ���� ===== */

/* cnt[��][��][�ڵ�] = �� ĭ�� ���� ����. ������ ���� �������� */
static uint8_t cnt[MAX_Y][MAX_X][CODE_N];

/* �̸� ������ ���� */
static char queue[QUEUE_N];
static int q_head, q_tail;

/* ���� �̵����� ��. 0�̸� ���� */
static char now_code;
static int now_col, now_dan;

static int seeded;

static void reply(const char *s) {
	tcp_reply(s);
}

static int ok_code(char c) {
	return c >= FIRST && c <= LAST;
}

/* ===== ���� ť ===== */

static int q_empty(void) {
	return q_head == q_tail;
}

static void q_clear(void) {
	q_head = q_tail = 0;
}

static int q_push(char c) {
	int next = (q_tail + 1) % QUEUE_N;

	if (next == q_head)
		return 0; /* �� �� */

	queue[q_tail] = c;
	q_tail = next;
	return 1;
}

static char q_pop(void) {
	char c = queue[q_head];

	q_head = (q_head + 1) % QUEUE_N;
	return c;
}

/* ===== ��ġ ===== */

/* ��� ĭ�� �������� ä���. ������ ���ĺ��� �����̰� �ߺ��ȴ� */
static void fill_all(void) {
	int xn = save_grid_x();
	int yn = save_grid_y();

	if (!seeded) {
		srand(HAL_GetTick());
		seeded = 1;
	}

	memset(cnt, 0, sizeof(cnt));

	for (int d = 0; d < yn; d++) {
		for (int c = 0; c < xn; c++) {
			int n = rand() % (MAX_PER + 1);

			for (int i = 0; i < n; i++)
				cnt[d][c][rand() % CODE_N]++;
		}
	}
}

/*
 * 1,1���� ���� ����� ĭ�� ã�´�.
 * �Ÿ��� (��-1)^2 + (��-1)^2. ������ �Ʒ� ��, ���� ���� ������.
 */
static int nearest(char code, int *col, int *dan) {
	int i = code - FIRST;
	int best = -1;

	for (int d = 0; d < save_grid_y(); d++) {
		for (int c = 0; c < save_grid_x(); c++) {
			int dist = c * c + d * d;

			if (!cnt[d][c][i])
				continue;

			if (best < 0 || dist < best) {
				best = dist;
				*col = c + 1;
				*dan = d + 1;
			}
		}
	}
	return best >= 0;
}

/* ===== ��� ===== */

/* ĭ �ϳ��� aabcddeee ���·� ����� */
static void cell_text(int d, int c, char *out) {
	int k = 0;

	for (int i = 0; i < CODE_N; i++)
		for (int j = 0; j < cnt[d][c][i]; j++)
			out[k++] = FIRST + i;

	out[k] = 0;
}

/* ĭ���� �� �پ� ����Ѵ� */
static void show_map(void) {
	char text[MAX_PER + 1];
	char msg[48];

	for (int d = 0; d < save_grid_y(); d++) {
		for (int c = 0; c < save_grid_x(); c++) {
			cell_text(d, c, text);
			snprintf(msg, sizeof(msg), "%d %d: %s\r\n", c + 1, d + 1, text);
			reply(msg);
		}
	}
}

/* ===== ���� ===== */

void item_init(void) {
	memset(cnt, 0, sizeof(cnt));
	q_clear();
	now_code = 0;
	seeded = 0;
}

/*
 * ����Ŭ�� �������� Ȯ���ϰ�, �������� ���� ������ �����Ѵ�.
 * ����Ŭ�� �������� ���� �ڿ��� �� ĭ���� �ϳ��� ����.
 */
void item_run(void) {
	char msg[48];
	char code;
	int col, dan, r;

	/* �̵����̸� ���� ������ ��ٸ��� */
	if (now_code) {
		if (save_busy())
			return;

		r = save_result();
		if (!r)
			return;

		if (r > 0) {
			/* ROT�� C�� ���ƿ����� �� ĭ���� �ϳ� ���� */
			cnt[now_dan - 1][now_col - 1][now_code - FIRST]--;
			snprintf(msg, sizeof(msg), "DONE %c %d %d\r\n", now_code, now_col,
					now_dan);
		} else {
			q_clear(); /* �����ϸ� ������ ����� */
			snprintf(msg, sizeof(msg), "FAIL %c %d %d\r\n", now_code, now_col,
					now_dan);
		}

		reply(msg);
		now_code = 0;
		return;
	}

	/* ������ ������ ���� ���� �����Ѵ� */
	if (q_empty() || save_busy())
		return;

	code = q_pop();

	if (!nearest(code, &col, &dan)) {
		snprintf(msg, sizeof(msg), "NONE %c\r\n", code);
		reply(msg);
		return;
	}

	if (!save_go(col, dan)) {
		q_clear();
		snprintf(msg, sizeof(msg), "ERR GO %c %d %d\r\n", code, col, dan);
		reply(msg);
		return;
	}

	now_code = code;
	now_col = col;
	now_dan = dan;

	snprintf(msg, sizeof(msg), "MOVE %c %d %d\r\n", code, col, dan);
	reply(msg);
}

int item_cmd(char *cmd) {
	char msg[32];

	/* start : ��ü�� �������� ä��� �����ش� */
	if (!strcmp(cmd, "start")) {
		if (save_busy()) {
			reply("ERR RUNNING\r\n");
			return 1;
		}

		fill_all();
		q_clear();
		reply("START OK\r\n");
		show_map();
		return 1;
	}

	if (!strcmp(cmd, "map")) {
		show_map();
		return 1;
	}

	/* a ~ g �� ����. �������̸� ������ �д� */
	if (ok_code(cmd[0]) && !cmd[1]) {
		if (!q_push(cmd[0]))
			reply("QUEUE FULL\r\n");
		else {
			snprintf(msg, sizeof(msg), "QUEUE %c\r\n", cmd[0]);
			reply(msg);
		}
		return 1;
	}

	return 0;
}
