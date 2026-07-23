/*
 * item.c
 *
 *  Created on: Jul 21, 2026
 *      Author: kotec
 */
#include "item.h"
#include "save.h"
#include "tcp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_X    8
#define MAX_Y    8

#define FIRST    'a'
#define LAST     'g'
#define CODE_N   (LAST - FIRST + 1)

#define MAX_PER  10     /* 한 칸에 들어갈 최대 개수 */
#define QUEUE_N  16     /* 미리 눌러둘 수 있는 개수 */

/* ===== 상태 ===== */

/* cnt[단][열][코드] = 그 칸에 남은 개수. 전원을 끄면 지워진다 */
static uint8_t cnt[MAX_Y][MAX_X][CODE_N];

/* 미리 눌러둔 명령 */
static char queue[QUEUE_N];
static int q_head, q_tail;

/* 지금 이동중인 것. 0이면 없음 */
static char now_code;
static int now_col, now_dan;

static int seeded;

static void reply(const char *s) {
	tcp_reply(s);
}

static int ok_code(char c) {
	return c >= FIRST && c <= LAST;
}

/* ===== 예약 큐 ===== */

static int q_empty(void) {
	return q_head == q_tail;
}

static void q_clear(void) {
	q_head = q_tail = 0;
}

static int q_push(char c) {
	int next = (q_tail + 1) % QUEUE_N;

	if (next == q_head)
		return 0; /* 꽉 참 */

	queue[q_tail] = c;
	q_tail = next;
	return 1;
}

static char q_pop(void) {
	char c = queue[q_head];

	q_head = (q_head + 1) % QUEUE_N;
	return c;
}

/* ===== 배치 ===== */

/* 모든 칸을 랜덤으로 채운다. 개수도 알파벳도 랜덤이고 중복된다 */
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
 * 1,1에서 가장 가까운 칸을 찾는다.
 * 거리는 (열-1)^2 + (단-1)^2. 같으면 아래 단, 왼쪽 열이 먼저다.
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

/* ===== 출력 ===== */

/* 칸 하나를 aabcddeee 형태로 만든다 */
static void cell_text(int d, int c, char *out) {
	int k = 0;

	for (int i = 0; i < CODE_N; i++)
		for (int j = 0; j < cnt[d][c][i]; j++)
			out[k++] = FIRST + i;

	out[k] = 0;
}

/* 칸마다 한 줄씩 출력한다 */
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

/* ===== 실행 ===== */

void item_init(void) {
	memset(cnt, 0, sizeof(cnt));
	q_clear();
	now_code = 0;
	seeded = 0;
}

/*
 * 사이클이 끝났는지 확인하고, 끝났으면 다음 예약을 시작한다.
 * 사이클이 성공으로 끝난 뒤에만 그 칸에서 하나를 뺀다.
 */
void item_run(void) {
	char msg[48];
	char code;
	int col, dan, r;

	/* 이동중이면 끝날 때까지 기다린다 */
	if (now_code) {
		if (save_busy())
			return;

		r = save_result();
		if (!r)
			return;

		if (r > 0) {
			/* ROT가 C로 돌아왔으니 그 칸에서 하나 뺀다 */
			cnt[now_dan - 1][now_col - 1][now_code - FIRST]--;
			snprintf(msg, sizeof(msg), "DONE %c %d %d\r\n", now_code, now_col,
					now_dan);
		} else {
			q_clear(); /* 실패하면 예약을 지운다 */
			snprintf(msg, sizeof(msg), "FAIL %c %d %d\r\n", now_code, now_col,
					now_dan);
		}

		reply(msg);
		now_code = 0;
		return;
	}

	/* 예약이 있으면 다음 것을 시작한다 */
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

	/* start : 전체를 랜덤으로 채우고 보여준다 */
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

	/* a ~ g 한 글자. 진행중이면 예약해 둔다 */
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

