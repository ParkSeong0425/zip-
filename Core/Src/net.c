/*
 * net.c
 *
 * 응답 프레임:  STX | 01 | (ACK 또는 NAK) | 내용 | ETX
 *   수락  STX 01 MI_1_1_080         ETX   받은 명령을 그대로 되돌린다
 *   완료  STX 01 <ACK> AI_1_1       ETX
 *   거절  STX 01 <NAK> MI_moving    ETX
 * ACK/NAK 은 아스키 제어문자라 터미널에는 - 와 ㅗ 처럼 보인다.
 * x y r l c 는 CLI 시험용이라 프레임 없이 UART3 에만 찍는다.
 */
#include "net.h"
#include "rfid.h"
#include "cli.h"
#include "rot_test.h"
#include "motor.h"
#include "button.h"
#include "fram.h"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "socket.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define NET_SPI   hspi1
#define SOCK      0
#define PORT      2500
#define STX       0x02    /* 명령 시작 */
#define ETX       0x03    /* 명령 끝 */
#define ACK       0x06    /* 완료. 터미널에는 - 로 보인다 */
#define NAK       0x15    /* 거절. 터미널에는 ㅗ 로 보인다 */
#define TCPS      0       /* 접속 모드 0=서버 1=클라이언트 */
#define TOLERANCE 50      /* 이만큼도 안 움직이면 멈춘 것으로 본다 */
#define START_MS  1000    /* 명령 걸고 이만큼은 무조건 기다린다 */
#define REV_PULSE 1000    /* 1회전 펄스 */
#define REV_MM    160     /* 1회전 이동 mm. 폴리 지름 50.93 의 원주 */
#define XY_GEAR   5       /* MD60모터에 들어있는 감속비 5:1 */
#define MAX_RPM   3000

/* 공유기 대역에 맞춰 여기만 고친다 */
#define IP        {172, 20, 0, 101}   /* 이 보드 DIP */
#define MASK      {255, 255, 255, 0}
#define GATE      {0, 0, 0, 0}
#define HOST      {172, 20, 0, 100}   /* 상대 PC SIP */

static State state = STATE_W; // 기본 상태는 대기중
static Step step = STEP_STOP; // 기본 상태는 멈춘 상태
static Move move;             // 지금 하고 있는 동작 하나
static uint32_t step_time;    // 지금 단계가 시작된 시각
static char line[128];        // 01FP 저장 값이 길어서 늘렸습니다
static int line_len;
static int linked;            // TCP 가 붙어 있으면 1

/* 부팅 로그는 UART3 으로만 */
void print(const char *s) {
	HAL_UART_Transmit(&huart3, (uint8_t*) s, strlen(s), 100);
}

/* UART3 은 보기 좋게 개행까지, TCP 는 프레임만 */
void reply(const char *s) {
	uint8_t frame[132];
	int n = strlen(s);
	/* 프레임에는 CR/LF를 넣지 않는다 */
	while (n && (s[n - 1] == '\r' || s[n - 1] == '\n'))
		n--;
	frame[0] = STX;
	memcpy(frame + 1, s, n);
	frame[n + 1] = ETX;
	frame[n + 2] = '\r';
	frame[n + 3] = '\n';
	HAL_UART_Transmit(&huart3, frame, n + 4, 100);
	if (getSn_SR(SOCK) == SOCK_ESTABLISHED)
		send(SOCK, frame, n + 4);
}
/* 수락. 01 + 내용 */
static void echo(const char *s) {
	char msg[128];
	snprintf(msg, sizeof(msg), "01%s\r\n", s);
	reply(msg);
}

/* 완료. 01 + ACK + 내용 */
static void ack(const char *s) {
	char msg[128];
	snprintf(msg, sizeof(msg), "01%c%s\r\n", ACK, s);
	reply(msg);
}

/* 거절. 01 + NAK + 내용 */
static void nak(const char *s) {
	char msg[128];
	snprintf(msg, sizeof(msg), "01%c%s\r\n", NAK, s);
	reply(msg);
}

/* 명령 이름에 거절 사유를 붙인다 */
static void nak_why(const char *name, const char *why) {
	char msg[48];
	snprintf(msg, sizeof(msg), "%s_%s", name, why);
	nak(msg);
}

/* ===== W6100 SPI ===== */

static void cs_on(void) {
	HAL_GPIO_WritePin(W610_CS_GPIO_Port, W610_CS_Pin, GPIO_PIN_RESET);
}
static void cs_off(void) {
	HAL_GPIO_WritePin(W610_CS_GPIO_Port, W610_CS_Pin, GPIO_PIN_SET);
}
static uint8_t spi_rb(void) {
	uint8_t b;
	HAL_SPI_Receive(&NET_SPI, &b, 1, 100);
	return b;
}
static void spi_wb(uint8_t b) {
	HAL_SPI_Transmit(&NET_SPI, &b, 1, 100);
}
static void spi_rbuf(uint8_t *buf, datasize_t n) {
	HAL_SPI_Receive(&NET_SPI, buf, n, 1000);
}
static void spi_wbuf(uint8_t *buf, datasize_t n) {
	HAL_SPI_Transmit(&NET_SPI, buf, n, 1000);
}

/* MAC 은 EEPROM 에서 읽고, NET 락을 풀어야 IP 가 써진다 */
static void net_init(void) {
	uint8_t size[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };
	wiz_NetInfo net = { .ip = IP, .sn = MASK, .gw = GATE, .ipmode =
			NETINFO_STATIC_V4 };
	int i;
	HAL_I2C_Mem_Read(&hi2c1, 0xA0, 0xFA, I2C_MEMADD_SIZE_8BIT, net.mac, 6, 100);
	HAL_GPIO_WritePin(W610_RST_GPIO_Port, W610_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(W610_RST_GPIO_Port, W610_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	reg_wizchip_cs_cbfunc(cs_on, cs_off);
	reg_wizchip_spi_cbfunc(spi_rb, spi_wb, spi_rbuf, spi_wbuf);
	wizchip_init(size, size);
	NETUNLOCK();
	wizchip_setnetinfo(&net);
	PHYUNLOCK();
	setPHYCR0(PHYCR0_AUTO);
	setPHYCR1(PHYCR1_RST);
	HAL_Delay(300);
	for (i = 0; i < 50 && !(getPHYSR() & PHYSR_LNK); i++)
		HAL_Delay(100);
}

/*
 * 01FI_1_구분2 조회. 칩에 실제로 쓰인 값을 되읽어 보낸다.
 * 구분2  1:SIP 2:SN 3:GW 4:DIP 5:PORT 6:MODE
 */
static void net_show(int kind) {
	wiz_NetInfo net;
	uint8_t host[4] = HOST;
	uint8_t *v;
	char msg[48];

	if (kind == 5) {                     /* 포트 번호 */
		snprintf(msg, sizeof(msg), "FI_1_5_%u", PORT);
		ack(msg);
		return;
	}
	if (kind == 6) {                     /* 접속 모드 */
		snprintf(msg, sizeof(msg), "FI_1_6_%d", TCPS);
		ack(msg);
		return;
	}

	wizchip_getnetinfo(&net);            /* 칩에서 되읽는다 */

	if (kind == 1) {
		/* 접속 중이면 실제 상대 주소를 읽고, 아니면 설정값을 쓴다 */
		if (getSn_SR(SOCK) == SOCK_ESTABLISHED)
			getSn_DIPR(SOCK, host);
		v = host;
	}
	else if (kind == 2)
		v = net.sn;
	else if (kind == 3)
		v = net.gw;
	else if (kind == 4)
		v = net.ip;
	else {
		nak("FI_bad_data");
		return;
	}

	snprintf(msg, sizeof(msg), "FI_1_%d_%d.%d.%d.%d",
			kind, v[0], v[1], v[2], v[3]);
	ack(msg);
}

/* 부팅과 접속 때 6개를 차례로 알린다 */
static void net_show_all(void) {
	int i;
	for (i = 1; i <= 6; i++)
		net_show(i);
}

/* ===== 단위 변환 ===== */

/* mm 을 X/Y 모터 펄스로 */
static int mm_to_pulse(int mm) {
	return mm * REV_PULSE * XY_GEAR / REV_MM;
}

/* 각도를 회전축 펄스로 */
static int deg_to_pulse(int deg) {
	return deg * MKS_REV * ROT_GEAR / 360;
}

/* 오차 안이면 1 */
static int same(int a, int b) {
	return a - b <= TOLERANCE && b - a <= TOLERANCE;
}

/* ===== 동작 ===== */

/* 다음 단계로. 대기 시간은 여기서부터 다시 센다 */
static void next(Step s) {
	step = s;
	step_time = HAL_GetTick();
}

/* 개별 이동과 MI/PO의 목표 위치 확인 */
static int xy_stop(void) {
	int x = 0, y = 0;
	if (!motor_pos(&motorX, &x) || !motor_pos(&motorY, &y))
		return 0;
	/* 원점복귀 후 X/Y 좌표가 모두 0인지 확인 */
	if (move.mode == MODE_HOME)
		return x == 0 && y == 0;
	/* X 개별 이동 */
	if (move.mode == MODE_X)
		return same(x, mm_to_pulse(move.x_mm));
	/* Y 개별 이동. motor_move 가 Y 를 뒤집어 보내니 여기도 마이너스 */
	if (move.mode == MODE_Y)
		return same(y, -mm_to_pulse(move.y_mm));
	/* MI/PO는 저장된 X/Y 위치 모두 확인 */
	return same(x, mm_to_pulse(move.x_mm))
			&& same(y, -mm_to_pulse(move.y_mm));
}

/* 수동 명령 완료. 프레임 없이 UART3 에만 */
static void done(const char *name) {
	char msg[24];
	state = STATE_W;
	next(STEP_STOP);
	snprintf(msg, sizeof(msg), "%s OK\r\n", name);
	print(msg);
}

/* MI / PO / 원점 완료 */
static void job_end(void) {
	char msg[32];
	state = STATE_W;
	next(STEP_STOP);
	if (move.mode == MODE_HOME) {
		ack("I");
		return;
	}
	/* 명령 준 칸에 맞게 도착했는지 확인하기 위해 실어 보낸다 */
	snprintf(msg, sizeof(msg),
			move.mode < 0 ? "AI_%d_%d" : "EO_%d_%d",
			move.prev_x, move.prev_y);
	ack(msg);
}

static int all_stop(void) {
	int ok;
	state = STATE_P;
	next(STEP_STOP); // 이 순서로 해야 다음 동작을 실행하지 않는다
	ok = motor_stop(&motorX);
	ok = motor_stop(&motorY) && ok;
	ok = mks_stop() && ok;
	return ok;
}

// 모터가 홈으로 가서 원점센서 인식을 하면 각자 멈추고 그 지점을 원점으로 지정한다
static int home(void) {
	int ok;
	move.mode = MODE_HOME;
	ok = motor_home_on(&motorX);
	ok = motor_home_on(&motorY) && ok;
	ok = mks_home() && ok;
	if (!ok) {
		all_stop();
		return 0;
	}
	state = STATE_I;
	next(STEP_XY);
	return 1;
}

/* 렉_열_단_XY%_틸트%_시작지연ms_복귀대기10ms */
static int go(char *s, int mode) {
	int rack, col, row, xy, tilt;    /* 명령에서 읽을 값 */
	int spare, angle, center, saved;   /* fram 에 저장된 회전 각도 */

	if (sscanf(s, "%d_%d_%d_%d_%d_%d_%d",   /* 7개가 다 와야 실행한다 */
			&rack, &col, &row, &xy, &tilt,
			&move.tilt_delay, &move.back_delay) != 7)
		return 0;
	if (!rot_load(rack, &spare, &angle, &center, &saved))
			return 0;                    /* 회전 각도를 못 읽으면 실패 */

	/* MI 는 입고 좌표, PO 는 출고 좌표를 fram 에서 읽는다 */
	pos_load(rack, mode == MODE_IN ? IN_X : OUT_X, col, &move.x_mm);
	pos_load(rack, mode == MODE_IN ? IN_Y : OUT_Y, row, &move.y_mm);

	move.mode = mode;                       /* 01MI 인지 01PO 인지 */
	move.prev_x = col;                      /* 응답에 실을 열 */
	move.prev_y = row;                      /* 응답에 실을 단 */
	move.speed = xy * MAX_RPM / 100;        /* % 를 rpm 으로 */
	move.tilt_speed = tilt * MAX_RPM / 100;
	move.back_delay *= 10;                  /* 10ms 단위로 들어온다 */

	/* MI 는 왼쪽, PO 는 오른쪽으로 기운다 */
	move.tilt = deg_to_pulse(mode == MODE_IN ? angle : -angle);

	/* 그 단이 만재면 각도를 0으로 두어 틸트만 건너뛴다 */
	if (mode == MODE_OUT && rack == 1 && row >= 1 && row <= 4
			&& (full_get() & (1 << (row - 1))))
		move.tilt = 0;

	if (!mks_c(move.tilt_speed))            /* 회전축을 0도로 먼저 */
		return 0;
	state = STATE_R;
	next(STEP_READY);                       /* 정렬이 끝나면 XY 출발 */
	return 1;
}

/* 10ms마다 현재 동작 단계를 확인한다 */
static void job_run(void) {
	uint32_t wait = HAL_GetTick() - step_time;  /* 이 단계에 머문 시간 */

	switch (step) {
	case STEP_READY:                     /* 0도 정렬을 기다렸다 XY 출발 */
		if (wait < START_MS || !mks_done(0))
			break;
		motor_move(&motorX, move.speed, mm_to_pulse(move.x_mm));
		motor_move(&motorY, move.speed, mm_to_pulse(move.y_mm));
		next(STEP_XY);
		break;
	case STEP_XY:
		if (wait < START_MS || !xy_stop())   /* 최소 대기 뒤 도착 확인 */
			break;
		if (move.mode == MODE_X)         /* 수동 이동은 여기서 끝 */
			done("x");
		else if (move.mode == MODE_Y)
			done("y");
		else if (move.mode == MODE_HOME) {
			if (mks_done(0))             /* 회전축까지 원점이면 끝 */
				job_end();
		} else if (!move.tilt)           /* 만재면 틸트 없이 끝 */
			job_end();
		else
			next(STEP_TILT);
		break;
	case STEP_TILT:                      /* 지연 뒤 목표 각도로 회전 */
		if (wait >= (uint32_t) move.tilt_delay
				&& mks_move(move.tilt_speed, move.tilt))
			next(STEP_TURN);
		else if (wait > 10000) {
			all_stop();
			nak("turn_fail");
		}
		break;
	case STEP_TURN:
		if (!mks_done(move.tilt))        /* 각도 도착을 기다린다 */
			break;
		if (move.mode == MODE_IN)        /* 01MI 는 여기서 끝 */
			job_end();
		else if (move.mode == MODE_RIGHT)
			done("r");
		else if (move.mode == MODE_LEFT)
			done("l");
		else if (move.mode == MODE_CENTER)
			done("c");
		else
			next(STEP_BACK);             /* 01PO 는 복귀로 */
		break;
	case STEP_BACK:                      /* 지연 뒤 0도로 되돌린다 */
		if (wait >= (uint32_t) move.back_delay
				&& mks_c(move.tilt_speed))
			next(STEP_CENTER);
		break;
	case STEP_CENTER:
		if (mks_done(0))
			job_end();
		else if (wait > 10000) {
			all_stop();
			nak("C_fail");
		}
		break;
	default:
		break;
	}
}

/* ===== 명령 ===== */

/* 01FP_렉_축 조회. 축 1=입고X 2=입고Y 3=틸트 4=출고X 5=출고Y */
static void pos_show(int rack, int axis) {
	char msg[96];
	int v[4], a = axis < 3 ? axis : axis - 1;
	int n, i, len;

	if (axis == 3) {
		rot_load(rack, &v[0], &v[1], &v[2], &v[3]);
		snprintf(msg, sizeof(msg), "FP_%d_3_%d_%d_%d_%d",
				rack, v[0], v[1], v[2], v[3]);
		ack(msg);
		return;
	}
	n = pos_count(a);
	len = snprintf(msg, sizeof(msg), "FP_%d_%d", rack, axis);
	for (i = 1; i <= n; i++) {
		pos_load(rack, a, i, &v[0]);
		len += snprintf(msg + len, sizeof(msg) - len, "_%d", v[0]);
	}
	ack(msg);
}

/* 01FP_렉_축_값... 저장 또는 조회 */
static void save_pos(char *s) {
	int rack, axis, count, pos[8];

	count = sscanf(s, "01FP_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d",
			&rack, &axis,
			&pos[0], &pos[1], &pos[2], &pos[3],
			&pos[4], &pos[5], &pos[6], &pos[7]);

	if (count == 1)                    /* 01FP_렉 : 축 1~5 전부 */
		for (axis = 1; axis <= 5; axis++)
			pos_show(rack, axis);
	else if (count == 2)               /* 01FP_렉_축 */
		pos_show(rack, axis);
	else if (count < 1)
		nak("FP_bad_data");
	else if (axis == 3) {              /* 틸트 L, R, C, rpm */
		if (count == 6 && rot_save(rack, pos[0], pos[1], pos[2], pos[3]))
			ack("FP");
		else
			nak("FP_save_fail");
	}
	/* 내부 번호에는 회전이 없으므로 프로토콜의 4와 5에서 1을 뺀다.
	   저장 순서는 입고 X, 입고 Y, 틸트, 출고 X, 출고 Y 이다 */
	else if (pos_save(rack, axis < 3 ? axis : axis - 1, pos, count - 2))
		ack("FP");
	else
		nak("FP_save_fail");
}

/* 01FS_1 조회 또는 01FS_1_S_X_Y 저장 */
static void save_cfg(char *s) {
	char msg[48], type;
	int rack, inx, iny, outx, outy;

	if (!strcmp(s, "1")) {                  /* 조회 */
		cfg_load(&rack, &inx, &iny, &outx, &outy);
		snprintf(msg, sizeof(msg), "FS_1_S_%d_%d_%d_%d",
				inx, iny, outx, outy);
		ack(msg);
		return;
	}
	if (sscanf(s, "%d_%c_%d_%d_%d_%d",      /* 저장 */
			&rack, &type, &inx, &iny, &outx, &outy) == 6
			&& rack == 1 && type == 'S'
			&& cfg_save(rack, inx, iny, outx, outy))
		ack("FS");
	else
		nak("FS_save_fail");
}

/* 01MI / 01PO 시작. 수락하면 받은 명령을 그대로 되돌린다 */
static void go_start(char *s, int mode, const char *name) {
	char msg[48];

	if (!card_ok) {                    /* 카드가 없으면 움직이지 않는다 */
		nak_why(name, "no_rfid");
		return;
	}
	if (!go(s, mode)) {
		nak_why(name, "move_fail");
		return;
	}
	snprintf(msg, sizeof(msg), "%s_%d_%d_%03d", name,
			move.prev_x, move.prev_y, move.speed * 100 / MAX_RPM);
	echo(msg);
}

/* 수동 회전. 목표를 한 번만 계산해 그대로 보낸다 */
static void turn(int deg, int rpm, int mode, const char *name) {
	char msg[24];

	move.tilt = deg_to_pulse(deg);
	move.mode = mode;
	if (mks_move(rpm, move.tilt)) {
		state = STATE_R;
		next(STEP_TURN);
		return;
	}
	snprintf(msg, sizeof(msg), "%s ERR\r\n", name);
	print(msg);
}

/* 02C 상태 조회 */
static void show_state(void) {
	char msg[32];
	int f = full_get();

	if (f)
		snprintf(msg, sizeof(msg), "S_1_%c&F%d%d%d%d", (char) state,
				!!(f & 1), !!(f & 2), !!(f & 4), !!(f & 8));
	else
		snprintf(msg, sizeof(msg), "S_1_%c&S", (char) state);
	ack(msg);
}

/* 동작 중이라 거절한다 */
static void nak_moving(char *s) {
	if (strncmp(s, "01MI_", 5) == 0)
		nak("MI_moving");
	else if (strncmp(s, "01PO_", 5) == 0)
		nak("PO_moving");
	else if (strcmp(s, "01I") == 0)
		nak("I_moving");
	else
		nak("moving");
}

/* TCP 와 UART3 CLI 가 같이 쓴다 */
void net_cmd(char *s) {
	int value, speed;

	/* 동작중에도 되는 명령만 통과시킨다 */
	if ((step != STEP_STOP || pause)
			&& strcmp(s, "01S_1")
			&& strcmp(s, "01D_1")
			&& strcmp(s, "02C")
			&& strcmp(s, "02R")) {
		nak_moving(s);
		return;
	}

	if (strncmp(s, "01MI_", 5) == 0)
		go_start(s + 5, MODE_IN, "MI");
	else if (strncmp(s, "01PO_", 5) == 0)
		go_start(s + 5, MODE_OUT, "PO");
	else if (sscanf(s, "x_%d_%d", &value, &speed) == 2) {
		move.x_mm = value;
		move.mode = MODE_X;
		if (motor_move(&motorX, speed, mm_to_pulse(value))) {
			state = STATE_R;
			next(STEP_XY);
		} else
			print("x ERR\r\n");
	}
	else if (sscanf(s, "y_%d_%d", &value, &speed) == 2) {
		move.y_mm = value;
		move.mode = MODE_Y;
		if (motor_move(&motorY, speed, mm_to_pulse(value))) {
			state = STATE_R;
			next(STEP_XY);
		} else
			print("y ERR\r\n");
	}
	else if (sscanf(s, "r_%d_%d", &value, &speed) == 2)
		turn(-value, speed, MODE_RIGHT, "r");   /* 오른쪽은 음수 */
	else if (sscanf(s, "l_%d_%d", &value, &speed) == 2)
		turn(value, speed, MODE_LEFT, "l");     /* 왼쪽은 양수 */
	else if (sscanf(s, "c_%d", &speed) == 1)
		turn(0, speed, MODE_CENTER, "c");       /* 0도로 */
	else if (strncmp(s, "01FP_", 5) == 0)
		save_pos(s);
	else if (strncmp(s, "01FS_", 5) == 0)
		save_cfg(s + 5);
	else if (sscanf(s, "01FI_1_%d", &value) == 1)   /* 하나만 조회 */
		net_show(value);
	else if (strcmp(s, "01FI_1") == 0)              /* 전부 조회 */
		net_show_all();
	else if (strcmp(s, "01I") == 0) {           /* 원점 이동 */
		if (home())
			echo("I");                          /* 수락. 도착하면 ACK I */
		else
			nak("I_move_fail");
	}
	else if (strcmp(s, "01S_1") == 0) {         /* 전체 정지 */
		if (all_stop())
			ack("S");
		else
			nak("S_stop_fail");
	}
	else if (strcmp(s, "01D_1") == 0) {         /* 만재 해제 */
		if (full_clear())
			ack("D");
		else
			nak("D_full");                      /* 센서가 아직 1이다 */
	}
	else if (strcmp(s, "02F") == 0) {       /* 만재 센서 생값 */
			char msg[32];
			snprintf(msg, sizeof(msg), "F_%d%d%d%d",
					HAL_GPIO_ReadPin(F1_GPIO_Port, F1_Pin),
					HAL_GPIO_ReadPin(F2_GPIO_Port, F2_Pin),
					HAL_GPIO_ReadPin(F3_GPIO_Port, F3_Pin),
					HAL_GPIO_ReadPin(F4_GPIO_Port, F4_Pin));
			ack(msg);
		}
	else if (strcmp(s, "02C") == 0)
		show_state();
	else if (strcmp(s, "02R") == 0)
		Rfid_Request();

	else {
			char msg[64];
			snprintf(msg, sizeof(msg), "bad_cmd_%s", s);
			nak(msg);
		}
}

/* 엔터가 올 때까지 모았다가 한 줄씩 실행한다 */
static void serve(void) {
	uint8_t buf[64];
	int n, i;

	switch (getSn_SR(SOCK)) {
	case SOCK_CLOSED:
		socket(SOCK, Sn_MR_TCP4, PORT, 0);
		line_len = 0;
		linked = 0;
		break;
	case SOCK_INIT:
		listen(SOCK);
		break;
	case SOCK_ESTABLISHED:
		if (!linked) {              /* 막 붙었으면 설정을 알린다 */
			linked = 1;
			net_show_all();
		}
		n = getSn_RX_RSR(SOCK);
		if (!n)
			break;
		if (n > (int) sizeof(buf))
			n = sizeof(buf);
		recv(SOCK, buf, n);
		for (i = 0; i < n; i++) {
			/* STX 를 받으면 새 명령 시작 */
			if (buf[i] == STX)
				line_len = 0;
			/* ETX 또는 엔터를 받으면 명령 실행 */
			else if (buf[i] == ETX || buf[i] == '\r' || buf[i] == '\n') {
				if (line_len) {
					line[line_len] = 0;
					line_len = 0;
					net_cmd(line);
				}
			}
			else if (line_len < (int) sizeof(line) - 1)
				line[line_len++] = buf[i];
		}
		break;
	case SOCK_CLOSE_WAIT:
		disconnect(SOCK);
		line_len = 0;
		linked = 0;
		break;
	}
}

void TM_TaskRun(void *arg) {
	int xok, yok, mok;

	(void) arg;
	button_init();
	net_init();
	net_show_all();               /* 아직 접속 전이라 UART3 에만 나온다 */
	print(fram_load() ? "fram ready\r\n" : "fram empty\r\n");
	xok = motor_init(&motorX);
	yok = motor_init(&motorY);
	mok = mks_init();
	print(xok ? "x ready\r\n" : "x init ERR\r\n");
	print(yok ? "y ready\r\n" : "y init ERR\r\n");
	print(mok ? "mks ready\r\n" : "mks init ERR\r\n");
	cli_start();

//	/* 전원 넣으면 명령 없이 바로 원점으로 간다 */
//	if (xok && yok && mok && !estop && !home())
//		print("home ERR\r\n");

	for (;;) {
		button_run();
		serve();
		cli_poll();
		job_run();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
