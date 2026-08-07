/*
 * net.c
 *
 * 명령은 한 줄씩. 끝은 엔터. TCP 2500 과 UART3 CLI 둘 다 같은 명령을 쓴다.
 *   01SAVE_렉_X칸_Y칸        렉 수와 칸 수를 정한다. 이 범위 안에서만 움직인다
 *   01J_축_값_rpm            손으로 움직인다. 축 1=X mm, 2=Y mm, 3=틸트 각
 *   01FS_렉_축_번호[_rpm]    지금 자리를 저장.
 *                            X/Y 는 번호 1=첫칸, 2=둘째칸 (나머지 칸은 자동)
 *                            틸트는 번호 1=L, 2=R, 3=C. rpm 은 틸트만 쓴다
 *   01FR_렉_축               저장값을 01FP_렉_축_값... 으로 돌려준다
 *   01MI_렉_열_단_rpm[_틸트rpm_wait_rxwait]   입고
 *   01PO_렉_열_단_rpm[...]                    출고
 *   01I 원점  01S_1 정지  02C 상태  02N 렉/칸 수
 */
#include "net.h"
#include "cli.h"
#include "rot_test.h"
#include "motor.h"
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
#define OCHA      50      /* 이만큼도 안 움직이면 멈춘 것으로 본다 */
#define START_MS  1000    /* 명령 걸고 이만큼은 무조건 기다린다 */
#define MKS_ID    2       /* rot_test.c 의 ID */
#define REV_PULSE 1000    /* 1회전 펄스 */
#define REV_MM    160     /* 1회전 이동 mm. 폴리 지름 50.93 의 원주 */

/* 공유기 대역에 맞춰 여기만 고친다 */
#define IP        {172, 20, 0, 101}
#define MASK      {255, 255, 255, 0}
#define GATE      {0, 0, 0, 0}

typedef enum {
	STATE_W = 'W', STATE_R = 'R', STATE_I = 'I', STATE_P = 'P'
} State;

typedef enum {
	STEP_STOP, STEP_XY, STEP_TILT, STEP_TURN, STEP_BACK, STEP_CENTER
} Step;

typedef struct {
	int x, y, rpm;            /* X/Y 목표 mm 와 속도 */
	int rot, center, rot_rpm; /* 틸트 목표와 정면과 속도 */
	int dir;                  /* -1 입고 +1 출고 */
	int wait, rx_wait;
	int lastx, lasty;
} Job;

static State state = STATE_W;
static Step step = STEP_STOP;
static Job job;
static uint32_t tick;
static char line[64];
static int line_len;

/* 부팅 로그는 UART3 으로만 */
void print(const char *s) {
	HAL_UART_Transmit(&huart3, (uint8_t*) s, strlen(s), 100);
}

/* 명령 응답은 UART3 과 TCP 둘 다 */
static void reply(const char *s) {
	print(s);
	if (getSn_SR(SOCK) == SOCK_ESTABLISHED)
		send(SOCK, (uint8_t*) s, strlen(s));
}

static void ack(const char *name, int ok) {
	char b[24];
	snprintf(b, sizeof(b), "%s_%s\r\n", name, ok ? "OK" : "ERR");
	reply(b);
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
	uint8_t ip4[4];
	char b[48];
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
	getSIPR(ip4);
	snprintf(b, sizeof(b), "01FI_1_%02X%02X%02X%02X_%u\r\n", ip4[0], ip4[1],
			ip4[2], ip4[3], PORT);
	print(b);
}

/* ===== mm 과 펄스 ===== */

static int mm_to_pulse(int mm) {
	return mm * REV_PULSE / REV_MM;
}
static int pulse_to_mm(int p) {
	return p * REV_MM / REV_PULSE;
}
static int same(int a, int b) {
	return a - b <= OCHA && b - a <= OCHA;
}

/* Y 모터는 방향이 반대라 읽은 값도 뒤집는다 */
static int axis_pos(int axis, int *mm) {
	int p;
	if (!motor_pos(axis == 1 ? &motorX : &motorY, &p))
		return 0;
	*mm = pulse_to_mm(axis == 2 ? -p : p);
	return 1;
}

/* ===== 동작 ===== */

/* 다음 단계로. 대기 시간은 여기서부터 다시 센다 */
static void next(Step s) {
	step = s;
	tick = HAL_GetTick();
}

/* 직전에 읽은 위치와 비교해서 더 안 움직이면 도착으로 본다 */
static int xy_stop(void) {
	int x = 0, y = 0, done;
	motor_pos(&motorX, &x);
	motor_pos(&motorY, &y);
	done = same(x, job.lastx) && same(y, job.lasty);
	job.lastx = x;
	job.lasty = y;
	return done;
}

static int all_stop(void) {
	int ok;
	ok = motor_stop(&motorX);
	ok = motor_stop(&motorY) && ok;
	ok = mks_stop() && ok;
	state = STATE_P;
	next(STEP_STOP);
	return ok;
}

static int home(void) {
	int ok;
	ok = motor_home_on(&motorX);
	ok = motor_home_on(&motorY) && ok;
	ok = mks_home() && ok;
	state = STATE_I;
	next(STEP_STOP);
	return ok;
}

/* "렉_열_단_rpm_틸트rpm_wait_rxwait". 위치는 FRAM 에서 꺼내 쓴다 */
static int go(char *s, int dir) {
	int rack, col, row, l, r, c, rpm, ok;
	job.wait = 0;
	job.rx_wait = 0;
	job.rot_rpm = 0;
	if (sscanf(s, "%d_%d_%d_%d_%d_%d_%d", &rack, &col, &row, &job.rpm,
			&job.rot_rpm, &job.wait, &job.rx_wait) < 4)
		return 0;
	if (!pos_load(rack, 1, col, &job.x) || !pos_load(rack, 2, row, &job.y)
			|| !rot_load(rack, &l, &r, &c, &rpm))
		return 0;
	if (job.rot_rpm < 1)
		job.rot_rpm = rpm;
	job.dir = dir;
	job.center = c;
	job.rot = dir < 0 ? r : l;   /* 입고 R, 출고 L. 반대면 이 줄만 바꾼다 */
	ok = motor_move(&motorX, job.rpm, mm_to_pulse(job.x));
	ok = motor_move(&motorY, job.rpm, mm_to_pulse(job.y)) && ok;
	ok = mks_move(job.rot_rpm, job.center) && ok;
	state = STATE_R;
	next(STEP_XY);
	return ok;
}

/* 자리 잡을 때 쓴다. "축_값_rpm" */
static int jog(char *s) {
	int axis, v, rpm;
	if (sscanf(s, "%d_%d_%d", &axis, &v, &rpm) != 3)
		return 0;
	if (axis == 1)
		return motor_move(&motorX, rpm, mm_to_pulse(v));
	if (axis == 2)
		return motor_move(&motorY, rpm, mm_to_pulse(v));
	if (axis == 3)
		return mks_move(rpm, v);
	return 0;
}

/* 지금 자리를 저장한다. "렉_축_번호[_rpm]" */
static int fs(char *s) {
	int rack, axis, no, rpm = 0;
	int p, v[2], l, r, c, old;
	if (sscanf(s, "%d_%d_%d_%d", &rack, &axis, &no, &rpm) < 3)
		return 0;
	if (axis == 3) {
		if (no < 1 || no > 3 || !mks_read(MKS_ID, CMD_AXIS, 6, &p)
				|| !rot_load(rack, &l, &r, &c, &old))
			return 0;
		if (no == 1)
			l = p;
		else if (no == 2)
			r = p;
		else
			c = p;
		return rot_save(rack, l, r, c, rpm > 0 ? rpm : old);
	}
	if ((axis != 1 && axis != 2) || no < 1 || no > 2
			|| !axis_pos(axis, &v[no - 1]))
		return 0;
	if (no == 2 && !pos_load(rack, axis, 1, &v[0]))
		return 0;
	return pos_save(rack, axis, v, no);
}

/* 저장값을 한 줄로 돌려준다. "렉_축" */
static void fr(char *s) {
	char b[256];
	int rack, axis, no, v[4], len;
	if (sscanf(s, "%d_%d", &rack, &axis) < 2) {
		reply("01FR_ERR\r\n");
		return;
	}
	len = snprintf(b, sizeof(b), "01FP_%d_%d", rack, axis);
	if (axis == 3) {
		if (!rot_load(rack, &v[0], &v[1], &v[2], &v[3])) {
			reply("01FR_ERR\r\n");
			return;
		}
		for (no = 0; no < 4; no++)
			len += snprintf(b + len, sizeof(b) - len, "_%d", v[no]);
	} else
		for (no = 1; no <= pos_count(axis) && len < 230; no++) {
			if (!pos_load(rack, axis, no, &v[0]))
				break;
			len += snprintf(b + len, sizeof(b) - len, "_%d", v[0]);
		}
	snprintf(b + len, sizeof(b) - len, "\r\n");
	reply(b);
}

/* "렉수_X칸수_Y칸수" */
static int save_cfg(char *s) {
	int rack, nx, ny;
	return sscanf(s, "%d_%d_%d", &rack, &nx, &ny) == 3
			&& cfg_save(rack, nx, ny);
}

static void job_end(void) {
	char b[32];
	state = STATE_W;
	next(STEP_STOP);
	snprintf(b, sizeof(b), job.dir < 0 ? "01AI_%d_%d\r\n" : "01EO_%d_%d\r\n",
			job.x, job.y);
	reply(b);
}

/* 10ms 마다 한 단계씩만 본다. 명령이 실패하면 다음 틱에 다시 건다 */
static void run(void) {
	uint32_t dt = HAL_GetTick() - tick;
	switch (step) {
	case STEP_XY:
		if (dt >= START_MS && xy_stop() && mks_done(job.center))
			next(STEP_TILT);
		break;
	case STEP_TILT:
		if (dt >= (uint32_t) job.wait && mks_move(job.rot_rpm, job.rot))
			next(STEP_TURN);
		break;
	case STEP_TURN:
		if (mks_done(job.rot))
			next(STEP_BACK);
		break;
	case STEP_BACK:
		if (dt >= (uint32_t) job.rx_wait && mks_move(job.rot_rpm, job.center))
			next(STEP_CENTER);
		break;
	case STEP_CENTER:
		if (mks_done(job.center))
			job_end();
		break;
	default:
		break;
	}
}

/* ===== 명령 ===== */

/* TCP 와 UART3 CLI 가 같이 쓴다 */
void net_cmd(char *s) {
	char b[64];
	int rack, nx, ny;
	if (strncmp(s, "01MI_", 5) == 0)
		ack("01MI", go(s + 5, -1));
	else if (strncmp(s, "01PO_", 5) == 0)
		ack("01PO", go(s + 5, 1));
	else if (strncmp(s, "01J_", 4) == 0)
		ack("01J", jog(s + 4));
	else if (strncmp(s, "01SAVE_", 7) == 0)
		ack("01SAVE", save_cfg(s + 7));
	else if (strncmp(s, "01FS_", 5) == 0)
		ack("01FS", fs(s + 5));
	else if (strncmp(s, "01FR_", 5) == 0)
		fr(s + 5);
	else if (strcmp(s, "01I") == 0)
		ack("01I", home());
	else if (strcmp(s, "01S_1") == 0)
		ack("01S", all_stop());
	else if (strcmp(s, "02C") == 0) {
		snprintf(b, sizeof(b), "01-S_1_%c&S\r\n", (char) state);
		reply(b);
	} else if (strcmp(s, "02N") == 0) {
		cfg_load(&rack, &nx, &ny);
		snprintf(b, sizeof(b), "02N_%d_%d_%d\r\n", rack, nx, ny);
		reply(b);
	} else
		reply("에러\r\n");
}

/* 엔터가 올 때까지 모았다가 한 줄씩 실행한다 */
static void serve(void) {
	uint8_t buf[64];
	int n, i;
	switch (getSn_SR(SOCK)) {
	case SOCK_CLOSED:
		socket(SOCK, Sn_MR_TCP4, PORT, 0);
		line_len = 0;
		break;
	case SOCK_INIT:
		listen(SOCK);
		break;
	case SOCK_ESTABLISHED:
		n = getSn_RX_RSR(SOCK);
		if (!n)
			break;
		if (n > (int) sizeof(buf))
			n = sizeof(buf);
		recv(SOCK, buf, n);
		for (i = 0; i < n; i++) {
			if (buf[i] == '\r' || buf[i] == '\n') {
				if (line_len) {
					line[line_len] = 0;
					line_len = 0;
					net_cmd(line);
				}
			} else if (line_len < (int) sizeof(line) - 1)
				line[line_len++] = buf[i];
		}
		break;
	case SOCK_CLOSE_WAIT:
		disconnect(SOCK);
		line_len = 0;
		break;
	}
}

void TM_TaskRun(void *arg) {
	(void) arg;
	net_init();
	print(fram_load() ? "fram ready\r\n" : "fram empty\r\n");
	print(motor_init(&motorX) ? "x ready\r\n" : "x init ERR\r\n");
	print(motor_init(&motorY) ? "y ready\r\n" : "y init ERR\r\n");
	print(mks_init() ? "mks ready\r\n" : "mks init ERR\r\n");
	cli_start();
	for (;;) {
		run();
		serve();
		cli_poll();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
