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
#include "rfid.h"
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
#define REV_PULSE 1000    /* 1회전 펄스 */
#define REV_MM    160     /* 1회전 이동 mm. 폴리 지름 50.93 의 원주 */
#define MAX_rpm   3000

/* 공유기 대역에 맞춰 여기만 고친다 */
#define IP        {172, 20, 0, 101}
#define MASK      {255, 255, 255, 0}
#define GATE      {0, 0, 0, 0}

static State state = STATE_W;
static Step step = STEP_STOP;
static Job job;
static uint32_t tick;
static char line[128]; // 01FP값이 많아서 늘렸습니다
static int line_len;

/* 부팅 로그는 UART3 으로만 */
void print(const char *s) {
	HAL_UART_Transmit(&huart3, (uint8_t*) s, strlen(s), 100);
}

/* 명령 응답은 UART3 과 TCP 둘 다 */
void reply(const char *s) {
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

static int same(int a, int b) {
	return a - b <= OCHA && b - a <= OCHA;
}


/* ===== 동작 ===== */

/* 다음 단계로. 대기 시간은 여기서부터 다시 센다 */
static void next(Step s) {
	step = s;
	tick = HAL_GetTick();
}

/* 개별 이동과 MI/PO의 목표 위치 확인 */
static int xy_stop(void) {
    int x = 0, y = 0, done;

    motor_pos(&motorX, &x);
    motor_pos(&motorY, &y);

    /* X 개별 이동 */
    if (job.dir == 2)
        return same(x, mm_to_pulse(job.x));

    /* Y 개별 이동 */
    if (job.dir == 3)
        return same(y, -mm_to_pulse(job.y));

    /* MI/PO는 저장된 X/Y 위치 모두 확인 */
    if (job.dir == -1 || job.dir == 1)
        return same(x, mm_to_pulse(job.x))
                && same(y, -mm_to_pulse(job.y));

    /* 원점 이동은 기존처럼 정지 여부 확인 */
    done = same(x, job.lastx) && same(y, job.lasty);
    job.lastx = x;
    job.lasty = y;

    return done;
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
	state = STATE_I;
	next(STEP_XY);
	ok = motor_home_on(&motorX);
	ok = motor_home_on(&motorY) && ok;
	ok = mks_home() && ok;
	job.center = 0;
	job.dir = 0;
	return ok;
}

/* 렉_열_단_XY%_틸트%_시작지연ms_복귀대기10ms */
static int go(char *s, int dir) {
	int rack, col, row, l, r, c, rpm;
	int xy_rpm, rot_rpm, ok;

	if (sscanf(s, "%d_%d_%d_%d_%d_%d_%d",
			&rack, &col, &row, &xy_rpm, &rot_rpm,
			&job.wait, &job.rx_wait) != 7)
		return 0;

	job.rpm = xy_rpm * MAX_rpm / 100;
	job.rot_rpm = rot_rpm * MAX_rpm / 100;
	job.rx_wait *= 10;

	// fram에 저장되어있는 값들을 이용하여 MI인지 PO인지 분간
	if (dir < 0) {
		pos_load(rack, IN_X, col, &job.x);
		pos_load(rack, IN_Y, row, &job.y);  /* 01MI */
	} else {
		pos_load(rack, OUT_X, col, &job.x);
		pos_load(rack, OUT_Y, row, &job.y); /* 01PO */
	}

	if (!rot_load(rack, &l, &r, &c, &rpm))
		return 0;

	job.dir = dir;
	job.center = 0; // 복귀 위치 ( 나중에 원점 센서 하면 그곳이 원점 )
	job.rot = -dir * r * MKS_REV / 360; // l,r 각도 값 들어가도록
	ok = motor_move(&motorX, job.rpm, mm_to_pulse(job.x));
	ok = motor_move(&motorY, job.rpm, mm_to_pulse(job.y)) && ok;
	ok = mks_move(job.rot_rpm, job.center) && ok;

	if (ok) {
		state = STATE_R;
		next(STEP_XY);
	}

	return ok;
}


/* 01FS_1 조회 또는 01FS_1_S_X_Y 저장 */
static void save_cfg(char *s) {
    char b[32], type;
    int rack, inx, iny, outx, outy;

    if (!strcmp(s, "1")) {
        cfg_load(&rack, &inx, &iny, &outx, &outy);
        snprintf(b, sizeof(b), "01FS_1_S_%d_%d_%d_%d\r\n",
                inx, iny, outx, outy);
        reply(b);
    } else if (sscanf(s, "%d_%c_%d_%d_%d_%d",
            &rack, &type, &inx, &iny, &outx, &outy) == 6
            && rack == 1 && type == 'S')
        ack("01FS", cfg_save(rack, inx, iny, outx, outy));
    else
        reply("01FS_ERR\r\n");
}

static void job_end(void) {
	char b[32];
	state = STATE_W;
	next(STEP_STOP);
	if (job.dir == 0) {
		motor_zero(&motorX);
		motor_zero(&motorY);
		mks_zero();
		snprintf(b, sizeof(b), "01IE_1\r\n");
	} else
		snprintf(b, sizeof(b), job.dir < 0 ? "01AI_%d_%d\r\n" : "01EO_%d_%d\r\n",
				job.x, job.y);
	reply(b);
}

/* 10ms마다 현재 동작 단계를 확인한다 */
static void run(void) {
    uint32_t dt = HAL_GetTick() - tick;

    switch (step) {
    case STEP_XY:
        if (dt >= START_MS && xy_stop()) {
            if (job.dir == 2 || job.dir == 3) {
                state = STATE_W;
                next(STEP_STOP);
                ack(job.dir == 2 ? "x" : "y", 1);
            } else if (mks_done(job.center))
                next(STEP_TILT);
        }
        break;

    case STEP_TILT:
        if (dt >= (uint32_t) job.wait
                && mks_move(job.rot_rpm, job.rot))
            next(STEP_TURN);
        break;

    case STEP_TURN:
        if (mks_done(job.rot)) {
            if (job.dir >= 4) {
                state = STATE_W;
                next(STEP_STOP);

                if (job.dir == 4)
                    ack("r", 1);
                else if (job.dir == 5)
                    ack("l", 1);
                else
                    ack("c", 1);
            } else
                next(STEP_BACK);
        }
        break;

    case STEP_BACK:
        if (dt >= (uint32_t) job.rx_wait
                && mks_move(job.rot_rpm, job.center))
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
	    int v, rpm, rack, axis, n, pos[8];

	if (step != STEP_STOP
	        && strcmp(s, "01S_1")
	        && strcmp(s, "02C")
	        && strcmp(s, "02R")) {
	    reply("동작 중\r\n");
	    return;
	}

	if (strncmp(s, "01MI_", 5) == 0) // 입고 이동 명령
		ack("01MI", go(s + 5, -1));
	else if (strncmp(s, "01PO_", 5) == 0) // 이동 + 분배 명령
		ack("01PO", go(s + 5, 1));
	else if (sscanf(s, "x_%d_%d", &v, &rpm) == 2) {
	    job.x = v; job.dir = 2;
	    if (motor_move(&motorX, rpm, mm_to_pulse(v))) {
	        state = STATE_R; next(STEP_XY);
	    } else ack("x", 0);
	}
	else if (sscanf(s, "y_%d_%d", &v, &rpm) == 2) {
	    job.y = v; job.dir = 3;
	    if (motor_move(&motorY, rpm, mm_to_pulse(v))) {
	        state = STATE_R; next(STEP_XY);
	    } else ack("y", 0);
	}
	else if (sscanf(s, "r_%d_%d", &v, &rpm) == 2) {
	    job.rot = -v * MKS_REV / 360; job.dir = 4;
	    if (mks_r(rpm, v)) {
	        state = STATE_R; next(STEP_TURN);
	    } else ack("r", 0);
	}
	else if (sscanf(s, "l_%d_%d", &v, &rpm) == 2) {
	    job.rot = v * MKS_REV / 360; job.dir = 5;
	    if (mks_l(rpm, v)) {
	        state = STATE_R; next(STEP_TURN);
	    } else ack("l", 0);
	}
	else if (sscanf(s, "c_%d", &rpm) == 1) {
	    job.rot = 0; job.dir = 6;
	    if (mks_c(rpm)) {
	        state = STATE_R; next(STEP_TURN);
	    } else ack("c", 0);
	}
	/* 01FP_렉_종류_위치값... 저장 */
	else if (strncmp(s, "01FP_", 5) == 0) {
	    n = sscanf(s,
	            "01FP_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d",
	            &rack, &axis,
	            &pos[0], &pos[1], &pos[2], &pos[3],
	            &pos[4], &pos[5], &pos[6], &pos[7]);

	    if (n < 2)
	        ack("01FP", 0);
	    else if (axis == 3)
	        ack("01FP", n == 6
	                && rot_save(rack, pos[0], pos[1],
	                        pos[2], pos[3]));

	    else
		    /*
		     | 내부번호 | 의미   |
	         | ----: | ----  |
	         |     1 | 입고 X |
	         |     2 | 입고 Y |
	         |     3 | 출고 X |
	         |     4 | 출고 Y |

		     * 내부 번호에는 회전이 없으므로 프로토콜의 4와 5에서 1을 빼는 것입니다
		     * 기존애 저장 할때는 입고x,y,틸트, 출고x,y 순인데
		     * 01FP
		     * */
	        ack("01FP", pos_save(rack,
	                axis < 3 ? axis : axis - 1,
	                pos, n - 2));
	}
	else if (strncmp(s, "01FS_", 5) == 0) // 렉 단 열 저장 명령
	    save_cfg(s + 5);
	else if (strcmp(s, "01I") == 0)
		ack("01I", home());
	else if (strcmp(s, "01S_1") == 0) // 동작 정지 명령
		ack("01S", all_stop());
	else if (strcmp(s, "02C") == 0) {
		snprintf(b, sizeof(b), "01-S_1_%c&S\r\n", (char) state);
		reply(b);
	}
	else if (strcmp(s, "02R") == 0)
		Rfid_Request();
	else
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
		if (rfid_check) {
			rfid_check = 0;
			all_stop();
		}
		serve();
		cli_poll();
		run();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
