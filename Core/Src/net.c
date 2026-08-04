/*
 * net.c
 *
 *  Created on: Aug 3, 2026
 *      Author: HWNOT
 */

#include "net.h"
#include "rot_test.h"
#include "motor.h"
#include "button.h"
#include "item.h"
#include "fram.h"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "socket.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

/* W6100이 물린 SPI. SPI3에 연결했으면 hspi3으로 바꾼다 */
#define NET_SPI     hspi1
#define SOCK        0
#define PORT        2500

/* 공유기 대역에 맞춰 여기만 고친다 */
#define IP          {172, 20, 0, 101}
#define MASK        {255, 255, 255, 0}
#define GATE        {0, 0, 0, 0}

static char out[1000]; // 도움말이 길어져서 버퍼를 늘린다
static int out_len;
static char line[64];
static int line_len;


/* 응답은 TCP로 보내고 디버깅용으로 UART3에도 찍는다 */
void print(const char *s) {
	HAL_UART_Transmit(&huart3, (uint8_t*) s, strlen(s), 100);
	out_len += snprintf(out + out_len, sizeof(out) - out_len, "%s", s);
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

/* MAC은 EEPROM에서 읽고, NET 락을 풀어야 IP가 써진다 */
static void net_init(void) {
	uint8_t size[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };
	wiz_NetInfo net = { .ip = IP, .sn = MASK, .gw = GATE,
			.ipmode = NETINFO_STATIC_V4 };
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

	NETUNLOCK(); // 이렇게 해야 TCP 가 연결이 됨을 알 수 있다.
	wizchip_setnetinfo(&net);

	PHYUNLOCK();
	setPHYCR0(PHYCR0_AUTO);
	setPHYCR1(PHYCR1_RST);
	HAL_Delay(300);

	for (i = 0; i < 50 && !(getPHYSR() & PHYSR_LNK); i++)
		HAL_Delay(100);

	getSIPR(ip4);
	// ip(1) , sn(2) , gw(3) , DIP(4), PORT(5), MODE(6) (TCPS,TCPC 차이가 뭐임?)
	snprintf(b, sizeof(b), "01FI_1_1_%u.%u.%u.%u mac=%02X%02X%02X\r\n",
			getPHYSR(), ip4[0], ip4[1], ip4[2], ip4[3],
			net.mac[0], net.mac[1], net.mac[2]);
	print(b);
}

/* 이동 중 들어온 입력은 전부 정지로 본다 */
static int check_stop(void) {
	uint8_t buf[16];
	int n = getSn_RX_RSR(SOCK);

	if (n <= 0)
		return 0;
	if (n > (int) sizeof(buf))
		n = sizeof(buf);
	recv(SOCK, buf, n);
	return 1;
}

static void command(char *s) {
	int rpm, target, xpos, ypos, rotpos , m_state;
	char b[64];
	int ok;

	if (sscanf(s, "go %d %d", &rpm, &target) == 2) {
		ok = motor_move(&motorX, rpm, target);
		ok = motor_move(&motorY, rpm, target) && ok;
	}

	else if (strcmp(s, "R") == 0)
		ok = mks_r();

	else if (strcmp(s, "L") == 0)
		ok = mks_l();

	else if (strcmp(s, "C") == 0)
		ok = mks_c();

	else if (strcmp(s, "home") == 0) {
		ok = motor_home_on(&motorX);
		ok = motor_home_on(&motorY) && ok;
		ok = mks_home() && ok;
	}

	// 멈추는 프로토콜 이라 모터 전부 멈춰야 함
	else if (strcmp(s, "01S_1") == 0) {
		ok = motor_stop(&motorX);
		ok = motor_stop(&motorY)&& ok;
		ok = mks_stop() && ok;
	}

	// 상태 체크 하는 형식으로 바꿀 예정
	else if (strcmp(s, "02C") == 0) {
		ok = motor_pos(&motorX, &xpos);
		ok = motor_pos(&motorY, &ypos) && ok;
		ok = mks_read(2, CMD_AXIS, 6, &rotpos);
		ok = mks_read(2, CMD_STATE, 1, &m_state) && ok;
		if (ok) {
			snprintf(b, sizeof(b), "x pos=%d | y pos=%d |m pos=%d | state=%d\r\n",
					xpos,ypos , rotpos, m_state);
			print(b);
			return;
		}

	}

	else {
		print(" go <rpm> <target> | home | stop | show\r\n");
		return;
	}

	print(ok ? "OK\r\n" : "ERR\r\n");
}

/* "GET /go/100/1"이면 HTTP, "go 100 1"이면 테라텀 */
static int http(char *s) {

	out_len = 0;
	if (strncmp(s, "GET /", 5) != 0) {
		command(s);
		send(SOCK, (uint8_t*) out, out_len);
	}
	return 0;
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
		if (n == 0)
			break;
		if (n > (int) sizeof(buf))
			n = sizeof(buf);
		recv(SOCK, buf, n);

		for (i = 0; i < n; i++) {
			if (buf[i] == '\r' || buf[i] == '\n') {
				if (line_len) {
					line[line_len] = 0;
					line_len = 0;
					if (http(line))
						break;
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
	(void)arg;

	net_init();

	print(motor_init(&motorX) ? "x ready\r\n" : "x init ERR\r\n");
	print(motor_init(&motorY) ? "y ready\r\n" : "y init ERR\r\n");
	print(mks_init() ? "mks ready\r\n" : "mks init ERR\r\n");


	for (;;) {
		serve();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

