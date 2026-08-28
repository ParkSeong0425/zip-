/*
 * cli.c
 *
 *  Created on: Aug 7, 2026
 *      Author: HWNOT
 *
 * UART3 Tera Term 에서 TCP 와 똑같은 명령을 친다.
 * 띄어쓰기로 쳐도 되고 (01MI 1 3 2 500) 밑줄로 쳐도 된다 (01MI_1_3_2_500).
 * 명령 처리는 net.c 의 net_cmd 가 그대로 한다.
 */
#include "cli.h"
#include "net.h"
#include "usart.h"
#include <string.h>

#define LINE_N 128
#define BS     8
#define DEL    127

static char line[LINE_N];
static int line_len;
static int framed;   /* STX 로 시작한 명령이면 1. 프롬프트를 찍지 않는다 */


/* UART3 문자열 출력 */
static void put(const char *s) {
	HAL_UART_Transmit(&huart3, (uint8_t*) s, strlen(s), 100);
}

/* UART3에서 받은 글자가 있으면 1 */
static int uart_get(uint8_t *c) {
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) == RESET)
		return 0;

	*c = (uint8_t)huart3.Instance->DR;
	return 1;
}

/* 외부에서 UART3 키 입력 확인 시 사용 */
int cli_key(void) {
	uint8_t c;
	return uart_get(&c);
}

/* 명령 목록 */
/* 현재 사용 가능한 명령 목록 */
static void help(void) {
	put("01FS 1                  : 설정값 조회\r\n");
	put("01FS 1 S IX IY OX OY    : 입고/출고 칸 수 저장\r\n");
	put("x 위치 rpm              : X축 수동 이동\r\n");
	put("y 위치 rpm              : Y축 수동 이동\r\n");
	put("r 각도 rpm              : 오른쪽 수동 회전\r\n");
	put("l 각도 rpm              : 왼쪽 수동 회전\r\n");
	put("c rpm                   : 회전축 0도로 이동\r\n");
	put("01MI 렉 열 단 XY% ROT% 시작ms 복귀10ms : 입고\r\n");
	put("01PO 렉 열 단 XY% ROT% 시작ms 복귀10ms : 출고\r\n");
	put("01I                     : 원점 이동\r\n");
	put("01S_1                   : 전체 정지\r\n");
	put("01C                     : 동작 상태 확인\r\n");
	put("01R                     : RFID 상태 요청\r\n");
}

/* 띄어쓰기를 밑줄로 바꿔 TCP 와 같은 형식으로 만든다 */
static void to_bar(char *s) {
	for (; *s; s++)
		if (*s == ' ' || *s == '\t')
			*s = '_';
}

/* 한 줄 실행 */
void cli_exec(char *s) {
	if (strcmp(s, "help") == 0 || strcmp(s, "?") == 0) {
		help();
		return;
	}
	to_bar(s);
	net_cmd(s);
}

/* 한 글자 입력 처리 */
/* 한 글자 입력 처리 */
static void cli_char(uint8_t c) {
	if (c == STX) {                 /* 프레임 시작. 줄을 비운다 */
		line_len = 0;
		framed = 1;
		return;
	}

	if (c == ETX || c == '\r' || c == '\n') {
		if (!line_len)
			return;

		if (!framed)
			put("\r\n");

		line[line_len] = 0;
		line_len = 0;
		cli_exec(line);

		if (!framed)
			put("\r\n> ");

		framed = 0;
		return;
	}

	if (c == BS || c == DEL) {
		if (line_len) {
			line_len--;
			put("\b \b");
		}
		return;
	}

	if (c >= ' ' && c < DEL && line_len < LINE_N - 1) {
		line[line_len++] = (char)c;

		if (!framed)            /* 사람이 친 글자만 되돌려 보여준다 */
			HAL_UART_Transmit(&huart3, &c, 1, 100);
	}
}

/* UART3 에 들어온 글자를 모두 처리 */
void cli_poll(void) {
	uint8_t c;
	while (uart_get(&c))
		cli_char(c);
}

/* 부팅 로그 뒤에 한 번 부른다 */
void cli_start(void) {
	line_len = 0;
	framed = 0;
	put("\r\nCLI ready. help 치면 명령 목록\r\n> ");
}
