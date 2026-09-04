/*
 * net.h
 */
#ifndef INC_NET_H_
#define INC_NET_H_

#include "main.h"
#include "move.h"

/* 프레임 문자. 터미널에는 ACK 가 - , NAK 이 ㅗ 처럼 보인다 */
#define STX       0x02    /* 명령 시작 */
#define ETX       0x03    /* 명령 끝 */
#define ACK       0x06    /* 수락 */
#define NAK       0x15    /* 거절 */

int alarm_get(void);
void alarm_set(int code);
void repeat_set(const char *message);          /* 이동 완료 메시지 반복 시작 */
void send_to_tcp_queue(const char *response);  /* 응답을 TCPQueue에 저장 */
void print(const char *s);   /* UART3 에만. 프레임 없음 */
void reply(const char *s);   /* STX 내용 ETX 로 감싸 보낸다 */
void pause_msg(char why);    /* 22P_PAUSE:F M S T */
void net_cmd(char *s);       /* TCP 와 UART3 CLI 가 같이 쓴다 */

/* freertos.c 의 StartNET_Task() 가 호출한다 */
void TM_TaskRun(void *argument);
#endif
