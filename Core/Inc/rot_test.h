/*
 * from m_motor.h  -  MKS SERVO57D_RS485.  매뉴얼 V1.0.6
 */
#ifndef ROT_TEST_H_
#define ROT_TEST_H_

#include "main.h"
#include <stdint.h>

#define ROT_GEAR  20      /* mks모터 웜기어에 들어있는 감속비 20:1  */
#define MKS_REV     16384   /* 한 바퀴 도는 위치값. 0x4000 */

/* 매뉴얼 Function 칸 */
typedef enum {
	CMD_AXIS = 0x31,        /* 5.1 인코더 누적값.  Data 6바이트 */
	CMD_IO = 0x34,          /* 5.1 IO 포트.  bit0=IN_1 bit1=IN_2 */
	CMD_MODE = 0x82,        /* 5.2 작업 모드.  5 = SR_vFOC */
	CMD_CURRENT = 0x83,     /* 5.2 작업 전류 mA.  Data 2바이트 */
	CMD_MSTEP = 0x84,       /* 5.2 세분화 */
	CMD_EN_LVL = 0x85,      /* 5.2 En 레벨.  0=Low 1=High 2=Hold */
	CMD_RESPOND = 0x8C,     /* 5.2 응답 패킷 */
	CMD_HOME_SET = 0x90,	/* 5.4 홈 파라미터 설정 */
	CMD_GO_HOME = 0x91,		/* 5.4 홈 가도록 설정 */
	CMD_ZERO = 0x92,        /* 5.4 지금 자리를 0 으로.  Data 없음 */
	CMD_STATE = 0xF1,       /* 6.2 상태 */
	CMD_ENABLE = 0xF3,      /* 6.2 사용 */
	CMD_ABS = 0xF5          /* 6.9 절대 이동 / 정지 */
} mks_cmd;

/* 6.2 F1 응답값 */
typedef enum {
	ST_FAIL = 0, ST_STOP = 1, ST_ACC = 2, ST_DEC = 3, ST_FULL = 4, ST_HOME = 5
} mks_st;


/* 레지스터 직접 접근. n 은 매뉴얼 Data 칸의 바이트 수 */
int mks_write(uint8_t id, uint8_t code, uint64_t data, int n);
int mks_read(uint8_t id, uint8_t code, int n, int *out);

int mks_done(int axis);
int mks_r(int rpm, int angle);
int mks_l(int rpm, int angle);
int mks_c(int rpm);
int mks_init(void);
int mks_move(int rpm, int axis);
int mks_home(void);     /* 0x34 센서 -> F5 정지 -> 0x92 */
int mks_zero(void);     /* 0x92 두 대 모두 */
int mks_stop(void);     /* F5 speed 0 acc DEC */
#endif
