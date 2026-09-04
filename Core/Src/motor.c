/*
 * motor.c
 *
 *  Created on: Jul 27, 2026
 *      Author: HWNOT
 */
#include "motor.h"
#include <string.h>

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/* X=UART4, Y=UART5 */
Motor motorX = { &huart4, rs485_GPIO_Port, rs485_Pin };
Motor motorY = { &huart5, rs4852_GPIO_Port, rs4852_Pin};

/* ===== 레지스터 주소 ===== */
#define ID              1

#define R_MODE          0x0200
#define R_DI1           0x0302
#define R_DI1L          0x0303
#define R_DI2           0x0304
#define R_DI2L          0x0305
#define R_DI3           0x0306
#define R_DI3L          0x0307
#define R_DI4           0x0308
#define R_DI4L          0x0309
#define R_DI5           0x030A
#define R_DI5L          0x030B

#define R_SRC           0x0500
#define R_HOME_SEL      0x051E  /* H05_30 */
#define R_HOME_MODE     0x051F  /* H05_31 */
#define R_HOME_FAST     0x0520  /* H05_32 */
#define R_HOME_SLOW     0x0521  /* H05_33 */
#define R_HOME_ACC      0x0522  /* H05_34 */
#define R_HOME_OFF      0x0524  /* H05_36 */

#define R_RUN           0x1100
#define R_REG           0x1101
#define R_BEGIN         0x1102
#define R_TYPE          0x1104  // 1: 절대모드 , 0 : 상대모드
#define R_POS           0x110C
#define R_SPEED         0x110E
#define R_ACC           0x110F
#define R_WAIT          0x1110

#define R_REALPOS       0x0B07
#define R_DI            0x0B03

#define R_ADDR          0x0C00

#define ACC_MS          300    /* 위치이동 가감속 시간 */
#define WAIT_MS         0     /* 한 구간 이동 후 대기 */


/* ===== RS485 하위 통신 ===== */

static uint16_t crc16(const uint8_t *data, uint16_t len) {
	uint16_t crc = 0xFFFF;

	for (uint16_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++)
			crc = (crc & 1U) ? (crc >> 1) ^ 0xA001U : crc >> 1;
	}
	return crc;
}

/* 에러 플래그와 남은 수신 데이터를 비운다 */
static void bus_clear(Motor *m) {
	__HAL_UART_CLEAR_OREFLAG(m->uart);
	__HAL_UART_CLEAR_FEFLAG(m->uart);
	__HAL_UART_CLEAR_NEFLAG(m->uart);
	__HAL_UART_CLEAR_PEFLAG(m->uart);

	while (__HAL_UART_GET_FLAG(m->uart, UART_FLAG_RXNE) != RESET) {
		volatile uint8_t d = m->uart->Instance->DR;
		(void) d;
	}
}

// 3번정도 명령 보내주는 함수
static HAL_StatusTypeDef bus_xfer(Motor *m, uint8_t *tx, uint16_t tn,
		uint8_t *rx, uint16_t rn) {
	HAL_StatusTypeDef r;
	uint16_t check;

	for (int retry = 0; retry < 3; retry++) {
		bus_clear(m);

		/* 송신 방향 */
		HAL_GPIO_WritePin(m->port, m->pin, GPIO_PIN_SET);
		HAL_Delay(1);

		r = HAL_UART_Transmit(m->uart, tx, tn, 30); // 통신 반응을 줄이기 위해 30으로 한번 테스트

		/* 송신 완료를 기다린 뒤 수신 방향 */
		while (__HAL_UART_GET_FLAG(m->uart, UART_FLAG_TC) == RESET) {
		}
		for (volatile uint32_t i = 0; i < 100; i++) {
		}
		HAL_GPIO_WritePin(m->port, m->pin, GPIO_PIN_RESET);

		if (r == HAL_OK)
			r = HAL_UART_Receive(m->uart, rx, rn, 100);

		if (r == HAL_OK) {
			check = crc16(rx, rn - 2);
			if (rx[0] == tx[0] && rx[1] == tx[1]
					&& rx[rn - 2] == (uint8_t)check
					&& rx[rn - 1] == (uint8_t)(check >> 8)) {
				HAL_Delay(20);
				return HAL_OK;
			}
		}
		HAL_Delay(10);
	}
	return HAL_ERROR;
}

/* ===== Modbus 읽기/쓰기 ===== */

/* 0x06 단일 레지스터 쓰기. 응답은 요청과 완전히 같아야 한다 */
static int write16(Motor *m, uint16_t reg, uint16_t val) {
	uint8_t tx[8], rx[8];
	uint16_t crc;

	tx[0] = ID;
	tx[1] = 0x06;
	tx[2] = reg >> 8;
	tx[3] = reg;
	tx[4] = val >> 8;
	tx[5] = val;

	crc = crc16(tx, 6);
	tx[6] = crc;
	tx[7] = crc >> 8;

	if (bus_xfer(m, tx, 8, rx, 8) != HAL_OK
			|| memcmp(rx, tx, 6)
			|| rx[6] != tx[6]
			|| rx[7] != tx[7])
		return 0;

	return 1;
}

/* 정상 쓰기 뒤 모터가 다음 명령을 받을 시간을 준다 */
static int set16(Motor *m, uint16_t reg, uint16_t val) {
	if (!write16(m, reg, val))
		return 0;
	HAL_Delay(10);
	return 1;
}

/* 0x03 단일 레지스터 읽기 */
static int read16(Motor *m, uint16_t reg, uint16_t *out) {
	uint8_t tx[8], rx[7];
	uint16_t crc;

	tx[0] = ID;
	tx[1] = 0x03;
	tx[2] = reg >> 8;
	tx[3] = reg;
	tx[4] = 0;
	tx[5] = 1;

	crc = crc16(tx, 6);
	tx[6] = crc;
	tx[7] = crc >> 8;

	if (bus_xfer(m, tx, 8, rx, 7) != HAL_OK)
		return 0;

	crc = crc16(rx, 5);

	if (rx[0] != ID || rx[1] != 0x03 || rx[2] != 2 || rx[5] != (uint8_t) crc
			|| rx[6] != (uint8_t) (crc >> 8))
		return 0;

	*out = ((uint16_t) rx[3] << 8) | rx[4];
	return 1;
}

/* 0x10 2레지스터(32bit) 쓰기 */
static int write32(Motor *m, uint16_t reg, int val) {
	uint8_t tx[13], rx[8];
	uint32_t raw = (uint32_t) val;
	uint16_t lo = raw, hi = raw >> 16, crc;

	tx[0] = ID;
	tx[1] = 0x10;
	tx[2] = reg >> 8;
	tx[3] = reg;
	tx[4] = 0;
	tx[5] = 2;
	tx[6] = 4;
	tx[7] = lo >> 8;
	tx[8] = lo;
	tx[9] = hi >> 8;
	tx[10] = hi;

	crc = crc16(tx, 11);
	tx[11] = crc;
	tx[12] = crc >> 8;

	if (bus_xfer(m, tx, 13, rx, 8) != HAL_OK)
		return 0;

	crc = crc16(rx, 6);

	return rx[0] == ID && rx[1] == 0x10 && rx[2] == tx[2] && rx[3] == tx[3]
			&& rx[4] == 0 && rx[5] == 2 && rx[6] == (uint8_t) crc
			&& rx[7] == (uint8_t) (crc >> 8);
}

/* 0x03 2레지스터(32bit) 읽기 */
static int read32(Motor *m, uint16_t reg, int *out) {
	uint8_t tx[8], rx[9];
	uint16_t crc, lo, hi;

	tx[0] = ID;
	tx[1] = 0x03;
	tx[2] = reg >> 8;
	tx[3] = reg;
	tx[4] = 0;
	tx[5] = 2;

	crc = crc16(tx, 6);
	tx[6] = crc;
	tx[7] = crc >> 8;

	if (bus_xfer(m, tx, 8, rx, 9) != HAL_OK)
		return 0;

	crc = crc16(rx, 7);

	if (rx[0] != ID || rx[1] != 0x03 || rx[2] != 4 || rx[7] != (uint8_t) crc
			|| rx[8] != (uint8_t) (crc >> 8))
		return 0;

	lo = ((uint16_t) rx[3] << 8) | rx[4];
	hi = ((uint16_t) rx[5] << 8) | rx[6];

	*out = (int) (((uint32_t) hi << 16) | lo);
	return 1;
}

/* ===== 초기 설정 ===== */

int motor_init(Motor *m) {
	uint16_t v;

	if (!read16(m, R_ADDR, &v) || v != ID)
		return 0;

	return set16(m, R_DI1L, 0)
			&& set16(m, R_DI2L, 0)
			&& set16(m, R_DI3L, 0)
			&& set16(m, R_DI4L, 0)
			&& set16(m, R_DI5L, 0)

		 /* 중복 방지를 위해 DI 기능을 먼저 해제 */
		    && set16(m, R_DI1, 0)
		    && set16(m, R_DI2, 0)
		    && set16(m, R_DI3, 0)
		    && set16(m, R_DI4, 0)
		    && set16(m, R_DI5, 0)

			&& set16(m, R_DI1, 31)
			&& set16(m, R_DI2, 1)
			&& set16(m, R_DI3, 32)
			&& set16(m, R_DI4, 28)
			&& set16(m, R_DI5, 34)

			&& set16(m, R_MODE, 1)
			&& set16(m, R_SRC, 2)
			&& set16(m, R_RUN, 0)
			&& set16(m, R_REG, 1)
			&& set16(m, R_BEGIN, 0)
			&& set16(m, R_TYPE, 1)

			&& set16(m, R_HOME_MODE,
					m->uart == &huart5 ? 0 : 1) // 홈 갈때 방향 감지
			&& set16(m, R_HOME_FAST, 350)
			&& set16(m, R_HOME_SLOW, 100)
			&& set16(m, R_HOME_ACC, 1000) // 홈으로 갈때 가감속
			&& write32(m, R_HOME_OFF, 0) // 전기,기계적 원점 확인하고 원점으로 해주는 데이터

			/* DI 중복 설정 확인 후 Servo ON */
			&& read16(m, R_DI1, &v) && v == 31
			&& read16(m, R_DI2, &v) && v == 1
			&& read16(m, R_DI3, &v) && v == 32
			&& read16(m, R_DI4, &v) && v == 28
			&& read16(m, R_DI5, &v) && v == 34
			&& set16(m, R_DI2L, 1);
}

/* ===== 동작 ===== */

/* 센서를 찾아 원점복귀 */
int motor_home_on(Motor *m) {
    return set16(m, R_HOME_SEL, 4);
}
/* 현재 위치를 소프트웨어 0으로 설정 */
int motor_zero(Motor *m) {
	return set16(m, R_HOME_SEL, 6);
}

/* H0B_07 현재 위치 읽기 */
int motor_pos(Motor *m, int *out) {
	return read32(m, R_REALPOS, out);
}

/* H11_04=1: 원점 기준 절대 목표 위치로 이동 */
int motor_move(Motor *m, int rpm, int target) {

	return set16(m, R_DI4L, 0)
			&& write32(m, R_POS, m->uart == &huart5 ? -target : target) // 위치 방향
			&& set16(m, R_SPEED, rpm)
			&& set16(m, R_ACC, ACC_MS)
			&& set16(m, R_WAIT, WAIT_MS)
			&& set16(m, R_DI4L, 1);
}

int motor_stop(Motor *m) {
	int ok;

	ok = set16(m, R_DI4L, 0);
	return set16(m, R_HOME_SEL, 0) && ok;
}

/* AIM 모터 비상정지 설정/해제 */
int motor_estop(Motor *m, int on) {
    return set16(m, R_DI5L, on);
}
