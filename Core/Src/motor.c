/*
 * motor.c
 *
 *  Created on: Jul 27, 2026
 *      Author: HWNOT
 */
#include "motor.h"
#include <string.h>
#include <stdarg.h>

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
/* 기존 motor.c의 32bit 순서 그대로: LOW -> HIGH */
#define REG32(v) \
	((uint16_t)(v)), ((uint16_t)((uint32_t)(v) >> 16))


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

/* ===== Modbus 06H / 10H / 03H ===== */

/* 06H : 단일 16bit 레지스터 쓰기 */
static int write16(Motor *m, uint16_t reg, uint16_t val)
{
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

	return bus_xfer(m, tx, 8, rx, 8) == HAL_OK
			&& !memcmp(rx, tx, 8);
}


/* 10H : 시작 주소부터 연속된 레지스터를 한 번에 쓴다 */
static int write_regs(Motor *m, uint16_t reg, uint8_t qty, ...)
{
	uint8_t tx[29], rx[8];
	uint16_t crc, val;
	va_list ap;

	if (qty < 2 || qty > 10)
		return 0;

	tx[0] = ID;
	tx[1] = 0x10;
	tx[2] = reg >> 8;
	tx[3] = reg;
	tx[4] = 0;
	tx[5] = qty;
	tx[6] = qty * 2;

	va_start(ap, qty);

	for (uint8_t i = 0; i < qty; i++) {
		val = (uint16_t)va_arg(ap, int);

		tx[7 + i * 2] = val >> 8;
		tx[8 + i * 2] = val;
	}

	va_end(ap);

	crc = crc16(tx, 7 + qty * 2);

	tx[7 + qty * 2] = crc;
	tx[8 + qty * 2] = crc >> 8;

	if (bus_xfer(m, tx, 9 + qty * 2, rx, 8) != HAL_OK)
		return 0;

	return rx[2] == tx[2]
			&& rx[3] == tx[3]
			&& rx[4] == 0
			&& rx[5] == qty;
}


/* 03H : 16bit / 32bit 읽기를 하나로 처리 */
static int read_regs(Motor *m, uint16_t reg, int *out, uint8_t qty)
{
	uint8_t tx[8], rx[9], rn;
	uint16_t crc, lo, hi;

	if (qty < 1 || qty > 2)
		return 0;

	tx[0] = ID;
	tx[1] = 0x03;
	tx[2] = reg >> 8;
	tx[3] = reg;
	tx[4] = 0;
	tx[5] = qty;

	crc = crc16(tx, 6);
	tx[6] = crc;
	tx[7] = crc >> 8;

	rn = 5 + qty * 2;

	if (bus_xfer(m, tx, 8, rx, rn) != HAL_OK
			|| rx[2] != qty * 2)
		return 0;

	/* 현재 오른쪽 motor.c의 word 순서 유지 */
	lo = ((uint16_t)rx[3] << 8) | rx[4];

	if (qty == 1) {
		*out = lo;
		return 1;
	}

	hi = ((uint16_t)rx[5] << 8) | rx[6];

	*out = (int)(((uint32_t)hi << 16) | lo);

	return 1;
}


/* ===== 초기 설정 ===== */

int motor_init(Motor *m)
{
	int v;

	if (!read_regs(m, R_ADDR, &v, 1) || v != ID)
		return 0;

	return
			/* DI 기능과 Logic을 먼저 모두 해제 */
			write_regs(m, R_DI1, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
			/* 기존 오른쪽 motor.c의 DI 설정 그대로 */
			&& write_regs(m, R_DI1, 10, 31, 0, 1,  0, 32, 0, 28, 0, 34, 0)
			/* 기존 설정값 그대로 */
			&& write16(m, R_MODE, 1)
			&& write16(m, R_SRC, 2)
			&& write_regs(m, R_RUN, 3, 0, 1, 0)
			&& write16(m, R_TYPE, 1)
			/* 기존 X/Y HOME 방향 및 값 그대로 */
			&& write_regs(m, R_HOME_MODE, 4,
					m->uart == &huart5 ? 0 : 1, 350, 100, 300) // 원점 찾는 속도 , 센서 근처에서 다시 찾는 속도 , 원점 복귀 가감속
			&& write_regs(m, R_HOME_OFF, 2, REG32(0))
			/* 기존 DI 중복 설정 확인 */
			&& read_regs(m, R_DI1, &v, 1) && v == 31
			&& read_regs(m, R_DI2, &v, 1) && v == 1
			&& read_regs(m, R_DI3, &v, 1) && v == 32
			&& read_regs(m, R_DI4, &v, 1) && v == 28
			&& read_regs(m, R_DI5, &v, 1) && v == 34

			/* Servo ON */
			&& write16(m, R_DI2L, 1);
}

/* ===== 동작 ===== */
/* 센서를 찾아 원점복귀 */
int motor_home_on(Motor *m)
{
	return write16(m, R_HOME_SEL, 4);
}


/* 현재 위치를 소프트웨어 0으로 설정 */
int motor_zero(Motor *m)
{
	return write16(m, R_HOME_SEL, 6);
}


/* H0B_07 현재 위치 읽기 */
int motor_pos(Motor *m, int *out)
{
	return read_regs(m, R_REALPOS, out, 2);
}


/* H11_04=1 : 원점 기준 절대 목표 위치로 이동 */
int motor_move(Motor *m, int rpm, int target)
{
	int pos = m->uart == &huart5 ? -target : target;

	return write16(m, R_DI4L, 0)
			&& write_regs(m, R_POS, 5, REG32(pos), rpm, ACC_MS, WAIT_MS)
			&& write16(m, R_DI4L, 1);
}


int motor_stop(Motor *m)
{
	int ok;

	ok = write16(m, R_DI4L, 0);

	return write16(m, R_HOME_SEL, 0) && ok;
}


/* AIM 모터 비상정지 설정/해제 */
int motor_estop(Motor *m, int on)
{
	return write16(m, R_DI5L, on);
}
