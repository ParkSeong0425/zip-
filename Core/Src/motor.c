#include "motor.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/* X = UART4, Y = UART5, 둘 다 AIMotor 주소 1 */
Motor motorX = { &huart4, rs485_GPIO_Port, rs485_Pin, 0 };
Motor motorY = { &huart5, rs4852_GPIO_Port, rs4852_Pin, 0 };

/* ===== 레지스터 주소 ===== */

#define ID              1U

#define R_MODE          0x0200
#define R_DIR           0x0202
#define R_DI1           0x0302
#define R_DI1L          0x0303
#define R_DI2           0x0304
#define R_DI2L          0x0305
#define R_DI3           0x0306
#define R_DI4           0x0308
#define R_DI4L          0x0309
#define R_DI5           0x030A
#define R_DI5L          0x030B

#define R_SPEED_A       0x0600
#define R_SPEED_SEL     0x0602
#define R_SPEED_CMD     0x0603
#define R_SPEED_ACC     0x0605
#define R_SPEED_DEC     0x0606

#define R_SRC           0x0500
#define R_RUN           0x1100
#define R_REG           0x1101
#define R_BEGIN         0x1102
#define R_TYPE          0x1104
#define R_POS           0x110C
#define R_SPEED         0x110E
#define R_ACC           0x110F
#define R_WAIT          0x1110

#define R_REALPOS       0x0B07
#define R_DI            0x0B03
#define R_ADDR          0x0C00
#define R_FAULT_RST     0x0D01

#define ACC_MS          1000    /* 위치이동 가감속 시간 */
#define WAIT_MS         500     /* 한 구간 이동 후 대기 */

/* ===== 전역 상태 ===== */

static int x_mode = -1; /* -1=미설정 0=속도 1=위치 */
static int y_mode = -1;

void print(const char *s) {
	HAL_UART_Transmit(&huart3, (uint8_t*) s, strlen(s), 100);
}

static char name(Motor *m) {
	return (m == &motorX) ? 'X' : 'Y';
}

static int* mode_ptr(Motor *m) {
	return (m == &motorX) ? &x_mode : &y_mode;
}

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

static HAL_StatusTypeDef bus_xfer(Motor *m, uint8_t *tx, uint16_t tn,
		uint8_t *rx, uint16_t rn) {
	HAL_StatusTypeDef r;

	bus_clear(m);

	/* 송신 방향 */
	HAL_GPIO_WritePin(m->port, m->pin, GPIO_PIN_SET);
	HAL_Delay(1);

	r = HAL_UART_Transmit(m->uart, tx, tn, 100);

	/* 송신 완료를 기다린 뒤 수신 방향 */
	while (__HAL_UART_GET_FLAG(m->uart, UART_FLAG_TC) == RESET) {
	}
	for (volatile uint32_t i = 0; i < 100; i++) {
	}
	HAL_GPIO_WritePin(m->port, m->pin, GPIO_PIN_RESET);

	if (r == HAL_OK)
		r = HAL_UART_Receive(m->uart, rx, rn, 500);

	HAL_Delay(20);
	return r;
}

/* ===== Modbus 읽기/쓰기 ===== */

/* 0x06 단일 레지스터 쓰기. 응답은 요청과 완전히 같아야 한다 */
static int write16(Motor *m, uint16_t reg, uint16_t val) {
	uint8_t tx[8], rx[8];
	uint16_t crc;
	char b[64];

	tx[0] = ID;
	tx[1] = 0x06;
	tx[2] = reg >> 8;
	tx[3] = reg;
	tx[4] = val >> 8;
	tx[5] = val;

	crc = crc16(tx, 6);
	tx[6] = crc;
	tx[7] = crc >> 8;

	if (bus_xfer(m, tx, 8, rx, 8) != HAL_OK || memcmp(rx, tx, 6)
			|| rx[6] != tx[6] || rx[7] != tx[7]) {

		snprintf(b, sizeof(b), "%c WR FAIL reg=%04X val=%04X\r\n", name(m), reg,
				val);
		print(b);
		return 0;
	}
	return 1;
}

/* 쓰기 3회 재시도 */
static int set16(Motor *m, uint16_t reg, uint16_t val) {
	for (int i = 0; i < 3; i++) {
		if (write16(m, reg, val)) {
			HAL_Delay(30);
			return 1;
		}
		HAL_Delay(50);
	}
	return 0;
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

static int cfg(Motor *m, int dir) {
	if (!set16(m, R_DI1L, 0))
		return 0;
	if (!set16(m, R_DI2L, 0))
		return 0;
	if (!set16(m, R_DI4L, 0))
		return 0;
	if (!set16(m, R_DI5L, 0))
		return 0;

	if (!set16(m, R_DI1, 31))
		return 0; /* 원점 센서 */
	if (!set16(m, R_DI2, 1))
		return 0; /* 서보 ON */
	if (!set16(m, R_DI3, 0))
		return 0;
	if (!set16(m, R_DI4, 28))
		return 0; /* 위치 시작 */
	if (!set16(m, R_DI5, 34))
		return 0; /* 비상 정지 */
	if (!set16(m, R_DIR, dir))
		return 0;

	set16(m, R_FAULT_RST, 1);
	set16(m, R_FAULT_RST, 0);
	return 1;
}

/* H0C_00 축 주소를 읽어 통신 확인 */
static int check(Motor *m) {
	uint16_t v;

	for (int i = 0; i < 2; i++) {
		if (read16(m, R_ADDR, &v) && v == ID)
			return 1;
		HAL_Delay(30);
	}
	return 0;
}

int motor_pos_mode(Motor *m) {
	int *mode = mode_ptr(m);

	if (*mode == 1)
		return 1;

	if (!set16(m, R_SPEED_CMD, 0) || !set16(m, R_DI2L, 0)
			|| !set16(m, R_MODE, 1) || !set16(m, R_SRC, 2)
			|| !set16(m, R_RUN, 0) || !set16(m, R_REG, 1)
			|| !set16(m, R_BEGIN, 0) || !set16(m, R_DI4L, 0)
			|| !set16(m, R_DI2L, 1)) {
		*mode = -1;
		return 0;
	}

	*mode = 1;
	return 1;
}

int motor_speed_mode(Motor *m) {
	int *mode = mode_ptr(m);

	if (*mode == 0)
		return 1;

	if (!set16(m, R_SPEED_CMD, 0) || !set16(m, R_DI2L, 0)
			|| !set16(m, R_MODE, 0) || !set16(m, R_SPEED_A, 0)
			|| !set16(m, R_SPEED_SEL, 0) || !set16(m, R_SPEED_ACC, 200) || /* 200ms 가속 */
			!set16(m, R_SPEED_DEC, 20) || /* 20ms 감속, 0이면 충격 */
			!set16(m, R_DI2L, 1)) {
		*mode = -1;
		return 0;
	}

	*mode = 0;
	return 1;
}

int motor_init(Motor *m, int dir) {
	char b[32];

	*mode_ptr(m) = -1;

	if (!check(m) || !cfg(m, dir) || !motor_pos_mode(m)) {
		snprintf(b, sizeof(b), "%c INIT FAIL\r\n", name(m));
		print(b);
		return 0;
	}
	return 1;
}

/* ===== 동작 ===== */

int motor_speed(Motor *m, int rpm) {
	if (rpm < -6000 || rpm > 6000)
		return 0;
	if (!motor_speed_mode(m))
		return 0;
	return set16(m, R_SPEED_CMD, (uint16_t) (int16_t) rpm);
}

/* DI1 원점 센서: 1=ON, 0=OFF, -1=통신 실패 */
int motor_home_on(Motor *m) {
	uint16_t di;

	if (!read16(m, R_DI, &di))
		return -1;
	return (di & 1) ? 1 : 0;
}

/* 현재 위치를 소프트웨어 0으로 설정 */
int motor_zero(Motor *m) {
	int raw;

	if (!read32(m, R_REALPOS, &raw) && !read32(m, R_REALPOS, &raw))
		return 0;
	m->off = -raw;
	return 1;
}

int motor_pos(Motor *m, int *out) {
	int raw;

	if (!read32(m, R_REALPOS, &raw))
		return 0;
	*out = raw + m->off;
	return 1;
}

/*
 * target은 원점 센서를 0으로 하는 소프트웨어 목표 좌표다.
 * H11_04=1 위치 명령에는 목표와 현재 위치의 차이만 보낸다.
 */
int motor_move(Motor *m, int rpm, int target) {
	int now;

	if (!motor_pos_mode(m))
		return 0;
	if (!motor_pos(m, &now) && !motor_pos(m, &now))
		return 0;

	if (!set16(m, R_DI4L, 0))
		return 0;
	if (!set16(m, R_TYPE, 1))
		return 0;
	if (!write32(m, R_POS, target - now))
		return 0;
	if (!set16(m, R_SPEED, rpm))
		return 0;
	if (!set16(m, R_ACC, ACC_MS))
		return 0;
	if (!set16(m, R_WAIT, WAIT_MS))
		return 0;

	return set16(m, R_DI4L, 1);
}

int motor_stop(Motor *m) {
	if (*mode_ptr(m) == 0)
		return set16(m, R_SPEED_CMD, 0);
	return set16(m, R_DI4L, 0);
}
