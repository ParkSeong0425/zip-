#include "motor.h"
//#include "button.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;
// extern UART_HandleTypeDef huart4;   /* X axis unused */
extern UART_HandleTypeDef huart5;

// Motor motorX = { &huart4, rs485_GPIO_Port, rs485_Pin, 0 };   /* X axis unused */
Motor motorY = { &huart5, rs4852_GPIO_Port, rs4852_Pin, 0 };

#define R_MODE      0x0200
#define R_DIR       0x0202
#define R_DI2       0x0304
#define R_DI4       0x0308
#define R_DI5       0x030A
#define R_DI5L      0x030B
#define R_START     0x0305
#define R_MUL       0x0309
#define R_SRC       0x0500

#define R_HOME_RUN  0x051E   // H05_30 원점복귀 실행 (통신으로 4를 쓰면 시작)
#define R_HOME_MODE 0x051F   // H05_31 원점복귀 방향/방식 (0=정방향,1=역방향, 원점스위치 기준)
#define R_HOME_HSPD 0x0520   // H05_32 고속 원점탐색 속도
#define R_HOME_LSPD 0x0521   // H05_33 저속 원점탐색 속도
#define R_HOME_ACC  0x0522   // H05_34 원점탐색 가감속
#define R_HOME_TIME 0x0523   // H05_35 원점탐색 타임아웃(ms)

#define R_FAULT_RST 0x0D01   // H0D_01 에러(알람) 리셋

#define R_RUN       0x1100
#define R_REG       0x1101
#define R_BEGIN     0x1102
#define R_TYPE      0x1104
#define R_POS       0x110C
#define R_SPEED     0x110E
#define R_ACC       0x110F
#define R_WAIT      0x1110

#define R_REALPOS   0x0B07
#define R_DI        0x0B03

#define MOTOR_ID    0x01
#define ARRIVE_GAP  100
#define ACCDEC      1000
#define WAIT_MS     500

void print(const char *s) {
	HAL_UART_Transmit(&huart3, (uint8_t*) s, strlen(s), 100);
}

static uint16_t crc16(uint8_t *data, uint16_t len) {
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++)
			crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
	}
	return crc;
}

static void bus_tx(Motor *m) {
	HAL_GPIO_WritePin(m->port, m->pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

static void bus_rx(Motor *m) {
	while (__HAL_UART_GET_FLAG(m->uart, UART_FLAG_TC) == RESET)
		;
	for (volatile uint32_t i = 0; i < 100; i++)
		;
	HAL_GPIO_WritePin(m->port, m->pin, GPIO_PIN_RESET);
}

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

static HAL_StatusTypeDef bus_xfer(Motor *m, uint8_t *tx, uint16_t tlen,
		uint8_t *rx, uint16_t rlen) {
	bus_clear(m);
	bus_tx(m);
	HAL_StatusTypeDef r = HAL_UART_Transmit(m->uart, tx, tlen, 100);
	bus_rx(m);
	if (r == HAL_OK)
		r = HAL_UART_Receive(m->uart, rx, rlen, 500);
	HAL_Delay(20);
	return r;
}

static int write16(Motor *m, uint16_t reg, uint16_t val) {
	uint8_t tx[8], rx[8];
	tx[0] = MOTOR_ID;
	tx[1] = 0x06;
	tx[2] = reg >> 8;
	tx[3] = reg & 0xFF;
	tx[4] = val >> 8;
	tx[5] = val & 0xFF;
	uint16_t crc = crc16(tx, 6);
	tx[6] = crc & 0xFF;
	tx[7] = crc >> 8;
	if (bus_xfer(m, tx, 8, rx, 8) != HAL_OK)
		return 0;
	return (rx[0] == MOTOR_ID && rx[1] == 0x06);
}

int motor_write(Motor *m, uint16_t reg, uint16_t val) {
	return write16(m, reg, val);
}

static int write32(Motor *m, uint16_t reg, int val) {
	uint8_t tx[13], rx[8];
	uint32_t raw = (uint32_t) val;
	uint16_t low = raw & 0xFFFF;
	uint16_t high = raw >> 16;
	tx[0] = MOTOR_ID;
	tx[1] = 0x10;
	tx[2] = reg >> 8;
	tx[3] = reg & 0xFF;
	tx[4] = 0x00;
	tx[5] = 0x02;
	tx[6] = 0x04;
	tx[7] = low >> 8;
	tx[8] = low & 0xFF;
	tx[9] = high >> 8;
	tx[10] = high & 0xFF;
	uint16_t crc = crc16(tx, 11);
	tx[11] = crc & 0xFF;
	tx[12] = crc >> 8;
	if (bus_xfer(m, tx, 13, rx, 8) != HAL_OK)
		return 0;
	return (rx[0] == MOTOR_ID && rx[1] == 0x10);
}

static uint16_t read16(Motor *m, uint16_t reg) {
	uint8_t tx[8], rx[7];
	tx[0] = MOTOR_ID;
	tx[1] = 0x03;
	tx[2] = reg >> 8;
	tx[3] = reg & 0xFF;
	tx[4] = 0x00;
	tx[5] = 0x01;
	uint16_t crc = crc16(tx, 6);
	tx[6] = crc & 0xFF;
	tx[7] = crc >> 8;
	if (bus_xfer(m, tx, 8, rx, 7) != HAL_OK)
		return 0xFFFF;
	if (rx[0] != MOTOR_ID || rx[1] != 0x03)
		return 0xFFFF;
	return (rx[3] << 8) | rx[4];
}

static int read32(Motor *m, uint16_t reg, int *out) {
	uint8_t tx[8], rx[9];
	tx[0] = MOTOR_ID;
	tx[1] = 0x03;
	tx[2] = reg >> 8;
	tx[3] = reg & 0xFF;
	tx[4] = 0x00;
	tx[5] = 0x02;
	uint16_t crc = crc16(tx, 6);
	tx[6] = crc & 0xFF;
	tx[7] = crc >> 8;
	if (bus_xfer(m, tx, 8, rx, 9) != HAL_OK)
		return 0;
	if (rx[0] != MOTOR_ID || rx[1] != 0x03 || rx[2] != 0x04)
		return 0;
	uint16_t low = (rx[3] << 8) | rx[4];
	uint16_t high = (rx[5] << 8) | rx[6];
	*out = (int) (((uint32_t) high << 16) | low);
	return 1;
}

int motor_check(Motor *m) {
	return read16(m, R_DI) != 0xFFFF;
}

int motor_init(Motor *m, int dir) {
	write16(m, R_MODE, 1);
	HAL_Delay(20);
	write16(m, R_DIR, dir);
	HAL_Delay(20);

	write16(m, R_DI1, 0);
	HAL_Delay(20);
	write16(m, R_DI2, 1);
	HAL_Delay(20);
	write16(m, R_DI3, 0);
	HAL_Delay(20);
	write16(m, R_DI4, 0);
	HAL_Delay(20);
	write16(m, R_DI5, 0);
	HAL_Delay(20);
	write16(m, R_DI9, 0);
	HAL_Delay(20);

	// DI1 = 원점센서 전용. JOG로 쓰면 센서 닿는 순간 그 방향으로 계속 움직이므로 절대 금지.
	write16(m, R_DI1, 31);
	HAL_Delay(20);
	write16(m, R_DI1L, 0);
	HAL_Delay(20);

	// DI3 = 역방향 조그 (기존과 동일)
	write16(m, R_DI3, 19);
	HAL_Delay(20);
	write16(m, R_DI3L, 0);
	HAL_Delay(20);

	// DI9 = 정방향 조그 (원래 DI1이 하던 역할, 원점센서 때문에 이전함)
	write16(m, R_DI9, 18);
	HAL_Delay(20);
	write16(m, R_DI9L, 0);
	HAL_Delay(20);

	// DI5 = 비상정지
	write16(m, R_DI5, 34);
	HAL_Delay(20);
	write16(m, R_DI5L, 0);
	HAL_Delay(20);

	// DI4 = 멀티 위치 실행 enable
	write16(m, R_DI4, 28);
	HAL_Delay(20);

	write16(m, R_MUL, 0);
	HAL_Delay(20);
	write16(m, R_SRC, 2);
	HAL_Delay(20);
	write16(m, R_RUN, 0);
	HAL_Delay(20);
	write16(m, R_REG, 1);
	HAL_Delay(20);
	write16(m, R_BEGIN, 0);
	HAL_Delay(20);
	write16(m, JOG_SPEED, JOG_RPM);
	HAL_Delay(20);

	write16(m, R_HOME_HSPD, 100);
	HAL_Delay(20);
	write16(m, R_HOME_LSPD, 20);
	HAL_Delay(20);
	write16(m, R_HOME_ACC, 200);
	HAL_Delay(20);
	write16(m, R_HOME_TIME, 60000);
	HAL_Delay(20);

	return write16(m, R_START, 1);
}

void motor_sync(Motor *m, int last) {
	int raw = 0;
	if (read32(m, R_REALPOS, &raw))
		m->off = last - raw;
	else
		m->off = 0;
}

int motor_pos(Motor *m, int *out) {
	int raw = 0;
	if (!read32(m, R_REALPOS, &raw))
		return 0;
	*out = raw + m->off;
	return 1;
}

int motor_move(Motor *m, int rpm, int target) {
	int now = 0;
	if (!motor_pos(m, &now) && !motor_pos(m, &now))
		return 0;
	int delta = target - now;

	write16(m, R_MUL, 0);
	HAL_Delay(20);
	write16(m, R_TYPE, 1);
	HAL_Delay(20);
	write32(m, R_POS, delta);
	HAL_Delay(20);
	write16(m, R_SPEED, rpm);
	HAL_Delay(20);
	write16(m, R_ACC, ACCDEC);
	HAL_Delay(20);
	write16(m, R_WAIT, WAIT_MS);
	HAL_Delay(20);

	if (write16(m, R_MUL, 1))
		return 1;
	return write16(m, R_MUL, 1);
}

// 통신 실패해도 성공한 척하지 않음 - 호출부(motor_wait 등)가 실패를 알아야 함
int motor_stop(Motor *m) {
	return write16(m, R_MUL, 0);
}

int motor_begin(Motor *m, int v) {
	return write16(m, R_BEGIN, v);
}

int motor_di5(Motor *m, int v) {
	return write16(m, R_DI5L, v);
}

// jog_reg: 원점센서 쪽으로 이동시키는 조그 트리거 레지스터
// (X는 R_DI3L 역방향, Y는 R_DI9L 정방향 - DI1은 원점센서 전용이라 못 씀)
// 드라이버 자체 원점복귀(H05_30) 대신, 이미 검증된 조그 방식을 그대로 쓰고
// DI1(원점센서) 상태를 직접 폴링해서 닿는 순간 멈추고 그 자리를 소프트웨어로 0으로 삼음
int motor_home(Motor *m, int jog_reg) {
	print("home start\r\n");

	write16(m, R_MUL, 0);
	HAL_Delay(50);

	motor_write(m, JOG_SPEED, 100);   // 센서 근처라 일반 조그(100)보다 느리게
	motor_write(m, jog_reg, 1);

	uint32_t t0 = HAL_GetTick();

	while (HAL_GetTick() - t0 < 65000) {
//		if (pause || estop) {
//			motor_write(m, jog_reg, 0);
//			motor_stop(m);
//			print("home stop\r\n");
//			return 0;
//		}

		uint16_t di = read16(m, R_DI);
		if (di != 0xFFFF && (di & 0x01)) {   // DI1 = bit0 = 원점센서
			motor_write(m, jog_reg, 0);
			HAL_Delay(100);
			motor_stop(m);

			int raw = 0;
			if (!read32(m, R_REALPOS, &raw))
				read32(m, R_REALPOS, &raw);
			m->off = -raw;

			print("home ok\r\n");
			return 1;
		}
	}

	motor_write(m, jog_reg, 0);
	motor_stop(m);
	write16(m, R_FAULT_RST, 1);
	HAL_Delay(50);
	print("home timeout\r\n");
	return 0;
}

int motor_wait(Motor *mx, int xtarget, Motor *my, int ytarget, int timeout) {
	uint32_t t0 = HAL_GetTick();
	int nowx = 0, nowy = 0;
	int xdone = 0, ydone = 0;

	while (HAL_GetTick() - t0 < (uint32_t) timeout) {
//		if (pause || estop) {
//			motor_stop(mx);
//			motor_stop(my);
//			print("Stop\r\n");
//			return 0;
//		}

		if (!xdone && motor_pos(mx, &nowx)) {
			int gap = nowx - xtarget;
			if (gap < 0)
				gap = -gap;
			if (gap <= ARRIVE_GAP)
				xdone = 1;
		}
		if (!ydone && motor_pos(my, &nowy)) {
			int gap = nowy - ytarget;
			if (gap < 0)
				gap = -gap;
			if (gap <= ARRIVE_GAP)
				ydone = 1;
		}
		if (xdone && ydone) {
			print("Done\r\n");
			return 1;
		}

		HAL_Delay(50);
	}

	motor_stop(mx);
	motor_stop(my);
	print("Timeout\r\n");
	return 0;
}
