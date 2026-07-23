#include "rot_test.h"
#include "save.h"
#include "tcp.h"
#include "motor.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>

extern UART_HandleTypeDef huart5;

/* ===== MKS 프로토콜 ===== */

#define ID              2U
#define TX_HEAD         0xFAU
#define RX_HEAD         0xFBU

#define C_MODE          0x82U
#define C_MSTEP         0x84U
#define C_READ_CFG      0x47U
#define C_RESPOND       0x8CU
#define C_HOME_PARAM    0x90U
#define C_HOME          0x91U
#define C_ZERO          0x92U
#define C_POS           0x31U
#define C_STATE         0xF1U
#define C_ENABLE        0xF3U
#define C_REL           0xF4U
#define C_ABS           0xF5U

#define HOME_TRIG       0U
#define HOME_DIR        0U      /* 0=CW, 1=CCW */
#define HOME_RPM        100U
#define HOME_END        0U
#define HOME_TIMEOUT    65000U
#define HOME_BACK_AXIS  -0x3555 /* R(+) 방향 약 300도 */

#define MOVE_ACC        255U    /* 255=가장 빠른 램프, 0=가감속 없음 */
#define MODE_SR_VFOC    5U
#define NORMAL_MSTEP    16U

/* ===== 하위 통신 ===== */

static uint8_t sum8(const uint8_t *p, uint16_t n) {
	uint16_t s = 0;

	while (n--)
		s += *p++;
	return (uint8_t) s;
}

static void clear_rx(void) {
	__HAL_UART_CLEAR_OREFLAG(&huart5);
	__HAL_UART_CLEAR_FEFLAG(&huart5);
	__HAL_UART_CLEAR_NEFLAG(&huart5);
	__HAL_UART_CLEAR_PEFLAG(&huart5);

	while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE) != RESET) {
		volatile uint8_t d = huart5.Instance->DR;
		(void) d;
	}
}

/* 송신 후 rn바이트 수신 */
static int xfer(uint8_t *tx, uint16_t tn, uint8_t *rx, uint16_t rn) {
	HAL_StatusTypeDef r;

	clear_rx();

	HAL_GPIO_WritePin(rs4852_GPIO_Port, rs4852_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

	r = HAL_UART_Transmit(&huart5, tx, tn, 100);

	while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC) == RESET) {
	}
	for (volatile uint32_t i = 0; i < 100; i++) {
	}
	HAL_GPIO_WritePin(rs4852_GPIO_Port, rs4852_Pin, GPIO_PIN_RESET);

	if (r != HAL_OK)
		return 0;

	r = HAL_UART_Receive(&huart5, rx, rn, 500);
	HAL_Delay(20);

	return r == HAL_OK;
}

/* 헤더, ID, 명령, 체크섬 확인 */
static int frame_ok(uint8_t *rx, uint16_t n, uint8_t cmd) {
	return rx[0] == RX_HEAD && rx[1] == ID && rx[2] == cmd
			&& rx[n - 1] == sum8(rx, n - 1);
}

/* ===== 명령 송수신 ===== */

/* 데이터 없는 명령 */
static int cmd0(uint8_t cmd, uint8_t *v) {
	uint8_t tx[4] = { TX_HEAD, ID, cmd, 0 };
	uint8_t rx[5];

	tx[3] = sum8(tx, 3);

	if (!xfer(tx, 4, rx, 5) || !frame_ok(rx, 5, cmd))
		return 0;
	if (v)
		*v = rx[3];
	return 1;
}

/* 데이터 1바이트 명령 */
static int cmd1(uint8_t cmd, uint8_t d, uint8_t *v) {
	uint8_t tx[5] = { TX_HEAD, ID, cmd, d, 0 };
	uint8_t rx[5];

	tx[4] = sum8(tx, 4);

	if (!xfer(tx, 5, rx, 5) || !frame_ok(rx, 5, cmd))
		return 0;
	if (v)
		*v = rx[3];
	return 1;
}

/* 응답이 1이어야 성공 */
static int set1(uint8_t cmd, uint8_t d) {
	uint8_t v;
	return cmd1(cmd, d, &v) && v == 1U;
}

/* 데이터 2바이트 명령. 응답이 1이어야 성공 */
static int set2(uint8_t cmd, uint8_t a, uint8_t b) {
	uint8_t tx[6] = { TX_HEAD, ID, cmd, a, b, 0 };
	uint8_t rx[5];

	tx[5] = sum8(tx, 5);

	return xfer(tx, 6, rx, 5) && frame_ok(rx, 5, cmd) && rx[3] == 1U;
}

/* 원점 파라미터 설정 */
static int home_param(void) {
	uint8_t tx[9], rx[5];

	tx[0] = TX_HEAD;
	tx[1] = ID;
	tx[2] = C_HOME_PARAM;
	tx[3] = HOME_TRIG;
	tx[4] = HOME_DIR;
	tx[5] = HOME_RPM >> 8;
	tx[6] = HOME_RPM;
	tx[7] = HOME_END;
	tx[8] = sum8(tx, 8);

	return xfer(tx, 9, rx, 5) && frame_ok(rx, 5, C_HOME_PARAM) && rx[3] == 1U;
}

static int read8(uint8_t cmd, uint8_t *out) {
	uint8_t tx[4] = { TX_HEAD, ID, cmd, 0 };
	uint8_t rx[5];

	tx[3] = sum8(tx, 3);

	if (!xfer(tx, 4, rx, 5) || !frame_ok(rx, 5, cmd))
		return 0;
	*out = rx[3];
	return 1;
}

/* 저장된 모드와 분주값을 읽는다. 47H 응답은 총 38바이트다 */
static int read_cfg(uint8_t *mode, uint8_t *mstep) {
	uint8_t tx[4] = { TX_HEAD, ID, C_READ_CFG, 0 };
	uint8_t rx[38];

	tx[3] = sum8(tx, 3);

	if (!xfer(tx, 4, rx, 38) || !frame_ok(rx, 38, C_READ_CFG))
		return 0;

	*mode = rx[3];
	*mstep = rx[7];
	return 1;
}

/* 48bit 부호값을 int로 변환해 읽는다 */
static int read48(uint8_t cmd, int *out) {
	uint8_t tx[4] = { TX_HEAD, ID, cmd, 0 };
	uint8_t rx[10];
	uint64_t raw = 0;
	int64_t v;

	tx[3] = sum8(tx, 3);

	if (!xfer(tx, 4, rx, 10) || !frame_ok(rx, 10, cmd))
		return 0;

	for (uint8_t i = 3; i <= 8; i++)
		raw = (raw << 8) | rx[i];

	if (raw & (1ULL << 47))
		raw |= 0xFFFF000000000000ULL;

	v = (int64_t) raw;
	if (v < INT_MIN || v > INT_MAX)
		return 0;

	*out = (int) v;
	return 1;
}

/* 속도, 가속도, 목표값을 함께 보내는 이동 명령 */
static int write_axis(uint8_t cmd, uint16_t rpm, uint8_t acc, int target) {
	uint8_t tx[11], rx[5];
	uint32_t p = (uint32_t) target;

	tx[0] = TX_HEAD;
	tx[1] = ID;
	tx[2] = cmd;
	tx[3] = rpm >> 8;
	tx[4] = rpm;
	tx[5] = acc;
	tx[6] = p >> 24;
	tx[7] = p >> 16;
	tx[8] = p >> 8;
	tx[9] = p;
	tx[10] = sum8(tx, 10);

	if (!xfer(tx, 11, rx, 5) || !frame_ok(rx, 5, cmd))
		return 0;

	/* 1=이동 시작, 2=이미 완료 또는 즉시 완료 */
	return rx[3] == 1U || rx[3] == 2U;
}

/* ROT 홈 전 R(+) 방향으로 약 300도 이탈한다 */
static int home_back(void) {
	uint32_t t0;
	uint8_t state;

	if (!write_axis(C_REL, HOME_RPM, MOVE_ACC, HOME_BACK_AXIS))
		return 0;

	t0 = HAL_GetTick();

	while (HAL_GetTick() - t0 < 5000U) {
		if (read8(C_STATE, &state) && state == 1U)
			return 1;
		HAL_Delay(20);
	}

	write_axis(C_REL, 0, MOVE_ACC, 0);
	return 0;
}

/* ===== 공개 함수 ===== */

int mks_pos(int *out) {
	return read48(C_POS, out);
}
int mks_state(uint8_t *out) {
	return read8(C_STATE, out);
}

/* 현재 위치를 축 좌표 0으로 설정 */
int mks_zero(void) {
	uint8_t v;
	return cmd0(C_ZERO, &v) && v == 1U;
}

int mks_stop(void) {
	return write_axis(C_ABS, 0, MOVE_ACC, 0);
}

int mks_move_abs(int rpm, int target) {
	if (rpm < 1 || rpm > 3000)
		return 0;
	return write_axis(C_ABS, (uint16_t) rpm, MOVE_ACC, target);
}

/* 부팅 직후 버스에 남은 바이트를 흘려보내며 응답을 찾는다 */
static int probe(uint8_t *state) {
	uint8_t tx[4] = { TX_HEAD, ID, C_STATE, 0 };
	uint8_t rx[16] = { 0 };
	int n = 0;

	tx[3] = sum8(tx, 3);

	clear_rx();
	HAL_GPIO_WritePin(rs4852_GPIO_Port, rs4852_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

	if (HAL_UART_Transmit(&huart5, tx, 4, 100) != HAL_OK)
		return 0;

	while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC) == RESET) {
	}
	for (volatile uint32_t i = 0; i < 100; i++) {
	}
	HAL_GPIO_WritePin(rs4852_GPIO_Port, rs4852_Pin, GPIO_PIN_RESET);

	if (HAL_UART_Receive(&huart5, &rx[0], 1, 500) != HAL_OK)
		return 0;

	for (n = 1; n < (int) sizeof(rx); n++)
		if (HAL_UART_Receive(&huart5, &rx[n], 1, 10) != HAL_OK)
			break;

	HAL_Delay(50);

	if (n != 5 || !frame_ok(rx, 5, C_STATE))
		return 0;

	*state = rx[3];
	return 1;
}

/* MKS 부팅이 끝나기 전에 여기 도달할 수 있어 재시도한다 */
int mks_init(void) {
	uint8_t state, mode = 0xFF, mstep = 0xFF;
	int ok = 0;

	HAL_Delay(1500);

	for (int i = 0; i < 10; i++) {
		if (probe(&state)) {
			ok = 1;
			break;
		}
		HAL_Delay(300);
	}

	if (!ok) {
		print("MKS F1 FAIL\r\n");
		return 0;
	}

	read_cfg(&mode, &mstep);

	/* 직렬 vFOC가 아니면 속도가 400/1500rpm으로 제한될 수 있다 */
	if (mode != MODE_SR_VFOC) {
		if (!set1(C_MODE, MODE_SR_VFOC)) {
			print("MKS 82 FAIL\r\n");
			return 0;
		}
		HAL_Delay(100);
	}

	/* 128분주에서는 명령 속도가 1/8로 줄어드므로 16분주로 맞춘다 */
	if (mstep != 16U && mstep != 32U && mstep != 64U) {
		if (!set1(C_MSTEP, NORMAL_MSTEP)) {
			print("MKS 84 FAIL\r\n");
			return 0;
		}
		HAL_Delay(100);
	}

	if (!set2(C_RESPOND, 1U, 0U)) {
		print("MKS 8C FAIL\r\n");
		return 0;
	}
	if (!home_param()) {
		print("MKS 90 FAIL\r\n");
		return 0;
	}
	if (!set1(C_ENABLE, 1U)) {
		print("MKS F3 FAIL\r\n");
		return 0;
	}

	return 1;
}

/* 광센서 원점 복귀. 상태 5를 본 뒤 1이 되면 완료 */
int mks_home(void) {
	uint8_t v;
	uint32_t t0;
	int saw_home = 0;

	if (!home_back())
		return 0;
	if (!cmd0(C_HOME, &v))
		return 0;

	if (v == 2U)
		return 1;
	if (v != 1U)
		return 0;

	t0 = HAL_GetTick();

	while (HAL_GetTick() - t0 < HOME_TIMEOUT) {
		if (mks_state(&v)) {
			if (v == 5U)
				saw_home = 1;
			if (v == 1U && (saw_home || HAL_GetTick() - t0 > 300U))
				return 1;
		}
		HAL_Delay(50);
	}

	mks_stop();
	return 0;
}

/* ===== main에서 부르는 진입점 ===== */

void rot_test_init(void) {
	tcp_init();
	save_init();
}

void rot_test_run(void) {
	tcp_run();
	save_run();
}
