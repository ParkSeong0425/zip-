#include "rot_test.h"
#include "save.h"
#include "tcp.h"

#include <limits.h>
#include <stdint.h>

extern UART_HandleTypeDef huart5;

/* UART5 / RS4852 shared bus
 * Y Siheung motor : Modbus-RTU, ID 1
 * MKS tilt motor  : FA/FB protocol, address 2
 *
 * MKS display settings:
 * Mode     = SR_vFOC
 * UartAddr = 2
 * UartBaud = 57600
 * UartRSP  = Enable
 * respond  = 1
 * active   = 0
 * Mb_RTU   = Disable
 */
#define MKS_ID          2U

/* MKS acceleration field is 0~255.
 * Move commands use START_ACC.
 * Stop commands use STOP_ACC.
 */
#define MKS_START_ACC   255U
#define MKS_STOP_ACC    255U
#define MKS_TIMEOUT     65000UL

/* MKS FA/FB one-byte command codes */
#define MKS_MODE        0x82U
#define MKS_RESPOND     0x8CU
#define MKS_IO          0x34U
#define MKS_POS         0x31U
#define MKS_RAW         0x35U
#define MKS_STATE       0xF1U
#define MKS_ENABLE      0xF3U
#define MKS_HOME        0x91U
#define MKS_ZERO        0x92U
#define MKS_REL_AXIS    0xF4U
#define MKS_ABS_AXIS    0xF5U

#define MKS_TX_HEAD     0xFAU
#define MKS_RX_HEAD     0xFBU

/* CHECKSUM-8: sum of all preceding bytes, low 8 bits */
static uint8_t sum8(const uint8_t *data, uint16_t len)
{
	uint16_t sum = 0;

	for (uint16_t i = 0; i < len; i++) {
		sum += data[i];
	}

	return (uint8_t)sum;
}

static void bus_tx(void)
{
	HAL_GPIO_WritePin(
			rs4852_GPIO_Port,
			rs4852_Pin,
			GPIO_PIN_SET);

	HAL_Delay(1);
}

static void bus_rx(void)
{
	while (__HAL_UART_GET_FLAG(
			&huart5,
			UART_FLAG_TC) == RESET) {
	}

	for (volatile uint32_t i = 0; i < 100; i++) {
	}

	HAL_GPIO_WritePin(
			rs4852_GPIO_Port,
			rs4852_Pin,
			GPIO_PIN_RESET);
}

static void bus_clear(void)
{
	__HAL_UART_CLEAR_OREFLAG(&huart5);
	__HAL_UART_CLEAR_FEFLAG(&huart5);
	__HAL_UART_CLEAR_NEFLAG(&huart5);
	__HAL_UART_CLEAR_PEFLAG(&huart5);

	while (__HAL_UART_GET_FLAG(
			&huart5,
			UART_FLAG_RXNE) != RESET) {

		volatile uint8_t d = huart5.Instance->DR;
		(void)d;
	}
}

static int frame_ok(
		const uint8_t *rx,
		uint16_t len,
		uint8_t cmd)
{
	if (len < 4) {
		return 0;
	}

	if (rx[0] != MKS_RX_HEAD ||
		rx[1] != MKS_ID ||
		rx[2] != cmd) {
		return 0;
	}

	return sum8(rx, len - 1) == rx[len - 1];
}

static int bus_xfer(
		const uint8_t *tx,
		uint16_t tlen,
		uint8_t *rx,
		uint16_t rlen)
{
	HAL_StatusTypeDef r;

	bus_clear();
	bus_tx();

	r = HAL_UART_Transmit(
			&huart5,
			(uint8_t *)tx,
			tlen,
			100);

	bus_rx();

	if (r != HAL_OK) {
		return 0;
	}

	r = HAL_UART_Receive(
			&huart5,
			rx,
			rlen,
			500);

	HAL_Delay(20);

	return r == HAL_OK;
}

/* No-data command: FA ID CMD SUM
 * Response: FB ID CMD STATUS SUM
 */
static int cmd0(uint8_t cmd, uint8_t *status)
{
	uint8_t tx[4];
	uint8_t rx[5];

	tx[0] = MKS_TX_HEAD;
	tx[1] = MKS_ID;
	tx[2] = cmd;
	tx[3] = sum8(tx, 3);

	if (!bus_xfer(tx, sizeof(tx), rx, sizeof(rx))) {
		return 0;
	}

	if (!frame_ok(rx, sizeof(rx), cmd)) {
		return 0;
	}

	if (status != 0) {
		*status = rx[3];
	}

	return 1;
}

/* One-byte data command */
static int cmd1(
		uint8_t cmd,
		uint8_t data,
		uint8_t *status)
{
	uint8_t tx[5];
	uint8_t rx[5];

	tx[0] = MKS_TX_HEAD;
	tx[1] = MKS_ID;
	tx[2] = cmd;
	tx[3] = data;
	tx[4] = sum8(tx, 4);

	if (!bus_xfer(tx, sizeof(tx), rx, sizeof(rx))) {
		return 0;
	}

	if (!frame_ok(rx, sizeof(rx), cmd)) {
		return 0;
	}

	if (status != 0) {
		*status = rx[3];
	}

	return 1;
}

/* Two-byte data command */
static int cmd2(
		uint8_t cmd,
		uint8_t data1,
		uint8_t data2,
		uint8_t *status)
{
	uint8_t tx[6];
	uint8_t rx[5];

	tx[0] = MKS_TX_HEAD;
	tx[1] = MKS_ID;
	tx[2] = cmd;
	tx[3] = data1;
	tx[4] = data2;
	tx[5] = sum8(tx, 5);

	if (!bus_xfer(tx, sizeof(tx), rx, sizeof(rx))) {
		return 0;
	}

	if (!frame_ok(rx, sizeof(rx), cmd)) {
		return 0;
	}

	if (status != 0) {
		*status = rx[3];
	}

	return 1;
}

/* Read one-byte value: FB ID CMD DATA SUM */
static int read8(uint8_t cmd, uint8_t *out)
{
	uint8_t tx[4];
	uint8_t rx[5];

	tx[0] = MKS_TX_HEAD;
	tx[1] = MKS_ID;
	tx[2] = cmd;
	tx[3] = sum8(tx, 3);

	if (!bus_xfer(tx, sizeof(tx), rx, sizeof(rx))) {
		return 0;
	}

	if (!frame_ok(rx, sizeof(rx), cmd)) {
		return 0;
	}

	*out = rx[3];
	return 1;
}

/* Read signed int48 Axis/RAW value.
 * Response: FB ID CMD D5 D4 D3 D2 D1 D0 SUM
 */
static int read48(uint8_t cmd, int *out)
{
	uint8_t tx[4];
	uint8_t rx[10];
	uint64_t raw = 0;
	int64_t value;

	tx[0] = MKS_TX_HEAD;
	tx[1] = MKS_ID;
	tx[2] = cmd;
	tx[3] = sum8(tx, 3);

	if (!bus_xfer(tx, sizeof(tx), rx, sizeof(rx))) {
		return 0;
	}

	if (!frame_ok(rx, sizeof(rx), cmd)) {
		return 0;
	}

	for (uint8_t i = 3; i <= 8; i++) {
		raw = (raw << 8) | rx[i];
	}

	if ((raw & (1ULL << 47)) != 0U) {
		raw |= 0xFFFF000000000000ULL;
	}

	value = (int64_t)raw;

	if (value < INT_MIN || value > INT_MAX) {
		return 0;
	}

	*out = (int)value;
	return 1;
}

/* F4/F5 frame:
 * FA ID CMD speedH speedL acc axis3 axis2 axis1 axis0 SUM
 *
 * respond=1, active=0:
 * immediate response only: FB ID CMD status SUM
 */
static int write_axis(
		uint8_t cmd,
		uint16_t rpm,
		uint8_t acc,
		int target)
{
	uint8_t tx[11];
	uint8_t rx[5];
	uint32_t p = (uint32_t)target;

	tx[0] = MKS_TX_HEAD;
	tx[1] = MKS_ID;
	tx[2] = cmd;
	tx[3] = (uint8_t)(rpm >> 8);
	tx[4] = (uint8_t)rpm;
	tx[5] = acc;
	tx[6] = (uint8_t)(p >> 24);
	tx[7] = (uint8_t)(p >> 16);
	tx[8] = (uint8_t)(p >> 8);
	tx[9] = (uint8_t)p;
	tx[10] = sum8(tx, 10);

	if (!bus_xfer(tx, sizeof(tx), rx, sizeof(rx))) {
		return 0;
	}

	if (!frame_ok(rx, sizeof(rx), cmd)) {
		return 0;
	}

	/* 0 = fail, 1 = command started */
	return rx[3] == 1U;
}

int mks_init(void)
{
	uint8_t value;

	/* Communication check */
	if (!mks_state(&value)) {
		return 0;
	}

	/* 82H = 5: SR_vFOC */
	if (!cmd1(MKS_MODE, 5U, &value) || value != 1U) {
		return 0;
	}

	HAL_Delay(20);

	/* 8CH: respond=1, active=0 */
	if (!cmd2(MKS_RESPOND, 1U, 0U, &value) || value != 1U) {
		return 0;
	}

	HAL_Delay(20);

	/* F3H = 1: Enable */
	if (!cmd1(MKS_ENABLE, 1U, &value) || value != 1U) {
		return 0;
	}

	return 1;
}

int mks_check(void)
{
	uint8_t state;
	return mks_state(&state);
}

int mks_pos(int *out)
{
	return read48(MKS_POS, out);
}

int mks_raw(int *out)
{
	return read48(MKS_RAW, out);
}

int mks_io(uint8_t *out)
{
	return read8(MKS_IO, out);
}

int mks_state(uint8_t *out)
{
	return read8(MKS_STATE, out);
}

int mks_zero(void)
{
	uint8_t status;

	return cmd0(MKS_ZERO, &status) &&
		   status == 1U;
}

int mks_home(void)
{
	uint8_t status;
	uint32_t t0;
	int saw_home = 0;

	/* 91H has no data byte */
	if (!cmd0(MKS_HOME, &status)) {
		return 0;
	}

	if (status == 2U) {
		return 1;
	}

	if (status != 1U) {
		return 0;
	}

	t0 = HAL_GetTick();

	while (HAL_GetTick() - t0 < MKS_TIMEOUT) {
		uint8_t state;

		if (mks_state(&state)) {
			if (state == 5U) {
				saw_home = 1;
			}
			else if (state == 1U) {
				/* Very short home may finish before state 5 is polled. */
				if (saw_home ||
					HAL_GetTick() - t0 >= 300U) {
					return 1;
				}
			}
		}

		HAL_Delay(50);
	}

	mks_stop();
	return 0;
}

int mks_move_abs(int rpm, int target)
{
	if (rpm < 1 || rpm > 3000) {
		return 0;
	}

	return write_axis(
			MKS_ABS_AXIS,
			(uint16_t)rpm,
			MKS_START_ACC,
			target);
}

int mks_move_rel(int rpm, int delta)
{
	if (rpm < 1 || rpm > 3000) {
		return 0;
	}

	return write_axis(
			MKS_REL_AXIS,
			(uint16_t)rpm,
			MKS_START_ACC,
			delta);
}

int mks_stop(void)
{
	/* F5 position-mode deceleration stop:
	 * speed=0, STOP_ACC=255, axis=0
	 */
	return write_axis(
			MKS_ABS_AXIS,
			0U,
			MKS_STOP_ACC,
			0);
}

void rot_test_init(void)
{
	tcp_init();
	save_init();
}

void rot_test_run(void)
{
	tcp_run();
	save_run();
}
