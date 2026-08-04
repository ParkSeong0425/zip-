/*
 * m_motor.c  -  MKS SERVO57D_RS485.  매뉴얼 V1.0.6
 *
 * UART4 57600 8N1, 방향핀 rs485_Pin. 모터 주소는 ids[] 에.
 * 레지스터는 mks_put / mks_get 둘로 다 쓴다. n 은 Data 칸 바이트 수.
 *   mks_put(id, CMD_CURRENT, 3500, 2)   쪼개는 건 put 이 한다
 *   mks_put(id, CMD_ZERO, 0, 0)         Data 없음
 *   mks_get(id, CMD_AXIS, 6, &v)        Data 6바이트
 * Data 가 5바이트인 0x9D 와 byte 수가 다른 F5 만 따로 만든다.
 *
 * Check 칸은 CHECKSUM 8bit (17쪽). send 가 채운다.
 * 그룹 주소로 보내면 여러 대가 동시에 받아 응답이 없다 (17쪽, 6.4).
 */
#include "rot_test.h"
#include "FreeRTOS.h"
#include "task.h"

extern UART_HandleTypeDef huart5;

#define TX_HEAD     0xFA
#define RX_HEAD     0xFB
#define HEAD_N      3       /* Head + Slave addr + Function */
#define ACK_N       5       /* + status + Check */
#define ID			2

/* 5.2 */
#define CURRENT     800    /* mA. 57D 상한 5200 (19쪽) */
#define MSTEP       16      /* 16/32/64 여야 speed 값이 곧 RPM (6.1) */
#define EN_HOLD     2       /* En 핀과 무관하게 항상 켜짐 */
#define MODE_VFOC   5       /* SR_vFOC. 최대 3000RPM (6.1) */

/* 원점. 0x34 로 모터1 센서를 본다 */
#define HM_TRIG       0     /* Low 감지 */
#define HM_DIR        0     /* 0=CW, 1=CCW */
#define HM_RPM        100
#define HM_LIMIT      0    /* 홈 후 축 잠금 유지 */

/* 6.1 / 6.9 */
#define ACC         150     /* 1rpm 당 (256-ACC)*50us */
#define DEC         200     /* 정지 감속. 0 이면 즉시 정지 */
#define Ocha    	200     /* 도착 판정 오차 */

#define DE_ON_US    20      /* 방향핀 송신 전환 후 대기 */
#define RX_WAIT     500     /* 응답 대기 */


/* ===== 통신 ===== */

/* 태스크 안에서 도므로 HAL_Delay 대신 태스크를 재운다 */
static void wait_ms(uint32_t ms) {
	TickType_t t = pdMS_TO_TICKS(ms);

	vTaskDelay(t ? t : 1);
}

/* Check 칸. 앞 n 바이트의 합 */
static uint8_t crc(const uint8_t *p, int n) {
	uint16_t s = 0;

	while (n--)
		s += *p++;
	return (uint8_t) s;
}

/*
 * tx[n] 에 Check 를 채워 n+1 바이트를 내보낸다.
 * 바이트 사이가 1바이트 이상 벌어지면 모터가 프레임을 놓친다(17쪽).
 * F4 의 USART 는 1바이트 버퍼뿐이라 송신 직후 바로 수신에 들어가야 한다.
 */
static void send(uint8_t *tx, int n) {
	uint32_t w = DE_ON_US * (SystemCoreClock / 4000000U);

	tx[n] = crc(tx, n);
	__HAL_UART_CLEAR_OREFLAG(&huart5);
	__HAL_UART_CLEAR_FEFLAG(&huart5);
	__HAL_UART_CLEAR_NEFLAG(&huart5);
	while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE) != RESET)
		(void) huart5.Instance->DR;

	HAL_GPIO_WritePin(rs4852_GPIO_Port, rs4852_Pin, GPIO_PIN_SET);
	while (w--)
		__NOP();
	HAL_UART_Transmit(&huart5, tx, n + 1, 100);
	while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC) == RESET) {
	}
	HAL_GPIO_WritePin(rs4852_GPIO_Port, rs4852_Pin, GPIO_PIN_RESET);
}

/* 보내고 rn 바이트를 받는다. Head/addr/Function/Check 가 맞으면 1 */
static int xfer(uint8_t *tx, int n, uint8_t *rx, int rn) {
	send(tx, n);
	if (HAL_UART_Receive(&huart5, rx, rn, RX_WAIT) != HAL_OK)
		return 0;
	wait_ms(3);

	return rx[0] == RX_HEAD && rx[1] == tx[1] && rx[2] == tx[2]
			&& rx[rn - 1] == crc(rx, rn - 1);
}

/*
 * 쓰기.  FA | addr | Function | Data(n) | Check
 * 응답   FB | addr | Function | status | Check.  status 1 이면 성공
 */
int mks_write(uint8_t id, uint8_t code, uint64_t data, int n) {
	uint8_t tx[12] = { TX_HEAD, id, code };
	uint8_t rx[ACK_N];
	int i;

	if (n < 0 || n > 7)
		return 0;

	for (i = 0; i < n; i++)
		tx[HEAD_N + i] =
				(uint8_t)(data >> (8 * (n - 1 - i)));

	return xfer(tx, HEAD_N + n, rx, ACK_N)
			&& rx[3] == 1U;
}

/*
 * 읽기.  FA | addr | Function | Check
 * 응답   FB | addr | Function | Data(n) | Check
 * n=6 인 0x31 은 48비트 부호값이라 음수를 되살린다
 */
int mks_read(uint8_t id, uint8_t code, int n, int *out) {
	uint8_t tx[4] = { TX_HEAD, id, code };
	uint8_t rx[10];
	int64_t v = 0;
	int i;

	if (!xfer(tx, HEAD_N, rx, HEAD_N + n + 1))
		return 0;
	for (i = 0; i < n; i++)
		v = (v << 8) | rx[HEAD_N + i];
	if (n == 6 && (v & ((int64_t) 1 << 47)))
		v -= (int64_t) 1 << 48;
	*out = (int) v;
	return 1;
}

/*
 * 6.9  byte1 Head | byte2 addr | byte3 f5 | byte4~5 rpm | byte6 acc | byte7~10 axis | CRC
 *
 */
static int f5(uint16_t rpm, uint8_t acc, int32_t axis) {
	uint64_t data;

	// 비트 위치 순서 axis먼저 (0~31) , 그 다음 ACC는 32부터 넣어야 해서 (32~39) 그다음 rpm은 40부터 넣어야 해서 (40 ~ 55)
	data = ((uint64_t)rpm << 40) | ((uint64_t)acc << 32) | (uint32_t)axis;

	return mks_write(ID, CMD_ABS, data, 7); // write 의 데이터 패기지는 7개 나머지는 write 함수에서 이뤄진다
}

/* ===== 설정 ===== */

/* 0x8C: respond 1바이트 | active 1바이트 */
static int rsp_set(uint8_t respond, uint8_t active) {
	uint64_t data;

	if (respond > 1 || active > 1)
		return 0;

	data = ((uint64_t)respond << 8)
			| (uint64_t)active;

	return mks_write(ID, CMD_RESPOND, data, 2);
}


/* 0x90: HmTrig | HmDir | HmSpeed 2바이트 | EndLimit */
static int home_set(uint8_t trig, uint8_t dir,
		uint16_t rpm, uint8_t limit) {
	uint64_t data;

	if (trig > 1 || dir > 1 || rpm > 3000 || limit > 1)
		return 0;

	data = ((uint64_t)trig << 32)
			| ((uint64_t)dir << 24)
			| ((uint64_t)rpm << 8)
			| (uint64_t)limit;

	return mks_write(ID, CMD_HOME_SET, data, 5);
}

int mks_init(void) {
	wait_ms(1000);

	return rsp_set(1, 0)
			&& mks_write(ID, CMD_MODE, MODE_VFOC, 1)
			&& mks_write(ID, CMD_CURRENT, CURRENT, 2)
			&& mks_write(ID, CMD_MSTEP, MSTEP, 1)
			&& mks_write(ID, CMD_EN_LVL, EN_HOLD, 1)
			&& mks_write(ID, CMD_STALL, 1, 1)
			&& home_set(HM_TRIG, HM_DIR, HM_RPM, HM_LIMIT)
			&& mks_write(ID, CMD_ENABLE, 1, 1);
}



/* ===== 이동 ===== */

// 도착 했는지 알려주는 함수
int mks_done(int axis) {
	int state;
	int now;
	int64_t error;

	if (!mks_read(ID, CMD_STATE, 1, &state))
		return 0;

	if (state != ST_STOP)
		return 0;

	if (!mks_read(ID, CMD_AXIS, 6, &now))
		return 0;

	error = (int64_t)now - (int64_t)axis;

	return error >= -(int64_t)Ocha
			&& error <= (int64_t)Ocha;
}

int mks_move(int rpm, int axis) {
	return rpm >= 1 && rpm <= 3000
			&& f5((uint16_t)rpm, ACC, (int32_t)axis);
}


int mks_home(void) {
	return mks_write(ID, CMD_GO_HOME, 0, 0);
}

int mks_zero(void) {

	return mks_write(ID, CMD_ZERO, 0, 0);
}

int mks_stop(void) {

	return f5(0, DEC, 0);
}


int mks_r(void) {
	return mks_move(ROT_RPM, ROT_R);
}

int mks_l(void) {
	return mks_move(ROT_RPM, ROT_L);
}

int mks_c(void) {
	return mks_move(ROT_RPM, ROT_C);
}
