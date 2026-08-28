/*
 * button.c
 *
 * 버튼은 PULLUP 입력이라 눌렀을 때 LOW 다. 50ms 이상 눌리면 한 번만 동작한다.
 * 만재는 정지하지 않는다. 해당 층 PO 를 MO 로 바꾸는 판단에만 쓴다.
 */

#include "button.h"
#include "net.h"
#include "motor.h"
#include <stdio.h>
int alarm_get(void); void alarm_set(int code); extern volatile int card_ok;

#define BUTTON_COUNT 5   /* button_run 10ms x 5 = 50ms */
#define TICK_FAST    12  /* 240ms 점멸 */
#define TICK_SLOW    25  /* 500ms 점멸 */
#define FULL_DELAY   1000 /* 만재가 비고 다시 시작하기까지 */
#define RFID_DELAY   1000 /* RFID가 인식되고 다시 시작하기까지 */

volatile int run;
volatile int pause;
volatile int estop;

static int lamp_count;  /* button_run 10ms 호출 횟수 */
static char lamp_mode;  /* W 운전, P 일시정지, E ESTOP, I 원점복귀, A 알람 */
static int tcp_mode[2], tcp_left[2], tcp_count, tcp_on, tcp_auto;
static volatile char pause_reason; /* M 버튼, F 만재, S RFID */
static int full_mask;       /* 정지시킨 층 비트 */
static uint32_t full_time;  /* 만재가 비어진 시각 */
static uint32_t rfid_time;  /* 만재가 비어진 시각 */

/* F1~F4 현재 만재 입력을 비트로 읽는다 */
static int full_now(void) {
	return (HAL_GPIO_ReadPin(F1_GPIO_Port, F1_Pin) ? 0 : 1)
			| (HAL_GPIO_ReadPin(F2_GPIO_Port, F2_Pin) ? 0 : 2)
			| (HAL_GPIO_ReadPin(F3_GPIO_Port, F3_Pin) ? 0 : 4)
			| (HAL_GPIO_ReadPin(F4_GPIO_Port, F4_Pin) ? 0 : 8);
}

/* 만재는 저장하지 않는다. 항상 지금 값을 준다 */
int full_read(void) { return full_now(); }
int full_get(void)  { return full_now(); }

/* 눌린 시간을 센다. 뗀 순간 0 이 되고 BUTTON_COUNT 에서 한 번만 걸린다 */
void button_50ms(GPIO_TypeDef *port, uint16_t pin, uint8_t *count) {
	if (HAL_GPIO_ReadPin(port, pin) != GPIO_PIN_RESET) { *count = 0; return; }
	if (*count < BUTTON_COUNT + 1) (*count)++;
}

/* 자동 운전 상태 램프 */
void lamp_ready_start(void) {
	lamp_mode = 'W'; lamp_count = 0;
	HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_RESET);
}

/* 일시정지: STOP 500ms 점멸 */
void lamp_pause(void) {
	if (lamp_mode != 'P') {
		lamp_mode = 'P'; lamp_count = 0;
		HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
		return;
	}
	if (++lamp_count < TICK_SLOW) return;
	lamp_count = 0;
	HAL_GPIO_TogglePin(STOP_GPIO_Port, STOP_Pin);
}

/* 램프 전체 점멸. tick 주기, motor는 MOTOR_ON 포함 여부 */
void lamp_blink(char mode, int tick, int motor) {
	if (lamp_mode != mode) {
		lamp_mode = mode; lamp_count = 0;
		HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_SET);
		return;
	}
	if (++lamp_count < tick) return;
	lamp_count = 0;
	HAL_GPIO_TogglePin(STOP_GPIO_Port, STOP_Pin);
	HAL_GPIO_TogglePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin);
	HAL_GPIO_TogglePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin);
	HAL_GPIO_TogglePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin);
	if (motor) HAL_GPIO_TogglePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin);
}

/* ESTOP과 RFID 미인식: 240ms, MOTOR_ON 유지 */
void lamp_estop(void) { lamp_blink('E', TICK_FAST, 0); }

/* 원점복귀: 500ms, MOTOR_ON 포함 */
void lamp_home(void) { lamp_blink('I', TICK_SLOW, 1); } // 모터 전원 램프도 같이 일때는 1

/* 모터 알람: 240ms, MOTOR_ON 포함 */
void lamp_alarm(void) { lamp_blink('A', TICK_FAST, 1); }

/* TCP LAMP 명령을 10ms마다 실행한다 */
static void lamp_tcp(void) {
	int i;
	if (tcp_mode[0] < 2) HAL_GPIO_WritePin(LAMP_RED_GPIO_Port,
			LAMP_RED_Pin, tcp_mode[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	if (tcp_mode[1] < 2) HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port,
			LAMP_GREEN_Pin, tcp_mode[1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	if (++tcp_count >= TICK_SLOW) {
		tcp_count = 0;
		if (tcp_mode[0] == 2) HAL_GPIO_TogglePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin);
		if (tcp_mode[1] == 2) HAL_GPIO_TogglePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin);
	}
	for (i = 0; i < 2; i++)
		if (tcp_left[i] > 0 && --tcp_left[i] == 0) tcp_mode[i] = 0;
	if (tcp_auto && !tcp_left[0] && !tcp_left[1]) tcp_on = 0;
}

/* 01LL_1_1_10&L_1_2_00: 적색 점등, 녹색 소등 */
void lamp_cmd(char *s) {
	int c1, id1, m1, t1, c2, id2, m2, t2; char b[64];
	if (sscanf(s, "01LL_%d_%d_%1d%d&L_%d_%d_%1d%d", &c1, &id1, &m1, &t1,
			&c2, &id2, &m2, &t2) != 8 || c1 != 1 || c2 != 1 || id1 < 1
			|| id1 > 2 || id2 < 1 || id2 > 2 || id1 == id2 || m1 < 0
			|| m1 > 2 || m2 < 0 || m2 > 2 || t1 < 0 || t1 > 60 || t2 < 0 || t2 > 60) {
		snprintf(b, sizeof(b), "01%cL_bad_data", NAK); reply(b); return;
	}
	tcp_mode[id1 - 1] = m1; tcp_mode[id2 - 1] = m2;
	tcp_left[id1 - 1] = m1 == 2 ? (t1 ? t1 * 100 : -1) : 0;
	tcp_left[id2 - 1] = m2 == 2 ? (t2 ? t2 * 100 : -1) : 0;
	tcp_auto = (m1 == 2 && t1) || (m2 == 2 && t2);
	tcp_count = 0; tcp_on = 1; lamp_ready_start();
	snprintf(b, sizeof(b), "01%c%s", ACK, s + 2); reply(b);
}

/* ESTOP, 알람, 원점복귀, TCP 명령, 일시정지 순서로 램프를 고른다 */
void lamp_run(char state, int card) {
	if (estop) lamp_estop();
	else if (state == 'A') lamp_alarm();
	else if (state == 'I')  lamp_home();
	else if (tcp_on) lamp_tcp();
	else if (!card) lamp_estop();
	else if (pause) lamp_pause();
	else lamp_ready_start();
}

void button_init(void) {
	run = 1;       /* 전원 시작 시 운전 가능 상태 */
	pause = 0;
	estop = HAL_GPIO_ReadPin(ESTOP_btn_GPIO_Port, ESTOP_btn_Pin);
	if (estop) {
		alarm_set(9);
		lamp_estop();
	} else {
		lamp_ready_start();
	}
}

/* 일시정지. M 버튼, F 만재, S RFID */
void pause_on(char why) {
	pause = 1;
	run = 0;
	pause_reason = why;
	lamp_pause();
	pause_msg(why);
}

/* 만재로 정지한다. mask 층이 비면 다시 시작한다 */
void pause_full(int mask) {
	full_mask = mask;
	full_time = 0;
	pause_on('F');
}

/* PAUSE를 풀고 작업을 이어간다 */
static void pause_off(void) {
	pause = 0; run = 1; pause_reason = 0; full_mask = 0;
	pause_msg(0); lamp_ready_start();
}

/* 블로킹 동작 중 ESTOP이나 알람이면 현재 명령을 끝낸다 */
int button_stop_requested(void) {
	return estop || alarm_get();
}

/* ESTOP 입력이 바뀐 순간만 처리한다 */
static void estop_change(int now)
{
    estop = now;
    motor_estop(&motorX, estop);
    motor_estop(&motorY, estop);
    pause_msg(0);

    if (estop) {
        /* ESTOP 전에 알람이 있었다면 A로 기억 */
        pause_reason = alarm_get() ? 'A' : 0;
        run = 0;
        pause = 0;
        alarm_set(9);
        lamp_estop();
        net_cmd("01S_1");
        print("ESTOP\r\n");
    } else if (run && pause && pause_reason == 'A') {
        /* RUN+PAUSE 복구 요청 후 ESTOP을 해제한 경우 */
        pause = 0;
        pause_reason = 0;
        lamp_ready_start();
        net_cmd("01I");
        print("ALARM CLEAR HOME\r\n");
    } else {
        run = 0;
        pause = 0;
        pause_reason = 0;
        alarm_set(0);
        lamp_ready_start();
        print("ESTOP CLEAR\r\n");
    }
}
/* 버튼 요청 처리 */
void button_run(void) {
	static uint8_t run_count, pause_count;
	int now = HAL_GPIO_ReadPin(ESTOP_btn_GPIO_Port, ESTOP_btn_Pin);

	button_50ms(MOTOR_START_btn_GPIO_Port, MOTOR_START_btn_Pin, &run_count);
	button_50ms(STOP_btn_GPIO_Port, STOP_btn_Pin, &pause_count);

	if (now != estop) {
	    estop_change(now);
	    return;
	}

	/* 기존 알람 후 ESTOP 상태에서 RUN+PAUSE를 같이 누르면 복구 요청 */
	if (estop) {
	    if (!run && pause_reason == 'A'
	            && run_count >= BUTTON_COUNT
	            && pause_count >= BUTTON_COUNT) {
	        run = 1;
	        pause = 1;
	        pause_msg(0);
	        print("ALARM CLEAR READY\r\n");
	    }
	    return;
	}

	/* 만재: 해당 층이 비고 FULL_DELAY가 지나면 이어서 한다 */
	if (pause && pause_reason == 'F') {
		if (full_now() & full_mask) full_time = 0;
		else if (!full_time) full_time = HAL_GetTick();
		else if (HAL_GetTick() - full_time >= FULL_DELAY) {
			pause_off(); print("FULL CLEAR\r\n");
		}
		return;
	}

	/* RFID: 카드가 다시 인식되고 일정 시간이 지나면 이어서 한다 */
	if (pause && pause_reason == 'S') {
	    if (!card_ok)
	        rfid_time = 0;
	    else if (!rfid_time)
	        rfid_time = HAL_GetTick();
	    else if (HAL_GetTick() - rfid_time >= RFID_DELAY) {
	        pause_off();
	        print("RFID CLEAR\r\n");
	    }
	    return;
	}

	/* PF9: 운전 중이면 일시정지 */
	if (pause_count == BUTTON_COUNT && !run_count) {
		if (!pause) { pause_on('M'); print("PAUSE\r\n"); }
		return;
	}

	/* PG1: 일시정지 해제, 알람 복구, 원점복귀 */
	if (run_count != BUTTON_COUNT || pause_count) return;

	if (pause && pause_reason == 'M') {
		pause_off(); print("START\r\n");
	} else if (alarm_get()) {
		run = 1; pause = 0; pause_reason = 0;
		pause_msg(0); net_cmd("01I"); print("ALARM CLEAR HOME\r\n");

	} else if (!run) {
		run = 1; lamp_ready_start(); net_cmd("01I"); print("HOME\r\n");
	}
}
