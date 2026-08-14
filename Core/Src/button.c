/*
 * button.c
 *
 *  Created on: Jul 22, 2026
 *      Author: kotec
 */

#include "button.h"
#include "net.h"
#include "motor.h"

#define DEBOUNCE 200U

volatile int run;
volatile int pause;
volatile int estop;

static volatile int start_req;
static volatile int pause_req;
static volatile int estop_req;
static volatile int full;  // 전부 1일때 만재 나느 상황

/* F1~F4 현재 만재 입력을 비트로 읽는다 */
static int full_now(void) {
	return (HAL_GPIO_ReadPin(F1_GPIO_Port, F1_Pin) ? 0 : 1)
			| (HAL_GPIO_ReadPin(F2_GPIO_Port, F2_Pin) ? 0 : 2)
			| (HAL_GPIO_ReadPin(F3_GPIO_Port, F3_Pin) ? 0 : 4)
			| (HAL_GPIO_ReadPin(F4_GPIO_Port, F4_Pin) ? 0 : 8);
}

/* 한 번 감지된 만재는 해제 전까지 유지한다 */
int full_get(void) {
	full |= full_now();
	return full;
}

/* 센서가 다 0일 때만 해제한다 */
int full_clear(void) {
	if (full_now())
		return 0;

	full = 0;
	return 1;
}
/* 대기 상태 램프 */
static void lamp_idle(void) {
	HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_SET);
}

/* 자동 운전 상태 램프 */
static void lamp_run(void) {
	HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_SET);
}

/* 일시정지 상태 램프 */
static void lamp_pause(void) {
	HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_RESET);
}

/* 비상정지 상태 램프 */
static void lamp_estop(void) {
	HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_RESET);
}

/* 버튼 및 모터 전원 초기화 */
void button_init(void) {
	run = 0;
	pause = 0;

	estop = HAL_GPIO_ReadPin(ESTOP_btn_GPIO_Port, ESTOP_btn_Pin);

	start_req = 0; // 시작 버튼
	pause_req = 0; // 일시정지 버튼
	estop_req = estop; // 전원 투입 때 눌려 있으면 정지 요청
	full = 0;

	/* MOTOR_ON은 Active High */
	HAL_GPIO_WritePin( MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET);

	if (estop)
		lamp_estop();
	else
		lamp_idle();
}

/* GPIO 외부 인터럽트 */
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	static uint32_t last[3];
	uint32_t now = HAL_GetTick();

	/* PG1: 시작 또는 재시작 */
	if (pin == MOTOR_START_btn_Pin) {
		if (now - last[0] < DEBOUNCE)
			return;

		last[0] = now;

		if (!estop)
			start_req = 1;

		return;
	}

	/* STOP 버튼: 요청만 세운다 */
	if (pin == STOP_btn_Pin) {
		if (now - last[1] < DEBOUNCE)
			return;

		last[1] = now;

		pause_req = 1;

		/* 눌린 것은 바로 알려준다 */
		HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
		return;
	}

	/* ESTOP 버튼: 비상정지 또는 해제 */
	if (pin == ESTOP_btn_Pin) {
		if (now - last[2] < DEBOUNCE)
			return;

		last[2] = now;

		estop = HAL_GPIO_ReadPin(
		ESTOP_btn_GPIO_Port,
		ESTOP_btn_Pin);

		estop_req = 1;

		if (estop) {
			run = 0;
			pause = 0;

			lamp_estop();
		} else {
			lamp_idle();
		}
	}
}

/* 블로킹 동작 중 확인. 하던 명령은 끝내니 ESTOP 만 끊는다 */
int button_stop_requested(void) {
	return estop;
}

/* 버튼 요청 처리 */
void button_run(void) {
	full |= full_now();   /* 만재 센서 순간 감지 */

	/* 비상정지 및 해제 */
	if (estop_req) {
		estop_req = 0;
		pause_req = 0;

		motor_estop(&motorX, estop);
		motor_estop(&motorY, estop);

		if (estop) {
			net_cmd("01S_1");
			print("ESTOP\r\n");
		} else {
			print("ESTOP CLEAR\r\n");
			net_cmd("01I");
		}
		return;
	}

	/* 일시정지 */
	if (pause_req) {
		pause_req = 0;
		pause = 1;
		run = 0;

		lamp_pause();
		print("01S_1\r\n");
		return;
	}

	if (!start_req)
		return;

	start_req = 0;
	full_clear();         /* 시작 버튼에서 만재 해제 */

	/* 일시정지 해제 */
	if (pause) {
		pause = 0;
		run = 1;

		lamp_run();
		print("START\r\n");
	}
}
