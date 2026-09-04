 /*
 * move.c
 *
 *  Created on: Sep 1, 2026
 *      Author: HWNOT
 */
/* move.c: 모터 명령 해석과 X/Y/회전축 동작을 담당한다. */
#include "move.h"
#include "net.h"
#include "rfid.h"
#include "rot_test.h"
#include "motor.h"
#include "button.h"
#include "fram.h"
#include "cfg.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern volatile int card_ok;
extern volatile int run;
extern volatile uint32_t command_number;

int mks_link(void);
void pause_full(int mask);
int button_stop_requested(void);
void repeat_set(const char *message);

#define POSITION_GAP  50    /* 목표 위치와 현재 위치의 허용 차이 */
#define REV_PULSE     1000  /* X/Y 모터 한 바퀴 위치값 */
#define REV_MM        160   /* 풀리 한 바퀴 이동 거리 */
#define XY_GEAR       5     /* X/Y 감속비 */
#define MAX_RPM       3000
#define ROT_MAX_RPM   3000  /* 틸트 속도 기준. 높이면 빨라진다 */
#define HOME_WAIT     80000 /* 홈 최대 대기 시간 */

volatile char status = 'W';
volatile int alarm;
int rack_now = 1;
char motor_line[64];
volatile int motor_ready;
volatile int motor_busy;

static void motor_check(void);

/* PAUSE 중에는 현재 명령을 보관하고 해제될 때까지 기다린다 */
static int wait_pause(void)
{
    for (;;) {
        motor_check();

        if (button_stop_requested())
            return 0;

        if (status != 'I' && !pause && !card_ok)
            pause_on('S');

        if (!pause)
            return 1;

        osDelay(10);
    }
}

static void fail(int code)
{
    status = 'A';
    alarm = code;
    run = 0;

    if (code == 2) {
        motor_estop(&motorY, 1);
        motor_stop(&motorY);
        mks_stop();
        motor_estop(&motorX, 1);
        motor_stop(&motorX);
    } else {
        motor_estop(&motorX, 1);
        motor_stop(&motorX);
        motor_estop(&motorY, 1);
        motor_stop(&motorY);
        mks_stop();
    }

    alarm_set(code);
}

static void fail_rot(void)
{
    fail(mks_link() ? 7 : 4);
}

/* 30ms마다 X/Y 위치와 ROT 상태·위치를 확인한다 */
static void motor_check(void)
{
    static int count;
    int pos, state;

    if (++count < 3)
        return;

    count = 0;

    if (!motor_pos(&motorX, &pos)) {
        if (status != 'A' || alarm != 2) fail(2);
        return;
    }
    if (!motor_pos(&motorY, &pos)) {
        if (status != 'A' || alarm != 3) fail(3);
        return;
    }
    if (!mks_read(2, CMD_STATE, 1, &state)) {
        if (status != 'A' || alarm != 4) fail(4);
        return;
    }
    if (state == ST_FAIL) {
        if (status != 'A' || alarm != 7) fail(7);
        return;
    }
    if (!mks_read(2, CMD_AXIS, 6, &pos)
            && (status != 'A' || alarm != 4))
        fail(4);
}

/* X/Y를 가까운 시점에 출발시켜 목표 위치까지 이동한다 */
static int move_xy(int x, int y, int rpm)
{
    int x_now, y_now, x_rpm = rpm, y_rpm = rpm;
    int64_t x_distance, y_distance;

    if (!wait_pause()) return 0;

    if (!motor_pos(&motorX, &x_now)) {
        fail(2);
        return 0;
    }
    if (!motor_pos(&motorY, &y_now)) {
        fail(3);
        return 0;
    }

    x_distance = llabs((int64_t)x - x_now);
    y_distance = llabs((int64_t)-y - y_now);

    /* 긴 거리는 명령 속도, 짧은 거리는 거리 비율만큼 낮춰 도착 시간을 맞춘다 */
    if (x_distance > y_distance && x_distance)
        y_rpm = (int)((int64_t)rpm * y_distance / x_distance);
    else if (y_distance > x_distance && y_distance)
        x_rpm = (int)((int64_t)rpm * x_distance / y_distance);

    if (x_rpm < 1) x_rpm = 1;
    if (y_rpm < 1) y_rpm = 1;

    /* 기존 motor_move만 사용하며 X보다 Y를 먼저 시작한다 */
    if (!motor_move(&motorY, y_rpm, y)) {
        motor_stop(&motorX);
        motor_stop(&motorY);
        fail(3);
        return 0;
    }
    if (!motor_move(&motorX, x_rpm, x)) {
        motor_stop(&motorX);
        motor_stop(&motorY);
        fail(2);
        return 0;
    }

    for (;;) {
        motor_check();

        /* ESTOP이나 알람이면 현재 명령을 버리고 세 축을 정지한다 */
        if (button_stop_requested()) {
            motor_estop(&motorX, 1);
            motor_estop(&motorY, 1);
            motor_stop(&motorX);
            motor_stop(&motorY);
            mks_stop();
            return 0;
        }
        if (!motor_pos(&motorX, &x_now)) {
            motor_stop(&motorX);
            motor_stop(&motorY);
            fail(2);
            return 0;
        }

        if (!motor_pos(&motorY, &y_now)) {
            motor_stop(&motorX);
            motor_stop(&motorY);
            fail(3);
            return 0;
        }

        if (x_now - x >= -POSITION_GAP
                && x_now - x <= POSITION_GAP
                && y_now + y >= -POSITION_GAP
                && y_now + y <= POSITION_GAP)
            return 1;

        osDelay(10);
    }
}

/* 회전축을 목표로 보낸다. 정지되면 세우고 풀린 뒤 다시 보낸다. */
static int move_rot(char dir, int rpm, int angle)
{
    int target = 0, done;

    if (dir == 'l') target = angle * MKS_REV * ROT_GEAR / 360;
    if (dir == 'r') target = -angle * MKS_REV * ROT_GEAR / 360;

    for (;;) {
        if (!wait_pause()) return 0;
        if (!mks_move(rpm, target)) { fail_rot(); return 0; }

        for (;;) {
            motor_check();
            if (button_stop_requested()) { mks_stop(); return 0; }
            if (pause || !card_ok) { mks_stop(); break; }
            done = mks_done(target);
            if (done > 0) return 1;
            if (done < 0) { fail(done == -1 ? 4 : 7); return 0; }
            osDelay(10);
        }
    }
}

/* 입고: 중앙 -> X/Y 이동 -> 정지 확인 -> 왼쪽 틸트 */
static void MI(int x, int y, int xy_rpm, int rot_rpm, int move_delay, int mask)
{
    status = 'R'; alarm = 0;

    if (!move_rot('c', rot_rpm, 0)) return;
    if (!move_xy(x, y, xy_rpm)) return;
    if (!wait_pause()) return;

    osDelay(move_delay);
    if (button_stop_requested()) return;

    if (full_read() & mask)
        pause_full(mask);

    if (!move_rot('l', rot_rpm, 12)) return;

    /* AI 승인 전까지 R 상태를 유지한다 */
    status = 'R';
}

/* 출고: 중앙 -> X/Y 이동 -> 정지 확인 */
static void MO(int x, int y, int xy_rpm, int rot_rpm)
{
    status = 'R'; alarm = 0;

    if (!move_rot('c', rot_rpm, 0)) return;
    if (!move_xy(x, y, xy_rpm)) return;
    if (!wait_pause()) return;

    /* AO 승인 후에도 R 상태를 유지한다 */
    status = 'R';
}

/* 분배: 중앙 -> X/Y 이동 -> 정지 확인 -> 오른쪽 틸트 -> 복귀 */
static void PO(int x, int y, int xy_rpm, int rot_rpm, int angle,
        int move_delay, int return_delay, int mask)
{
    status = 'R'; alarm = 0;

    if (!move_rot('c', rot_rpm, 0)) return;
    if (!move_xy(x, y, xy_rpm)) return;
    if (!wait_pause()) return;

    osDelay(move_delay);
    if (button_stop_requested()) return;

    if (full_read() & mask)
        pause_full(mask);

    if (!move_rot('r', rot_rpm, angle)) return;

    osDelay(return_delay);
    if (button_stop_requested()) return;
    if (!move_rot('c', rot_rpm, 0)) return;

    /* EO 승인 전까지 R 상태를 유지한다 */
    status = 'R';
}

/* 틸트와 X/Y 원점복귀 */
static void I(void)
{
    int x, y, done;
    uint32_t start;

    status = 'I';
    alarm = 0;

    /* 이전 이동과 원점복귀 명령을 폐기한다 */
    if (!motor_stop(&motorX)) { fail(2); return; }
    if (!motor_stop(&motorY)) { fail(3); return; }
    mks_stop();

    /* X/Y ESTOP 해제 */
    if (!motor_estop(&motorX, 0)) { fail(2); return; }
    if (!motor_estop(&motorY, 0)) { fail(3); return; }

    /* 틸트 원점복귀 */
    if (!mks_home()) {
        fail(mks_link() ? 10 : 4);
        return;
    }

    osDelay(50);
    start = HAL_GetTick();

    for (;;) {
        motor_check();

        if (button_stop_requested()) {
            motor_estop(&motorX, 1);
            motor_estop(&motorY, 1);
            motor_stop(&motorX);
            motor_stop(&motorY);
            mks_stop();
            return;
        }

        if (pause) {
            mks_stop();
            if (!wait_pause()) return;

            if (!mks_home()) {
                fail(mks_link() ? 10 : 4);
                return;
            }

            start = HAL_GetTick();
            osDelay(50);
            continue;
        }

        done = mks_done(0);

        if (done > 0) break;

        if (done < 0) {
            fail(done == -1 ? 4 : 10);
            return;
        }

        if (HAL_GetTick() - start >= HOME_WAIT) {
            fail(10);
            return;
        }

        osDelay(10);
    }

    if (!mks_zero()) {
        fail(mks_link() ? 10 : 4);
        return;
    }

    /* 두 축 통신을 먼저 확인한 뒤 원점복귀를 시작한다 */
    if (!motor_pos(&motorX, &x)) { fail(2); return; }
    if (!motor_pos(&motorY, &y)) { fail(3); return; }
    if (button_stop_requested()) {
        motor_estop(&motorX, 1);
        motor_estop(&motorY, 1);
        motor_stop(&motorX);
        motor_stop(&motorY);
        mks_stop();
        return;
    }

    /* X/Y 원점복귀 시작 */
    if (!motor_home_on(&motorX)) { fail(2); return; }
    if (button_stop_requested()) {
        motor_estop(&motorX, 1);
        motor_estop(&motorY, 1);
        motor_stop(&motorX);
        motor_stop(&motorY);
        mks_stop();
        return;
    }
    if (!motor_home_on(&motorY)) { fail(3); return; }

    osDelay(10);
    start = HAL_GetTick();

    for (;;) {
        motor_check();

        if (button_stop_requested()) {
            motor_estop(&motorX, 1);
            motor_estop(&motorY, 1);
            motor_stop(&motorX);
            motor_stop(&motorY);
            mks_stop();
            return;
        }

        if (pause) {
            motor_stop(&motorX);
            motor_stop(&motorY);

            if (!wait_pause()) return;
            if (!motor_home_on(&motorX)) { fail(2); return; }
            if (!motor_home_on(&motorY)) { fail(3); return; }

            start = HAL_GetTick();
            osDelay(10);
            continue;
        }

        if (!motor_pos(&motorX, &x)) { fail(2); return; }
        if (!motor_pos(&motorY, &y)) { fail(3); return; }

        if (abs(x) <= POSITION_GAP && abs(y) <= POSITION_GAP)
            break;

        if (HAL_GetTick() - start >= HOME_WAIT) {
            fail(10);
            return;
        }

        osDelay(10);
    }

    /* X/Y 현재 위치를 0으로 설정 */
    if (!motor_zero(&motorX)) { fail(2); return; }
    if (!motor_zero(&motorY)) { fail(3); return; }

    /* RFID 인식 대기 */
    if (!card_ok) {
        pause_on('S');
        if (!wait_pause()) return;
    }

    status = 'W';
}

static void Stop(void)
{
    motor_stop(&motorX);
    motor_stop(&motorY);
    mks_stop();
    if (status != 'A') status = 'W';
    ack("S");
}

static void motor_command(char *command)
{
	char message[64], *cmd = command + 2;
	uint32_t number = command_number;
	int rack, col, row, xy, rot, move_delay, return_delay;
	int x, y, x_mm, y_mm, left, right, unused;
	int xy_rpm, rot_rpm, mm, speed, angle;

	if (!strncmp(cmd, "MI_", 3) || !strncmp(cmd, "MO_", 3)
			|| !strncmp(cmd, "PO_", 3)) {
		if (!card_ok) { pause_on('S'); return; }

		if (sscanf(cmd + 3, "%d_%d_%d_%d_%d_%d_%d", &rack,
				&col, &row, &xy, &rot, &move_delay, &return_delay) != 7) {
			snprintf(message, sizeof(message), "%.2s%cbad_data", command, NAK);
			send_to_tcp_queue(message);
			return;
		}

		rack_now = rack;

		if (!rot_load(rack, &left, &right, &unused, &unused)) {
			snprintf(message, sizeof(message), "%.2s%cfram_data", command, NAK);
			send_to_tcp_queue(message);
			return;
		}

		/* MI는 입고 X/Y, MO/PO는 출고 X/Y 범위를 확인한다 */
		if (!strncmp(cmd, "MI_", 3)) {
			if (!pos_load(rack, IN_X, col, &x_mm)
					|| !pos_load(rack, IN_Y, row, &y_mm)) {
				snprintf(message, sizeof(message),
						"%.2s%cfram_data", command, NAK);
				send_to_tcp_queue(message);
				return;
			}
		} else {
			if (!pos_load(rack, OUT_X, col, &x_mm)
					|| !pos_load(rack, OUT_Y, row, &y_mm)) {
				snprintf(message, sizeof(message),
						"%.2s%cfram_data", command, NAK);
				send_to_tcp_queue(message);
				return;
			}
		}

		x = x_mm * REV_PULSE * XY_GEAR / REV_MM;
		y = y_mm * REV_PULSE * XY_GEAR / REV_MM;
		xy_rpm = xy * MAX_RPM / 100;
		rot_rpm = rot * ROT_MAX_RPM / 100;
		return_delay *= 10;

		snprintf(message, sizeof(message), "%.2s%c%.2s_%d_%d_%03d",
				command, ACK, cmd, col, row, xy);
		send_to_tcp_queue(message);

		if (!strncmp(cmd, "MI_", 3)) {
			MI(x, y, xy_rpm, rot_rpm, 12, move_delay);
			if (status == 'A') return;

			snprintf(message, sizeof(message), "%.2sAI_%d_%d_%d",
					command, rack, col, row);

		} else if (!strncmp(cmd, "MO_", 3)) {
			MO(x, y, xy_rpm, rot_rpm);
			if (status == 'A') return;

			snprintf(message, sizeof(message), "%.2sAO_%d_%d_%d",
					command, rack, col, row);

		} else {
			PO(x, y, xy_rpm, rot_rpm, right, move_delay, return_delay,
					1 << (row - 1));
			if (status == 'A') return;

			snprintf(message, sizeof(message), "%.2sEO_%d_%d_%d",
					command, rack, col, row);
		}

		if (number == command_number && !motor_ready)
			repeat_set(message);

	} else if (sscanf(cmd, "x_%d_%d", &mm, &speed) == 2) {
		if (motor_move(&motorX, speed * MAX_RPM / 100,
				mm * REV_PULSE * XY_GEAR / REV_MM)) {
			snprintf(message, sizeof(message), "%.2s%c%s", command, ACK, cmd);
			send_to_tcp_queue(message);
		} else {
			status = 'A';
			alarm = 2;
		}

	} else if (sscanf(cmd, "y_%d_%d", &mm, &speed) == 2) {
		if (motor_move(&motorY, speed * MAX_RPM / 100,
				mm * REV_PULSE * XY_GEAR / REV_MM)) {
			snprintf(message, sizeof(message), "%.2s%c%s", command, ACK, cmd);
			send_to_tcp_queue(message);
		} else {
			status = 'A';
			alarm = 3;
		}

	} else if (sscanf(cmd, "r_%d_%d", &angle, &speed) == 2) {
		if (mks_r(speed * MAX_RPM / 100, angle)) {
			snprintf(message, sizeof(message), "%.2s%c%s", command, ACK, cmd);
			send_to_tcp_queue(message);
		} else {
			status = 'A';
			alarm = mks_link() ? 7 : 4;
		}

	} else if (sscanf(cmd, "l_%d_%d", &angle, &speed) == 2) {
		if (mks_l(speed * MAX_RPM / 100, angle)) {
			snprintf(message, sizeof(message), "%.2s%c%s", command, ACK, cmd);
			send_to_tcp_queue(message);
		} else {
			status = 'A';
			alarm = mks_link() ? 7 : 4;
		}

	} else if (sscanf(cmd, "c_%d", &speed) == 1) {
		if (mks_c(speed * MAX_RPM / 100)) {
			snprintf(message, sizeof(message), "%.2s%c%s", command, ACK, cmd);
			send_to_tcp_queue(message);
		} else {
			status = 'A';
			alarm = mks_link() ? 7 : 4;
		}

	} else if (!strcmp(cmd, "I")) {
		I();

		if (status == 'W') {
			snprintf(message, sizeof(message), "%.2s%cI", command, ACK);
			send_to_tcp_queue(message);
		}

	} else if (!strcmp(cmd, "S_1")) {
		Stop();

	} else {
		snprintf(message, sizeof(message), "%.2s%cbad_cmd", command, NAK);
		send_to_tcp_queue(message);
	}
}

void MOTOR_TaskRun(void *argument)
{
	char command[64];
	int state;

	(void)argument;

	while (!motor_init(&motorX))
		osDelay(10);
	print("x ready\r\n");

	while (!motor_init(&motorY))
		osDelay(10);
	print("y ready\r\n");

	/* MKS가 통신 가능해질 때까지 기다린다 */
	while (!mks_read(2, CMD_STATE, 1, &state))
		osDelay(10);

	/* 통신 확인 후 MKS 설정 */
	if (mks_init()) {
		print("mks ready\r\n");

		/* 모든 모터 준비가 끝난 뒤 처음 원점복귀 */
		if (!estop)
			net_cmd("00I");
	}
	else {
		print("mks init ERR\r\n");
		status = 'A';
		alarm = mks_link() ? 7 : 4;
	}

	motor_estop(&motorX, estop);
	motor_estop(&motorY, estop);

	for (;;) {
		/* 모든 모터 통신은 MOTOR Task에서만 실행한다 */
		if (estop) {
			motor_estop(&motorX, 1);
			motor_estop(&motorY, 1);
			motor_stop(&motorX);
			motor_stop(&motorY);
			mks_stop();

			while (estop)
				osDelay(10);

			continue;
		}

		if (motor_ready) {
			snprintf(command, sizeof(command), "%s", motor_line);

			motor_ready = 0;
			motor_busy = 1;

			motor_command(command);

			motor_busy = 0;

			if (status != 'A' && !motor_ready)
				motor_line[0] = 0;
		}

		motor_check();
		osDelay(10);
	}
}
