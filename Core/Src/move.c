/* move.c: 모터 명령과 실제 이동만 담당한다. */
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
extern volatile int card_ok; extern volatile int run; extern volatile uint32_t command_number;
int mks_link(void); int button_stop_requested(void);
void repeat_set(const char *message); void pause_po(int mask);
#define POSITION_GAP  50
#define REV_PULSE     1000
#define REV_MM        160
#define XY_GEAR       5
#define MAX_RPM       3000
#define ROT_MAX_RPM   3000
#define HOME_WAIT     80000
volatile char status = 'W';
volatile int alarm;
int rack_now = 1;
char motor_line[64];
volatile int motor_ready; volatile int motor_busy;
volatile int event_number;
static int po_mask;
static void motor_check(void);
static void event_set(const char *command) {
    event_number = (command[0] - '0') * 10 + command[1] - '0';
}
static int wait_pause(void) {
    while (pause) {
        motor_check();
        if (button_stop_requested()) return 0;
        osDelay(10);
    }
    return 1;
}
static void fail(int code) {
    status = 'A'; alarm = code; run = 0;
    motor_stop(&motorX); motor_stop(&motorY); mks_stop();
    motor_estop(&motorX, 1); motor_estop(&motorY, 1);
    alarm_set(code);
}
static void fail_rot(void) { fail(mks_link() ? 7 : 4); }
static void motor_check(void)
{
    static int count;
    int pos, state;

    if (++count < 3)
        return;

    count = 0;

    if (!motor_pos(&motorX, &pos)) {
        if (status != 'A' || alarm != 2)
            fail(2);
        return;
    }

    if (!motor_pos(&motorY, &pos)) {
        if (status != 'A' || alarm != 3)
            fail(3);
        return;
    }

    if (!mks_read(2, CMD_STATE, 1, &state)) {
        if (status != 'A' || alarm != 4)
            fail(4);
        return;
    }

    if (state == ST_FAIL) {
        if (status != 'A' || alarm != 7)
            fail(7);
        return;
    }
}
static int move_xy(int x, int y, int rpm) {
    int x_now, y_now, x_rpm = rpm, y_rpm = rpm;
    int64_t xd, yd;
    if (!wait_pause()) return 0;
    if (!motor_pos(&motorX, &x_now)) { fail(2); return 0; }
    if (!motor_pos(&motorY, &y_now)) { fail(3); return 0; }
    xd = llabs((int64_t)x - x_now); yd = llabs((int64_t)-y - y_now);
    if (xd > yd && xd) y_rpm = (int)((int64_t)rpm * yd / xd);
    else if (yd > xd && yd) x_rpm = (int)((int64_t)rpm * xd / yd);
    if (x_rpm < 1) x_rpm = 1;
    if (y_rpm < 1) y_rpm = 1;
    if (!motor_move(&motorY, y_rpm, y)) { motor_stop(&motorX); motor_stop(&motorY); fail(3); return 0; }
    if (!motor_move(&motorX, x_rpm, x)) { motor_stop(&motorX); motor_stop(&motorY); fail(2); return 0; }
    for (;;) {
        motor_check();
        if (button_stop_requested()) { motor_estop(&motorX, 1); motor_estop(&motorY, 1); motor_stop(&motorX); motor_stop(&motorY); mks_stop(); return 0; }
        if (!motor_pos(&motorX, &x_now)) { motor_stop(&motorX); motor_stop(&motorY); fail(2); return 0; }
        if (!motor_pos(&motorY, &y_now)) { motor_stop(&motorX); motor_stop(&motorY); fail(3); return 0; }
        if (abs(x_now - x) <= POSITION_GAP && abs(y_now + y) <= POSITION_GAP) return 1;
        osDelay(10);
    }
}
static int move_rot(char dir, int rpm, int angle) {
    int target = 0, done;
    if (dir == 'l') target = angle * MKS_REV * ROT_GEAR / 360;
    if (dir == 'r') target = -angle * MKS_REV * ROT_GEAR / 360;
    for (;;) {
        if (!wait_pause()) return 0;
        if (po_mask && ((full_read() & po_mask) || !card_ok)) { pause_po(po_mask); continue; }
        if (!mks_move(rpm, target)) { fail_rot(); return 0; }
        for (;;) {
            motor_check();
            if (button_stop_requested()) { mks_stop(); return 0; }
            if (pause) { mks_stop(); break; }
            if (po_mask && ((full_read() & po_mask) || !card_ok)) { mks_stop(); pause_po(po_mask); break; }
            done = mks_done(target);
            if (done > 0) return 1;
            if (done < 0) { fail(done == -1 ? 4 : 7); return 0; }
            osDelay(10);
        }
    }
}
static void MI(int x, int y, int xy_rpm, int rot_rpm, int move_delay) {
    status = 'R'; alarm = 0;
    if (!move_rot('c', rot_rpm, 0)) return;
    if (!move_xy(x, y, xy_rpm)) return;
    if (!wait_pause()) return;
    osDelay(move_delay);
    if (button_stop_requested()) return;
    if (!move_rot('l', rot_rpm, 12)) return;
    status = 'R';
}
static void MO(int x, int y, int xy_rpm, int rot_rpm) {
    status = 'R'; alarm = 0;
    if (!move_rot('c', rot_rpm, 0)) return;
    if (!move_xy(x, y, xy_rpm)) return;
    if (!wait_pause()) return;
    status = 'R';
}
static void PO(int x, int y, int xy_rpm, int rot_rpm, int angle,
        int move_delay, int return_delay, int mask) {
    status = 'R'; alarm = 0; po_mask = 0;
    if (!move_rot('c', rot_rpm, 0)) return;
    if (!move_xy(x, y, xy_rpm)) return;
    if (!wait_pause()) return;
    osDelay(move_delay);
    if (button_stop_requested()) return;
    po_mask = mask; pause_po(mask);
    if (!wait_pause()) { po_mask = 0; return; }
    if (!move_rot('r', rot_rpm, angle)) { po_mask = 0; return; }
    po_mask = 0; osDelay(return_delay);
    if (button_stop_requested()) return;
    if (!move_rot('c', rot_rpm, 0)) return;
    status = 'R';
}
static void I(void)
{
    int x, y, done;
    uint32_t start;

    status = 'I';
    alarm = 0;
    po_mask = 0;

    /* X/Y ESTOP 해제 */
    if (!motor_estop(&motorX, 0)) {
        fail(2);
        return;
    }

    if (!motor_estop(&motorY, 0)) {
        fail(3);
        return;
    }

    /* 이전 동작 정지 */
    if (!motor_stop(&motorX)) {
        fail(2);
        return;
    }

    if (!motor_stop(&motorY)) {
        fail(3);
        return;
    }

    mks_stop();

    /* ROT HOME */
    if (!mks_home()) {
        fail(mks_link() ? 10 : 4);
        return;
    }

    osDelay(50);
    start = HAL_GetTick();

    for (;;) {

        if (button_stop_requested())
            goto abort;

        if (pause) {
            mks_stop();

            if (!wait_pause())
                return;

            if (!mks_home()) {
                fail(mks_link() ? 10 : 4);
                return;
            }

            start = HAL_GetTick();
            osDelay(50);
            continue;
        }

        /* ROT HOME 완료 확인 */
        if (!mks_read(2, CMD_STATE, 1, &done)) {
            fail(4);
            return;
        }

        if (done == ST_FAIL) {
            fail(7);
            return;
        }

        if (done == ST_STOP)
            break;

        if (HAL_GetTick() - start >= HOME_WAIT) {
            fail(10);
            return;
        }

        osDelay(10);
    }

    /* ROT 현재 위치를 0으로 */
    if (!mks_zero()) {
        fail(mks_link() ? 10 : 4);
        return;
    }

    /* X/Y 통신 확인 */
    if (!motor_pos(&motorX, &x)) {
        fail(2);
        return;
    }

    if (!motor_pos(&motorY, &y)) {
        fail(3);
        return;
    }

    if (button_stop_requested())
        goto abort;

    /* X HOME */
    if (!motor_home_on(&motorX)) {
        fail(2);
        return;
    }

    if (button_stop_requested())
        goto abort;

    /* Y HOME */
    if (!motor_home_on(&motorY)) {
        fail(3);
        return;
    }

    osDelay(10);
    start = HAL_GetTick();

    /* X/Y HOME 완료 대기 */
    for (;;) {

        if (button_stop_requested())
            goto abort;

        if (pause) {
            motor_stop(&motorX);
            motor_stop(&motorY);

            if (!wait_pause())
                return;

            if (!motor_home_on(&motorX)) {
                fail(2);
                return;
            }

            if (!motor_home_on(&motorY)) {
                fail(3);
                return;
            }

            start = HAL_GetTick();
            osDelay(10);
            continue;
        }

        if (!motor_pos(&motorX, &x)) {
            fail(2);
            return;
        }

        if (!motor_pos(&motorY, &y)) {
            fail(3);
            return;
        }

        if (abs(x) <= POSITION_GAP &&
            abs(y) <= POSITION_GAP)
            break;

        if (HAL_GetTick() - start >= HOME_WAIT) {
            fail(10);
            return;
        }

        osDelay(10);
    }

    /* X/Y 현재 위치 0 */
    if (!motor_zero(&motorX)) {
        fail(2);
        return;
    }

    if (!motor_zero(&motorY)) {
        fail(3);
        return;
    }

    status = 'W';
    return;

abort:
    motor_estop(&motorX, 1);
    motor_estop(&motorY, 1);
    motor_stop(&motorX);
    motor_stop(&motorY);
    mks_stop();
}
static void Stop(void) {
    motor_stop(&motorX); motor_stop(&motorY); mks_stop();
    if (status != 'A') status = 'W';
    ack("S");
}
static void motor_command(char *command) {
    char message[64], *cmd = command + 2;
    uint32_t number = command_number;
    int rack, col, row, xy, rot, move_delay, return_delay;
    int x, y, x_mm, y_mm, left, right, unused, xy_rpm, rot_rpm;
    int mm, speed, angle;
    if (!strncmp(cmd, "MI_", 3) || !strncmp(cmd, "MO_", 3) || !strncmp(cmd, "PO_", 3)) {
        if (sscanf(cmd + 3, "%d_%d_%d_%d_%d_%d_%d", &rack, &col, &row,
                &xy, &rot, &move_delay, &return_delay) != 7) {
            snprintf(message, sizeof(message), "%.2s%cbad_data", command, NAK); send_to_tcp_queue(message); return;
        }
        rack_now = rack;
        if (!rot_load(rack, &left, &right, &unused, &unused)) {
            snprintf(message, sizeof(message), "%.2s%cfram_data", command, NAK); send_to_tcp_queue(message); return;
        }
        if (!strncmp(cmd, "MI_", 3)) {
            if (!pos_load(rack, IN_X, col, &x_mm) || !pos_load(rack, IN_Y, row, &y_mm)) {
                snprintf(message, sizeof(message), "%.2s%cfram_data", command, NAK); send_to_tcp_queue(message); return;
            }
        } else if (!pos_load(rack, OUT_X, col, &x_mm) || !pos_load(rack, OUT_Y, row, &y_mm)) {
            snprintf(message, sizeof(message), "%.2s%cfram_data", command, NAK); send_to_tcp_queue(message); return;
        }
        x = x_mm * REV_PULSE * XY_GEAR / REV_MM; y = y_mm * REV_PULSE * XY_GEAR / REV_MM; xy_rpm = xy * MAX_RPM / 100;
        rot_rpm = rot * ROT_MAX_RPM / 100; return_delay *= 10; event_set(command);
        snprintf(message, sizeof(message), "%.2s%c%.2s_%d_%d_%03d", command, ACK, cmd, col, row, xy); send_to_tcp_queue(message);
        if (!strncmp(cmd, "MI_", 3)) {
            MI(x, y, xy_rpm, rot_rpm, move_delay);
            if (status == 'A') return;
            snprintf(message, sizeof(message), "%.2sAI_%d_%d_%d", command, rack, col, row);
        } else if (!strncmp(cmd, "MO_", 3)) {
            MO(x, y, xy_rpm, rot_rpm);
            if (status == 'A') return;
            snprintf(message, sizeof(message), "%.2sAO_%d_%d_%d", command, rack, col, row);
        } else {
            PO(x, y, xy_rpm, rot_rpm, right, move_delay, return_delay, 1 << (row - 1));
            if (status == 'A') return;
            snprintf(message, sizeof(message), "%.2sEO_%d_%d_%d", command, rack, col, row);
        }
        if (number == command_number && !motor_ready) repeat_set(message);
        return;
    }
    if (sscanf(cmd, "x_%d_%d", &mm, &speed) == 2) {
        if (motor_move(&motorX, speed * MAX_RPM / 100, mm * REV_PULSE * XY_GEAR / REV_MM))
            snprintf(message, sizeof(message), "%.2s%c%s", command, ACK, cmd);
        else { status = 'A'; alarm = 2; return; }
        send_to_tcp_queue(message); return;
    }
    if (sscanf(cmd, "y_%d_%d", &mm, &speed) == 2) {
        if (motor_move(&motorY, speed * MAX_RPM / 100, mm * REV_PULSE * XY_GEAR / REV_MM))
            snprintf(message, sizeof(message), "%.2s%c%s", command, ACK, cmd);
        else { status = 'A'; alarm = 3; return; }
        send_to_tcp_queue(message); return;
    }
    if (sscanf(cmd, "r_%d_%d", &angle, &speed) == 2) {
        if (mks_r(speed * MAX_RPM / 100, angle)) snprintf(message, sizeof(message), "%.2s%c%s", command, ACK, cmd);
        else { status = 'A'; alarm = mks_link() ? 7 : 4; return; }
        send_to_tcp_queue(message); return;
    }
    if (sscanf(cmd, "l_%d_%d", &angle, &speed) == 2) {
        if (mks_l(speed * MAX_RPM / 100, angle)) snprintf(message, sizeof(message), "%.2s%c%s", command, ACK, cmd);
        else { status = 'A'; alarm = mks_link() ? 7 : 4; return; }
        send_to_tcp_queue(message); return;
    }
    if (sscanf(cmd, "c_%d", &speed) == 1) {
        if (mks_c(speed * MAX_RPM / 100)) snprintf(message, sizeof(message), "%.2s%c%s", command, ACK, cmd);
        else { status = 'A'; alarm = mks_link() ? 7 : 4; return; }
        send_to_tcp_queue(message); return;
    }
    if (!strcmp(cmd, "I")) {
        I();
        if (status == 'W') { snprintf(message, sizeof(message), "%.2s%cI", command, ACK); send_to_tcp_queue(message); }
        return;
    }
    if (!strcmp(cmd, "S_1")) { Stop(); return; }
    snprintf(message, sizeof(message), "%.2s%cbad_cmd", command, NAK); send_to_tcp_queue(message);
}
void MOTOR_TaskRun(void *argument)
{
    char command[64];
    int state;

    (void)argument;

    while (!motor_init(&motorX, 0))
        osDelay(10);

    print("x ready\r\n");

    while (!motor_init(&motorY, 0))
        osDelay(10);

    print("y ready\r\n");

    while (!mks_read(2, CMD_STATE, 1, &state))
        osDelay(10);

    if (!mks_init()) {
        print("mks init ERR\r\n");
        fail(mks_link() ? 7 : 4);
    }
     else {
            print("mks ready\r\n");

            /* MKS 전원/통신 안정화 후 원점복귀 1회 */
            osDelay(300);

            while (!run && !estop)
                osDelay(10);

            if (!estop)
                net_cmd("00I");
        }
    for (;;) {

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

