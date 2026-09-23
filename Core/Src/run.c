#include "run.h"
#include "i_motor.h"
#include "a_motor.h"
#include "t_motor.h"
#include "cli.h"
#include "cmsis_os2.h"
#include "status.h"
#include "cmd.h"
#include <stdio.h>      // snprintf

#define BOOT_WAIT    3000   /* 드라이브 전원 투입 후 통신이 살아날 때까지 */
#define AXIS_WAIT    300    /* 축 사이 간격 */
#define INIT_RETRY   3      /* 초기화 재시도 횟수 */
#define CHECK_COUNT  100    /* 메인 루프 10ms × 100 = 1초마다 한 축 */
#define GAP          10     /* 도착 판정 오차 : X,Y는 mm */
#define TILT_GAP     2

extern osMessageQueueId_t MotorQueueHandle;

static int x_ok;
static int y_ok;
static int t_ok;

/* 드라이브가 깨어날 때까지 기다린 뒤 실패한 축만 다시 시도 */
int motor_basic_init(void)
{

    for (int n = 1; n <= INIT_RETRY; n++) {

        if (!x_ok) x_ok = IM_MapInit();
        osDelay(AXIS_WAIT);

        if (!y_ok) y_ok = AM_Init();
        osDelay(AXIS_WAIT);

        if (!t_ok) t_ok = TM_Init();
        osDelay(AXIS_WAIT);

        if (x_ok && y_ok && t_ok) break;
    }
    return I();
}

/* 기본 모터 시험 */
int motor_basic_run(MotorCommand cmd)
{
    if (cmd.command == 'X') {
        if (!IM_Move(cmd.pulse, cmd.rpm)) return 0;
        while (!IM_Done());
        return 1;
    }
    else if (cmd.command == 'Y') {
        if (!AM_Move(cmd.pulse, cmd.rpm)) return 0;
        while (!AM_Done());
        return 1;
    }
    else if (cmd.command == 'C') {
        if (!TM_C(cmd.rpm)) return 0;
        while (!TM_Done());
        return 1;
    }
    else if (cmd.command == 'R') {
        if (!TM_R(cmd.pulse, cmd.rpm)) return 0;
        while (!TM_Done());
        return 1;
    }
    else if (cmd.command == 'L') {
        if (!TM_L(cmd.pulse, cmd.rpm)) return 0;
        while (!TM_Done());
        return 1;
    }
    else if (cmd.command == CMD_S) {
        return S();
    }
    else if (cmd.command == CMD_I) {
    	return I();
    }
    else if (cmd.command == CMD_MI) {
        return MI(cmd.x, cmd.y,
                  cmd.xy_speed, cmd.tilt_speed);
    }
    else if (cmd.command == CMD_MO) {
        return MO(cmd.x, cmd.y,
                  cmd.xy_speed, cmd.tilt_speed);
    }
    else if (cmd.command == CMD_PO) {
        return PO(cmd.rack, cmd.x, cmd.y,
                  cmd.xy_speed, cmd.tilt_speed,
                  cmd.move_delay, cmd.return_delay,
                  cmd.tilt_angle, cmd.tilting_y,
                  cmd.tilting_y_speed);
    }

    return 0;
}

/* 초기화된 축만 1초에 하나씩 확인 : X -> Y -> TILT */
void Motor_Check(void)
{
    static int turn = 0;
    static int count = 0;

    IM_Info info;
    uint16_t state;
    uint16_t alarm;
    int32_t mm;
    int rpm;

    count++;
    if (count < CHECK_COUNT) return;
    count = 0;

    if (turn == 0 && x_ok) {
        IM_MapRead(&info);          /* 상태+위치+속도+알람을 한 번에 */
    }
    else if (turn == 1 && y_ok) {
        AM_Speed(&rpm);
        AM_Pos(&mm);
        AM_AlarmRead(&alarm);
    }
    else if (turn == 2 && t_ok) {
        TM_State(&state);
        TM_Pos(&mm);
    }

    turn++;
    if (turn > 2) turn = 0;
}
#define GAP  10      /* 도착 판정 오차 : X,Y는 mm / TILT는 도 */

/* X, Y 도착 대기 */
static int wait_xy(int x, int y)
{
    int32_t x_now = 0;
    int32_t y_now = 0;
    int x_ok;
    int y_ok;
    int count = 0;
    char buf[64];

    for (;;) {
        osDelay(200);

        x_ok = IM_Pos(&x_now);
        y_ok = AM_Pos(&y_now);

        if (x_ok && y_ok &&
            x_now > x - GAP && x_now < x + GAP &&
            y_now > y - GAP && y_now < y + GAP)
            return 1;

        count++;

        /* 현재값/목표값 : 스케일 오류인지 읽기 실패인지 구분 */
        if (count == 1 || count % 10 == 0 || count > 150) {
            snprintf(buf, sizeof(buf), "XY %ld/%d %ld/%d %s%s\r\n",
                     (long)x_now, x, (long)y_now, y,
                     x_ok ? "" : "X_READ_ERR ",
                     y_ok ? "" : "Y_READ_ERR");
            CLI_Print(buf);
        }

        if (count > 150) {              /* 약 32초 */
            CLI_Print("XY TIMEOUT\r\n");
            return 0;
        }
    }
}

/* TILT 도착 대기 */
static int wait_tilt(int deg)
{
    int32_t now = 0;
    int ok;
    for (;;) {
    	   osDelay(200);

    	        ok = TM_Pos(&now);

    	        if (ok &&
    	            now >= deg - TILT_GAP &&
    	            now <= deg + TILT_GAP &&
    	            TM_Done())
    	            return 1;
    }
}
/* 입고 */
int MI(int x, int y, int xy_speed, int tilt_speed)
{
    TM_C(tilt_speed);
    if (!wait_tilt(0)) return 0;

    IM_Move(x, xy_speed);
    AM_Move(y, xy_speed);
    if (!wait_xy(x, y)) return 0;

    return 1;
}

/* 출고 : 동작은 입고와 같다 */
int MO(int x, int y, int xy_speed, int tilt_speed)
{
    return MI(x, y, xy_speed, tilt_speed);
}

/* 분배 */
int PO(int rack, int x, int y,
       int xy_speed, int tilt_speed,
       int move_delay, int return_delay,
       int tilt_angle, int tilting_y, int tilting_y_speed)
{
    int deg;

    return_delay *= 10;

    if (!MO(x, y, xy_speed, tilt_speed))
        return 0;

    osDelay(move_delay);

    AM_Move(y + tilting_y, tilting_y_speed);

    if (rack == 1) {
        deg = -tilt_angle;
        TM_L(tilt_angle, tilt_speed);
    }
    else {
        deg = tilt_angle;
        TM_R(tilt_angle, tilt_speed);
    }

    if (!wait_xy(x, y + tilting_y)) return 0;
    if (!wait_tilt(deg)) return 0;

    osDelay(return_delay);

    AM_Move(y, tilting_y_speed);
    TM_C(tilting_y_speed);

    if (!wait_xy(x, y)) return 0;
    if (!wait_tilt(0)) return 0;

    return 1;
}

int S (void){
 if( MI||PO )
 {
  }

 }

	return 1;
}

/* 원점 : Tilt -> X -> Y 순서 */
int I (void){
    status_1 = HOME;
    status_2 = HOME;

    TM_Home();
    if (TM_Done()) {
        IM_Home();
        if (IM_Done()) {
            return AM_Home();
        }
    }
    return 0;
}

void Motor_Run(void)
{
    MotorCommand cmd;
    int ok;

    ok = motor_basic_init();
    CLI_Start(ok);

    for (;;) {

        Motor_Check();

        if (osMessageQueueGet(MotorQueueHandle,
                              &cmd, NULL, 10) == osOK) {

            Status_Start(cmd.rack);

            ok = motor_basic_run(cmd);
            CLI_Result(cmd.command, ok);

            Status_End(cmd.command, cmd.rack);

            if (ok)
                CMD_Done(cmd);
        }
    }
}
