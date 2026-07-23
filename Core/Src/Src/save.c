#include "save.h"
#include "motor.h"
#include "rot_test.h"
#include "fram.h"
#include "tcp.h"
#include "item.h"
#include "button.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ===== 설정값 ===== */

#define ROT_RPM         1500
#define X_RPM           1000
#define Y_RPM           1000
#define HOME_RPM        100     /* 원점 탐색 속도 */

#define TIMEOUT         60000U
#define HOME_TIMEOUT    65000U

#define ROT_GAP         300     /* ROT 도착 허용오차 */
#define XY_GAP          1000    /* XY 도착 허용오차 */
#define XY_STABLE       2       /* 연속 몇 번 들어와야 도착 */
#define HOLD_MS         100U    /* 단계 사이 대기 */

#define MAX_X           8
#define MAX_Y           8

#define DATA_ADDR       0x0000
#define DATA_VER        0x000A  /* 호기별 보정값 추가 */

#define ROT_600         0x6AAA  /* C에서 R/L까지 거리 */
#define C_POS           0
#define R_POS           ROT_600
#define L_POS           (-ROT_600)

#define REV_MM          160
#define REV_CNT         131072
#define MM(v)           ((v) * REV_MM / REV_CNT)

#define AXIS_X          1
#define AXIS_Y          2
#define AXIS_IN_X       3       /* 입력칸 X. 따로 움직이고 따로 저장 */
#define AXIS_IN_Y       4       /* 입력칸 Y */
#define AXIS_ADJ_X      5       /* 호기별 X 보정 */
#define AXIS_ADJ_Y      6       /* 호기별 Y 보정 */

#define IN_X_OK         1
#define IN_Y_OK         2
#define IN_OK           3

/* 한 사이클: C -> 입력칸 -> R -> C -> 목표칸 -> L -> C */
typedef enum {
    S_IDLE,
    S_C1_START, S_C1_WAIT,      /* 1. ROT -> C */
    S_IN_START, S_IN_WAIT,      /* 2. X/Y -> 입력칸 */
    S_R_START,  S_R_WAIT,       /* 3. ROT -> L (이름은 R이지만 목적지는 L) */
    S_C2_START, S_C2_WAIT,      /* 4. ROT -> C */
    S_XY_START, S_XY_WAIT,      /* 5. X/Y -> 목표칸 */
    S_L_START,  S_L_WAIT,       /* 6. ROT -> R (이름은 L이지만 목적지는 R) */
    S_C3_START, S_C3_WAIT       /* 7. ROT -> C */
} Seq;

/* ===== FRAM 데이터 ===== */

typedef struct {
    int ver;
    int x_n, y_n;               /* 실제 사용할 열 수, 단 수 */
    int x[MAX_X], y[MAX_Y];     /* 각 열/단 위치 */
    int in_x, in_y;             /* 입력칸 위치 */
    int in_ready;               /* bit0=X 저장됨 bit1=Y 저장됨 */
    int x_ref, y_ref;           /* 첫 기준 위치 저장됨 */
    int x_ready, y_ready;       /* 전체 위치 계산 완료 */
    int x_adj, y_adj;           /* 이 기계만의 보정. load로 덮어쓰지 않는다 */
} Data;

/* ===== 전역 상태 ===== */

static Data data;

static int rot_homed, x_homed, y_homed;

static int save_axis;                   /* 저장 대기중인 축, 0=없음 */
static int save_n;                      /* 저장 대기중인 번호 */

static Seq seq;
static int run_col = 1;
static int run_dan = 1;

static int seq_result;                  /* 0=진행중 1=성공 -1=실패 */
static uint32_t move_t0, poll_t0, hold_t0;
static int x_now, y_now;
static int x_count, y_count;

/* ===== 공통 ===== */

static int gap(int a, int b) { return abs(a - b); }

static void reply(const char *s) { tcp_reply(s); }

/* item.c가 grid 크기를 알아야 해서 열어준다 */
int save_result(void) { return seq_result; }
int save_grid_x(void) { return data.x_n; }
int save_grid_y(void) { return data.y_n; }

/* 저장값에 이 기계의 보정을 더한 실제 목표 */
static int tx(int i)   { return data.x[i] + data.x_adj; }
static int ty(int i)   { return data.y[i] + data.y_adj; }
static int in_tx(void) { return data.in_x + data.x_adj; }
static int in_ty(void) { return data.in_y + data.y_adj; }

static int grid_ok(void)
{
    return data.x_n >= 2 && data.x_n <= MAX_X &&
           data.y_n >= 2 && data.y_n <= MAX_Y;
}

/* ===== FRAM ===== */

static void data_write(void) { fram_write(DATA_ADDR, &data, sizeof(data)); }

static void data_clear(void)
{
    memset(&data, 0, sizeof(data));
    data.ver = DATA_VER;
    data_write();
}

static void data_read(void)
{
    fram_read(DATA_ADDR, &data, sizeof(data));

    if (data.ver != DATA_VER) { data_clear(); return; }

    if (data.x_n < 0 || data.x_n > MAX_X ||
        data.y_n < 0 || data.y_n > MAX_Y) data_clear();
}

/* ===== ROT ===== */

/* F1 상태가 1이면 정지. 위치가 ROT_GAP 안이면 완료 */
static int wait_rot(int target)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t state;
    int now;

    while (HAL_GetTick() - t0 < TIMEOUT) {
        if (mks_state(&state) && state == 1U &&
            mks_pos(&now) && gap(now, target) <= ROT_GAP) return 1;
        HAL_Delay(50);
    }
    mks_stop();
    return 0;
}

static int move_rot(int t) { return mks_move_abs(ROT_RPM, t) && wait_rot(t); }

/* 센서까지 이동하고 그 자리를 C=0으로 잡는다 */
static int rot_home_c(void)
{
    rot_homed = 0;

    if (!mks_home() || !mks_zero()) return 0;

    rot_homed = 1;
    return 1;
}

/* ===== X/Y 홈 ===== */

/*
 * 한 축만 홈 방향으로 보낸다.
 * 그 축 드라이버의 DI1 광센서가 켜지면 그 축만 멈추고 그 자리를 0으로 잡는다.
 * m 외의 축은 건드리지 않는다.
 */
static int home_one(Motor *m, int rpm)
{
    uint32_t t0;

    if (!motor_speed_mode(m) || !motor_speed(m, rpm)) return 0;

    t0 = HAL_GetTick();

    while (HAL_GetTick() - t0 < HOME_TIMEOUT) {
        if (button_stop_requested()) break;

        if (motor_home_on(m) == 1) {
            motor_speed(m, 0);
            HAL_Delay(50);
            return motor_zero(m) && motor_pos_mode(m);
        }

        HAL_Delay(10);
    }

    motor_speed(m, 0);
    motor_pos_mode(m);
    return 0;
}

/* X를 먼저 끝내고 그다음 Y. 각자 움직이고 각자 멈춘다 */
static int home_xy(void)
{
    x_homed = y_homed = 0;

    print("HOME X START\r\n");
    if (!home_one(&motorX, -HOME_RPM)) { print("HOME X FAIL\r\n"); return 0; }
    x_homed = 1;

    print("HOME Y START\r\n");
    if (!home_one(&motorY, HOME_RPM)) { print("HOME Y FAIL\r\n"); return 0; }
    y_homed = 1;

    print("HOME XY OK\r\n");
    return 1;
}

int save_home(void)
{
    if (save_busy() || save_axis) return 0;

    if (!rot_home_c()) {
        HAL_Delay(300);
        if (!rot_home_c()) { print("HOME ROT FAIL\r\n"); return 0; }
    }

    print("HOME ROT OK\r\n");
    return home_xy();
}

/* ===== grid 설정 ===== */

/* 크기가 바뀐 축만 위치값을 지운다. 같은 값이면 유지 */
static void grid_set(int xn, int yn)
{
    char msg[96];

    if (xn < 2 || xn > MAX_X || yn < 2 || yn > MAX_Y) {
        snprintf(msg, sizeof(msg), "GRID ERROR X=2~%d Y=2~%d\r\n", MAX_X, MAX_Y);
        reply(msg);
        return;
    }

    if (data.x_n != xn) {
        memset(data.x, 0, sizeof(data.x));
        data.x_n = xn;
        data.x_ref = data.x_ready = 0;
        data.in_ready &= ~IN_X_OK;
    }
    if (data.y_n != yn) {
        memset(data.y, 0, sizeof(data.y));
        data.y_n = yn;
        data.y_ref = data.y_ready = 0;
        data.in_ready &= ~IN_Y_OK;
    }

    run_col = run_dan = 1;
    data_write();

    snprintf(msg, sizeof(msg), "GRID OK X=%d Y=%d\r\n", data.x_n, data.y_n);
    reply(msg);
}

/* ===== 위치 저장 ===== */

/*
 * X는 마지막 열, 그다음 마지막 전 열 순서. grid 4 5면 x4 -> s, x3 -> s
 * Y는 y1, y2 순서.
 * 입력칸은 in x -> s, in y -> s 로 축마다 따로.
 */
static int save_start(int axis, int n)
{
    Motor *m;
    int rpm;

    if (save_busy() || save_axis || !grid_ok()) return 0;

    if (axis == AXIS_X) {
        if (!x_homed) return 0;
        /* 마지막 열과 마지막 전 열만 직접 저장 */
        if (n != data.x_n && n != data.x_n - 1) return 0;
        /* 마지막 전 열은 마지막 열이 저장된 뒤에만 */
        if (n == data.x_n - 1 && !data.x_ref) return 0;
        m = &motorX;
        rpm = HOME_RPM;         /* 홈 방향의 반대 */
    }
    else if (axis == AXIS_Y) {
        if (!y_homed) return 0;
        if (n != 1 && n != 2) return 0;         /* y1, y2만 */
        if (n == 2 && !data.y_ref) return 0;    /* y2는 y1 뒤에만 */
        m = &motorY;
        rpm = -HOME_RPM;
    }
    else if (axis == AXIS_IN_X) {
        if (!x_homed) return 0;
        m = &motorX;
        rpm = HOME_RPM;
    }
    else if (axis == AXIS_IN_Y) {
        if (!y_homed) return 0;
        m = &motorY;
        rpm = -HOME_RPM;
    }
    else if (axis == AXIS_ADJ_X) {
        /* 기준칸을 지나쳐 있으면 되돌아갈 수 없으므로 홈부터 다시 잡는다 */
        if (!data.x_ready || !home_one(&motorX, -HOME_RPM)) return 0;
        m = &motorX;
        rpm = HOME_RPM;
    }
    else if (axis == AXIS_ADJ_Y) {
        if (!data.y_ready || !home_one(&motorY, HOME_RPM)) return 0;
        m = &motorY;
        rpm = -HOME_RPM;
    }
    else return 0;

    if (!move_rot(C_POS)) return 0;             /* 저장 이동 전에 C로 */
    if (!motor_speed_mode(m) || !motor_speed(m, rpm)) return 0;

    /* s 명령이 올 때 쓸 축과 번호를 기억 */
    save_axis = axis;
    save_n = n;
    return 1;
}

/* s 명령: 움직이던 축 하나를 멈추고 그 위치를 저장한다 */
static int save_end(void)
{
    Motor *m;
    int pos, step;
    int axis = save_axis;
    int n = save_n;

    if (!axis) return 0;

    m = (axis == AXIS_X || axis == AXIS_IN_X || axis == AXIS_ADJ_X)
        ? &motorX : &motorY;

    if (!motor_speed(m, 0)) goto bad;
    HAL_Delay(100);
    if (!motor_pos(m, &pos) && !motor_pos(m, &pos)) goto bad;

    if (axis == AXIS_ADJ_X) {
        data.x_adj = pos - data.x[data.x_n - 1];
    }
    else if (axis == AXIS_ADJ_Y) {
        data.y_adj = pos - data.y[0];
    }
    else if (axis == AXIS_IN_X) {
        data.in_x = pos;
        data.in_ready |= IN_X_OK;
    }
    else if (axis == AXIS_IN_Y) {
        data.in_y = pos;
        data.in_ready |= IN_Y_OK;
    }
    else if (axis == AXIS_X) {
        int last = data.x_n - 1;

        /* 첫 저장: 마지막 열 */
        if (n == data.x_n) {
            memset(data.x, 0, sizeof(data.x));
            data.x[last] = pos;
            data.x_ref = 1;
            data.x_ready = 0;
        }
        /* 두 번째 저장: 간격을 구해 X1까지 계산 */
        else if (n == data.x_n - 1) {
            if (!data.x_ref) goto bad;
            step = pos - data.x[last];
            if (!step) goto bad;

            data.x[last - 1] = pos;
            /* 뒤에서 앞으로. 예: X4=1000 X3=2000 X2=3000 X1=4000 */
            for (int i = last - 2; i >= 0; i--) data.x[i] = data.x[i + 1] + step;
            data.x_ready = 1;
        }
        else goto bad;
    }
    else {
        /* 첫 저장: Y1 */
        if (n == 1) {
            memset(data.y, 0, sizeof(data.y));
            data.y[0] = pos;
            data.y_ref = 1;
            data.y_ready = 0;
        }
        /* 두 번째 저장: 간격을 구해 마지막 단까지 계산 */
        else if (n == 2) {
            if (!data.y_ref) goto bad;
            step = pos - data.y[0];
            if (!step) goto bad;

            for (int i = 1; i < data.y_n; i++) data.y[i] = data.y[0] + step * i;
            data.y_ready = 1;
        }
        else goto bad;
    }

    if (!motor_pos_mode(m)) goto bad;   /* 저장 후 위치제어 복귀 */

    save_axis = save_n = 0;
    data_write();
    return 1;

bad:
    motor_speed(m, 0);
    motor_pos_mode(m);
    save_axis = save_n = 0;
    return 0;
}

/* ===== 자동 운전 ===== */

int save_busy(void) { return seq != S_IDLE; }

/* 485를 너무 자주 쓰지 않도록 50ms마다 한 번만 */
static int poll50(void)
{
    uint32_t now = HAL_GetTick();

    if (now - poll_t0 < 50U) return 0;
    poll_t0 = now;
    return 1;
}

/* ROT가 정지했고 위치가 허용오차 안인지 확인 */
static int rot_done(int target)
{
    uint8_t state;
    int now;

    if (!poll50() || !mks_state(&state)) return 0;
    if (state != 1U) return 0;          /* F1=1 이면 정지 */
    if (!mks_pos(&now)) return 0;
    return gap(now, target) <= ROT_GAP;
}

static int xy_start(int xt, int yt)
{
    if (!motor_move(&motorX, X_RPM, xt)) { reply("XY X START FAIL\r\n"); return 0; }

    if (!motor_move(&motorY, Y_RPM, yt)) {
        reply("XY Y START FAIL\r\n");
        motor_stop(&motorX);
        return 0;
    }

    x_count = y_count = 0;
    return 1;
}

/* 두 축 모두 XY_STABLE번 연속 허용오차 안이어야 도착 */
static int xy_done(int xt, int yt)
{
    int xok, yok;

    if (!poll50()) return 0;

    /* 한 번 실패하면 한 번 더 읽는다 */
    xok = motor_pos(&motorX, &x_now) || motor_pos(&motorX, &x_now);
    yok = motor_pos(&motorY, &y_now) || motor_pos(&motorY, &y_now);

    x_count = (xok && gap(x_now, xt) <= XY_GAP) ? x_count + 1 : 0;
    y_count = (yok && gap(y_now, yt) <= XY_GAP) ? y_count + 1 : 0;

    return x_count >= XY_STABLE && y_count >= XY_STABLE;
}

static void seq_reset(void)
{
    seq = S_IDLE;
    run_col = run_dan = 1;
}

static void seq_fail(const char *msg)
{
    reply(msg);
    mks_stop();
    motor_stop(&motorX);
    motor_stop(&motorY);
    seq_reset();
    seq_result = -1;
}

/* go X Y : 한 칸에 대해 사이클을 한 번만 돈다 */
static int seq_start(int col, int dan)
{
    if (!grid_ok() || !data.x_ready || !data.y_ready) return 0;
    if (data.in_ready != IN_OK) return 0;
    if (save_busy() || save_axis || !rot_homed || !x_homed || !y_homed) return 0;
    if (col < 1 || col > data.x_n || dan < 1 || dan > data.y_n) return 0;

    run_col = col;
    run_dan = dan;
    seq_result = 0;

    hold_t0 = HAL_GetTick();
    seq = S_C1_START;
    return 1;
}

int save_go(int col, int dan) { return seq_start(col, dan); }

/* 자동운전 또는 수동 저장 이동을 즉시 정지 */
void save_stop(void)
{
    if (save_axis) {
        motor_stop(&motorX);
        motor_stop(&motorY);
        save_axis = save_n = 0;
    }

    if (save_busy()) {
        mks_stop();
        motor_stop(&motorX);
        motor_stop(&motorY);
        seq_reset();
        seq_result = -1;
    }

    /* stop이면 항상 ROT를 C로 되돌린다 */
    if (rot_homed) {
        HAL_Delay(100);         /* mks_stop 감속이 끝날 시간 */
        reply(move_rot(C_POS) ? "STOP C OK\r\n" : "STOP C FAIL\r\n");
    }
}

/* 이동을 시작했으니 타임아웃 시계를 켠다 */
static void seq_next(Seq s)
{
    move_t0 = HAL_GetTick();
    poll_t0 = 0;
    seq = s;
}

/* 도착했으니 다음 단계로. HOLD_MS 대기부터 시작한다 */
static void seq_done(Seq s)
{
    hold_t0 = HAL_GetTick();
    seq = s;
}

/* 대기 시간이 지났으면 1 */
static int held(void)
{
    return HAL_GetTick() - hold_t0 >= HOLD_MS;
}

/* ROT 이동 시작. 공통 부분이라 묶었다 */
static int rot_go(int target, const char *fail_msg)
{
    if (!held()) return 0;

    if (!mks_move_abs(ROT_RPM, target)) {
        seq_fail(fail_msg);
        return 0;
    }
    return 1;
}

/*
 * 한 사이클 상태머신
 * C -> 입력칸 -> L -> C -> 목표칸 -> R -> C -> 끝
 */
static void seq_run(void)
{
    uint32_t now = HAL_GetTick();
    int xt = tx(run_col - 1);
    int yt = ty(run_dan - 1);
    char msg[64];

    switch (seq) {

    case S_IDLE:
        return;

    /* 1. ROT -> C */
    case S_C1_START:
        if (!rot_go(C_POS, "ROT C1 START FAIL\r\n")) return;
        seq_next(S_C1_WAIT);
        return;

    case S_C1_WAIT:
        if (rot_done(C_POS)) { reply("ROT C OK\r\n"); seq_done(S_IN_START); }
        else if (now - move_t0 >= TIMEOUT) seq_fail("ROT C1 TIMEOUT\r\n");
        return;

    /* 2. X/Y -> 입력칸 */
    case S_IN_START:
        if (!held()) return;
        if (!xy_start(in_tx(), in_ty())) { seq_fail("IN START FAIL\r\n"); return; }
        seq_next(S_IN_WAIT);
        return;

    case S_IN_WAIT:
        if (xy_done(in_tx(), in_ty())) {
            motor_stop(&motorX);
            motor_stop(&motorY);
            reply("IN OK\r\n");
            seq_done(S_R_START);
        }
        else if (now - move_t0 >= TIMEOUT) seq_fail("IN TIMEOUT\r\n");
        return;

    /* 3. ROT -> L */
    case S_R_START:
        if (!rot_go(L_POS, "ROT L START FAIL\r\n")) return;
        seq_next(S_R_WAIT);
        return;

    case S_R_WAIT:
        if (rot_done(L_POS)) { reply("ROT L OK\r\n"); seq_done(S_C2_START); }
        else if (now - move_t0 >= TIMEOUT) seq_fail("ROT L TIMEOUT\r\n");
        return;

    /* 4. ROT -> C */
    case S_C2_START:
        if (!rot_go(C_POS, "ROT C2 START FAIL\r\n")) return;
        seq_next(S_C2_WAIT);
        return;

    case S_C2_WAIT:
        if (rot_done(C_POS)) { reply("ROT C OK\r\n"); seq_done(S_XY_START); }
        else if (now - move_t0 >= TIMEOUT) seq_fail("ROT C2 TIMEOUT\r\n");
        return;

    /* 5. X/Y -> 목표칸 */
    case S_XY_START:
        if (!held()) return;
        if (!xy_start(xt, yt)) { seq_fail("XY START FAIL\r\n"); return; }
        seq_next(S_XY_WAIT);
        return;

    case S_XY_WAIT:
        if (xy_done(xt, yt)) {
            motor_stop(&motorX);
            motor_stop(&motorY);

            /* Python과 PowerShell이 읽기 쉬운 형식 */
            snprintf(msg, sizeof(msg), "ARRIVE %d %d\r\n", run_col, run_dan);
            reply(msg);

            seq_done(S_L_START);
        }
        else if (now - move_t0 >= TIMEOUT) seq_fail("XY TIMEOUT\r\n");
        return;

    /* 6. ROT -> R */
    case S_L_START:
        if (!rot_go(R_POS, "ROT R START FAIL\r\n")) return;
        seq_next(S_L_WAIT);
        return;

    case S_L_WAIT:
        if (rot_done(R_POS)) { reply("ROT R OK\r\n"); seq_done(S_C3_START); }
        else if (now - move_t0 >= TIMEOUT) seq_fail("ROT R TIMEOUT\r\n");
        return;

    /* 7. ROT -> C, 사이클 종료 */
    case S_C3_START:
        if (!rot_go(C_POS, "ROT C3 START FAIL\r\n")) return;
        seq_next(S_C3_WAIT);
        return;

    case S_C3_WAIT:
        if (rot_done(C_POS)) {
            reply("ROT C OK\r\n");
            reply("CYCLE END\r\n");
            seq_reset();
            seq_result = 1;
        }
        else if (now - move_t0 >= TIMEOUT) seq_fail("ROT C3 TIMEOUT\r\n");
        return;
    }
}

/* ===== 문자열 ===== */

static void trim(char *s)
{
    size_t n = strlen(s);

    while (n && (s[n - 1] == '\r' || s[n - 1] == '\n' ||
                 s[n - 1] == ' '  || s[n - 1] == '\t')) s[--n] = 0;
}

/* x4 또는 x4 save 형식 확인. y도 같은 함수로 처리 */
static int parse_save_cmd(const char *cmd, char axis, int *n)
{
    char rest[16] = "";

    if (cmd[0] != axis) return 0;
    if (sscanf(&cmd[1], "%d%15s", n, rest) < 1) return 0;

    return !rest[0] || !strcmp(rest, "save");
}

/* ===== 상태 출력 ===== */

/* show : grid와 각 칸 위치를 mm로 출력 */
static void show(void)
{
    char msg[512];
    int k = 0;

    k += snprintf(&msg[k], sizeof(msg) - k, "GRID X=%d Y=%d\r\nX:", data.x_n, data.y_n);

    for (int i = 0; i < data.x_n && k < (int)sizeof(msg) - 24; i++)
        k += snprintf(&msg[k], sizeof(msg) - k, " %d=%dmm", i + 1, MM(data.x[i]));

    k += snprintf(&msg[k], sizeof(msg) - k, "\r\nY:");

    for (int i = 0; i < data.y_n && k < (int)sizeof(msg) - 24; i++)
        k += snprintf(&msg[k], sizeof(msg) - k, " %d=%dmm", i + 1, MM(data.y[i]));

    snprintf(&msg[k], sizeof(msg) - k,
             "\r\nIN: x=%dmm y=%dmm\r\nADJ: x=%dmm y=%dmm\r\n"
             "X_READY=%d Y_READY=%d IN_READY=%d\r\n",
             MM(data.in_x), MM(data.in_y), MM(data.x_adj), MM(data.y_adj),
             data.x_ready, data.y_ready, data.in_ready);
    reply(msg);
}

/* dump : FRAM 값을 한 줄로 출력. 다른 기계에 load로 넣는다 */
static void dump(void)
{
    char msg[400];
    int k = snprintf(msg, sizeof(msg), "DATA %d %d", data.x_n, data.y_n);

    for (int i = 0; i < data.x_n; i++)
        k += snprintf(&msg[k], sizeof(msg) - k, " %d", data.x[i]);

    for (int i = 0; i < data.y_n; i++)
        k += snprintf(&msg[k], sizeof(msg) - k, " %d", data.y[i]);

    snprintf(&msg[k], sizeof(msg) - k, " %d %d\r\n", data.in_x, data.in_y);
    reply(msg);
}

/* load : dump로 뽑은 숫자열을 그대로 넣는다 */
static int load(const char *s)
{
    Data d;
    int n = 0, v;

    memset(&d, 0, sizeof(d));
    d.ver = DATA_VER;

    if (sscanf(s, "%d %d%n", &d.x_n, &d.y_n, &n) != 2) return 0;
    if (d.x_n < 2 || d.x_n > MAX_X || d.y_n < 2 || d.y_n > MAX_Y) return 0;
    s += n;

    for (int i = 0; i < d.x_n; i++) {
        if (sscanf(s, "%d%n", &v, &n) != 1) return 0;
        d.x[i] = v; s += n;
    }
    for (int i = 0; i < d.y_n; i++) {
        if (sscanf(s, "%d%n", &v, &n) != 1) return 0;
        d.y[i] = v; s += n;
    }
    if (sscanf(s, "%d %d", &d.in_x, &d.in_y) != 2) return 0;

    d.x_ref = d.y_ref = d.x_ready = d.y_ready = 1;
    d.in_ready = IN_OK;
    d.x_adj = data.x_adj;   /* 보정은 이 기계 것이라 유지 */
    d.y_adj = data.y_adj;

    data = d;
    data_write();
    return 1;
}

/* status : 현재 모터 위치와 플래그를 한 줄로 출력 */
static void status(void)
{
    char msg[300];
    int rot = 0, x = 0, y = 0;

    mks_pos(&rot);
    motor_pos(&motorX, &x);
    motor_pos(&motorY, &y);

    snprintf(msg, sizeof(msg),
             "grid=%d,%d rot_home=%d x_home=%d y_home=%d "
             "x_ready=%d y_ready=%d in_ready=%d "
             "rot=%d x=%d y=%d step=%d,%d busy=%d save=%d\r\n",
             data.x_n, data.y_n, rot_homed, x_homed, y_homed,
             data.x_ready, data.y_ready, data.in_ready,
             rot, x, y, run_col, run_dan,
             save_busy(), save_axis);
    reply(msg);
}

static int rot_cmd(int target)
{
    return rot_homed && !save_axis && !save_busy() && move_rot(target);
}

/* ===== TCP 명령 처리 ===== */

static void command(char *cmd)
{
    int a, b;
    char extra;

    trim(cmd);

    /* item 명령은 진행중에도 받는다. 미리 누르면 예약된다 */
    if (item_cmd(cmd)) return;

    /* 자동운전 중에는 stop, status, show만 허용 */
    if (save_busy() &&
        strcmp(cmd, "stop") && strcmp(cmd, "status") && strcmp(cmd, "show")) {
        reply("ERR RUNNING\r\n");
        return;
    }

    if (!strcmp(cmd, "status")) { status(); return; }
    if (!strcmp(cmd, "show"))   { show();   return; }
    if (!strcmp(cmd, "dump"))   { dump();   return; }

    if (!strncmp(cmd, "load ", 5)) {
        if (save_axis) reply("ERR SAVING\r\n");
        else reply(load(cmd + 5) ? "LOAD OK\r\n" : "LOAD FAIL\r\n");
        return;
    }

    /* grid 4 5 : 정수 두 개만 있을 때 처리 */
    if (sscanf(cmd, "grid %d %d %c", &a, &b, &extra) == 2) {
        if (save_axis) reply("ERR SAVING\r\n");
        else           grid_set(a, b);
        return;
    }

    if (!strcmp(cmd, "rot c")) { reply(rot_cmd(C_POS) ? "OK C\r\n" : "ERR C\r\n"); return; }
    if (!strcmp(cmd, "rot r")) { reply(rot_cmd(R_POS) ? "OK R\r\n" : "ERR R\r\n"); return; }
    if (!strcmp(cmd, "rot l")) { reply(rot_cmd(L_POS) ? "OK L\r\n" : "ERR L\r\n"); return; }

    if (!strcmp(cmd, "home")) {
        reply(save_home() ? "OK HOME\r\n" : "ERR HOME\r\n");
        return;
    }

    /* 저장이나 이동 전에 grid가 있어야 한다 */
    if ((cmd[0] == 'x' || cmd[0] == 'y' ||
         !strncmp(cmd, "go", 2) || !strncmp(cmd, "in", 2) ||
         !strncmp(cmd, "adj", 3)) && !grid_ok()) {
        reply("GRID FIRST: grid X Y\r\n");
        return;
    }

    /* X 저장. grid 4 5면 x4, x3 */
    if (parse_save_cmd(cmd, 'x', &a)) {
        if (a != data.x_n && a != data.x_n - 1) reply("ERR X NUMBER\r\n");
        else reply(save_start(AXIS_X, a) ? "X MOVE, PRESS S\r\n" : "X MOVE FAIL\r\n");
        return;
    }

    /* Y 저장은 y1, y2만 */
    if (parse_save_cmd(cmd, 'y', &a)) {
        if (a != 1 && a != 2) reply("ERR Y NUMBER\r\n");
        else reply(save_start(AXIS_Y, a) ? "Y MOVE, PRESS S\r\n" : "Y MOVE FAIL\r\n");
        return;
    }

    /* 입력칸 저장. X와 Y를 따로 움직이고 s를 각각 누른다 */
    if (!strcmp(cmd, "in x")) {
        reply(save_start(AXIS_IN_X, 0) ? "IN X MOVE, PRESS S\r\n" : "IN X FAIL\r\n");
        return;
    }

    if (!strcmp(cmd, "in y")) {
        reply(save_start(AXIS_IN_Y, 0) ? "IN Y MOVE, PRESS S\r\n" : "IN Y FAIL\r\n");
        return;
    }

    /* 호기별 보정. 기준칸에 맞춘 뒤 s */
    if (!strcmp(cmd, "adj x")) {
        reply(save_start(AXIS_ADJ_X, 0) ? "ADJ X MOVE, PRESS S\r\n" : "ADJ X FAIL\r\n");
        return;
    }

    if (!strcmp(cmd, "adj y")) {
        reply(save_start(AXIS_ADJ_Y, 0) ? "ADJ Y MOVE, PRESS S\r\n" : "ADJ Y FAIL\r\n");
        return;
    }

    if (!strcmp(cmd, "adj clear")) {
        data.x_adj = data.y_adj = 0;
        data_write();
        reply("ADJ CLEAR\r\n");
        return;
    }

    /* 움직이던 축의 위치를 저장 */
    if (!strcmp(cmd, "s")) {
        reply(save_end() ? "SAVE OK\r\n" : "SAVE FAIL\r\n");
        return;
    }

    /* go 4 5 */
    if (sscanf(cmd, "go %d %d %c", &a, &b, &extra) == 2) {
        reply(save_go(a, b) ? "OK GO\r\n" : "ERR GO\r\n");
        return;
    }

    /* go 3 : X3, Y3으로 이동 */
    if (sscanf(cmd, "go %d %c", &a, &extra) == 1) {
        reply(save_go(a, a) ? "OK GO\r\n" : "ERR GO\r\n");
        return;
    }

    if (!strcmp(cmd, "stop")) { save_stop(); reply("OK STOP\r\n"); return; }

    reply("ERR CMD\r\n");
}

void save_cmd(char *cmd) { command(cmd); }

/* ===== 초기화 ===== */

void save_init(void)
{
    int xok, yok;
    char msg[64];

    HAL_Delay(1000);
    data_read();
    item_init();

    if (!mks_init()) { print("MKS INIT FAIL\r\n"); return; }
    print("MKS INIT OK\r\n");

    yok = motor_init(&motorY, 1);       /* Y = UART5, AIMotor ID1 */
    print(yok ? "Y INIT OK\r\n" : "Y INIT FAIL\r\n");

    xok = motor_init(&motorX, 0);       /* X = UART4, AIMotor ID1 */
    print(xok ? "X INIT OK\r\n" : "X INIT FAIL\r\n");

    if (!xok || !yok) { print("XY INIT FAIL\r\n"); return; }

    /* 부팅 시 ROT를 먼저 잡고 X, Y가 차례로 각자의 DI1을 찾는다 */
    if (!save_home()) { print("POWER HOME FAIL\r\n"); return; }

    print("POWER HOME OK\r\n");
    print("INIT OK\r\n");

    if (!grid_ok()) print("GRID FIRST: grid X Y\r\n");
    else {
        snprintf(msg, sizeof(msg), "GRID LOAD X=%d Y=%d\r\n", data.x_n, data.y_n);
        print(msg);
    }
}

void save_run(void)
{
    if (save_busy()) seq_run();

    item_run();
}
