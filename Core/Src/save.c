#include "save.h"
#include "motor.h"
#include "rot_test.h"
#include "fram.h"
#include "tcp.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ===================== 설정값 ===================== */

#define ROT_RPM         2500
#define X_RPM           1000
#define Y_RPM           1000
#define HOME_RPM        100

#define TIMEOUT         60000U
#define HOME_TIMEOUT    65000U

#define ROT_GAP         300
#define XY_GAP          1000
#define XY_STABLE       2

#define MAX_X           8
#define MAX_Y           8

#define DATA_ADDR       0x0000
#define DATA_VER        0x0007

#define ROT_ADDR        0x0200
#define ROT_MAGIC       0x54494C54UL
#define ROT_VER         5

#define ROT_600         0x6AAA

#define REV_MM          160
#define REV_CNT         131072
#define MM(v)           ((v) * REV_MM / REV_CNT)

#define AXIS_X          1
#define AXIS_Y          2

/* ===================== 자동 운전 상태 ===================== */

/*
 * 자동운전:
 * R 또는 L 이동
 * -> C 이동
 * -> X/Y 이동
 * -> 다음 좌표
 */
typedef enum {
    S_IDLE,
    S_SIDE_START,
    S_SIDE_WAIT,
    S_C_START,
    S_C_WAIT,
    S_XY_START,
    S_XY_WAIT
} Seq;

/* ===================== FRAM 데이터 ===================== */

typedef struct {
    int ver;

    /* 실제 사용할 X 열 수와 Y 단 수 */
    int x_n;
    int y_n;

    /* 최대 MAX_X, MAX_Y까지 위치 저장 */
    int x[MAX_X];
    int y[MAX_Y];

    int home_x;
    int home_y;
    int last_x;
    int last_y;
    int home_ok;

    /*
     * ref:
     * 첫 번째 기준 위치가 저장됐는지 표시
     *
     * ready:
     * 전체 위치 계산이 끝났는지 표시
     */
    int x_ref;
    int y_ref;
    int x_ready;
    int y_ready;
} Data;

/*
 * ROT C 위치 데이터 형식을 유지한다.
 * 현재는 부팅 위치를 C=0으로 설정한다.
 */
typedef struct {
    uint32_t magic;
    int32_t c_offset;
    uint16_t version;
    uint16_t sum;
} Tilt;

/* ===================== 전역 상태 ===================== */

static Data data;
static Tilt tilt;

static int tilt_ok;

static int c_pos;
static int r_pos;
static int l_pos;

static int rot_homed;
static int x_homed;
static int y_homed;

static int save_axis;
static int save_n;

static Seq seq;

static int run_on;
static int run_col = 1;
static int run_dan = 1;

/* 1=R, -1=L */
static int side = 1;

static volatile uint8_t stop_req;

static uint32_t move_t0;
static uint32_t poll_t0;

static int x_now;
static int y_now;
static int x_count;
static int y_count;

/* ===================== 공통 함수 ===================== */

static int gap(int a, int b)
{
    return abs(a - b);
}

/*
 * UART3 출력과 TCP 응답을 동시에 보낸다.
 */
static void reply(const char *s)
{
    tcp_reply(s);
}

/*
 * 설정된 grid가 정상인지 검사한다.
 */
static int grid_ok(void)
{
    return
        data.x_n >= 2 &&
        data.x_n <= MAX_X &&
        data.y_n >= 2 &&
        data.y_n <= MAX_Y;
}

/* ===================== FRAM 함수 ===================== */

static uint16_t tilt_sum(const Tilt *p)
{
    const uint8_t *b = (const uint8_t *)p;
    uint16_t sum = 0;

    for (uint16_t i = 0;
         i < offsetof(Tilt, sum);
         i++) {

        sum += b[i];
    }

    return sum;
}

static void data_write(void)
{
    fram_write(
        DATA_ADDR,
        &data,
        sizeof(data));
}

static void data_read(void)
{
    fram_read(
        DATA_ADDR,
        &data,
        sizeof(data));

    /*
     * DATA_VER가 다르면 기존 구조와 맞지 않으므로 초기화한다.
     * memset으로 x_n과 y_n도 0이 된다.
     */
    if (data.ver != DATA_VER) {
        memset(
            &data,
            0,
            sizeof(data));

        data.ver = DATA_VER;

        data_write();
        return;
    }

    /*
     * FRAM 값이 깨져 범위를 벗어난 경우
     * grid와 위치값을 초기화한다.
     */
    if (data.x_n < 0 ||
        data.x_n > MAX_X ||
        data.y_n < 0 ||
        data.y_n > MAX_Y) {

        memset(
            &data,
            0,
            sizeof(data));

        data.ver = DATA_VER;

        data_write();
    }
}

static void tilt_write(void)
{
    tilt.magic = ROT_MAGIC;
    tilt.version = ROT_VER;
    tilt.sum = tilt_sum(&tilt);

    fram_write(
        ROT_ADDR,
        &tilt,
        sizeof(tilt));
}

/* ===================== ROT 위치 ===================== */

static void rot_set(void)
{
    c_pos = tilt.c_offset;
    r_pos = c_pos + ROT_600;
    l_pos = c_pos - ROT_600;
}

/*
 * 수동 c/r/l, HOME C, 저장 시작 전 C 이동에 사용한다.
 *
 * F1 상태가 1이면 모터가 정지한 상태다.
 * 정지 후 최종 위치가 ROT_GAP 안에 들어오면 완료한다.
 */
static int wait_rot(int target)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t state;
    int now;

    while (HAL_GetTick() - t0 <
           TIMEOUT) {

        if (mks_state(&state) &&
            state == 1U &&
            mks_pos(&now) &&
            gap(now, target) <= ROT_GAP) {

            return 1;
        }

        HAL_Delay(50);
    }

    mks_stop();

    return 0;
}

static int move_rot(int target)
{
    return
        mks_move_abs(
            ROT_RPM,
            target) &&
        wait_rot(target);
}

/*
 * 전원을 켠 현재 ROT 위치를 C=0으로 설정한다.
 * 광센서를 찾으러 움직이지 않는다.
 */
static int rot_init_c(void)
{
    rot_homed = 0;

    /*
     * 현재 MKS 위치를 움직이지 않고
     * Axis 좌표 0으로 설정한다.
     */
    if (!mks_zero()) {
        return 0;
    }

    tilt.c_offset = 0;

    tilt_write();

    tilt_ok = 1;

    /*
     * C=0
     * R=+ROT_600
     * L=-ROT_600
     */
    rot_set();

    rot_homed = 1;

    return 1;
}

/* ===================== X/Y 홈 ===================== */

/*
 * X와 Y를 동시에 홈 방향으로 움직인다.
 *
 * X DI1=1:
 * X만 정지
 *
 * Y DI1=1:
 * Y만 정지
 *
 * 두 축 모두 센서를 인식하면
 * 각각의 정지 위치를 소프트웨어 원점 0으로 설정한다.
 */
static int home_xy(void)
{
    uint32_t t0;

    int xd = 0;
    int yd = 0;

    int xs;
    int ys;

    print("HOME XY START\r\n");

    /*
     * X/Y를 속도제어 모드로 변경한다.
     */
    if (!motor_speed_mode(&motorX) ||
        !motor_speed_mode(&motorY)) {

        goto fail;
    }

    /*
     * X 홈 방향: -100rpm
     */
    if (!motor_speed(
            &motorX,
            -HOME_RPM)) {

        goto fail;
    }

    /*
     * Y 홈 방향: +100rpm
     */
    if (!motor_speed(
            &motorY,
            HOME_RPM)) {

        goto fail;
    }

    t0 = HAL_GetTick();

    while (HAL_GetTick() - t0 <
           HOME_TIMEOUT) {

        /*
         * X가 아직 센서를 찾지 못했을 때만 검사한다.
         */
        if (!xd) {
            xs = motor_home_on(
                    &motorX);

            if (xs < 0) {
                goto fail;
            }

            /*
             * X DI1=1이면 X만 정지한다.
             */
            if (xs == 1) {
                motor_speed(
                    &motorX,
                    0);

                xd = 1;

                print(
                    "HOME X SENSOR\r\n");
            }
        }

        /*
         * Y가 아직 센서를 찾지 못했을 때만 검사한다.
         */
        if (!yd) {
            ys = motor_home_on(
                    &motorY);

            if (ys < 0) {
                goto fail;
            }

            /*
             * Y DI1=1이면 Y만 정지한다.
             */
            if (ys == 1) {
                motor_speed(
                    &motorY,
                    0);

                yd = 1;

                print(
                    "HOME Y SENSOR\r\n");
            }
        }

        if (xd && yd) {
            break;
        }

        HAL_Delay(10);
    }

    if (!xd || !yd) {
        goto fail;
    }

    /*
     * DEC 감속이 끝날 시간을 기다린다.
     */
    HAL_Delay(50);

    /*
     * 센서에서 정지한 위치를
     * 각각의 소프트웨어 좌표 0으로 설정한다.
     */
    if (!motor_zero(&motorX) ||
        !motor_zero(&motorY)) {

        goto fail;
    }

    /*
     * 홈 완료 후 위치제어 모드로 돌아간다.
     */
    if (!motor_pos_mode(&motorX) ||
        !motor_pos_mode(&motorY)) {

        goto fail;
    }

    x_homed = 1;
    y_homed = 1;

    data.home_x = 0;
    data.home_y = 0;

    data.last_x = 0;
    data.last_y = 0;

    data.home_ok |= 0x03;

    data_write();

    print("HOME XY OK\r\n");

    return 1;

fail:
    motor_speed(
        &motorX,
        0);

    motor_speed(
        &motorY,
        0);

    motor_pos_mode(
        &motorX);

    motor_pos_mode(
        &motorY);

    x_homed = 0;
    y_homed = 0;

    print("HOME XY FAIL\r\n");

    return 0;
}

int save_home(void)
{
    if (save_busy() ||
        save_axis ||
        !rot_homed ||
        !tilt_ok) {

        return 0;
    }

    /*
     * X/Y 홈 전에 ROT를 C로 이동한다.
     */
    if (!move_rot(c_pos)) {
        HAL_Delay(300);

        if (!move_rot(c_pos)) {
            print(
                "HOME C FAIL\r\n");

            return 0;
        }
    }

    print("HOME C OK\r\n");

    return home_xy();
}

/* ===================== grid 설정 ===================== */

/*
 * grid 크기가 변경된 축만 위치값을 초기화한다.
 *
 * 같은 grid를 다시 입력하면
 * 기존 위치값은 유지한다.
 */
static void grid_set(int xn, int yn)
{
    char msg[96];

    if (xn < 2 ||
        xn > MAX_X ||
        yn < 2 ||
        yn > MAX_Y) {

        snprintf(
            msg,
            sizeof(msg),
            "GRID ERROR X=2~%d Y=2~%d\r\n",
            MAX_X,
            MAX_Y);

        reply(msg);
        return;
    }

    /*
     * X 열 수가 변경되면
     * X 위치와 저장 상태만 초기화한다.
     */
    if (data.x_n != xn) {
        memset(
            data.x,
            0,
            sizeof(data.x));

        data.x_n = xn;
        data.last_x = 0;
        data.x_ref = 0;
        data.x_ready = 0;
    }

    /*
     * Y 단 수가 변경되면
     * Y 위치와 저장 상태만 초기화한다.
     */
    if (data.y_n != yn) {
        memset(
            data.y,
            0,
            sizeof(data.y));

        data.y_n = yn;
        data.last_y = 0;
        data.y_ref = 0;
        data.y_ready = 0;
    }

    run_col = 1;
    run_dan = 1;

    data_write();

    snprintf(
        msg,
        sizeof(msg),
        "GRID OK X=%d Y=%d\r\n",
        data.x_n,
        data.y_n);

    reply(msg);
}

/* ===================== 위치 저장 ===================== */

/*
 * X:
 * 마지막 열을 먼저 저장한다.
 * 그다음 마지막 전 열을 저장한다.
 *
 * grid 4 5:
 * x4 -> s
 * x3 -> s
 *
 * grid 6 5:
 * x6 -> s
 * x5 -> s
 *
 * Y:
 * y1 -> s
 * y2 -> s
 */
static int save_start(int axis, int n)
{
    Motor *m;
    int rpm;

    if (save_busy() ||
        save_axis) {

        return 0;
    }

    if (!grid_ok()) {
        return 0;
    }

    switch (axis) {

    case AXIS_X:
        if (!x_homed) {
            return 0;
        }

        /*
         * X는 마지막 열과
         * 마지막 전 열만 직접 저장한다.
         */
        if (n != data.x_n &&
            n != data.x_n - 1) {

            return 0;
        }

        /*
         * 마지막 전 열을 저장하기 전에
         * 마지막 열이 먼저 저장돼 있어야 한다.
         */
        if (n == data.x_n - 1 &&
            !data.x_ref) {

            return 0;
        }

        m = &motorX;

        /*
         * X 저장 방향은 홈 방향의 반대다.
         */
        rpm = HOME_RPM;

        break;

    case AXIS_Y:
        if (!y_homed) {
            return 0;
        }

        /*
         * Y는 y1과 y2만 직접 저장한다.
         */
        if (n != 1 &&
            n != 2) {

            return 0;
        }

        /*
         * y2 저장 전에 y1이 먼저 저장돼 있어야 한다.
         */
        if (n == 2 &&
            !data.y_ref) {

            return 0;
        }

        m = &motorY;

        /*
         * Y 저장 방향은 홈 방향의 반대다.
         */
        rpm = -HOME_RPM;

        break;

    default:
        return 0;
    }

    /*
     * 저장 이동 전에 ROT를 C로 이동한다.
     */
    if (!move_rot(c_pos)) {
        return 0;
    }

    if (!motor_speed_mode(m) ||
        !motor_speed(m, rpm)) {

        return 0;
    }

    /*
     * s 명령을 받을 때 사용할
     * 축과 저장 번호를 기억한다.
     */
    save_axis = axis;
    save_n = n;

    return 1;
}

static int save_end(void)
{
    Motor *m;

    int pos;
    int step;

    int axis = save_axis;
    int n = save_n;

    if (!axis) {
        return 0;
    }

    m = (axis == AXIS_X) ?
        &motorX :
        &motorY;

    /*
     * 저장 이동을 정지한다.
     */
    if (!motor_speed(m, 0)) {
        goto bad;
    }

    HAL_Delay(100);

    /*
     * 정지 위치를 읽는다.
     */
    if (!motor_pos(m, &pos) &&
        !motor_pos(m, &pos)) {

        goto bad;
    }

    if (axis == AXIS_X) {
        int last =
            data.x_n - 1;

        /*
         * 첫 번째 X 저장:
         * 마지막 열 위치를 저장한다.
         */
        if (n == data.x_n) {
            memset(
                data.x,
                0,
                sizeof(data.x));

            data.x[last] = pos;

            data.x_ref = 1;
            data.x_ready = 0;
        }

        /*
         * 두 번째 X 저장:
         * 마지막 전 열을 저장한 뒤
         * X1까지 같은 간격으로 계산한다.
         */
        else if (n ==
                 data.x_n - 1) {

            if (!data.x_ref) {
                goto bad;
            }

            step =
                pos -
                data.x[last];

            if (!step) {
                goto bad;
            }

            data.x[last - 1] = pos;

            /*
             * 뒤에서 앞으로 계산한다.
             *
             * 예:
             * X4=1000, X3=2000
             * X2=3000, X1=4000
             */
            for (int i = last - 2;
                 i >= 0;
                 i--) {

                data.x[i] =
                    data.x[i + 1] +
                    step;
            }

            data.x_ready = 1;
        }
        else {
            goto bad;
        }

        data.last_x = pos;
    }
    else {
        /*
         * 첫 번째 Y 저장:
         * Y1 위치를 저장한다.
         */
        if (n == 1) {
            memset(
                data.y,
                0,
                sizeof(data.y));

            data.y[0] = pos;

            data.y_ref = 1;
            data.y_ready = 0;
        }

        /*
         * 두 번째 Y 저장:
         * Y2를 기준으로 마지막 단까지 계산한다.
         */
        else if (n == 2) {
            if (!data.y_ref) {
                goto bad;
            }

            step =
                pos -
                data.y[0];

            if (!step) {
                goto bad;
            }

            /*
             * Y1부터 같은 간격으로
             * 실제 사용하는 마지막 단까지 계산한다.
             */
            for (int i = 1;
                 i < data.y_n;
                 i++) {

                data.y[i] =
                    data.y[0] +
                    step * i;
            }

            data.y_ready = 1;
        }
        else {
            goto bad;
        }

        data.last_y = pos;
    }

    /*
     * 저장 완료 후 위치제어 모드로 복귀한다.
     */
    if (!motor_pos_mode(m)) {
        goto bad;
    }

    save_axis = 0;
    save_n = 0;

    data_write();

    return 1;

bad:
    motor_speed(
        m,
        0);

    motor_pos_mode(m);

    save_axis = 0;
    save_n = 0;

    return 0;
}

/* ===================== 자동 운전 ===================== */

int save_busy(void)
{
    return seq != S_IDLE;
}

/*
 * UART5 통신을 너무 자주 하지 않도록
 * 50ms마다 한 번만 확인한다.
 */
static int poll50(void)
{
    uint32_t now =
        HAL_GetTick();

    if (now - poll_t0 <
        50U) {

        return 0;
    }

    poll_t0 = now;

    return 1;
}

/*
 * ROT가 실제로 정지한 뒤
 * 최종 위치가 허용 오차 안에 있는지 확인한다.
 */
static int rot_done(int target)
{
    uint8_t state;
    int now;

    if (!poll50()) {
        return 0;
    }

    if (!mks_state(&state)) {
        return 0;
    }

    /*
     * F1=1: 모터 정지
     */
    if (state != 1U) {
        return 0;
    }

    if (!mks_pos(&now)) {
        return 0;
    }

    return
        gap(now, target) <=
        ROT_GAP;
}

static int xy_start(int xt, int yt)
{
    if (!motor_move(
            &motorX,
            X_RPM,
            xt)) {

        reply(
            "XY X START FAIL\r\n");

        return 0;
    }

    if (!motor_move(
            &motorY,
            Y_RPM,
            yt)) {

        reply(
            "XY Y START FAIL\r\n");

        motor_stop(
            &motorX);

        return 0;
    }

    x_count = 0;
    y_count = 0;

    return 1;
}

static int xy_done(int xt, int yt)
{
    int xok;
    int yok;

    if (!poll50()) {
        return 0;
    }

    /*
     * 한 번 실패하면 한 번 더 읽는다.
     */
    xok =
        motor_pos(
            &motorX,
            &x_now) ||
        motor_pos(
            &motorX,
            &x_now);

    yok =
        motor_pos(
            &motorY,
            &y_now) ||
        motor_pos(
            &motorY,
            &y_now);

    if (xok &&
        gap(x_now, xt) <=
        XY_GAP) {

        x_count++;
    }
    else {
        x_count = 0;
    }

    if (yok &&
        gap(y_now, yt) <=
        XY_GAP) {

        y_count++;
    }
    else {
        y_count = 0;
    }

    return
        x_count >= XY_STABLE &&
        y_count >= XY_STABLE;
}

/*
 * X와 Y 전체 위치 계산이 완료됐는지 확인한다.
 */
static int all_saved(void)
{
    return
        grid_ok() &&
        data.x_ready &&
        data.y_ready;
}

static void seq_reset(void)
{
    seq = S_IDLE;

    run_on = 0;

    run_col = 1;
    run_dan = 1;

    side = 1;

    stop_req = 0;
}

static void seq_fail(const char *msg)
{
    reply(msg);

    mks_stop();

    motor_stop(
        &motorX);

    motor_stop(
        &motorY);

    seq_reset();
}

/*
 * 자동 run에서는 X 열을 먼저 증가시킨다.
 *
 * grid 4 3 예:
 * 1,1 -> 2,1 -> 3,1 -> 4,1
 * -> 1,2 -> 2,2 ... -> 4,3
 * -> 다시 1,1
 */
static void next_xy(void)
{
    run_col++;

    if (run_col >
        data.x_n) {

        run_col = 1;
        run_dan++;
    }

    if (run_dan >
        data.y_n) {

        run_dan = 1;
    }
}

static int seq_start(
        int col,
        int dan,
        int repeat)
{
    if (!grid_ok() ||
        !data.x_ready ||
        !data.y_ready) {

        return 0;
    }

    if (save_busy() ||
        save_axis ||
        !rot_homed ||
        !x_homed ||
        !y_homed) {

        return 0;
    }

    if (col < 1 ||
        col > data.x_n ||
        dan < 1 ||
        dan > data.y_n) {

        return 0;
    }

    /*
     * repeat=1은 자동 run이다.
     */
    if (repeat &&
        !all_saved()) {

        return 0;
    }

    run_col = col;
    run_dan = dan;

    run_on = repeat;

    stop_req = 0;

    /*
     * 첫 ROT 이동은 R부터 시작한다.
     */
    side = 1;

    seq = S_SIDE_START;

    return 1;
}

int save_go(int col, int dan)
{
    return seq_start(
        col,
        dan,
        0);
}

int save_auto(void)
{
    return seq_start(
        1,
        1,
        1);
}

/*
 * 자동운전 또는 수동 위치 저장 이동을 즉시 정지한다.
 */
void save_stop(void)
{
    if (save_axis == AXIS_X) {
        motor_stop(
            &motorX);

        save_axis = 0;
        save_n = 0;
    }
    else if (save_axis ==
             AXIS_Y) {

        motor_stop(
            &motorY);

        save_axis = 0;
        save_n = 0;
    }

    if (!save_busy()) {
        return;
    }

    stop_req = 1;

    mks_stop();

    motor_stop(
        &motorX);

    motor_stop(
        &motorY);

    seq_reset();
}

/*
 * 자동운전 상태머신
 */
static void seq_run(void)
{
    uint32_t now =
        HAL_GetTick();

    int xt =
        data.x[run_col - 1];

    int yt =
        data.y[run_dan - 1];

    int target =
        (side > 0) ?
        r_pos :
        l_pos;

    switch (seq) {

    case S_IDLE:
        return;

    case S_SIDE_START:
        if (!mks_move_abs(
                ROT_RPM,
                target)) {

            seq_fail(
                "ROT SIDE START FAIL\r\n");

            return;
        }

        move_t0 = now;
        poll_t0 = 0;

        seq = S_SIDE_WAIT;

        return;

    case S_SIDE_WAIT:
        if (rot_done(target)) {
            reply(
                side > 0 ?
                "ROT R OK\r\n" :
                "ROT L OK\r\n");

            seq = S_C_START;
        }
        else if (now - move_t0 >=
                 TIMEOUT) {

            seq_fail(
                "ROT SIDE TIMEOUT\r\n");
        }

        return;

    case S_C_START:
        if (!mks_move_abs(
                ROT_RPM,
                c_pos)) {

            seq_fail(
                "ROT C START FAIL\r\n");

            return;
        }

        move_t0 = now;
        poll_t0 = 0;

        seq = S_C_WAIT;

        return;

    case S_C_WAIT:
        if (rot_done(c_pos)) {
            reply(
                "ROT C OK\r\n");

            seq = S_XY_START;
        }
        else if (now - move_t0 >=
                 TIMEOUT) {

            seq_fail(
                "ROT C TIMEOUT\r\n");
        }

        return;

    case S_XY_START:
        if (!xy_start(
                xt,
                yt)) {

            seq_fail(
                "XY START FAIL\r\n");

            return;
        }

        move_t0 = now;
        poll_t0 = 0;

        seq = S_XY_WAIT;

        return;

    case S_XY_WAIT:
        if (xy_done(
                xt,
                yt)) {

            char msg[64];

            motor_stop(
                &motorX);

            motor_stop(
                &motorY);

            /*
             * Python과 PowerShell이 읽기 쉬운 형식이다.
             * UART3에도 같은 문장이 출력된다.
             */
            snprintf(
                msg,
                sizeof(msg),
                "ARRIVE %d %d\r\n",
                run_col,
                run_dan);

            reply(msg);

            data.last_x = x_now;
            data.last_y = y_now;

            data_write();

            /*
             * go 명령은 한 위치 이동 후 종료한다.
             */
            if (!run_on ||
                stop_req) {

                seq_reset();
                return;
            }

            /*
             * run 명령은 다음 grid 위치로 넘어간다.
             */
            next_xy();

            side = -side;

            seq = S_SIDE_START;
        }
        else if (now - move_t0 >=
                 TIMEOUT) {

            seq_fail(
                "XY TIMEOUT\r\n");
        }

        return;
    }
}

/* ===================== 문자열 함수 ===================== */

static void trim(char *s)
{
    size_t n =
        strlen(s);

    while (n &&
           (s[n - 1] == '\r' ||
            s[n - 1] == '\n' ||
            s[n - 1] == ' ' ||
            s[n - 1] == '\t')) {

        s[--n] = 0;
    }
}

/*
 * x4 또는 x4 save 형식을 검사한다.
 * y1 또는 y1 save도 같은 함수로 처리한다.
 */
static int parse_save_cmd(
        const char *cmd,
        char axis,
        int *n)
{
    char short_cmd[24];
    char long_cmd[32];

    if (cmd[0] != axis) {
        return 0;
    }

    if (sscanf(
            &cmd[1],
            "%d",
            n) != 1) {

        return 0;
    }

    snprintf(
        short_cmd,
        sizeof(short_cmd),
        "%c%d",
        axis,
        *n);

    snprintf(
        long_cmd,
        sizeof(long_cmd),
        "%c%d save",
        axis,
        *n);

    return
        !strcmp(cmd, short_cmd) ||
        !strcmp(cmd, long_cmd);
}

/* ===================== 상태 출력 ===================== */

static void show(void)
{
    char msg[512];
    int k = 0;

    k += snprintf(
        &msg[k],
        sizeof(msg) - k,
        "GRID X=%d Y=%d\r\n",
        data.x_n,
        data.y_n);

    k += snprintf(
        &msg[k],
        sizeof(msg) - k,
        "X:");

    for (int i = 0;
         i < data.x_n &&
         k < (int)sizeof(msg) - 24;
         i++) {

        k += snprintf(
            &msg[k],
            sizeof(msg) - k,
            " %d=%dmm",
            i + 1,
            MM(data.x[i]));
    }

    k += snprintf(
        &msg[k],
        sizeof(msg) - k,
        "\r\nY:");

    for (int i = 0;
         i < data.y_n &&
         k < (int)sizeof(msg) - 24;
         i++) {

        k += snprintf(
            &msg[k],
            sizeof(msg) - k,
            " %d=%dmm",
            i + 1,
            MM(data.y[i]));
    }

    snprintf(
        &msg[k],
        sizeof(msg) - k,
        "\r\nX_READY=%d Y_READY=%d\r\n",
        data.x_ready,
        data.y_ready);

    reply(msg);
}

static void status(void)
{
    char msg[300];

    int rot = 0;
    int x = 0;
    int y = 0;

    mks_pos(&rot);
    motor_pos(
        &motorX,
        &x);
    motor_pos(
        &motorY,
        &y);

    snprintf(
        msg,
        sizeof(msg),
        "grid=%d,%d rot_home=%d x_home=%d y_home=%d "
        "x_ready=%d y_ready=%d c=%d r=%d l=%d "
        "rot=%d x=%d y=%d step=%d,%d side=%c busy=%d save=%d\r\n",
        data.x_n,
        data.y_n,
        rot_homed,
        x_homed,
        y_homed,
        data.x_ready,
        data.y_ready,
        c_pos,
        r_pos,
        l_pos,
        rot,
        x,
        y,
        run_col,
        run_dan,
        side > 0 ? 'R' : 'L',
        save_busy(),
        save_axis);

    reply(msg);
}

static int rot_cmd(int target)
{
    return
        rot_homed &&
        !save_axis &&
        !save_busy() &&
        move_rot(target);
}

/* ===================== TCP 명령 처리 ===================== */

static void command(char *cmd)
{
    int a;
    int b;

    char extra;

    trim(cmd);

    /*
     * 자동운전 중에는
     * stop, status, show만 허용한다.
     */
    if (save_busy() &&
        strcmp(cmd, "stop") &&
        strcmp(cmd, "status") &&
        strcmp(cmd, "show")) {

        reply(
            "ERR RUNNING\r\n");

        return;
    }

    if (!strcmp(
            cmd,
            "status")) {

        status();
        return;
    }

    if (!strcmp(
            cmd,
            "show")) {

        show();
        return;
    }

    /*
     * grid 4 5
     *
     * 정확히 정수 두 개가 있을 때만 처리한다.
     */
    if (sscanf(
            cmd,
            "grid %d %d %c",
            &a,
            &b,
            &extra) == 2) {

        if (save_axis) {
            reply(
                "ERR SAVING\r\n");

            return;
        }

        grid_set(
            a,
            b);

        return;
    }

    if (!strcmp(cmd, "c")) {
        reply(
            rot_cmd(c_pos) ?
            "OK C\r\n" :
            "ERR C\r\n");

        return;
    }

    if (!strcmp(cmd, "r")) {
        reply(
            rot_cmd(r_pos) ?
            "OK R\r\n" :
            "ERR R\r\n");

        return;
    }

    if (!strcmp(cmd, "l")) {
        reply(
            rot_cmd(l_pos) ?
            "OK L\r\n" :
            "ERR L\r\n");

        return;
    }

    if (!strcmp(
            cmd,
            "home")) {

        reply(
            save_home() ?
            "OK HOME\r\n" :
            "ERR HOME\r\n");

        return;
    }

    /*
     * 위치 저장이나 이동 전에
     * grid가 설정돼 있어야 한다.
     */
    if ((cmd[0] == 'x' ||
         cmd[0] == 'y' ||
         !strncmp(cmd, "go", 2) ||
         !strcmp(cmd, "run")) &&
        !grid_ok()) {

        reply(
            "GRID FIRST: grid X Y\r\n");

        return;
    }

    /*
     * X 저장 명령
     *
     * grid 4 5:
     * x4, x3
     *
     * grid 6 5:
     * x6, x5
     */
    if (parse_save_cmd(
            cmd,
            'x',
            &a)) {

        if (a != data.x_n &&
            a != data.x_n - 1) {

            reply(
                "ERR X NUMBER\r\n");

            return;
        }

        reply(
            save_start(
                AXIS_X,
                a) ?
            "X MOVE, PRESS S\r\n" :
            "X MOVE FAIL\r\n");

        return;
    }

    /*
     * Y 저장 명령은 y1, y2만 사용한다.
     */
    if (parse_save_cmd(
            cmd,
            'y',
            &a)) {

        if (a != 1 &&
            a != 2) {

            reply(
                "ERR Y NUMBER\r\n");

            return;
        }

        reply(
            save_start(
                AXIS_Y,
                a) ?
            "Y MOVE, PRESS S\r\n" :
            "Y MOVE FAIL\r\n");

        return;
    }

    /*
     * 현재 움직이던 X 또는 Y 위치를 저장한다.
     */
    if (!strcmp(cmd, "s")) {
        reply(
            save_end() ?
            "SAVE OK\r\n" :
            "SAVE FAIL\r\n");

        return;
    }

    /*
     * go 4 5
     */
    if (sscanf(
            cmd,
            "go %d %d %c",
            &a,
            &b,
            &extra) == 2) {

        reply(
            save_go(
                a,
                b) ?
            "OK GO\r\n" :
            "ERR GO\r\n");

        return;
    }

    /*
     * go 3
     * X3, Y3으로 이동한다.
     */
    if (sscanf(
            cmd,
            "go %d %c",
            &a,
            &extra) == 1) {

        reply(
            save_go(
                a,
                a) ?
            "OK GO\r\n" :
            "ERR GO\r\n");

        return;
    }

    /*
     * grid 전체를 순서대로 반복한다.
     */
    if (!strcmp(cmd, "run")) {
        reply(
            save_auto() ?
            "OK RUN\r\n" :
            "ERR RUN\r\n");

        return;
    }

    if (!strcmp(cmd, "stop")) {
        save_stop();

        reply(
            "OK STOP\r\n");

        return;
    }

    reply("ERR CMD\r\n");
}

void save_cmd(char *cmd)
{
    command(cmd);
}

/* ===================== 초기화 ===================== */

void save_init(void)
{
    int mok;
    int xok;
    int yok;

    HAL_Delay(1000);

    data_read();

    mok = mks_init();

    print(
        mok ?
        "MKS INIT OK\r\n" :
        "MKS INIT FAIL\r\n");

    if (!mok) {
        return;
    }

    /*
     * Y는 UART5, AIMotor ID1
     */
    yok = motor_init(
        &motorY,
        1);

    print(
        yok ?
        "Y INIT OK\r\n" :
        "Y INIT FAIL\r\n");

    /*
     * X는 UART4, AIMotor ID1
     */
    xok = motor_init(
        &motorX,
        0);

    print(
        xok ?
        "X INIT OK\r\n" :
        "X INIT FAIL\r\n");

    /*
     * 전원 ON 당시 ROT 위치를 C로 설정한다.
     */
    if (!rot_init_c()) {
        print(
            "ROT C SET FAIL\r\n");

        return;
    }

    print(
        "ROT C SET OK\r\n");

    /*
     * X/Y는 home 명령을 실행해야
     * 원점 완료 상태가 된다.
     */
    x_homed = 0;
    y_homed = 0;

    if (xok && yok) {
        print(
            "INIT OK\r\n");
    }
    else {
        print(
            "ROT OK, XY INIT FAIL\r\n");
    }

    if (!grid_ok()) {
        print(
            "GRID FIRST: grid X Y\r\n");
    }
    else {
        char msg[64];

        snprintf(
            msg,
            sizeof(msg),
            "GRID LOAD X=%d Y=%d\r\n",
            data.x_n,
            data.y_n);

        print(msg);
    }
}

void save_run(void)
{
    if (save_busy()) {
        seq_run();
    }
}
