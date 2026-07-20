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
#define ROT_RPM         600   /* lower speed for accurate C stop */
#define X_RPM           1000
#define Y_RPM           1000
#define HOME_X_RPM      100    /* X home speed, rpm */
#define HOME_Y_RPM      100    /* Y seek speed, rpm */
#define HOME_RELEASE_RPM 30    /* leave an already-active sensor slowly */
#define HOME_RELEASE_MS  5000U
#define TIMEOUT         60000U
#define HOME_TIMEOUT    65000U
#define ROT_GAP         20
#define ROT_STABLE      3
#define XY_GAP          1000
#define XY_STABLE       2
#define DATA_ADDR       0x0000
#define DATA_VER        0x0005
#define ROT_ADDR        0x0200
#define ROT_MAGIC       0x54494C54UL
#define ROT_VER         5
#define ROT_600         0x6AAA
#define REV_MM          160
#define REV_CNT         131072
#define MM(v)           ((v) * REV_MM / REV_CNT)
#define AXIS_X          1
#define AXIS_Y          2
/* RUN_N=3: (1,1)->(2,2)->(3,3)->다시 (1,1), 현재 범위 1~4 */
#define RUN_N           3
#if RUN_N < 1 || RUN_N > 4
#error "RUN_N must be 1..4"
#endif
/* RUN: C -> R -> C -> XY -> L -> C -> XY -> R -> C ... */
typedef enum {
    S_IDLE,
    S_SIDE_START,
    S_SIDE_WAIT,
    S_C_START,
    S_C_WAIT,
    S_XY_START,
    S_XY_WAIT
} Seq;
typedef struct {
    int ver;
    int x[4];
    int y[4];
    int home_x;
    int home_y;
    int last_x;
    int last_y;
    int home_ok;
} Data;
typedef struct {
    uint32_t magic;
    int32_t c_offset;
    uint16_t version;
    uint16_t sum;
} Tilt;
static Data data;
static Tilt tilt;
static int tilt_ok;
static int c_pos, r_pos, l_pos;
static int rot_homed, x_homed, y_homed;
static int save_axis, save_n;
static Seq seq;
static int run_on;
static int run_col = 1, run_dan = 1;
static int side = 1;                 /* 1=R, -1=L */
static volatile uint8_t stop_req;
static uint32_t move_t0, poll_t0, xy_log_t0;
static int x_now, y_now, x_count, y_count;
static int rot_count;
static int gap(int a, int b)
{
    return abs(a - b);
}
static uint16_t tilt_sum(const Tilt *p)
{
    const uint8_t *b = (const uint8_t *)p;
    uint16_t s = 0;
    for (uint16_t i = 0; i < offsetof(Tilt, sum); i++) s += b[i];
    return s;
}
static int tilt_valid(const Tilt *p)
{
    return p->magic == ROT_MAGIC && p->version == ROT_VER &&
           p->sum == tilt_sum(p);
}
static void data_write(void)
{
    fram_write(DATA_ADDR, &data, sizeof(data));
}
static void data_read(void)
{
    fram_read(DATA_ADDR, &data, sizeof(data));
    if (data.ver != DATA_VER) {
        memset(&data, 0, sizeof(data));
        data.ver = DATA_VER;
        data_write();
    }
}
static void tilt_write(void)
{
    tilt.magic = ROT_MAGIC;
    tilt.version = ROT_VER;
    tilt.sum = tilt_sum(&tilt);
    fram_write(ROT_ADDR, &tilt, sizeof(tilt));
}
static void tilt_read(void)
{
    Tilt retry;
    fram_read(ROT_ADDR, &tilt, sizeof(tilt));
    if (tilt_valid(&tilt)) {
        tilt_ok = 1;
        print("ROT DATA OK\r\n");
        return;
    }
    HAL_Delay(20);
    fram_read(ROT_ADDR, &retry, sizeof(retry));
    if (tilt_valid(&retry)) {
        tilt = retry;
        tilt_ok = 1;
        print("ROT DATA RETRY OK\r\n");
        return;
    }
    memset(&tilt, 0, sizeof(tilt));
    tilt_ok = 0;
    print("ROT DATA INVALID\r\n");
}
static void rot_set(void)
{
    c_pos = tilt.c_offset;
    r_pos = c_pos + ROT_600;
    l_pos = c_pos - ROT_600;
}
static int wait_rot(int target)
{
    uint32_t t0 = HAL_GetTick();
    int now;
    while (HAL_GetTick() - t0 < TIMEOUT) {
        if (mks_pos(&now) && gap(now, target) <= ROT_GAP) return 1;
        HAL_Delay(50);
    }
    mks_stop();
    return 0;
}
static int move_rot(int target)
{
    return mks_move_abs(ROT_RPM, target) && wait_rot(target);
}
static int rot_init_home(void)
{
    int raw_c = 0;
    int raw_home = 0;
    int axis_home = 0;

    rot_homed = 0;

    /* Capture the physical encoder value at this power-on position C. */
    if (!mks_raw(&raw_c) && !mks_raw(&raw_c)) return 0;

    /* Find the optical sensor. */
    if (!mks_home()) return 0;

    if ((!mks_raw(&raw_home) && !mks_raw(&raw_home)) ||
        (!mks_pos(&axis_home) && !mks_pos(&axis_home))) return 0;

    /*
     * F5 absolute motion uses the command-31 axis coordinate.
     * RAW gives the physical C-to-home distance. Add that distance to
     * the actual axis value after homing instead of assuming it is zero.
     */
    tilt.c_offset = axis_home + (raw_c - raw_home);
    tilt_write();
    tilt_ok = 1;

    rot_set();
    rot_homed = 1;
    return 1;
}
static int home_axis(Motor *m, int rpm, const char *ok, const char *fail)
{
    uint32_t t0;
    int sensor;

    if (!motor_speed_mode(m)) goto bad;

    /*
     * If the axis is already touching DI1, moving toward home makes only
     * a quick tick. First leave the sensor slowly, then seek it again.
     */
    sensor = motor_home_on(m);
    if (sensor < 0) goto bad;

    if (sensor == 1) {
        if (!motor_speed(m, rpm > 0 ? -HOME_RELEASE_RPM : HOME_RELEASE_RPM))
            goto bad;

        t0 = HAL_GetTick();
        do {
            sensor = motor_home_on(m);
            if (sensor < 0) goto bad;
            if (sensor == 0) break;
            HAL_Delay(10);
        } while (HAL_GetTick() - t0 < HOME_RELEASE_MS);

        motor_speed(m, 0);
        if (sensor != 0) goto bad;
        HAL_Delay(100);
    }

    if (!motor_speed(m, rpm)) goto bad;
    t0 = HAL_GetTick();

    while (HAL_GetTick() - t0 < HOME_TIMEOUT) {
        sensor = motor_home_on(m);
        if (sensor < 0) goto bad;
        if (sensor == 1) {
            motor_speed(m, 0);
            HAL_Delay(100);
            if (!motor_zero(m) || !motor_pos_mode(m)) goto bad;
            print(ok);
            return 1;
        }
        HAL_Delay(10);
    }

bad:
    motor_speed(m, 0);
    motor_pos_mode(m);
    print(fail);
    return 0;
}
static int home_xy(void)
{
    int xok, yok;
    print("HOME XY START\r\n");
    xok = home_axis(&motorX, -HOME_X_RPM,
                    "HOME X OK\r\n", "HOME X FAIL\r\n");
    yok = home_axis(&motorY, HOME_Y_RPM,
                    "HOME Y OK\r\n", "HOME Y FAIL\r\n");
    x_homed = xok;
    y_homed = yok;
    if (!xok || !yok) return 0;
    data.home_x = data.home_y = 0;
    data.last_x = data.last_y = 0;
    data.home_ok |= 0x03;
    data_write();
    print("HOME XY OK\r\n");
    return 1;
}
int save_home(void)
{
    if (save_busy() || save_axis || !rot_homed || !tilt_ok) return 0;
    if (!move_rot(c_pos)) {
        HAL_Delay(300);
        if (!move_rot(c_pos)) {
            print("HOME C FAIL\r\n");
            return 0;
        }
    }
    print("HOME C OK\r\n");
    return home_xy();
}
static int save_start(int axis, int n)
{
    Motor *m;
    int rpm;
    if (save_busy() || save_axis || n < 1 || n > 2) return 0;
    if (axis == AXIS_X && !x_homed) return 0;
    if (axis == AXIS_Y && !y_homed) return 0;
    if (n == 2 && ((axis == AXIS_X && !data.x[0]) ||
                   (axis == AXIS_Y && !data.y[0]))) return 0;
    if (!move_rot(c_pos)) return 0;
    m = (axis == AXIS_X) ? &motorX : &motorY;
    rpm = (axis == AXIS_X) ? HOME_X_RPM : -HOME_Y_RPM;
    if (!motor_speed_mode(m) || !motor_speed(m, rpm)) return 0;
    save_axis = axis;
    save_n = n;
    return 1;
}
static int save_end(void)
{
    Motor *m;
    int pos, step;
    int axis = save_axis;
    int n = save_n;
    if (!axis) return 0;
    m = (axis == AXIS_X) ? &motorX : &motorY;
    if (!motor_speed(m, 0)) goto bad;
    HAL_Delay(100);
    if (!motor_pos(m, &pos) && !motor_pos(m, &pos)) goto bad;
    if (axis == AXIS_X) {
        if (n == 1) {
            data.x[0] = pos;
            data.x[1] = data.x[2] = data.x[3] = 0;
        } else {
            step = pos - data.x[0];
            if (!step) goto bad;
            data.x[1] = pos;
            data.x[2] = data.x[0] + step * 2;
            data.x[3] = data.x[0] + step * 3;
        }
        data.last_x = pos;
    } else {
        if (n == 1) {
            data.y[0] = pos;
            data.y[1] = data.y[2] = data.y[3] = 0;
        } else {
            step = pos - data.y[0];
            if (!step) goto bad;
            data.y[1] = pos;
            data.y[2] = data.y[0] + step * 2;
            data.y[3] = data.y[0] + step * 3;
        }
        data.last_y = pos;
    }
    if (!motor_pos_mode(m)) goto bad;
    save_axis = save_n = 0;
    data_write();
    return 1;
bad:
    motor_speed(m, 0);
    motor_pos_mode(m);
    save_axis = save_n = 0;
    return 0;
}
int save_busy(void)
{
    return seq != S_IDLE;
}
static int poll50(void)
{
    uint32_t now = HAL_GetTick();
    if (now - poll_t0 < 50U) return 0;
    poll_t0 = now;
    return 1;
}
static int rot_done(int target)
{
    int now;

    if (!poll50()) return 0;
    if (!mks_pos(&now)) {
        rot_count = 0;
        return 0;
    }

    if (gap(now, target) <= ROT_GAP) rot_count++;
    else rot_count = 0;

    return rot_count >= ROT_STABLE;
}
static int xy_start(int xt, int yt)
{
    if (!motor_move(&motorX, X_RPM, xt)) {
        print("XY X START FAIL\r\n");
        return 0;
    }
    if (!motor_move(&motorY, Y_RPM, yt)) {
        print("XY Y START FAIL\r\n");
        motor_stop(&motorX);
        return 0;
    }
    x_count = y_count = 0;
    xy_log_t0 = 0;
    return 1;
}
static int xy_done(int xt, int yt)
{
    int xok, yok;
    uint32_t now;
    if (!poll50()) return 0;
    now = HAL_GetTick();
    xok = motor_pos(&motorX, &x_now) || motor_pos(&motorX, &x_now);
    yok = motor_pos(&motorY, &y_now) || motor_pos(&motorY, &y_now);
    x_count = (xok && gap(x_now, xt) <= XY_GAP) ? x_count + 1 : 0;
    y_count = (yok && gap(y_now, yt) <= XY_GAP) ? y_count + 1 : 0;
    if (now - xy_log_t0 >= 1000U) {
        char b[100];
        xy_log_t0 = now;
        snprintf(b, sizeof(b), "XY x=%d/%d y=%d/%d\r\n",
                 x_now, xt, y_now, yt);
        print(b);
    }
    return x_count >= XY_STABLE && y_count >= XY_STABLE;
}
static int all_saved(void)
{
    /* RUN에서 사용하는 위치까지만 검사한다. */
    for (int i = 0; i < RUN_N; i++) {
        if (!data.x[i] || !data.y[i]) return 0;
    }
    return 1;
}
static void seq_reset(void)
{
    seq = S_IDLE;
    run_on = 0;
    run_col = run_dan = 1;
    side = 1;
    stop_req = 0;
}
static void seq_fail(const char *msg)
{
    print(msg);
    mks_stop();
    motor_stop(&motorX);
    motor_stop(&motorY);
    seq_reset();
}
static void next_xy(void)
{
    /* 이동 완료 후 다음 (n,n)으로 넘긴다. */
    run_col++;
    if (run_col > RUN_N) run_col = 1;
    run_dan = run_col;
}
static int seq_start(int col, int dan, int repeat)
{
    if (save_busy() || save_axis || !rot_homed || !x_homed || !y_homed) return 0;
    if (col < 1 || col > RUN_N || dan < 1 || dan > RUN_N) return 0;
    if (!data.x[col - 1] || !data.y[dan - 1]) return 0;
    if (repeat && !all_saved()) return 0;
    run_col = col;
    run_dan = dan;
    run_on = repeat;
    stop_req = 0;
    side = 1;
    seq = S_SIDE_START;
    return 1;
}
int save_go(int col, int dan)
{
    return seq_start(col, dan, 0);
}
int save_auto(void)
{
    return seq_start(1, 1, 1);
}
void save_stop(void)
{
    /* Stop immediately instead of waiting for the current XY move. */
    if (!save_busy()) return;

    stop_req = 1;
    mks_stop();
    motor_stop(&motorX);
    motor_stop(&motorY);
    seq_reset();
}
static void seq_run(void)
{
    uint32_t now = HAL_GetTick();
    int xt = data.x[run_col - 1];
    int yt = data.y[run_dan - 1];
    int target = (side > 0) ? r_pos : l_pos;
    switch (seq) {
    case S_IDLE:
        return;
    case S_SIDE_START:
        if (!mks_move_abs(ROT_RPM, target)) {
            seq_fail("ROT SIDE START FAIL\r\n");
            return;
        }
        move_t0 = now;
        poll_t0 = 0;
        rot_count = 0;
        seq = S_SIDE_WAIT;
        return;
    case S_SIDE_WAIT:
        if (rot_done(target)) {
            print(side > 0 ? "ROT R OK\r\n" : "ROT L OK\r\n");
            seq = S_C_START;
        }
        else if (now - move_t0 >= TIMEOUT) seq_fail("ROT SIDE TIMEOUT\r\n");
        return;
    case S_C_START:
        if (!mks_move_abs(ROT_RPM, c_pos)) {
            seq_fail("ROT C START FAIL\r\n");
            return;
        }
        move_t0 = now;
        poll_t0 = 0;
        rot_count = 0;
        seq = S_C_WAIT;
        return;
    case S_C_WAIT:
        if (rot_done(c_pos)) {
            print("ROT C OK\r\n");
            seq = S_XY_START;
        }
        else if (now - move_t0 >= TIMEOUT) seq_fail("ROT C TIMEOUT\r\n");
        return;
    case S_XY_START:
        if (!xy_start(xt, yt)) {
            seq_fail("XY START FAIL\r\n");
            return;
        }
        move_t0 = now;
        poll_t0 = 0;
        seq = S_XY_WAIT;
        return;
    case S_XY_WAIT:
        if (xy_done(xt, yt)) {
            motor_stop(&motorX);
            motor_stop(&motorY);
            data.last_x = x_now;
            data.last_y = y_now;
            data_write();
            if (!run_on || stop_req) {
                seq_reset();
                return;
            }
            next_xy();
            side = -side;
            seq = S_SIDE_START;
        } else if (now - move_t0 >= TIMEOUT) {
            seq_fail("XY TIMEOUT\r\n");
        }
        return;
    }
}
static void reply(const char *s)
{
    tcp_reply(s);
}
static void trim(char *s)
{
    size_t n = strlen(s);
    while (n && (s[n - 1] == '\r' || s[n - 1] == '\n' ||
                 s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = 0;
}
static void show(void)
{
    char b[160];
    snprintf(b, sizeof(b),
             "x %dmm %dmm %dmm %dmm\r\n"
             "y %dmm %dmm %dmm %dmm\r\n",
             MM(data.x[0]), MM(data.x[1]), MM(data.x[2]), MM(data.x[3]),
             MM(data.y[0]), MM(data.y[1]), MM(data.y[2]), MM(data.y[3]));
    reply(b);
}
static void status(void)
{
    char b[220];
    int rot = 0, x = 0, y = 0;
    mks_pos(&rot);
    motor_pos(&motorX, &x);
    motor_pos(&motorY, &y);
    snprintf(b, sizeof(b),
             "rot_home=%d x_home=%d y_home=%d c=%d r=%d l=%d "
             "rot=%d x=%d y=%d step=%d,%d side=%c busy=%d stop=%d\r\n",
             rot_homed, x_homed, y_homed, c_pos, r_pos, l_pos,
             rot, x, y, run_col, run_dan, side > 0 ? 'R' : 'L',
             save_busy(), stop_req);
    reply(b);
}
static int rot_cmd(int target)
{
    return rot_homed && !save_axis && !save_busy() && move_rot(target);
}
static void command(char *cmd)
{
    int a, b;
    trim(cmd);
    if (save_busy() && strcmp(cmd, "stop") && strcmp(cmd, "status") &&
        strcmp(cmd, "show")) {
        reply("ERR RUNNING\r\n");
        return;
    }
    if (!strcmp(cmd, "status")) { status(); return; }
    if (!strcmp(cmd, "show"))   { show(); return; }
    if (!strcmp(cmd, "c")) { reply(rot_cmd(c_pos) ? "OK C\r\n" : "ERR C\r\n"); return; }
    if (!strcmp(cmd, "r")) { reply(rot_cmd(r_pos) ? "OK R\r\n" : "ERR R\r\n"); return; }
    if (!strcmp(cmd, "l")) { reply(rot_cmd(l_pos) ? "OK L\r\n" : "ERR L\r\n"); return; }
    if (!strcmp(cmd, "home")) { reply(save_home() ? "OK HOME\r\n" : "ERR HOME\r\n"); return; }
    if ((sscanf(cmd, "x%d save", &a) == 1 || sscanf(cmd, "x%dsave", &a) == 1) &&
        a >= 1 && a <= 2) {
        reply(save_start(AXIS_X, a) ? "OK, PRESS S\r\n" : "ERR X SAVE\r\n");
        return;
    }
    if ((sscanf(cmd, "y%d save", &a) == 1 || sscanf(cmd, "y%dsave", &a) == 1) &&
        a >= 1 && a <= 2) {
        reply(save_start(AXIS_Y, a) ? "OK, PRESS S\r\n" : "ERR Y SAVE\r\n");
        return;
    }
    if (!strcmp(cmd, "s")) { reply(save_end() ? "OK SAVE\r\n" : "ERR SAVE\r\n"); return; }
    if (sscanf(cmd, "go %d %d", &a, &b) == 2 ||
        sscanf(cmd, "go%d %d", &a, &b) == 2) {
        reply(save_go(a, b) ? "OK GO\r\n" : "ERR GO\r\n");
        return;
    }
    if (sscanf(cmd, "go %d", &a) == 1 || sscanf(cmd, "go%d", &a) == 1) {
        reply(save_go(a, a) ? "OK GO\r\n" : "ERR GO\r\n");
        return;
    }
    if (!strcmp(cmd, "run")) { reply(save_auto() ? "OK RUN\r\n" : "ERR RUN\r\n"); return; }
    if (!strcmp(cmd, "stop")) {
        save_stop();
        reply("OK STOP\r\n");
        return;
    }
    reply("ERR CMD\r\n");
}
void save_cmd(char *cmd)
{
    command(cmd);
}
void save_init(void)
{
    int mok, xok = 0, yok = 0;
    HAL_Delay(1000);
    tilt_read();
    data_read();
    mok = mks_init();
    print(mok ? "MKS INIT OK\r\n" : "MKS INIT FAIL\r\n");
    if (!mok) return;
    /* One initialization attempt per axis is enough after communication is stable. */
    yok = motor_init(&motorY, 1);
    print(yok ? "Y INIT OK\r\n" : "Y INIT FAIL\r\n");

    xok = motor_init(&motorX, 0);
    print(xok ? "X INIT OK\r\n" : "X INIT FAIL\r\n");
    if (!rot_init_home()) {
        print("ROT INIT HOME FAIL\r\n");
        return;
    }
    print("ROT HOME OK\r\n");
    if (!move_rot(c_pos)) {
        print("ROT C FAIL\r\n");
        return;
    }
    print("ROT C OK\r\n");
    x_homed = y_homed = 0;
    print((xok && yok) ? "INIT OK\r\n" : "ROT OK, XY INIT FAIL\r\n");
}
void save_run(void)
{
    if (save_busy()) seq_run();
}
