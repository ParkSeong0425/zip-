/* net.c: NET_Task는 TCP, MOTOR_Task는 모터 명령만 담당한다. */
#include "net.h"
#include "rfid.h"
#include "cli.h"
#include "rot_test.h"
#include "motor.h"
#include "button.h"
#include "fram.h"
#include "cfg.h"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "socket.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
extern osMessageQueueId_t TCPQueueHandle;
extern volatile int card_ok;

int mks_link(void);
void pause_full(int mask);
int button_stop_requested(void);
void lamp_cmd(char *s);
void lamp_run(char state, int card);

#define NET_SPI       hspi1
#define SOCK          0
#define POSITION_GAP  50    /* 목표 위치와 현재 위치의 허용 차이 */
#define REV_PULSE     1000  /* X/Y 모터 한 바퀴 위치값 */
#define REV_MM        160   /* 풀리 한 바퀴 이동 거리 */
#define XY_GEAR       5     /* X/Y 감속비 */
#define MAX_RPM       3000
#define ROT_MAX_RPM   3000  /* 틸트 속도 기준. 높이면 빨라진다 */
#define HOME_WAIT     30000 /* 홈 최대 대기 시간 */
static char repeat_message[64];
static volatile char status = 'W';
static volatile int alarm;
static uint32_t repeat_time;
static uint32_t repeat_number;
static volatile uint32_t pause_time;
static volatile char repeat_mode;
static volatile char pause_repeat;
static volatile uint32_t command_number;
static int rack_now = 1;   /* 마지막 명령의 렉 번호 */
static char motor_line[64];        /* MOTOR_Task 가 실행할 명령 */
static volatile int motor_ready;   /* 대기 중인 명령이 있으면 1 */
static volatile int motor_busy;    /* 명령을 실행하는 중이면 1 */

static void Check(void);
static void motor_check(void);

int alarm_get(void)
{
    return status == 'A';
}

void alarm_set(int code)
{
    status = code ? 'A' : 'W';
    alarm = code;
}

void send_to_tcp_queue(const char *response)
{
    uint8_t byte = STX;

    if (osMessageQueuePut(TCPQueueHandle, &byte, 0, 0) != osOK)
        return;

    while (*response && *response != '\r' && *response != '\n') {
        byte = (uint8_t)*response++;
        if (osMessageQueuePut(TCPQueueHandle, &byte, 0, 0) != osOK)
            return;
    }

    byte = ETX;
    osMessageQueuePut(TCPQueueHandle, &byte, 0, 0);
}

static void repeat_ready(const char *message)
{
    repeat_mode = 0;
    snprintf(repeat_message, sizeof(repeat_message), "%s", message);
    repeat_number = command_number;
}

static void repeat_start(void)
{
    if (repeat_number != command_number)
        return;

    send_to_tcp_queue(repeat_message);
    repeat_time = HAL_GetTick();
    repeat_mode = 'M';
}

static void repeat_set(const char *message)
{
    repeat_ready(message);
    repeat_start();
}

/* PAUSE가 있으면 PAUSE를 우선하고, 없으면 완료 메시지를 반복한다 */
static void repeat_run(void)
{
    uint32_t now = HAL_GetTick();

    /* 알람일 때만 상태 확인 응답을 1초마다 자동으로 보낸다 */
    if (status == 'A') {
        if (now - repeat_time >= 1000) {
            Check();
            repeat_time = now;
        }
        return;
    }

    if (pause_repeat) {
        if (now - pause_time >= 1000)
            pause_msg(pause_repeat);
        return;
    }

    if (!repeat_mode || now - repeat_time < 1000)
        return;

    send_to_tcp_queue(repeat_message);
    repeat_time = now;
}

static void tcp_queue_run(void)
{
    uint8_t frame[64];
    uint32_t length = 0;

    while (length < sizeof(frame) && osMessageQueueGet(
            TCPQueueHandle, &frame[length], NULL, 0) == osOK)
        length++;

    if (!length)
        return;

    HAL_UART_Transmit(&huart3, frame, length, 100);

    if (getSn_SR(SOCK) == SOCK_ESTABLISHED)
        send(SOCK, frame, length);
}

void print(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)s, strlen(s), 100);
}

void reply(const char *s)
{
    send_to_tcp_queue(s);
}

void pause_msg(char why)
{
    char message[16];

    if (!why) {
        pause_repeat = 0;
        return;
    }

    snprintf(message, sizeof(message), "01P_PAUSE:%c", why);
    send_to_tcp_queue(message);
    pause_repeat = why;
    pause_time = HAL_GetTick();
}


/* PAUSE 중에는 현재 명령을 보관하고 RUN 또는 자동 해제를 기다린다 */
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

static void fail(int code) { status = 'A'; alarm = code; repeat_mode = 0; }
static void fail_rot(void) { fail(mks_link() ? 7 : 4); }

/* 100ms마다 X/Y 위치와 ROT 상태·위치를 확인한다 */
static void motor_check(void)
{
    static int count;
    int pos, state, code = 0;

    if (++count < 10)
        return;

    count = 0;

    if (!motor_pos(&motorX, &pos))
        code = 2;

    if (!motor_pos(&motorY, &pos))
        code = 3;

    if (!mks_read(2, CMD_STATE, 1, &state))
        code = 4;
    else if (state == ST_FAIL)
        code = 7;

    if (!mks_read(2, CMD_AXIS, 6, &pos))
        code = 4;

    if (code && (status != 'A' || alarm != code))
        fail(code);
}

/* X/Y를 목표 위치까지 이동한다 */
static int move_xy(int x, int y, int rpm)
{
    int x_now, y_now;

    if (!wait_pause()) return 0;
    if (!motor_move(&motorX, rpm, x)) { fail(2); return 0; }
    if (!motor_move(&motorY, rpm, y)) {
        motor_stop(&motorX); fail(3); return 0;
    }

    for (;;) {
        motor_check();
        if (button_stop_requested()) return 0;
        if (!motor_pos(&motorX, &x_now)) { fail(2); return 0; }
        if (!motor_pos(&motorY, &y_now)) { fail(3); return 0; }

        /* 현재 위치와 목표 위치 차이가 허용 범위 안이면 도착 */
        if (x_now - x >= -POSITION_GAP
                && x_now - x <= POSITION_GAP
                && y_now + y >= -POSITION_GAP
                && y_now + y <= POSITION_GAP)
            return 1;

        osDelay(10);
    }
}

/* 회전축을 목표로 보낸다. 정지되면 세우고 풀린 뒤 다시 보낸다.
   dir 은 c 중앙, l 왼쪽, r 오른쪽 */
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
            if (button_stop_requested()) return 0;
            if (pause || !card_ok) { mks_stop(); break; }
            done = mks_done(target);
            if (done > 0) return 1;
            if (done < 0) { fail(done == -1 ? 4 : 7); return 0; }
            osDelay(10);
        }
    }
}

/* 입고: 중앙 -> X/Y 이동 -> 정지 확인 -> 왼쪽 틸트 */
static void MI(int x, int y, int xy_rpm, int rot_rpm, int angle, int move_delay)
{
    status = 'R'; alarm = 0;

    if (!move_rot('c', rot_rpm, 0)) return;
    if (!move_xy(x, y, xy_rpm)) return;
    if (!wait_pause()) return;

    osDelay(move_delay);
    if (!move_rot('l', rot_rpm, angle)) return;

    status = 'W';
}

/* 출고: 중앙 -> X/Y 이동 -> 정지 확인 */
static void MO(int x, int y, int xy_rpm, int rot_rpm)
{
    status = 'R'; alarm = 0;

    if (!move_rot('c', rot_rpm, 0)) return;
    if (!move_xy(x, y, xy_rpm)) return;
    if (!wait_pause()) return;

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

    if (full_read() & mask)
        pause_full(mask);

    if (!move_rot('r', rot_rpm, angle)) return;

    osDelay(return_delay);
    if (!move_rot('c', rot_rpm, 0)) return;

    status = 'W';
}

/* 원점복귀: 회전축 -> X/Y 동시 */
static void I(void) {
    int x, y, done; uint32_t start;
    status = 'I'; alarm = 0;
    if (!mks_home()) {
        fail(mks_link() ? 10 : 4); return;
    }

    osDelay(1000);

    for (;;) {
        motor_check();
        if (button_stop_requested()) return;

        if (pause) {
            mks_stop();
            if (!wait_pause()) return;
            if (!mks_home()) { fail(mks_link() ? 10 : 4); return; }
            osDelay(1000);
            continue;
        }

        done = mks_done(0);
        if (done > 0) break;
        if (done < 0) { fail(done == -1 ? 4 : 10); return; }
        osDelay(10);
    }

    if (!mks_zero()) {
        fail(mks_link() ? 10 : 4); return;
    }

    /* X와 Y 를 이어서 보내고 둘 다 같이 기다린다 */
    if (!motor_home_on(&motorX)) {
        fail(2); return;
    }
    if (!motor_home_on(&motorY)) {
        fail(3); return;
    }

    osDelay(10);
    start = HAL_GetTick();
    for (;;) {
        motor_check();
        if (button_stop_requested()) return;

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

        if (!motor_pos(&motorX, &x)) {
            fail(2); return;
        }
        if (!motor_pos(&motorY, &y)) {
            fail(3); return;
        }
        if (abs(x) <= POSITION_GAP && abs(y) <= POSITION_GAP) break;
        if (HAL_GetTick() - start >= HOME_WAIT) {  /* 홈을 못 찾았다 */
            fail(abs(x) > POSITION_GAP ? 2 : 3); return;
        }
        osDelay(10);
    }
    if (!motor_zero(&motorX)) {
        fail(2); return;
    }
    if (!motor_zero(&motorY)) {
        fail(3); return;
    }

    /* 원점복귀 후 RFID가 없으면 인식될 때까지 기다린다 */
    if (!card_ok) {
        pause_on('S');
        if (!wait_pause()) return;
    }

    status = 'W';
}
/* 현재 운전 상태와 버튼·만재 입력을 표시한다. */
static void Check(void) {
    char message[48];
    int full = full_get();

    if (estop) {
        status = 'A';
        alarm = 9;
    }

    if (status == 'A')
        snprintf(message, sizeof(message), "S_1_A_%02d&F_%d_%d%d%d%d",
                alarm, rack_now, !!(full & 1), !!(full & 2),
                !!(full & 4), !!(full & 8));
    else
        snprintf(message, sizeof(message), "S_1_%c&F_%d_%d%d%d%d",
                pause ? 'P' : status, rack_now, !!(full & 1),
                !!(full & 2), !!(full & 4), !!(full & 8));

    ack(message);
}

static void Stop(void) {
    motor_stop(&motorX);
    motor_stop(&motorY);
    mks_stop();
    if (status != 'A') status = 'W';
    ack("S");
}

/* 정지 중이거나 명령을 도는 중에도 받는 명령 */
static int always_ok(const char *command)
{
    return !strcmp(command, "01C") || !strcmp(command, "01R")
            || !strcmp(command, "01S_1") || !strcmp(command, "01D_1")
            || !strncmp(command, "01LL_", 5);
}

void net_cmd(char *command) {
    if ((pause || motor_busy || motor_ready) && !always_ok(command)) {
        nak("busy");
        return;
    }

    if (!strcmp(command, "01C")) {
        Check();          /* 물어볼 때만 한 번 답한다 */
    } else if (!strcmp(command, "01D_1")) {
        if (pause) pause_full(0);  /* 만재를 풀면 2초 뒤 이어서 한다 */
        ack("D_1");
    } else if (!strncmp(command, "01LL_", 5)) {
        lamp_cmd(command);
    } else if (!strncmp(command, "01FP_", 5) || !strncmp(command, "01FI_", 5)
            || !strcmp(command, "01FD_1")) {
        cfg_cmd(command);
    } else if (!strcmp(command, "01R")) {
        Rfid_Request();   /* 조회는 반복을 건드리지 않는다 */
    } else {
    	repeat_mode = 0; command_number++;
    	strcpy(motor_line, command); motor_ready = 1;
    }
 }

static void motor_command(char *command) {
    char message[32]; uint32_t number = command_number;
    int rack, col, row, xy, rot, move_delay, return_delay;
    int x, y, x_mm, y_mm, left, right, unused;
    int xy_rpm, rot_rpm, mm, speed, angle;
    if (!strncmp(command, "01MI_", 5) || !strncmp(command, "01MO_", 5)
            || !strncmp(command, "01PO_", 5)) {
        if (!card_ok) { pause_on('S'); return; }
        if (sscanf(command + 5, "%d_%d_%d_%d_%d_%d_%d", &rack,
                &col, &row, &xy, &rot, &move_delay, &return_delay) != 7) {
            nak("bad_data"); return;
        }
        rack_now = rack;
        if (!rot_load(rack, &left, &right, &unused, &unused)) {
            nak("fram_data"); return;
        }
        if (!strncmp(command, "01MI_", 5)) {
            pos_load(rack, IN_X, col, &x_mm);
            pos_load(rack, IN_Y, row, &y_mm);
        } else {
            pos_load(rack, OUT_X, col, &x_mm);
            pos_load(rack, OUT_Y, row, &y_mm);
        }
        x = x_mm * REV_PULSE * XY_GEAR / REV_MM;
        y = y_mm * REV_PULSE * XY_GEAR / REV_MM;
        xy_rpm = xy * MAX_RPM / 100;
        rot_rpm = rot * ROT_MAX_RPM / 100;
        return_delay *= 10; /* 마지막 값은 10ms 단위 */
        snprintf(message, sizeof(message), "01%c%c%c_%d_%d_%03d", ACK,
                command[2], command[3], col, row, xy);
        reply(message);   /* 수신 확인은 한 번만 */
        if (!strncmp(command, "01MI_", 5)) {
            MI(x, y, xy_rpm, rot_rpm, 10, move_delay); if (status == 'A') return;
            snprintf(message, sizeof(message), "01AI_%d_%d_%d", rack, col, row);
        } else if (!strncmp(command, "01MO_", 5)) {
            MO(x, y, xy_rpm, rot_rpm); if (status == 'A') return;
            snprintf(message, sizeof(message), "01AO_%d_%d_%d", rack, col, row);
        } else {
            PO(x, y, xy_rpm, rot_rpm, right, move_delay, return_delay,
                    1 << (row - 1)); if (status == 'A') return;
            snprintf(message, sizeof(message), "01EO_%d_%d_%d", rack, col, row);
        }
        if (number == command_number && !motor_ready) repeat_set(message);
    } else if (sscanf(command, "x_%d_%d", &mm, &speed) == 2) {
        if (motor_move(&motorX, speed * MAX_RPM / 100,
                mm * REV_PULSE * XY_GEAR / REV_MM)) ack(command); else { status = 'A'; alarm = 2; }
    } else if (sscanf(command, "y_%d_%d", &mm, &speed) == 2) {
        if (motor_move(&motorY, speed * MAX_RPM / 100,
                mm * REV_PULSE * XY_GEAR / REV_MM)) ack(command); else { status = 'A'; alarm = 3; }
    } else if (sscanf(command, "r_%d_%d", &angle, &speed) == 2) {
        if (mks_r(speed * MAX_RPM / 100, angle)) ack(command); else { status = 'A'; alarm = mks_link() ? 7 : 4; }
    } else if (sscanf(command, "l_%d_%d", &angle, &speed) == 2) {
        if (mks_l(speed * MAX_RPM / 100, angle)) ack(command); else { status = 'A'; alarm = mks_link() ? 7 : 4; }
    } else if (sscanf(command, "c_%d", &speed) == 1) {
        if (mks_c(speed * MAX_RPM / 100)) ack(command); else { status = 'A'; alarm = mks_link() ? 7 : 4; }
    } else if (!strcmp(command, "01I")) { I(); if (status != 'A') ack("I"); }
    else if (!strcmp(command, "01S_1")) Stop();
    else nak("bad_cmd");
}
void MOTOR_TaskRun(void *argument) {
    char command[64];
    (void)argument;
    if (motor_init(&motorX)) print("x ready\r\n"); else { print("x init ERR\r\n"); status = 'A'; alarm = 2; }
    if (motor_init(&motorY)) print("y ready\r\n"); else { print("y init ERR\r\n"); status = 'A'; alarm = 3; }
    if (mks_init()) print("mks ready\r\n"); else { print("mks init ERR\r\n"); status = 'A'; alarm = mks_link() ? 7 : 4; }
    motor_estop(&motorX, estop); motor_estop(&motorY, estop); /* 전원 시 ESTOP 상태 반영 */
    for (;;) {
        if (motor_ready) {
            snprintf(command, sizeof(command), "%s", motor_line);
            motor_ready = 0; motor_busy = 1;
            motor_command(command);
            motor_busy = 0;
        }
        motor_check();
        osDelay(10);
    }
}
static void cs_on(void)  { HAL_GPIO_WritePin(W610_CS_GPIO_Port, W610_CS_Pin, GPIO_PIN_RESET); }
static void cs_off(void) { HAL_GPIO_WritePin(W610_CS_GPIO_Port, W610_CS_Pin, GPIO_PIN_SET); }
static uint8_t spi_rb(void) { uint8_t b; HAL_SPI_Receive(&NET_SPI, &b, 1, 100); return b; }
static void spi_wb(uint8_t b) { HAL_SPI_Transmit(&NET_SPI, &b, 1, 100); }
static void spi_rbuf(uint8_t *b, datasize_t n) { HAL_SPI_Receive(&NET_SPI, b, n, 1000); }
static void spi_wbuf(uint8_t *b, datasize_t n) { HAL_SPI_Transmit(&NET_SPI, b, n, 1000); }
/* int 한 개를 IP 네 칸으로 푸다 */
static void ip_put(uint8_t *out, int v) {
    out[0] = (v >> 24) & 255; out[1] = (v >> 16) & 255;
    out[2] = (v >> 8) & 255;  out[3] = v & 255;
}

/* W6100 에 들어 있는 값을 그대로 읽어서 보낸다 */
static void net_show(void)
{
    wiz_NetInfo now;
    char b[40];

    wizchip_getnetinfo(&now);

    snprintf(b, sizeof(b), "01FI_1_1_%d.%d.%d.%d",
            now.ip[0], now.ip[1], now.ip[2], now.ip[3]);
    reply(b); tcp_queue_run();

    snprintf(b, sizeof(b), "01FI_1_2_%d.%d.%d.%d",
            now.sn[0], now.sn[1], now.sn[2], now.sn[3]);
    reply(b); tcp_queue_run();

    snprintf(b, sizeof(b), "01FI_1_3_%d.%d.%d.%d",
            now.gw[0], now.gw[1], now.gw[2], now.gw[3]);
    reply(b); tcp_queue_run();

    snprintf(b, sizeof(b), "01FI_1_4_%d.%d.%d.%d",
            (cfg_net(4) >> 24) & 255,
            (cfg_net(4) >> 16) & 255,
            (cfg_net(4) >> 8) & 255,
            cfg_net(4) & 255);
    reply(b); tcp_queue_run();

    snprintf(b, sizeof(b), "01FI_1_5_%d", cfg_net(5));
    reply(b); tcp_queue_run();

    snprintf(b, sizeof(b), "01FI_1_6_%d", cfg_net(6));
    reply(b); tcp_queue_run();
}

static void net_init(void) {
    uint8_t size[8] = {2, 2, 2, 2, 2, 2, 2, 2};
    wiz_NetInfo net = {.ipmode = NETINFO_STATIC_V4};

    ip_put(net.ip, cfg_net(1));
    ip_put(net.sn, cfg_net(2));
    ip_put(net.gw, cfg_net(3));
    HAL_I2C_Mem_Read(&hi2c1, 0xA0, 0xFA, I2C_MEMADD_SIZE_8BIT, net.mac, 6, 100);
    HAL_GPIO_WritePin(W610_RST_GPIO_Port, W610_RST_Pin, GPIO_PIN_RESET); HAL_Delay(10);
    HAL_GPIO_WritePin(W610_RST_GPIO_Port, W610_RST_Pin, GPIO_PIN_SET); HAL_Delay(100);
    reg_wizchip_cs_cbfunc(cs_on, cs_off); reg_wizchip_spi_cbfunc(spi_rb, spi_wb, spi_rbuf, spi_wbuf);
    wizchip_init(size, size); NETUNLOCK(); wizchip_setnetinfo(&net);
    PHYUNLOCK(); setPHYCR0(PHYCR0_AUTO); setPHYCR1(PHYCR1_RST);
}
static void serve(void) {
    static char command[64]; static int length;
    uint8_t data[64]; int count, i;
    switch (getSn_SR(SOCK)) {
    case SOCK_CLOSED: socket(SOCK, Sn_MR_TCP4, cfg_net(5), 0); break;
    case SOCK_INIT: listen(SOCK); break;
    case SOCK_ESTABLISHED:
        /* TCP가 새로 연결된 순간 설정값을 한 번 보낸다 */
        if (getSn_IR(SOCK) & Sn_IR_CON) {
            setSn_IR(SOCK, Sn_IR_CON);
            net_show();
        }
        count = getSn_RX_RSR(SOCK);
        if (count > (int)sizeof(data))
            count = sizeof(data);
        if (!count)
            break;
        recv(SOCK, data, count);
        for (i = 0; i < count; i++) {
            if (data[i] == STX) {
                length = 0;
            } else if (data[i] == ETX) {
                command[length] = 0;
                net_cmd(command);
                length = 0;
            } else if (length < (int)sizeof(command) - 1) {
                command[length++] = (char)data[i];
            }
        }
        break;
    case SOCK_CLOSE_WAIT: disconnect(SOCK); length = 0; break;
    }
}
void TM_TaskRun(void *argument) {
    (void)argument;
    button_init();
    /* 버튼 없이 테스트 중이라 전원 시 원점복귀는 막아둔다 */
    if (!estop) net_cmd("01I");
    cfg_init(); net_init();
    print(fram_load() ? "fram ready\r\n" : "fram empty\r\n");
    cli_start();
    for (;;) {
        button_run();
    	lamp_run(status, card_ok);
    	serve(); repeat_run(); tcp_queue_run(); cli_poll(); osDelay(10);
    }
}
