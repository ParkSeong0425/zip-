/* net.c: NET_Task는 TCP, MOTOR_Task는 모터 명령만 담당한다. */
#include "net.h"
#include "move.h"
#include "rfid.h"
#include "cli.h"
#include "button.h"
#include "fram.h"
#include "cfg.h"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "socket.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

extern osMessageQueueId_t TCPQueueHandle;
extern osMutexId_t TCPMutexHandle;
extern volatile int card_ok; // rfid.c에 있는 변수
extern volatile int run; // 버튼.c에 있는 변수

void pause_full(int mask);
void lamp_cmd(char *s);
void lamp_run(char state, int card);

#define NET_SPI       hspi1
#define SOCK          0
static char repeat_message[64];
static uint32_t repeat_time;
static uint32_t repeat_number;
static volatile uint32_t pause_time;
static volatile char repeat_mode;
static volatile char pause_repeat;
volatile uint32_t command_number;
static char no[3] = "00";          /* 지금 받은 명령 번호 */
static char r_no[3] = "00";        /* RFID 요청 번호 */

static void Check(int event);

int alarm_get(void)
{
    return status == 'A' ? alarm : 0;
}

void alarm_set(int code)
{
    status = code ? 'A' : 'W';
    alarm = code;
    if (code) {
        repeat_mode = 0;
        Check(1);
        repeat_time = HAL_GetTick();
    }
    if (code == 9) {
        motor_ready = 0;
        repeat_mode = 0;
        command_number++;
    }
}

void send_to_tcp_queue(const char *response)
{
    uint8_t byte = STX;

    if (osMutexAcquire(TCPMutexHandle, osWaitForever) != osOK)
        return;

    if (osMessageQueueGetSpace(TCPQueueHandle) < strlen(response) + 2) {
        osMutexRelease(TCPMutexHandle);
        return;
    }

    osMessageQueuePut(TCPQueueHandle, &byte, 0, 0);

    while (*response && *response != '\r' && *response != '\n') {
        byte = (uint8_t)*response++;
        osMessageQueuePut(TCPQueueHandle, &byte, 0, 0);
    }

    byte = ETX;
    osMessageQueuePut(TCPQueueHandle, &byte, 0, 0);
    osMutexRelease(TCPMutexHandle);
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

    repeat_time = HAL_GetTick();
    repeat_mode = 'M';
    send_to_tcp_queue(repeat_message);
}

void repeat_set(const char *message)
{
    repeat_ready(message);
    repeat_start();
}

/* PAUSE가 있으면 PAUSE를 우선하고, 없으면 완료 메시지를 반복한다 */
static void repeat_run(void)
{
    uint32_t now = HAL_GetTick();

    /* ESTOP 순간에는 아직 알람 상태 반영 전이어도 완료 반복을 폐기한다 */
    if (estop)
        repeat_mode = 0;

    /* 알람일 때만 상태 확인 응답을 1초마다 자동으로 보낸다 */
    if (status == 'A') {
        if (now - repeat_time >= 1000) {
            Check(1);
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
    uint8_t frame[66];
    uint32_t length = 0;

    if (osMutexAcquire(TCPMutexHandle, 0) != osOK)
        return;

    while (length < sizeof(frame) && osMessageQueueGet(
            TCPQueueHandle, &frame[length], NULL, 0) == osOK) {
        length++;
        if (frame[length - 1] == ETX)
            break;
    }

    osMutexRelease(TCPMutexHandle);

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
    char b[64];

    if (s[0] == '0' && s[1] == '1') {
        snprintf(b, sizeof(b), "%.2s%s", s[2] == 'U' ? r_no : no, s + 2);
        send_to_tcp_queue(b);
    } else {
        send_to_tcp_queue(s);
    }
}

void pause_msg(char why)
{
    char message[16];

    if (!why) {
        pause_repeat = 0;
        return;
    }

    snprintf(message, sizeof(message), "22P_PAUSE:%c_%.2s", why,
            motor_line[0] ? motor_line : "00");
    send_to_tcp_queue(message);
    pause_repeat = why;
    pause_time = HAL_GetTick();
}


/* 현재 운전 상태와 버튼·만재 입력을 표시한다. */
static void Check(int event) {
    char message[48];
    int full = full_get();

    if (estop) {
        status = 'A';
        alarm = 9;
    }

    if (event) {
        if (estop || alarm == 9)
            snprintf(message, sizeof(message), "00ESTOP");
        else
            snprintf(message, sizeof(message), "44A_%02d", alarm);
        reply(message);
        return;
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

/* 정지 중이거나 명령을 도는 중에도 받는 명령 */
static int always_ok(const char *command)
{
    return !strcmp(command, "C") || !strcmp(command, "R")
            || !strcmp(command, "S_1") || !strcmp(command, "D_1")
            || !strncmp(command, "LL_", 3);
}

void net_cmd(char *command) {
    char b[64], *cmd;

    if (command[0] < '0' || command[0] > '9'
            || command[1] < '0' || command[1] > '9' || !command[2]) {
        no[0] = '0'; no[1] = '0';
        nak("bad_number");
        return;
    }

    no[0] = command[0]; no[1] = command[1];
    cmd = command + 2;

    /* 00=내부 원점복귀, 22=PAUSE, 44=알람 전용 */
    if ((!strcmp(no, "00") && strcmp(cmd, "I")) || !strcmp(no, "44")
            || (!strcmp(no, "22") && strcmp(cmd, "S_1"))
            || (!strcmp(cmd, "S_1") && strcmp(no, "22"))) {
        nak("bad_number");
        return;
    }

    /* 네트워크 설정은 알람이나 원점복귀 중에도 FRAM에 저장한다 */
    if (!strncmp(cmd, "FI_", 3) || !strcmp(cmd, "FD_1")) {
        snprintf(b, sizeof(b), "01%s", cmd);
        cfg_cmd(b);
        return;
    }

    /* AI/AO/EO 완료 메시지와 번호·내용이 모두 같은 승인만 처리한다 */
    if (cmd[0] == '-') {
        if (repeat_mode == 'M'
                && command[0] == repeat_message[0]
                && command[1] == repeat_message[1]
                && !strcmp(cmd + 1, repeat_message + 2)) {
            repeat_mode = 0;
            repeat_message[0] = 0;

            /* MO의 AO 승인은 반복만 끝내고 R 상태를 유지한다 */
            status = !strncmp(cmd + 1, "AO_", 3) ? 'R' : 'W';
            return;
        }

        nak("can_not_command");
        return;
    }

    /* ESTOP 중에는 모든 외부 명령을 막는다 */
    if (estop) {
        nak("can_not_command");
        return;
    }

    /* 내부 원점복귀: PG1 또는 전원 시작에서 운전 허가한 경우만 예약한다 */
    if (!strcmp(no, "00") && !strcmp(cmd, "I")) {
        if (!run || pause) {
            nak("can_not_command");
            return;
        }
        repeat_mode = 0; command_number++;
        strcpy(motor_line, command); motor_ready = 1;
        return;
    }

    /* 알람 중에도 상태·RFID·램프 명령은 계속 처리한다 */
    if (((!run && !pause) || alarm_get() || ((motor_ready || motor_busy)
    		&& !strcmp(motor_line + 2, "I")))
    		&& !always_ok(cmd)) {
    	nak("can_not_command");
    	return;
    }

    if ((!run || pause || motor_busy || motor_ready || repeat_mode == 'M')
            && !always_ok(cmd)) {
        nak("can_not_command");
        return;
    }

    if (!strcmp(cmd, "C")) {
        Check(0);          /* 물어볼 때만 한 번 답한다 */
    } else if (!strcmp(cmd, "S_1")) {
        if (!estop && !alarm_get() && !pause) {
            pause_on('M'); ack("S");
        }
    } else if (!strcmp(cmd, "D_1")) {
        if (pause) pause_full(0);  /* 만재를 풀면 2초 뒤 이어서 한다 */
        ack("D_1");
    } else if (!strncmp(cmd, "LL_", 3)) {
        snprintf(b, sizeof(b), "01%s", cmd); lamp_cmd(b);
    } else if (!strncmp(cmd, "FP_", 3)) {
        snprintf(b, sizeof(b), "01%s", cmd); cfg_cmd(b);
    } else if (!strcmp(cmd, "R")) {
        r_no[0] = no[0]; r_no[1] = no[1];
        Rfid_Request();   /* 조회는 반복을 건드리지 않는다 */
    } else {
    	repeat_mode = 0; command_number++;
    	strcpy(motor_line, command); motor_ready = 1;
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
            no[0] = '0'; no[1] = '0';
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

    cfg_init();
    net_init();

    print(fram_load() ? "fram ready\r\n" : "fram empty\r\n");
    cli_start();

    for (;;) {
        button_run();
        lamp_run(status, card_ok);
        serve();
        repeat_run();
        tcp_queue_run();
        cli_poll();
        osDelay(10);
    }
}
