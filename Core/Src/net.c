/* net.c: NET_Task는 TCP, MOTOR_Task는 모터 명령만 담당한다. */
#include "net.h"
#include "rfid.h"
#include "cli.h"
#include "rot_test.h"
#include "motor.h"
#include "button.h"
#include "fram.h"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "socket.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
extern osMessageQueueId_t MOTOR_QueueHandle; /* NET_Task -> MOTOR_Task */
extern osMessageQueueId_t TCPQueueHandle;   /* MOTOR_Task -> NET_Task */
#define NET_SPI       hspi1
#define SOCK          0
#define PORT          2500
#define POSITION_GAP  50    /* 목표 위치와 현재 위치의 허용 차이 */
#define REV_PULSE     1000  /* X/Y 모터 한 바퀴 위치값 */
#define REV_MM        160   /* 풀리 한 바퀴 이동 거리 */
#define XY_GEAR       5     /* X/Y 감속비 */
#define MAX_RPM       3000
static char repeat_message[64]; static uint32_t repeat_time, repeat_number; static volatile char repeat_mode; static volatile uint32_t command_number; static void C(void);
void save_to_motor_queue(const char *command) {
    uint8_t byte = STX;
    repeat_mode = 0; command_number++; /* 새 명령이면 이전 반복 중지 */
    osMessageQueuePut(MOTOR_QueueHandle, &byte, 0, osWaitForever);
    while (*command) {
        byte = (uint8_t)*command++;
        osMessageQueuePut(MOTOR_QueueHandle, &byte, 0, osWaitForever);
    }
    byte = ETX;
    osMessageQueuePut(MOTOR_QueueHandle, &byte, 0, osWaitForever);
}
void send_to_tcp_queue(const char *response) {
    uint8_t byte = STX;
    osMessageQueuePut(TCPQueueHandle, &byte, 0, osWaitForever);
    while (*response && *response != '\r' && *response != '\n') {
        byte = (uint8_t)*response++;
        osMessageQueuePut(TCPQueueHandle, &byte, 0, osWaitForever);
    }
    byte = ETX;
    osMessageQueuePut(TCPQueueHandle, &byte, 0, osWaitForever);
}
static void repeat_ready(const char *message) {
    repeat_mode = 0;
    snprintf(repeat_message, sizeof(repeat_message), "%s", message);
    repeat_number = command_number;
}
static void repeat_start(void) {
    if (repeat_number != command_number) return;
    send_to_tcp_queue(repeat_message);
    repeat_time = HAL_GetTick(); repeat_mode = 'M';
}
static void repeat_set(const char *message) { repeat_ready(message); repeat_start(); }
static void repeat_run(void) {
    uint32_t now = HAL_GetTick();
    if (!repeat_mode || now - repeat_time < 1000) return;
    if (repeat_mode == 'C') C();
    else send_to_tcp_queue(repeat_message);
    repeat_time = now;
}
static void tcp_queue_run(void) {
    uint8_t frame[64]; uint32_t length = 0;
    while (length < sizeof(frame) && osMessageQueueGet(
            TCPQueueHandle, &frame[length], NULL, 0) == osOK)
        length++;
    if (!length) return;
    HAL_UART_Transmit(&huart3, frame, length, 100);
    if (getSn_SR(SOCK) == SOCK_ESTABLISHED) send(SOCK, frame, length);
}
void print(const char *s) { HAL_UART_Transmit(&huart3, (uint8_t *)s, strlen(s), 100); }
void reply(const char *s) { send_to_tcp_queue(s); }
void pause_msg(char why) {
    char message[16]; snprintf(message, sizeof(message), "01PAUSE:%c", why); send_to_tcp_queue(message);
}
static void ack(const char *s) {
    char message[64]; snprintf(message, sizeof(message), "01%c%s", ACK, s); send_to_tcp_queue(message);
}
static void nak(const char *s) {
    char message[64]; snprintf(message, sizeof(message), "01%c%s", NAK, s); send_to_tcp_queue(message);
}
/* 입고: 중앙 -> X/Y 이동 -> 왼쪽 틸트 */
static void MI(int x, int y, int xy_rpm, int rot_rpm, int angle, int move_delay) {
    int x_now, y_now;
    if (!mks_c(rot_rpm)) return;
    repeat_start();
    while (!mks_done(0)) osDelay(10);
    motor_move(&motorX, xy_rpm, x);
    motor_move(&motorY, xy_rpm, y);
    motor_pos(&motorX, &x_now);
    motor_pos(&motorY, &y_now);
    while (abs(x_now - x) > POSITION_GAP
            || abs(y_now + y) > POSITION_GAP) {
        osDelay(10);
        motor_pos(&motorX, &x_now);
        motor_pos(&motorY, &y_now);
    }
    osDelay(move_delay);
    mks_l(rot_rpm, angle);
    while (!mks_done(angle * MKS_REV * ROT_GEAR / 360)) osDelay(10);
}
/* 출고: 중앙 -> X/Y 이동, 틸트 없음 */
static void MO(int x, int y, int xy_rpm, int rot_rpm) {
    int x_now, y_now;
    if (!mks_c(rot_rpm)) return;
    repeat_start();
    while (!mks_done(0)) osDelay(10);
    motor_move(&motorX, xy_rpm, x);
    motor_move(&motorY, xy_rpm, y);
    motor_pos(&motorX, &x_now);
    motor_pos(&motorY, &y_now);
    while (abs(x_now - x) > POSITION_GAP
            || abs(y_now + y) > POSITION_GAP) {
        osDelay(10);
        motor_pos(&motorX, &x_now);
        motor_pos(&motorY, &y_now);
    }
}
/* 분배: 중앙 -> X/Y 이동 -> 오른쪽 틸트 -> 중앙 */
static void PO(int x, int y, int xy_rpm, int rot_rpm, int angle, int move_delay, int return_delay) {
    int x_now, y_now;
    if (!mks_c(rot_rpm)) return;
    repeat_start();
    while (!mks_done(0)) osDelay(10);
    motor_move(&motorX, xy_rpm, x);
    motor_move(&motorY, xy_rpm, y);
    motor_pos(&motorX, &x_now);
    motor_pos(&motorY, &y_now);
    while (abs(x_now - x) > POSITION_GAP
            || abs(y_now + y) > POSITION_GAP) {
        osDelay(10);
        motor_pos(&motorX, &x_now);
        motor_pos(&motorY, &y_now);
    }
    osDelay(move_delay);
    mks_r(rot_rpm, angle);
    while (!mks_done(-angle * MKS_REV * ROT_GEAR / 360)) osDelay(10);
    osDelay(return_delay);
    mks_c(rot_rpm);
    while (!mks_done(0)) osDelay(10);
}
/* 원점복귀: 회전축 -> Y축 -> X축 */
static void I(void) {
    int position;
    mks_home();
    osDelay(1000);
    while (!mks_done(0)) osDelay(10);
    mks_zero();
    motor_home_on(&motorY);
    osDelay(1000);
    motor_pos(&motorY, &position);
    while (abs(position) > POSITION_GAP) {
        osDelay(10); motor_pos(&motorY, &position);
    }
    motor_zero(&motorY);
    motor_home_on(&motorX);
    osDelay(1000);
    motor_pos(&motorX, &position);
    while (abs(position) > POSITION_GAP) {
        osDelay(10); motor_pos(&motorX, &position);
    }
    motor_zero(&motorX);
}
/* 상태는 별도 상태머신 없이 버튼과 만재 입력만 표시한다. */
static void C(void) {
    char message[32]; int full = full_get();
    char state = estop || pause ? 'P' : 'W';
    if (full) snprintf(message, sizeof(message), "S_1_%c&F%d%d%d%d", state, !!(full & 1), !!(full & 2), !!(full & 4), !!(full & 8));
    else snprintf(message, sizeof(message), "S_1_%c&S", state);
    ack(message);
}
static void S(void) { motor_stop(&motorX); motor_stop(&motorY); mks_stop(); ack("S"); }
void net_cmd(char *command) {
    if (!strcmp(command, "02C")) {
        repeat_mode = 0; command_number++; C(); repeat_time = HAL_GetTick(); repeat_mode = 'C';
    } else if (!strcmp(command, "02R")) { repeat_mode = 0; command_number++; Rfid_Request(); }
    else save_to_motor_queue(command);
}
static void motor_command(char *command) {
    char message[32]; uint32_t number = command_number;
    int rack, col, row, xy, rot, move_delay, return_delay;
    int x, y, x_mm, y_mm, left, right, unused;
    int xy_rpm, rot_rpm, mm, speed, angle;
    if (!strncmp(command, "01MI_", 5) || !strncmp(command, "01MO_", 5)
            || !strncmp(command, "01PO_", 5)) {
        if (sscanf(command + 5, "%d_%d_%d_%d_%d_%d_%d", &rack,
                &col, &row, &xy, &rot, &move_delay, &return_delay) != 7) {
            nak("bad_data"); return;
        }
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
        rot_rpm = rot * MAX_RPM / 100;
        return_delay *= 10; /* 마지막 값은 10ms 단위 */
        snprintf(message, sizeof(message), "01%c%c%c_%d_%d_%03d", ACK, command[2], command[3], col, row, xy);
        repeat_ready(message);
        if (!strncmp(command, "01MI_", 5)) {
            MI(x, y, xy_rpm, rot_rpm, 10, move_delay);
            snprintf(message, sizeof(message), "01AI_%d_%d", col, row);
            if (number == command_number && !osMessageQueueGetCount(MOTOR_QueueHandle)) repeat_set(message);
        } else if (!strncmp(command, "01MO_", 5))
            MO(x, y, xy_rpm, rot_rpm);
        else {
            PO(x, y, xy_rpm, rot_rpm, right, move_delay, return_delay);
            snprintf(message, sizeof(message), "01EO_%d_%d", col, row);
            if (number == command_number && !osMessageQueueGetCount(MOTOR_QueueHandle)) repeat_set(message);
        }
    } else if (sscanf(command, "x_%d_%d", &mm, &speed) == 2) {
        motor_move(&motorX, speed * MAX_RPM / 100,
                mm * REV_PULSE * XY_GEAR / REV_MM); ack(command);
    } else if (sscanf(command, "y_%d_%d", &mm, &speed) == 2) {
        motor_move(&motorY, speed * MAX_RPM / 100,
                mm * REV_PULSE * XY_GEAR / REV_MM); ack(command);
    } else if (sscanf(command, "r_%d_%d", &angle, &speed) == 2) {
        mks_r(speed * MAX_RPM / 100, angle); ack(command);
    } else if (sscanf(command, "l_%d_%d", &angle, &speed) == 2) {
        mks_l(speed * MAX_RPM / 100, angle); ack(command);
    } else if (sscanf(command, "c_%d", &speed) == 1) {
        mks_c(speed * MAX_RPM / 100); ack(command);
    } else if (!strcmp(command, "01I")) { I(); ack("I"); }
    else if (!strcmp(command, "01S_1")) S();
    else nak("bad_cmd");
}
void MOTOR_TaskRun(void *argument) {
    char command[64]; uint8_t byte; int length = 0;
    (void)argument;
    print(motor_init(&motorX) ? "x ready\r\n" : "x init ERR\r\n");
    print(motor_init(&motorY) ? "y ready\r\n" : "y init ERR\r\n");
    print(mks_init() ? "mks ready\r\n" : "mks init ERR\r\n");
    for (;;) {
        osMessageQueueGet(MOTOR_QueueHandle, &byte, NULL, osWaitForever);
        if (byte == STX) length = 0;
        else if (byte == ETX) {
            command[length] = 0;
            motor_command(command);
            length = 0;
        } else if (length < (int)sizeof(command) - 1)
            command[length++] = (char)byte;
    }
}
static void cs_on(void)  { HAL_GPIO_WritePin(W610_CS_GPIO_Port, W610_CS_Pin, GPIO_PIN_RESET); }
static void cs_off(void) { HAL_GPIO_WritePin(W610_CS_GPIO_Port, W610_CS_Pin, GPIO_PIN_SET); }
static uint8_t spi_rb(void) { uint8_t b; HAL_SPI_Receive(&NET_SPI, &b, 1, 100); return b; }
static void spi_wb(uint8_t b) { HAL_SPI_Transmit(&NET_SPI, &b, 1, 100); }
static void spi_rbuf(uint8_t *b, datasize_t n) { HAL_SPI_Receive(&NET_SPI, b, n, 1000); }
static void spi_wbuf(uint8_t *b, datasize_t n) { HAL_SPI_Transmit(&NET_SPI, b, n, 1000); }
static void net_init(void) {
    uint8_t size[8] = {2, 2, 2, 2, 2, 2, 2, 2};
    wiz_NetInfo net = {.ip = {172, 20, 0, 101}, .sn = {255, 255, 255, 0},
            .gw = {0, 0, 0, 0}, .ipmode = NETINFO_STATIC_V4};
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
    case SOCK_CLOSED: socket(SOCK, Sn_MR_TCP4, PORT, 0); break;
    case SOCK_INIT: listen(SOCK); break;
    case SOCK_ESTABLISHED:
        count = getSn_RX_RSR(SOCK); if (count > (int)sizeof(data)) count = sizeof(data);
        if (!count) break;
        recv(SOCK, data, count);
        for (i = 0; i < count; i++) {
            if (data[i] == STX) length = 0;
            else if (data[i] == ETX) {
                command[length] = 0; net_cmd(command); length = 0;
            } else if (length < (int)sizeof(command) - 1)
                command[length++] = (char)data[i];
        }
        break;
    case SOCK_CLOSE_WAIT: disconnect(SOCK); length = 0; break;
    }
}
void TM_TaskRun(void *argument) {
    (void)argument;
    button_init(); net_init(); print(fram_load() ? "fram ready\r\n" : "fram empty\r\n"); cli_start();
    for (;;) {
        button_run(); serve(); repeat_run(); tcp_queue_run(); cli_poll(); osDelay(10);
    }
}
