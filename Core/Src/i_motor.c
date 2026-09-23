#include "i_motor.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <string.h>

/* 통신 */
#define RETRY        3
#define BUS_GAP      20
#define RX_TIMEOUT   500
#define IM_ID        3       /* DIP 스위치로 설정한 주소 */

/* 데이터시트 레지스터 */
#define R_DIP         0x0187   /* Pr4.35 DIP switches status */
#define R_STATE       0x1003   /* Motion state */
#define R_FB_POS_H    0x1014   /* Feedback position H */
#define R_FB_POS_L    0x1015   /* Feedback position L */
#define SET_ZERO      0x0021   /* Set Zero */
#define R_ALARM       0x2203   /* Current alarm */
#define R_CTRL        0x6002   /* Pr8.02 Trigger register */
#define R_HOME_MODE   0x600A   /* Pr8.10 Homing mode */
#define R_HOME_HI     0x600F   /* Pr8.15 Homing high velocity */
#define R_HOME_WARN   0x601D   /* Pr8.29 PR warning */
#define R_PR0         0x6200   /* Pr9.00 ~ Pr9.07 */
#define R_REAL_POS_H  0x602C   /* Pr8.44 Actual position H */

/* Mapping: 설정 0x0F10~, 데이터시트 예제 읽기 0x0F00~ */
#define R_MAP_DATA    0x0F00
#define R_MAP_SET     0x0F10
#define MAP_COUNT     5

/* Trigger 값 */
#define PR_POSITION   0x0001   /* TYPE=1 Position, Bit6=0 Absolute */
#define START_PR0     0x0010
#define START_HOME    0x0020
#define QUICK_STOP    0x0040

static uint8_t id = IM_ID;

/* Modbus RTU CRC16 */
static uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

/* UART2 수신 정리 */
static void bus_clear(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET) {
        volatile uint8_t d = (uint8_t)huart2.Instance->RDR;
        (void)d;
    }
}

/* UART2 Hardware RS485 송수신 */
static int bus_xfer(uint8_t *tx, uint16_t tn, uint8_t *rx, uint16_t rn)
{
    HAL_StatusTypeDef r;
    bus_clear();
    r = HAL_UART_Transmit(&huart2, tx, tn, 100);
    if (r == HAL_OK) r = HAL_UART_Receive(&huart2, rx, rn, RX_TIMEOUT);
    osDelay(BUS_GAP);
    return r == HAL_OK;
}

/* FC06 : 16비트 레지스터 1개 쓰기 */
static int write16(uint16_t reg, uint16_t val)
{
    uint8_t tx[8], rx[8];
    uint16_t crc;
    tx[0] = id; tx[1] = 0x06;
    tx[2] = reg >> 8; tx[3] = (uint8_t)reg;
    tx[4] = val >> 8; tx[5] = (uint8_t)val;
    crc = crc16(tx, 6);
    tx[6] = (uint8_t)crc; tx[7] = (uint8_t)(crc >> 8);
    for (uint8_t retry = 0; retry < RETRY; retry++) {
        if (bus_xfer(tx, 8, rx, 8) && memcmp(rx, tx, 8) == 0) return 1;
    }
    return 0;
}

/* FC03 : 연속 레지스터 읽기 */
static int read_reg(uint16_t reg, uint8_t qty, uint16_t *out)
{
    uint8_t tx[8], rx[32];
    uint16_t n = 5 + qty * 2, crc;
    tx[0] = id; tx[1] = 0x03;
    tx[2] = reg >> 8; tx[3] = (uint8_t)reg;
    tx[4] = 0; tx[5] = qty;
    crc = crc16(tx, 6);
    tx[6] = (uint8_t)crc; tx[7] = (uint8_t)(crc >> 8);

    for (uint8_t retry = 0; retry < RETRY; retry++) {
        if (!bus_xfer(tx, 8, rx, n)) continue;
        crc = crc16(rx, n - 2);
        if (rx[0] != id || rx[1] != 0x03 || rx[2] != qty * 2) continue;
        if (rx[n - 2] != (uint8_t)crc || rx[n - 1] != (uint8_t)(crc >> 8)) continue;
        for (uint8_t i = 0; i < qty; i++)
            out[i] = ((uint16_t)rx[3 + i * 2] << 8) | rx[4 + i * 2];
        return 1;
    }
    return 0;
}

/* FC10 : 연속 레지스터 여러 개 쓰기 */
static int write_multi(uint16_t reg, const uint16_t *data, uint8_t qty)
{
    uint8_t tx[32], rx[8];
    uint16_t n = 7 + qty * 2, crc;
    tx[0] = id; tx[1] = 0x10;
    tx[2] = reg >> 8; tx[3] = (uint8_t)reg;
    tx[4] = 0; tx[5] = qty; tx[6] = qty * 2;
    for (uint8_t i = 0; i < qty; i++) {
        tx[7 + i * 2] = data[i] >> 8;
        tx[8 + i * 2] = (uint8_t)data[i];
    }
    crc = crc16(tx, n);
    tx[n] = (uint8_t)crc; tx[n + 1] = (uint8_t)(crc >> 8);

    for (uint8_t retry = 0; retry < RETRY; retry++) {
        if (!bus_xfer(tx, n + 2, rx, 8)) continue;
        crc = crc16(rx, 6);
        if (rx[0] == id && rx[1] == 0x10 && rx[2] == tx[2] && rx[3] == tx[3]
                && rx[4] == 0 && rx[5] == qty && rx[6] == (uint8_t)crc
                && rx[7] == (uint8_t)(crc >> 8)) return 1;
    }
    return 0;
}

/* mm -> pulse */
static int32_t move_pos(int32_t mm)
{
    return (int32_t)((int64_t)mm * X_PULSE *X_GEAR / X_PI);
}
/* 32비트 값 읽기 : High + Low */
static int read32(uint16_t reg, int32_t *value)
{
    uint16_t data[2];

    if (!read_reg(reg, 2, data))
        return 0;

    *value = (int32_t)(((uint32_t)data[0] << 16) | data[1]);

    return 1;
}

/* 현재 위치 mm */
static int read_pos(int32_t *mm)
{
    int32_t pulse;

    if (!read32(R_REAL_POS_H, &pulse))
        return 0;

    *mm = (int32_t)((int64_t)pulse * X_PI / (X_PULSE * X_GEAR));

    return 1;
}

/* PR0 Immediate Trigger 전송 */
static int speed_rpm(int percent)
{
    return percent * IM_MAX_RPM / 100;
}

/* PR0 Immediate Trigger : 100% = 3000rpm */
static int write_move(uint16_t reg, int32_t pulse, int percent)
{
    uint32_t raw = (uint32_t)pulse;
    int rpm = speed_rpm(percent);
    uint16_t data[8] = {
        PR_POSITION, (uint16_t)(raw >> 16), (uint16_t)raw, (uint16_t)rpm,
        IM_ACC_MS, IM_DEC_MS, 0, START_PR0
    };
    return write_multi(reg, data, 8);
}

/* Mapping 설정 */
static int write_map(void)
{
    const uint16_t data[MAP_COUNT] = {
        R_STATE,
        R_FB_POS_H,
        R_FB_POS_L,
        R_ALARM,
        R_HOME_WARN
    };

    return write_multi(R_MAP_SET, data, MAP_COUNT);
}

static int read_map(IM_Info *info)
{
    uint16_t data[MAP_COUNT];

    if (!read_reg(R_MAP_DATA, MAP_COUNT, data))
        return 0;

    info->state = data[0];
    info->pos   = (int32_t)(((uint32_t)data[1] << 16) | data[2]);
    info->alarm = data[3];
    info->warn  = data[4];

    return 1;
}
/* 모터 기능 */
/* mm 단위 절대위치 이동 */
int IM_Move(int32_t mm, int percent){ return write_move(R_PR0, move_pos(mm), percent); }
int IM_Stop(void) { return write16(R_CTRL, QUICK_STOP); }
/* 현재 위치를 0으로 */
int IM_Zero(void)
{
    return write16(R_CTRL, SET_ZERO);
}

/* 토크 원점 -> 100mm 이동 -> 0점 */
int IM_Home(void)
{
    write16(R_HOME_MODE, IM_HOME_TORQUE | IM_HOME_DIR);

    write_multi(R_HOME_HI, IM_HOME_DATA, IM_HOME_COUNT);

    write16(R_CTRL, START_HOME);

    while ((IM_State() & IM_HOMED) == 0)
        osDelay(20);

    IM_Move(100, 10);

    while (IM_Done() == 0)
        osDelay(IM_ACC_MS);  // 이동 완료를 너무 일찍 하면  문제가 생길 수 있음.

    IM_Zero();

    return 1;
}

/* 상태 읽기 */
uint16_t  IM_State(void)
{
    uint16_t state = 0;
    read_reg(R_STATE, 1, &state);
    return state;
}

/* PR 이동 완료 */
int IM_Done(void)
{
    return (IM_State() & IM_PATH_DONE);
}

/* 상태 읽기 */
int IM_Pos(int32_t *mm){ return read_pos(mm); }
int IM_AlarmRead(uint16_t *alarm) { return read_reg(R_ALARM, 1, alarm); }

int IM_DIPRead(uint16_t *dip) { return read_reg(R_DIP, 1, dip); }

/* Register Mapping */
int IM_MapInit(void) { return write_map(); }
int IM_MapRead(IM_Info *info) { return read_map(info); }
int IM_Read(uint16_t reg, uint16_t *val) { return read_reg(reg, 1, val); }
