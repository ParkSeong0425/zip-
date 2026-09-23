/*
 * t_motor.c
 *
 *  Created on: Sep 11, 2026
 *      Author: HWNOT
 */
#include "t_motor.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <string.h>

/* 통신 */
#define RETRY         3
#define BUS_GAP       20
#define RX_TIMEOUT    500
#define TM_ID         2       /* DIP 스위치로 설정한 주소 */

/* TL57-R 레지스터 */
#define R_STATE       0x0004   /* PA_004 Run state */
#define R_POS         0x0008   /* PA_008/009 Current position H/L */
#define R_DI0         0x0011   /* PA_011 DI0 function */
#define R_DIR         0x0022   /* PA_022 Default direction */
#define R_PPR         0x0023   /* PA_023 Subdivision */
#define R_START       0x0033   /* PA_033 Start rpm */
#define R_ACC         0x0034   /* PA_034 Acc */
#define R_DEC         0x0035   /* PA_035 Dec */
#define R_SPEED       0x0036   /* PA_036 Position rpm */
#define R_TARGET      0x0037   /* PA_037/038 Target H/L */
#define R_HOME_MODE   0x0040   /* PA_040 Home mode */
#define R_HOME_RPM    0x0041   /* PA_041 Home rpm */
#define R_CREEP_RPM   0x0042   /* PA_042 Zero search rpm */
#define R_HOME_ACC    0x0043   /* PA_043 Home acc/dec */
#define R_CTRL        0x004E   /* PA_04E Control word */
#define R_AUX         0x004F   /* PA_04F Auxiliary control */

#define GO_ABS        0x0003
#define HOME          0x0010
#define STOP          0x0020
#define ZERO_POS      0x0400
#define ENABLE        0x0500

static const uint8_t id = TM_ID;

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

/* G473 UART2 Hardware RS485 DE */
static int bus_xfer(uint8_t *tx, uint16_t tn, uint8_t *rx, uint16_t rn)
{
    HAL_StatusTypeDef r;
    bus_clear();
    r = HAL_UART_Transmit(&huart2, tx, tn, 100);
    if (r == HAL_OK) r = HAL_UART_Receive(&huart2, rx, rn, RX_TIMEOUT);
    osDelay(BUS_GAP);
    return r == HAL_OK;
}

/* FC06 */
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

/* FC03 */
static int read_reg(uint16_t reg, uint8_t qty, uint16_t *out)
{
    uint8_t tx[8], rx[16];
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

/* FC10 : 32bit High -> Low */
static int write32(uint16_t reg, int32_t val)
{
    uint8_t tx[13], rx[8];
    uint32_t raw = (uint32_t)val;
    uint16_t crc;
    tx[0] = id; tx[1] = 0x10;
    tx[2] = reg >> 8; tx[3] = (uint8_t)reg;
    tx[4] = 0; tx[5] = 2; tx[6] = 4;
    tx[7] = (uint8_t)(raw >> 24); tx[8] = (uint8_t)(raw >> 16);
    tx[9] = (uint8_t)(raw >> 8); tx[10] = (uint8_t)raw;
    crc = crc16(tx, 11);
    tx[11] = (uint8_t)crc; tx[12] = (uint8_t)(crc >> 8);
    for (uint8_t retry = 0; retry < RETRY; retry++) {
        if (!bus_xfer(tx, 13, rx, 8)) continue;
        crc = crc16(rx, 6);
        if (rx[0] == id && rx[1] == 0x10 && rx[2] == tx[2] && rx[3] == tx[3]
                && rx[4] == 0 && rx[5] == 2 && rx[6] == (uint8_t)crc
                && rx[7] == (uint8_t)(crc >> 8)) return 1;
    }
    return 0;
}

static int speed_rpm(int percent)
{
    return percent * TM_MAX_RPM / 100;
}

void TM_State(uint16_t *state)
{
    read_reg(R_STATE, 1, state);
}

/* PA_004 Bit0 = 도착 */
int TM_Done(void)
{
	uint16_t state;

	TM_State(&state);

    return (state & TM_DONE) && !(state & TM_RUN);
}

int TM_Pos(int32_t *deg)
{
    uint16_t w[2];
    int32_t pulse;
    if (!read_reg(R_POS, 2, w)) return 0;
    pulse = (int32_t)(((uint32_t)w[0] << 16) | w[1]);
    *deg = (int32_t)((int64_t)pulse * 360 / (TM_PULSE * TM_GEAR));
    return 1;
}

/* 절대 각도 이동 */
int TM_Move(int32_t deg, int percent)
{
    int rpm = speed_rpm(percent);
    int32_t pulse = (int32_t)((int64_t)deg * TM_PULSE * TM_GEAR / 360);
    return write16(R_ACC, TM_ACC_MS)
        && write16(R_DEC, TM_DEC_MS)
        && write16(R_SPEED, rpm)
        && write32(R_TARGET, pulse)
        && write16(R_CTRL, GO_ABS)
        && write16(R_CTRL, 0);
}

int TM_C(int percent) { return TM_Move(0, percent); }
int TM_R(int32_t deg, int percent) { return TM_Move(deg, percent); }
int TM_L(int32_t deg, int percent) { return TM_Move(-deg, percent); }
int TM_Stop(void) { return write16(R_CTRL, STOP); }

/* 기존 원점센서 HOME */
int TM_Home(void)
{
    uint16_t state;

    write16(R_DI0, 1);
    write16(R_HOME_MODE, TM_HOME_MODE);
    write16(R_HOME_RPM, TM_HOME_RPM);
    write16(R_CREEP_RPM, TM_CREEP_RPM);
    write16(R_HOME_ACC, TM_ACC_MS);

    write16(R_CTRL, HOME);

    TM_State(&state);

    /* 원점 완료를 기다리기 위해 사용  */
    while ((state & TM_HOMED) == 0)
    {
        osDelay(20);
        TM_State(&state);
    }

    write16(R_CTRL, 0);
    write16(R_AUX, ZERO_POS);

    return 1;
}
/* 부팅 시 RAM 설정 */
int TM_Init(void)
{
    uint16_t state = 0;

    TM_State(&state);

    if (state & TM_FAULT)
        return 0;

    if (state & TM_RUN)
        TM_Stop();

    if ((state & TM_ENABLE) == 0)
        write16(R_AUX, ENABLE);

    return write16(R_DIR, 1)
        && write16(R_PPR, TM_PULSE)
        && write16(R_START, TM_START_RPM);
}
int TM_Read(uint16_t reg, uint16_t *val) { return read_reg(reg, 1, val); }
