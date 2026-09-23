#include "a_motor.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdarg.h>
/* 통신 */
#define RETRY          3
#define BUS_GAP        5
#define RX_TIMEOUT     50
/* DI Function / Logic */
#define R_DI1_FUNC      0x0302   /* H03_02 : Home Switch */
#define R_DI1_LOGIC     0x0303   /* H03_03 */
#define R_DI2_FUNC      0x0304   /* H03_04 : Servo Enable */
#define R_DI2_LOGIC     0x0305   /* H03_05 */
#define R_DI3_FUNC      0x0306   /* H03_06 : Homeing Start */
#define R_DI3_LOGIC     0x0307   /* H03_07 */
#define R_DI4_FUNC      0x0308   /* H03_08 : Position Run */
#define R_DI4_LOGIC     0x0309   /* H03_09 */
#define R_DI5_FUNC      0x030A   /* H03_10 : Emergency Stop */
#define R_DI5_LOGIC     0x030B   /* H03_11 */
/* 위치 / HOME */
#define R_MODE          0x0200   /* H02_00 Control mode */
#define R_POS_SOURCE    0x0500   /* H05_00 Position source */
#define R_HOME_START    0x051E   /* H05_30 */
#define R_HOME_MODE     0x051F   /* H05_31 */
#define R_HOME_FAST     0x0520   /* H05_32 */
#define R_HOME_SLOW     0x0521   /* H05_33 */
#define R_HOME_ACC      0x0522   /* H05_34 */
#define R_HOME_TIMEOUT  0x0523   /* H05_35 */
#define R_HOME_TORQUE   0x053A   /* H05_58 */
/* 상태 */
#define R_REAL_SPEED    0x0B00   /* H0B_00 Real speed */
#define R_REAL_TORQUE   0x0B02   /* H0B_02 Real torque */
#define R_REAL_POS      0x0B07   /* H0B_07 Absolute position */
#define R_ALARM         0x0B34   /* H0B_34 Fault code */
#define R_ALARM_RESET   0x0D01   /* H0D_01 Error reset */
#define R_DO1_FUNC      0x0400   /* H04_00 DO1 function */
#define R_DO2_FUNC      0x0402   /* H04_02 DO2 function */
#define R_DO_STATE      0x0B05   /* H0B_05 DO monitor */
#define R_POS_DONE_GAP  0x0515   /* H05_21 Position complete gap */
/* 다단 위치운전 */
#define R_RUN_MODE      0x1100   /* H11_00 Single cycle */
#define R_MOVE_TYPE     0x1104   /* H11_04 1=Absolute */
#define R_MOVE_POS      0x110C   /* H11_12 ~ H11_16 */
/* EEPROM */
#define R_ADDR          0x0C00   /* H0C_00 Servo axis address */
#define R_BAUD          0x0C02   /* H0C_02 Baud rate */
#define R_EEPROM_SAVE   0x0C0D   /* H0C_13 Save EEPROM */
#define R_RESET         0x0D00   /* H0D_00 Software reset */
#define OLD_BAUD        57600    /* A motor 공장 설정 */
#define BAUD_CODE       6        /* H0C_02 : 6 = 115200 */
#define SAVE_WAIT       100      /* EEPROM 기록 대기 */
#define RESET_WAIT      3000     /* 드라이브 재시작 대기 */

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
        volatile uint8_t data = (uint8_t)huart2.Instance->RDR;
        (void)data;
    }
}
/* UART2 Hardware RS485 송수신 */
static int bus_xfer(uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len)
{
    HAL_StatusTypeDef result;
    bus_clear();
    result = HAL_UART_Transmit(&huart2, tx, tx_len, 100);
    if (result == HAL_OK)
        result = HAL_UART_Receive(&huart2, rx, rx_len, RX_TIMEOUT);
    osDelay(BUS_GAP);
    return result == HAL_OK;
}
/* 06H / 10H : 16bit, 32bit, 연속 레지스터 쓰기 */
static int write_regs(uint16_t reg, uint8_t qty, ...)
{
    uint8_t tx[29], rx[8];
    uint16_t crc, value;
    va_list ap;
    va_start(ap, qty);
    if (qty == 1) {
        value = (uint16_t)va_arg(ap, int);
        va_end(ap);
        tx[0] = AM_ID; tx[1] = 0x06;
        tx[2] = reg >> 8; tx[3] = (uint8_t)reg;
        tx[4] = value >> 8; tx[5] = (uint8_t)value;
        crc = crc16(tx, 6);
        tx[6] = (uint8_t)crc; tx[7] = (uint8_t)(crc >> 8);
        for (uint8_t retry = 0; retry < RETRY; retry++) {
            if (bus_xfer(tx, 8, rx, 8) && memcmp(rx, tx, 8) == 0)
                return 1;
        }
        return 0;
    }
    tx[0] = AM_ID; tx[1] = 0x10;
    tx[2] = reg >> 8; tx[3] = (uint8_t)reg;
    tx[4] = 0; tx[5] = qty; tx[6] = qty * 2;
    for (uint8_t i = 0; i < qty; i++) {
        value = (uint16_t)va_arg(ap, int);
        tx[7 + i * 2] = value >> 8;
        tx[8 + i * 2] = (uint8_t)value;
    }
    va_end(ap);
    crc = crc16(tx, 7 + qty * 2);
    tx[7 + qty * 2] = (uint8_t)crc;
    tx[8 + qty * 2] = (uint8_t)(crc >> 8);
    for (uint8_t retry = 0; retry < RETRY; retry++) {
        if (!bus_xfer(tx, 9 + qty * 2, rx, 8)) continue;
        crc = crc16(rx, 6);
        if (rx[0] == AM_ID && rx[1] == 0x10 && rx[2] == tx[2] && rx[3] == tx[3]
                && rx[4] == 0 && rx[5] == qty && rx[6] == (uint8_t)crc
                && rx[7] == (uint8_t)(crc >> 8)) return 1;
    }
    return 0;
}
/* 03H : 16bit / 32bit 읽기 */
static int read_regs(uint16_t reg, int32_t *out, uint8_t qty)
{
    uint8_t tx[8], rx[9];
    uint16_t crc, low, high;
    uint8_t rx_len = 5 + qty * 2;
    tx[0] = AM_ID;
    tx[1] = 0x03;
    tx[2] = reg >> 8;
    tx[3] = (uint8_t)reg;
    tx[4] = 0;
    tx[5] = qty;
    crc = crc16(tx, 6);
    tx[6] = (uint8_t)crc;
    tx[7] = (uint8_t)(crc >> 8);
    for (uint8_t retry = 0; retry < RETRY; retry++) {
        if (!bus_xfer(tx, 8, rx, rx_len))
            continue;
        crc = crc16(rx, rx_len - 2);
        if (rx[0] != AM_ID || rx[1] != 0x03 || rx[2] != qty * 2)
            continue;
        if (rx[rx_len - 2] != (uint8_t)crc ||
            rx[rx_len - 1] != (uint8_t)(crc >> 8))
            continue;
        low = ((uint16_t)rx[3] << 8) | rx[4];
        if (qty == 1) {
            *out = low;
            return 1;
        }
        high = ((uint16_t)rx[5] << 8) | rx[6];
        *out = (int32_t)(((uint32_t)high << 16) | low);
        return 1;
    }
    return 0;
}
/* mm -> pulse */
static int32_t move_pos(int32_t mm)
{
    return (int32_t)((int64_t)mm * Y_PULSE * Y_GEAR / Y_PI);
}

/* 현재 위치 mm */
static int read_pos(int32_t *mm)
{
    int32_t pulse;

    if (!read_regs(R_REAL_POS, &pulse, 2))
        return 0;

    *mm = (int32_t)(-(int64_t)pulse * Y_PI
                    / (Y_PULSE * Y_GEAR));

    return 1;
}
/* 100% = 3000rpm */
static int speed_rpm(int percent)
{
    return percent * AM_MAX_RPM / 100;
}
/* H11_12~H11_16 : 위치 + 속도 + ACC/DEC + WAIT */
static int write_move(int32_t pulse, int percent)
{
    int32_t target = -pulse;
    uint32_t raw = (uint32_t)target;
    int rpm = speed_rpm(percent);
    return write_regs(R_DI4_LOGIC, 1, 0)
        && write_regs(R_MOVE_POS, 5,
            (uint16_t)raw,
            (uint16_t)(raw >> 16),
            rpm,
            AM_ACC_DEC_MS,
            AM_WAIT_MS)
        && write_regs(R_DI4_LOGIC, 1, 1);
}
/* ID 1 통신 확인 */
static int check_id(void)
{
    int32_t address;
    return read_regs(R_ADDR, &address, 1) && address == AM_ID;
}

/* 모터 기능 */
int AM_Move(int32_t mm, int percent){ return write_move(move_pos(mm), percent); }
/* 토크 원점 -> 100mm 이동 -> 0점 */
int AM_Home(void)
{
    write_regs(R_HOME_SLOW,    1, AM_HOME_LO_RPM);
    write_regs(R_HOME_ACC,     1, AM_HOME_ACC_MS);
    write_regs(R_HOME_TIMEOUT, 1, AM_HOME_TIMEOUT_MS);
    write_regs(R_HOME_TORQUE,  1, AM_HOME_TORQUE);

    write_regs(R_HOME_START, 1, 4);

    while (AM_HomeDone() == 0)
        osDelay(20);

    AM_Move(100, 10);
    osDelay(AM_ACC_DEC_MS);        /* 출발 전 COIN 오판 방지 */

    while (AM_Done() == 0)
        osDelay(20);

    AM_Zero();

    return 1;
}
int AM_Stop(void)
{
    return write_regs(R_DI4_LOGIC, 1, 0)
        && write_regs(R_HOME_START, 1, 0);
}
int AM_EStop(int on) { return write_regs(R_DI5_LOGIC, 1, on); }
int AM_Servo(int on) { return write_regs(R_DI2_LOGIC, 1, on); }

/* 상태 읽기 */
int AM_Pos(int32_t *mm){ return read_pos(mm); }

/* 위치운전 완료 : DO1 = COIN */
int AM_Done(void)
{
    int32_t state;
    return read_regs(R_DO_STATE, &state, 1) && (state & 1);
}
int AM_Speed(int *rpm)
{
    int32_t value;
    if (!read_regs(R_REAL_SPEED, &value, 1)) return 0;
    *rpm = (int16_t)value;
    return 1;
}
int AM_Torque(int *torque)
{
    int32_t value;
    if (!read_regs(R_REAL_TORQUE, &value, 1)) return 0;
    *torque = (int16_t)value;
    return 1;
}
int AM_AlarmRead(uint16_t *alarm)
{
    int32_t value;
    if (!read_regs(R_ALARM, &value, 1)) return 0;
    *alarm = (uint16_t)value;
    return 1;
}
int AM_AlarmReset(void) { return write_regs(R_ALARM_RESET, 1, 1); }
int AM_HomeDone(void)
{
    int32_t state;

    return read_regs(R_DO_STATE, &state, 1) && (state & 2);
}
/* 현재 위치를 0으로 */
int AM_Zero(void)
{
    return write_regs(R_HOME_START, 1, 6);
}
/* ID 확인 + DI + 이동 + 토크 HOME 설정 */
int AM_Init(void)
{
    if (!check_id()) return 0;
    return write_regs(R_DI1_FUNC, 10,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        && write_regs(R_DI1_FUNC, 10,
            31, 0,
            1,  0,
            32, 0,
            28, 0,
            34, 0)
        && write_regs(R_DO1_FUNC, 2, 5, 0)
		&& write_regs(R_DO2_FUNC, 2, 16, 0)
        && write_regs(R_POS_DONE_GAP, 1, AM_DONE_GAP)
        && write_regs(R_MODE, 1, 1)
        && write_regs(R_POS_SOURCE, 1, 2)
        && write_regs(R_RUN_MODE, 3, 0, 1, 0)
        && write_regs(R_MOVE_TYPE, 1, 1)
		&&write_regs(R_HOME_MODE, 1, AM_HOME_MODE)   /* Enable OFF에서만 변경 */
        && AM_Servo(1);
}
int AM_Read(uint16_t reg, uint16_t *val)
{
    int32_t value;
    if (!read_regs(reg, &value, 1)) return 0;
    *val = (uint16_t)value;
    return 1;
}
///////////////////////////EEPROM///////////////////////////////////////////
/* STM UART2 속도 */
static void uart_speed(uint32_t baud)
{
    HAL_UART_DeInit(&huart2);
    huart2.Init.BaudRate = baud;
    HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0);
}

/* A motor 속도를 AM_BAUD로 저장 후 재시작 */
int AM_BaudInit(void)
{
    if (check_id())
        return 1;

    /* 1. 옛 속도로 접속 */
    uart_speed(OLD_BAUD);

    /* 2. 속도 변경(ER.941) 후 저장 */
    write_regs(R_BAUD, 1, BAUD_CODE);
    write_regs(R_EEPROM_SAVE, 1, 1);
    osDelay(SAVE_WAIT);

    /* 3. 드라이브 재시작 */
    write_regs(R_RESET, 1, 1);

    /* 4. STM 복귀 후 확인 */
    uart_speed(AM_BAUD);
    osDelay(RESET_WAIT);

    return check_id();
}
