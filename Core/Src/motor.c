#include "motor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/* X = UART4, Y = UART5, both AIMotor address 1 */
Motor motorX = { &huart4, rs485_GPIO_Port,  rs485_Pin,  0 };
Motor motorY = { &huart5, rs4852_GPIO_Port, rs4852_Pin, 0 };

#define ID              1U

#define R_MODE          0x0200
#define R_DIR           0x0202
#define R_DI1           0x0302
#define R_DI2           0x0304
#define R_DI2L          0x0305
#define R_DI3           0x0306
#define R_DI4           0x0308
#define R_DI4L          0x0309
#define R_DI5           0x030A
#define R_DI5L          0x030B

#define R_SPEED_A       0x0600
#define R_SPEED_SEL     0x0602
#define R_SPEED_CMD     0x0603
#define R_SPEED_ACC     0x0605
#define R_SPEED_DEC     0x0606

#define R_SRC           0x0500
#define R_RUN           0x1100
#define R_REG           0x1101
#define R_BEGIN         0x1102
#define R_TYPE          0x1104
#define R_POS           0x110C
#define R_SPEED         0x110E
#define R_ACC           0x110F
#define R_WAIT          0x1110

#define R_REALPOS       0x0B07
#define R_DI            0x0B03
#define R_ADDR          0x0C00
#define R_FAULT_RST     0x0D01

#define ACC_MS          1000   /* position move acceleration/deceleration */
#define WAIT_MS         500    /* wait after one position segment */
#define GAP             100    /* arrival tolerance used by motor_wait */

static int x_mode = -1;
static int y_mode = -1;

void print(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)s, strlen(s), 100);
}

static char name(Motor *m)
{
    return (m == &motorX) ? 'X' : 'Y';
}

static int *mode_ptr(Motor *m)
{
    return (m == &motorX) ? &x_mode : &y_mode;
}

static uint16_t crc16(
        const uint8_t *data,
        uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1U) {
                crc = (crc >> 1) ^ 0xA001U;
            }
            else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static void bus_tx(Motor *m)
{
    HAL_GPIO_WritePin(m->port, m->pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

static void bus_rx(Motor *m)
{
    while (__HAL_UART_GET_FLAG(
            m->uart,
            UART_FLAG_TC) == RESET) {
    }

    for (volatile uint32_t i = 0; i < 100; i++) {
    }

    HAL_GPIO_WritePin(m->port, m->pin, GPIO_PIN_RESET);
}

static void bus_clear(Motor *m)
{
    __HAL_UART_CLEAR_OREFLAG(m->uart);
    __HAL_UART_CLEAR_FEFLAG(m->uart);
    __HAL_UART_CLEAR_NEFLAG(m->uart);
    __HAL_UART_CLEAR_PEFLAG(m->uart);

    while (__HAL_UART_GET_FLAG(
            m->uart,
            UART_FLAG_RXNE) != RESET) {

        volatile uint8_t d = m->uart->Instance->DR;
        (void)d;
    }
}

static HAL_StatusTypeDef bus_xfer(
        Motor *m,
        uint8_t *tx,
        uint16_t tn,
        uint8_t *rx,
        uint16_t rn)
{
    HAL_StatusTypeDef r;

    bus_clear(m);
    bus_tx(m);

    r = HAL_UART_Transmit(
            m->uart,
            tx,
            tn,
            100);

    bus_rx(m);

    if (r == HAL_OK) {
        r = HAL_UART_Receive(
                m->uart,
                rx,
                rn,
                500);
    }

    HAL_Delay(20);
    return r;
}

static int write16(
        Motor *m,
        uint16_t reg,
        uint16_t val)
{
    uint8_t tx[8];
    uint8_t rx[8];
    uint16_t crc;
    uint16_t rx_crc;
    HAL_StatusTypeDef st;
    char b[120];

    tx[0] = ID;
    tx[1] = 0x06;
    tx[2] = reg >> 8;
    tx[3] = reg;
    tx[4] = val >> 8;
    tx[5] = val;

    crc = crc16(tx, 6);
    tx[6] = crc;
    tx[7] = crc >> 8;

    st = bus_xfer(m, tx, 8, rx, 8);

    if (st != HAL_OK) {
        snprintf(
                b,
                sizeof(b),
                "%c WR TIMEOUT reg=%04X val=%04X st=%d\r\n",
                name(m),
                reg,
                val,
                (int)st);
        print(b);
        return 0;
    }

    rx_crc = (uint16_t)rx[6] | ((uint16_t)rx[7] << 8);

    if (rx[0] != ID ||
        rx[1] != 0x06 ||
        rx[2] != tx[2] ||
        rx[3] != tx[3] ||
        rx[4] != tx[4] ||
        rx[5] != tx[5] ||
        rx_crc != crc16(rx, 6)) {

        snprintf(
                b,
                sizeof(b),
                "%c WR BAD %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                name(m),
                rx[0], rx[1], rx[2], rx[3],
                rx[4], rx[5], rx[6], rx[7]);
        print(b);
        return 0;
    }

    return 1;
}

static int set16(
        Motor *m,
        uint16_t reg,
        uint16_t val)
{
    for (int i = 0; i < 3; i++) {
        if (write16(m, reg, val)) {
            HAL_Delay(30);
            return 1;
        }

        HAL_Delay(50);
    }

    return 0;
}

static uint16_t read16(
        Motor *m,
        uint16_t reg)
{
    uint8_t tx[8];
    uint8_t rx[7];
    uint16_t crc;
    uint16_t rx_crc;

    tx[0] = ID;
    tx[1] = 0x03;
    tx[2] = reg >> 8;
    tx[3] = reg;
    tx[4] = 0;
    tx[5] = 1;

    crc = crc16(tx, 6);
    tx[6] = crc;
    tx[7] = crc >> 8;

    if (bus_xfer(m, tx, 8, rx, 7) != HAL_OK) {
        return 0xFFFF;
    }

    rx_crc = (uint16_t)rx[5] | ((uint16_t)rx[6] << 8);

    if (rx[0] != ID ||
        rx[1] != 0x03 ||
        rx[2] != 2 ||
        rx_crc != crc16(rx, 5)) {
        return 0xFFFF;
    }

    return ((uint16_t)rx[3] << 8) | rx[4];
}

static int read16_ok(
        Motor *m,
        uint16_t reg,
        uint16_t *out)
{
    uint16_t v = read16(m, reg);

    if (v == 0xFFFF) {
        return 0;
    }

    *out = v;
    return 1;
}

static int write32(
        Motor *m,
        uint16_t reg,
        int val)
{
    uint8_t tx[13];
    uint8_t rx[8];
    uint32_t raw = (uint32_t)val;
    uint16_t lo = raw;
    uint16_t hi = raw >> 16;
    uint16_t crc;
    uint16_t rx_crc;

    tx[0] = ID;
    tx[1] = 0x10;
    tx[2] = reg >> 8;
    tx[3] = reg;
    tx[4] = 0;
    tx[5] = 2;
    tx[6] = 4;
    tx[7] = lo >> 8;
    tx[8] = lo;
    tx[9] = hi >> 8;
    tx[10] = hi;

    crc = crc16(tx, 11);
    tx[11] = crc;
    tx[12] = crc >> 8;

    if (bus_xfer(m, tx, 13, rx, 8) != HAL_OK) {
        return 0;
    }

    rx_crc = (uint16_t)rx[6] | ((uint16_t)rx[7] << 8);

    return
            rx[0] == ID &&
            rx[1] == 0x10 &&
            rx[2] == tx[2] &&
            rx[3] == tx[3] &&
            rx[4] == 0 &&
            rx[5] == 2 &&
            rx_crc == crc16(rx, 6);
}

static int read32(
        Motor *m,
        uint16_t reg,
        int *out)
{
    uint8_t tx[8];
    uint8_t rx[9];
    uint16_t crc;
    uint16_t rx_crc;
    uint16_t lo;
    uint16_t hi;

    tx[0] = ID;
    tx[1] = 0x03;
    tx[2] = reg >> 8;
    tx[3] = reg;
    tx[4] = 0;
    tx[5] = 2;

    crc = crc16(tx, 6);
    tx[6] = crc;
    tx[7] = crc >> 8;

    if (bus_xfer(m, tx, 8, rx, 9) != HAL_OK) {
        return 0;
    }

    rx_crc = (uint16_t)rx[7] | ((uint16_t)rx[8] << 8);

    if (rx[0] != ID ||
        rx[1] != 0x03 ||
        rx[2] != 4 ||
        rx_crc != crc16(rx, 7)) {
        return 0;
    }

    lo = ((uint16_t)rx[3] << 8) | rx[4];
    hi = ((uint16_t)rx[5] << 8) | rx[6];

    *out = (int)(((uint32_t)hi << 16) | lo);
    return 1;
}

static int cfg(Motor *m, int dir)
{
    /* H03_03 is intentionally never written. */
    if (!set16(m, R_DI2L, 0)) return 0;
    if (!set16(m, R_DI4L, 0)) return 0;
    if (!set16(m, R_DI5L, 0)) return 0;

    if (!set16(m, R_DI1, 0)) return 0;   /* sensor is read from H0B_03 */
    if (!set16(m, R_DI2, 1)) return 0;   /* servo enable */
    if (!set16(m, R_DI3, 0)) return 0;
    if (!set16(m, R_DI4, 28)) return 0;  /* position start */
    if (!set16(m, R_DI5, 34)) return 0;  /* emergency stop */
    if (!set16(m, R_DIR, dir)) return 0;

    set16(m, R_FAULT_RST, 1);
    set16(m, R_FAULT_RST, 0);
    return 1;
}

int motor_write(Motor *m, uint16_t reg, uint16_t val)
{
    return write16(m, reg, val);
}

int motor_check(Motor *m)
{
    uint16_t v;

    /* H0C_00 axis address is UInt16. */
    for (int i = 0; i < 2; i++) {
        if (read16_ok(m, R_ADDR, &v) && v == ID) {
            return 1;
        }
        HAL_Delay(30);
    }

    return 0;
}

int motor_pos_mode(Motor *m)
{
    int *mode = mode_ptr(m);
    if (*mode == 1) return 1;

    if (!set16(m, R_SPEED_CMD, 0) ||
        !set16(m, R_DI2L, 0) ||
        !set16(m, R_MODE, 1) ||
        !set16(m, R_SRC, 2) ||
        !set16(m, R_RUN, 0) ||
        !set16(m, R_REG, 1) ||
        !set16(m, R_BEGIN, 0) ||
        !set16(m, R_DI4L, 0) ||
        !set16(m, R_DI2L, 1)) {
        *mode = -1;
        return 0;
    }

    *mode = 1;
    return 1;
}

int motor_speed_mode(Motor *m)
{
    int *mode = mode_ptr(m);
    if (*mode == 0) return 1;

    if (!set16(m, R_SPEED_CMD, 0) ||
        !set16(m, R_DI2L, 0) ||
        !set16(m, R_MODE, 0) ||
        !set16(m, R_SPEED_A, 0) ||
        !set16(m, R_SPEED_SEL, 0) ||
        !set16(m, R_SPEED_ACC, 200) ||
        !set16(m, R_SPEED_DEC, 200) ||
        !set16(m, R_DI2L, 1)) {
        *mode = -1;
        return 0;
    }

    *mode = 0;
    return 1;
}

int motor_init(Motor *m, int dir)
{
    *mode_ptr(m) = -1;

    if (!motor_check(m)) {
        char b[48];
        snprintf(b, sizeof(b), "%c COMM FAIL\r\n", name(m));
        print(b);
        return 0;
    }

    if (!cfg(m, dir) || !motor_pos_mode(m)) {
        char b[48];
        snprintf(b, sizeof(b), "%c INIT CFG FAIL\r\n", name(m));
        print(b);
        return 0;
    }
    return 1;
}

int motor_speed(Motor *m, int rpm)
{
    if (rpm < -6000 || rpm > 6000) return 0;
    if (!motor_speed_mode(m)) return 0;
    return set16(m, R_SPEED_CMD, (uint16_t)(int16_t)rpm);
}

int motor_home_on(Motor *m)
{
    int di;

    /*
     * H0B_03 is UInt32 in the AIMotor manual.
     * Bit0 is the physical DI1 home sensor state.
     */
    if (!read32(m, R_DI, &di)) return -1;
    return (di & 1) ? 1 : 0;
}

int motor_zero(Motor *m)
{
    int raw;
    if (!read32(m, R_REALPOS, &raw) && !read32(m, R_REALPOS, &raw)) return 0;
    m->off = -raw;
    return 1;
}

void motor_sync(Motor *m, int last)
{
    int raw;
    m->off = read32(m, R_REALPOS, &raw) ? last - raw : 0;
}

int motor_pos(Motor *m, int *out)
{
    int raw;
    if (!read32(m, R_REALPOS, &raw)) return 0;
    *out = raw + m->off;
    return 1;
}

int motor_move(Motor *m, int rpm, int target)
{
    int raw_target;

    /*
     * target is a software coordinate whose home sensor is 0.
     * The motor itself moves with the raw H0B_07 coordinate, so convert
     * software target -> motor raw target by removing the software offset.
     *
     * H11_04 = 1 means absolute displacement. This avoids accumulating
     * relative-move errors and works from any current X/Y position.
     */
    if (!motor_pos_mode(m)) return 0;

    raw_target = target - m->off;

    if (!set16(m, R_DI4L, 0)) return 0;
    if (!set16(m, R_TYPE, 1)) return 0;
    if (!write32(m, R_POS, raw_target)) return 0;
    if (!set16(m, R_SPEED, rpm)) return 0;
    if (!set16(m, R_ACC, ACC_MS)) return 0;
    if (!set16(m, R_WAIT, WAIT_MS)) return 0;

    return set16(m, R_DI4L, 1);
}

int motor_stop(Motor *m)
{
    if (*mode_ptr(m) == 0) return set16(m, R_SPEED_CMD, 0);
    return set16(m, R_DI4L, 0);
}

int motor_begin(Motor *m, int v)
{
    return set16(m, R_BEGIN, v);
}

int motor_di5(Motor *m, int v)
{
    return set16(m, R_DI5L, v);
}

int motor_wait(Motor *mx, int xt, Motor *my, int yt, int timeout)
{
    uint32_t t0 = HAL_GetTick();
    int x = 0, y = 0, xd = 0, yd = 0;

    while (HAL_GetTick() - t0 < (uint32_t)timeout) {
        if (!xd && motor_pos(mx, &x) && abs(x - xt) <= GAP) xd = 1;
        if (!yd && motor_pos(my, &y) && abs(y - yt) <= GAP) yd = 1;
        if (xd && yd) return 1;
        HAL_Delay(50);
    }

    motor_stop(mx);
    motor_stop(my);
    return 0;
}
