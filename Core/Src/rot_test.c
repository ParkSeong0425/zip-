#include "rot_test.h"

#include "save.h"

#include "tcp.h"

#include "motor.h"



#include <limits.h>

#include <stdint.h>

#include <stdio.h>



extern UART_HandleTypeDef huart5;



/* ===== MKS 프로토콜 ===== */



#define ID              2U

#define TX_HEAD         0xFAU

#define RX_HEAD         0xFBU



#define C_MODE          0x82U

#define C_CURRENT       0x83U

#define C_RESPOND       0x8CU

#define C_HOME_PARAM    0x90U

#define C_HOME          0x91U

#define C_ZERO          0x92U

#define C_POS           0x31U

#define C_RAW           0x35U

#define C_IO            0x34U

#define C_STATE         0xF1U

#define C_ENABLE        0xF3U

#define C_REL           0xF4U

#define C_ABS           0xF5U



#define HOME_TRIG       0U

#define HOME_DIR        1U      /* 0=CW, 1=CCW */

#define HOME_RPM        100U

#define HOME_END        0U

#define HOME_TIMEOUT    65000U

#define MOVE_ACC        255U



/* ===== 하위 통신 ===== */



static uint8_t sum8(const uint8_t *p, uint16_t n)

{

    uint16_t s = 0;



    while (n--) s += *p++;

    return (uint8_t)s;

}



static void tx_mode(void)

{

    HAL_GPIO_WritePin(rs4852_GPIO_Port, rs4852_Pin, GPIO_PIN_SET);

    HAL_Delay(1);

}



static void rx_mode(void)

{

    while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC) == RESET) { }

    for (volatile uint32_t i = 0; i < 100; i++) { }

    HAL_GPIO_WritePin(rs4852_GPIO_Port, rs4852_Pin, GPIO_PIN_RESET);

}



static void clear_rx(void)

{

    __HAL_UART_CLEAR_OREFLAG(&huart5);

    __HAL_UART_CLEAR_FEFLAG(&huart5);

    __HAL_UART_CLEAR_NEFLAG(&huart5);

    __HAL_UART_CLEAR_PEFLAG(&huart5);



    while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE) != RESET) {

        volatile uint8_t d = huart5.Instance->DR;

        (void)d;

    }

}



/* 송신 후 rn바이트 수신. RX 전환 뒤 지연 없음 */

static int xfer(uint8_t *tx, uint16_t tn, uint8_t *rx, uint16_t rn)

{

    HAL_StatusTypeDef r;



    clear_rx();

    tx_mode();

    r = HAL_UART_Transmit(&huart5, tx, tn, 100);

    rx_mode();



    if (r != HAL_OK) return 0;



    r = HAL_UART_Receive(&huart5, rx, rn, 500);

    HAL_Delay(20);



    return r == HAL_OK;

}



/* 헤더, ID, 명령, 체크섬 확인 */

static int frame_ok(uint8_t *rx, uint16_t n, uint8_t cmd)

{

    return n >= 4 && rx[0] == RX_HEAD && rx[1] == ID &&

           rx[2] == cmd && rx[n - 1] == sum8(rx, n - 1);

}



/* ===== 진단 출력 ===== */



static void hex_line(const char *tag, const uint8_t *p, int n)

{

    char b[160];

    int k = snprintf(b, sizeof(b), "%s n=%d err=%08lX :",

                     tag, n, (unsigned long)HAL_UART_GetError(&huart5));



    for (int i = 0; i < n && k < (int)sizeof(b) - 5; i++)

        k += snprintf(&b[k], sizeof(b) - k, " %02X", p[i]);



    snprintf(&b[k], sizeof(b) - k, "\r\n");

    print(b);

}



/* F1 상태 확인. 1바이트씩 받아 타임아웃/불량 프레임을 눈으로 본다 */

static int probe_f1(uint8_t *state)

{

    uint8_t tx[4] = { TX_HEAD, ID, C_STATE, 0 };

    uint8_t rx[16] = {0};

    HAL_StatusTypeDef st;

    int n = 0;



    tx[3] = sum8(tx, 3);

    hex_line("MKS TX", tx, 4);



    clear_rx();

    tx_mode();

    st = HAL_UART_Transmit(&huart5, tx, 4, 100);

    rx_mode();



    if (st != HAL_OK) {

        print("MKS TX HAL FAIL\r\n");

        return 0;

    }



    /* 첫 바이트만 오래 기다린다 */

    st = HAL_UART_Receive(&huart5, &rx[0], 1, 500);



    if (st == HAL_OK) {

        n = 1;

        /* 나머지는 연달아 들어와야 한다 */

        while (n < (int)sizeof(rx)) {

            st = HAL_UART_Receive(&huart5, &rx[n], 1, 10);

            if (st != HAL_OK) break;

            n++;

        }

    }



    hex_line("MKS RX", rx, n);

    HAL_Delay(50);



    if (n != 5 || !frame_ok(rx, 5, C_STATE)) return 0;



    *state = rx[3];

    return 1;

}



/* ===== 명령 송수신 ===== */



/* 데이터 없는 명령 */

static int cmd0(uint8_t cmd, uint8_t *v)

{

    uint8_t tx[4] = { TX_HEAD, ID, cmd, 0 };

    uint8_t rx[5];



    tx[3] = sum8(tx, 3);



    if (!xfer(tx, 4, rx, 5) || !frame_ok(rx, 5, cmd)) return 0;

    if (v) *v = rx[3];

    return 1;

}



/* 데이터 1바이트 명령 */

static int cmd1(uint8_t cmd, uint8_t d, uint8_t *v)

{

    uint8_t tx[5] = { TX_HEAD, ID, cmd, d, 0 };

    uint8_t rx[5];



    tx[4] = sum8(tx, 4);



    if (!xfer(tx, 5, rx, 5) || !frame_ok(rx, 5, cmd)) return 0;

    if (v) *v = rx[3];

    return 1;

}



/* 데이터 2바이트 명령 */

static int cmd2(uint8_t cmd, uint8_t a, uint8_t b, uint8_t *v)

{

    uint8_t tx[6] = { TX_HEAD, ID, cmd, a, b, 0 };

    uint8_t rx[5];



    tx[5] = sum8(tx, 5);



    if (!xfer(tx, 6, rx, 5) || !frame_ok(rx, 5, cmd)) return 0;

    if (v) *v = rx[3];

    return 1;

}



/* 응답이 1이어야 성공 */

static int set1(uint8_t cmd, uint8_t d)

{

    uint8_t v;

    return cmd1(cmd, d, &v) && v == 1U;

}



static int set2(uint8_t cmd, uint8_t a, uint8_t b)

{

    uint8_t v;

    return cmd2(cmd, a, b, &v) && v == 1U;

}



/* 원점 파라미터 설정 */

static int home_param(void)

{

    uint8_t tx[9], rx[5];



    tx[0] = TX_HEAD;

    tx[1] = ID;

    tx[2] = C_HOME_PARAM;

    tx[3] = HOME_TRIG;

    tx[4] = HOME_DIR;

    tx[5] = HOME_RPM >> 8;

    tx[6] = HOME_RPM;

    tx[7] = HOME_END;

    tx[8] = sum8(tx, 8);



    return xfer(tx, 9, rx, 5) && frame_ok(rx, 5, C_HOME_PARAM) && rx[3] == 1U;

}



static int read8(uint8_t cmd, uint8_t *out)

{

    uint8_t tx[4] = { TX_HEAD, ID, cmd, 0 };

    uint8_t rx[5];



    tx[3] = sum8(tx, 3);



    if (!xfer(tx, 4, rx, 5) || !frame_ok(rx, 5, cmd)) return 0;

    *out = rx[3];

    return 1;

}



/* 48bit 부호값을 int로 변환해 읽는다 */

static int read48(uint8_t cmd, int *out)

{

    uint8_t tx[4] = { TX_HEAD, ID, cmd, 0 };

    uint8_t rx[10];

    uint64_t raw = 0;

    int64_t v;



    tx[3] = sum8(tx, 3);



    if (!xfer(tx, 4, rx, 10) || !frame_ok(rx, 10, cmd)) return 0;



    for (uint8_t i = 3; i <= 8; i++) raw = (raw << 8) | rx[i];



    if (raw & (1ULL << 47)) raw |= 0xFFFF000000000000ULL;



    v = (int64_t)raw;

    if (v < INT_MIN || v > INT_MAX) return 0;



    *out = (int)v;

    return 1;

}



/* 속도, 가속도, 목표값을 함께 보내는 이동 명령 */

static int write_axis(uint8_t cmd, uint16_t rpm, uint8_t acc, int target)

{

    uint8_t tx[11], rx[5];

    uint32_t p = (uint32_t)target;



    tx[0] = TX_HEAD;

    tx[1] = ID;

    tx[2] = cmd;

    tx[3] = rpm >> 8;

    tx[4] = rpm;

    tx[5] = acc;

    tx[6] = p >> 24;

    tx[7] = p >> 16;

    tx[8] = p >> 8;

    tx[9] = p;

    tx[10] = sum8(tx, 10);



    return xfer(tx, 11, rx, 5) && frame_ok(rx, 5, cmd) && rx[3] == 1U;

}



/* ===== 공개 함수 ===== */



/* MKS 부팅이 끝나기 전에 여기 도달할 수 있어 재시도한다 */

int mks_init(void)

{

    uint8_t state = 0;

    int ok = 0;



    HAL_Delay(1500);



    for (int i = 0; i < 10; i++) {

        if (probe_f1(&state)) { ok = 1; break; }

        HAL_Delay(300);

    }



    if (!ok) { print("MKS F1 FAIL\r\n"); return 0; }

    print("MKS F1 OK\r\n");

    HAL_Delay(50);



    if (!set2(C_RESPOND, 1U, 0U)) { print("MKS 8C FAIL\r\n"); return 0; }

    print("MKS 8C OK\r\n");

    HAL_Delay(50);



    if (!home_param()) { print("MKS 90 FAIL\r\n"); return 0; }

    print("MKS 90 OK\r\n");

    HAL_Delay(50);



    if (!set1(C_ENABLE, 1U)) { print("MKS F3 FAIL\r\n"); return 0; }

    print("MKS F3 OK\r\n");



    return 1;

}



int mks_check(void)

{

    uint8_t v;

    return mks_state(&v);

}



int mks_pos(int *out)       { return read48(C_POS, out); }

int mks_raw(int *out)       { return read48(C_RAW, out); }

int mks_io(uint8_t *out)    { return read8(C_IO, out); }

int mks_state(uint8_t *out) { return read8(C_STATE, out); }



/* 현재 위치를 축 좌표 0으로 설정 */

int mks_zero(void)

{

    uint8_t v;

    return cmd0(C_ZERO, &v) && v == 1U;

}



/* 광센서 원점 복귀. 상태 5를 본 뒤 1이 되면 완료 */

int mks_home(void)

{

    uint8_t v;

    uint32_t t0;

    int saw_home = 0;



    if (!cmd0(C_HOME, &v)) return 0;

    if (v == 2U) return 1;

    if (v != 1U) return 0;



    t0 = HAL_GetTick();



    while (HAL_GetTick() - t0 < HOME_TIMEOUT) {

        if (mks_state(&v)) {

            if (v == 5U) saw_home = 1;

            if (v == 1U && (saw_home || HAL_GetTick() - t0 > 300U)) return 1;

        }

        HAL_Delay(50);

    }



    mks_stop();

    return 0;

}



int mks_move_abs(int rpm, int target)

{

    return rpm >= 1 && rpm <= 3000 &&

           write_axis(C_ABS, (uint16_t)rpm, MOVE_ACC, target);

}



int mks_move_rel(int rpm, int delta)

{

    return rpm >= 1 && rpm <= 3000 &&

           write_axis(C_REL, (uint16_t)rpm, MOVE_ACC, delta);

}



int mks_stop(void)

{

    return write_axis(C_ABS, 0, MOVE_ACC, 0);

}



/* ===== main에서 부르는 진입점 ===== */



void rot_test_init(void)

{

    tcp_init();

    save_init();

}



void rot_test_run(void)

{

    tcp_run();

    save_run();

}
