/* button.c: 버튼, 만재/RFID 램프, PO 대기 상태만 처리한다. */
#include "button.h"
#include "net.h"
#include "motor.h"
#include "move.h"
#include "rot_test.h"
#include <stdio.h>
int alarm_get(void);
void alarm_set(int code);
extern volatile int card_ok;
extern volatile int event_number;
#define BUTTON_COUNT 5
#define TICK_FAST    12   /* 240ms */
#define TICK_SLOW    25   /* 500ms */
volatile int run;
volatile int pause;
volatile int estop;
static int lamp_count, rfid_lamp_count;
static char lamp_mode;
static int tcp_mode[2], tcp_left[2], tcp_count, tcp_blink = 1, tcp_on;
static volatile char pause_reason; /* M=버튼, F=만재, S=RFID, A=알람, E=ESTOP */
static int full_mask;
static int last_full;
static int alarm_estop;      /* 알람 중 ESTOP을 눌렀으면 1 */
/* F1~F4 현재 만재 입력 */
static int full_now(void)
{
    return (HAL_GPIO_ReadPin(F1_GPIO_Port, F1_Pin) ? 0 : 1)
         | (HAL_GPIO_ReadPin(F2_GPIO_Port, F2_Pin) ? 0 : 2)
         | (HAL_GPIO_ReadPin(F3_GPIO_Port, F3_Pin) ? 0 : 4)
         | (HAL_GPIO_ReadPin(F4_GPIO_Port, F4_Pin) ? 0 : 8);
}
int full_read(void) { return full_now(); }
int full_get(void)  { return full_now(); }
static void send_full_event(int full)
{
    char b[32];
    snprintf(b, sizeof(b), "%02dFF_1_1_%d%d%d%d", event_number,
            !!(full & 1), !!(full & 2), !!(full & 4), !!(full & 8));
    send_to_tcp_queue(b);
}
void button_50ms(GPIO_TypeDef *port, uint16_t pin, uint8_t *count)
{
    if (HAL_GPIO_ReadPin(port, pin) != GPIO_PIN_RESET) { *count = 0; return; }
    if (*count < BUTTON_COUNT + 1) (*count)++;
}
void lamp_ready_start(void)
{
    lamp_mode = 'W'; lamp_count = 0; rfid_lamp_count = 0;
    HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_RESET);
}
/* RFID 없음: START 240ms 점멸 */
void lamp_ni_rfid(void)
{
    if (++rfid_lamp_count < TICK_FAST) return;
    rfid_lamp_count = 0;
    HAL_GPIO_TogglePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin);
}

/* PAUSE 또는 만재: STOP 500ms 점멸 */
void lamp_pause(void)
{
    if (++lamp_count < TICK_SLOW) return;
    lamp_count = 0;
    HAL_GPIO_TogglePin(STOP_GPIO_Port, STOP_Pin);
}
void lamp_blink(char mode, int tick, int motor)
{
    if (lamp_mode != mode) {
        lamp_mode = mode; lamp_count = 0;
        HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_SET);
        return;
    }
    if (++lamp_count < tick) return;
    lamp_count = 0;
    HAL_GPIO_TogglePin(STOP_GPIO_Port, STOP_Pin);
    HAL_GPIO_TogglePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin);
    HAL_GPIO_TogglePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin);
    HAL_GPIO_TogglePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin);
    if (motor) HAL_GPIO_TogglePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin);
}
void lamp_estop(void) { lamp_blink('E', TICK_FAST, 0); }
void lamp_home(void)  { lamp_blink('I', TICK_SLOW, 0); }
void lamp_alarm(void) { lamp_blink('A', TICK_FAST, 1); }
static void lamp_tcp(void)
{
    int i;
    if (++tcp_count >= TICK_SLOW) { tcp_count = 0; tcp_blink = !tcp_blink; }
    for (i = 0; i < 2; i++) {
        if (tcp_left[i] > 0 && --tcp_left[i] == 0) tcp_mode[i] = 0;
    }
    HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin,
            tcp_mode[0] == 1 || (tcp_mode[0] == 2 && tcp_blink) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin,
            tcp_mode[1] == 1 || (tcp_mode[1] == 2 && tcp_blink) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    tcp_on = tcp_mode[0] || tcp_mode[1];
}
void lamp_cmd(char *s)
{
    int c1, id1, m1, t1, c2, id2, m2, t2, p1, p2;
    char b[64];
    if (sscanf(s, "01LL_%d_%d_%1d%d&L_%d_%d_%1d%d",
            &c1, &id1, &m1, &t1, &c2, &id2, &m2, &t2) != 8
            || c1 != 1 || c2 != 1 || id1 < 1 || id1 > 9 || id2 < 1 || id2 > 9
            || ((id1 & 1) == (id2 & 1)) || m1 < 0 || m1 > 2 || m2 < 0 || m2 > 2
            || t1 < 0 || t1 > 60 || t2 < 0 || t2 > 60) {
        snprintf(b, sizeof(b), "01%cL_bad_data", NAK); reply(b); return;
    }
    p1 = (id1 & 1) ? 0 : 1; p2 = (id2 & 1) ? 0 : 1;
    tcp_mode[p1] = m1; tcp_mode[p2] = m2;
    tcp_left[p1] = m1 ? (t1 ? t1 * 100 : -1) : 0;
    tcp_left[p2] = m2 ? (t2 ? t2 * 100 : -1) : 0;
    tcp_count = 0; tcp_blink = 1; tcp_on = tcp_mode[0] || tcp_mode[1];
    HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, tcp_mode[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, tcp_mode[1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    snprintf(b, sizeof(b), "01%c%s", ACK, s + 2); reply(b);
}
void lamp_run(char state, int card)
{
    int full = full_now();

    /* 일반 ESTOP 해제 후에는 START 램프 상태 유지 */
    if (estop == 0 && run == 0 && pause_reason == 'E') return;

    if (estop) lamp_estop();
    else if (state == 'A') lamp_alarm();
    else if (state == 'I') lamp_home();
    else if (!card && (pause || full)) {
        if (lamp_mode != 'S') {
            lamp_mode = 'S'; lamp_count = 0; rfid_lamp_count = 0;
            HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
        }
        lamp_ni_rfid(); lamp_pause();
    }
    else if (!card) {
        if (lamp_mode != 's') {
            lamp_mode = 's'; rfid_lamp_count = 0;
            HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
        }
        lamp_ni_rfid();
    }
    else if (pause || full) {
        if (lamp_mode != 'P') {
            lamp_mode = 'P'; lamp_count = 0;
            HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
        }
        lamp_pause();
    }
    else lamp_ready_start();

    if (estop == 0 && state != 'A' && tcp_on) lamp_tcp();
}
void button_init(void)
{
    run = 1; pause = 0; full_mask = 0; last_full = 0; alarm_estop = 0;
    estop = HAL_GPIO_ReadPin(ESTOP_btn_GPIO_Port, ESTOP_btn_Pin);
    if (estop) { pause_reason = 'E'; alarm_set(9); lamp_estop(); }
    else lamp_ready_start();
}

void pause_on(char why)
{
    pause = 1; run = 0; pause_reason = why;
    lamp_pause(); pause_msg(why);
}

void pause_full(int mask) { full_mask = mask; pause_on('F'); }

/* PO에서만 만재/RFID를 실제 PAUSE로 만든다 */
void pause_po(int mask)
{
    full_mask = mask;
    if (full_now() & mask) pause_on('F');
    else if (!card_ok) pause_on('S');
}

static void pause_off(void)
{
    pause = 0; run = 1; pause_reason = 0; full_mask = 0;
    pause_msg(0); lamp_ready_start();
}

int button_stop_requested(void) { return estop || alarm_get(); }

/* ESTOP 해제 후 PG1을 기다리는 램프 */
static void lamp_wait_start(void)
{
    lamp_mode = 'E'; lamp_count = 0; rfid_lamp_count = 0;
    HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_RESET);
}

/* ESTOP 눌림 */
static void estop_on(void)
{
    int code = alarm_get();
    estop = 1; run = 0; pause = 0; pause_msg(0);

    if (code == 0 || code == 9) {
        alarm_estop = 0; pause_reason = 'E'; print("ESTOP\r\n");
    } else {
        alarm_estop = 1; pause_reason = 'A'; print("ALARM ESTOP\r\n");
    }

    alarm_set(9); lamp_estop();
}

/* ESTOP 해제 */
static void estop_off(void)
{
    estop = 0; run = 0; pause = 0; pause_msg(0);

    if (alarm_estop) {
        pause_reason = 'A'; lamp_alarm();
        print("ALARM ESTOP CLEAR READY\r\n");
    } else {
        pause_reason = 'E'; lamp_wait_start();
        print("ESTOP CLEAR READY\r\n");
    }
}

/* 일반 ESTOP: 해제 후 PG1 -> 원점복귀 */
static int estop_home(uint8_t start_count, uint8_t pause_count)
{
    if (pause_reason == 'E') {
        if (start_count == BUTTON_COUNT && pause_count == 0) {
            run = 1; pause = 0; pause_reason = 0;
            lamp_home(); net_cmd("00I"); print("ESTOP CLEAR HOME\r\n");
        }
        return 1;
    }
    return 0;
}

/* 알람: 일반은 PF9, 알람+ESTOP은 ESTOP 해제 후 PG1+PF9 */
static int alarm_run(uint8_t start_count, uint8_t pause_count)
{
    int code = alarm_get();

    if (code > 0) {
        if (code == 9) {
            /* ESTOP 알람은 E/A 상태에서 따로 처리 */
        } else if (pause_reason == 'A') {
            /* 이미 알람 상태 */
        } else {
            pause_reason = 'A'; run = 0; pause = 0;
        }
    }

    if (pause_reason == 'A') {
        if (alarm_estop) {
            if (start_count >= BUTTON_COUNT && pause_count >= BUTTON_COUNT) {
                alarm_estop = 0; run = 1; pause = 0; pause_reason = 0;
                lamp_home(); net_cmd("00I"); print("ALARM ESTOP CLEAR HOME\r\n");
            }
            return 1;
        }

        if (pause_count == BUTTON_COUNT && start_count == 0) {
            run = 1; pause = 0; pause_reason = 0;
            lamp_home(); net_cmd("00I"); print("ALARM CLEAR HOME\r\n");
        }
        return 1;
    }
    return 0;
}

/* PO 만재/RFID 대기 해제 */
static int sensor_pause_run(void)
{
    int full = full_now();

    if (pause && (pause_reason == 'F' || pause_reason == 'S')) {
        if ((full & full_mask) || !card_ok) {
            char next = (full & full_mask) ? 'F' : 'S';
            if (pause_reason == next) return 1;
            pause_reason = next; pause_msg(next);
            return 1;
        }
        pause_off();
        return 1;
    }
    return 0;
}

/* 일반 PAUSE: PF9로 정지, PG1로 해제 */
static void pause_run(uint8_t start_count, uint8_t pause_count)
{
    if (pause && pause_reason == 'M') {
        if (start_count == BUTTON_COUNT && pause_count == 0) {
            pause_off(); print("START\r\n");
        }
        return;
    }

    if (pause == 0 && pause_count == BUTTON_COUNT && start_count == 0) {
        pause_on('M'); print("PAUSE\r\n");
    }
}

void button_run(void)
{
    static uint8_t start_count, pause_count;
    int now = HAL_GPIO_ReadPin(ESTOP_btn_GPIO_Port, ESTOP_btn_Pin);
    int full = full_now();

    button_50ms(MOTOR_START_btn_GPIO_Port, MOTOR_START_btn_Pin, &start_count);
    button_50ms(STOP_btn_GPIO_Port, STOP_btn_Pin, &pause_count);

    if (full == last_full) {
        /* 만재 변화 없음 */
    } else {
        last_full = full; send_full_event(full);
    }

    if (now == 1 && estop == 0) { estop_on(); return; }
    if (now == 0 && estop == 1) { estop_off(); return; }
    if (estop == 1) return;

    if (alarm_run(start_count, pause_count)) return;
    if (estop_home(start_count, pause_count)) return;
    if (sensor_pause_run()) return;

    pause_run(start_count, pause_count);
}
