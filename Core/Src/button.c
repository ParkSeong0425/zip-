/* button.c: 버튼, 램프, 만재/RFID STOP */
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

static int motor_alarm_on(void);

#define BUTTON_COUNT 5

volatile int run;
volatile int pause;   /* 기존 move.c와 공유하므로 이름 유지 */
volatile int estop;

static volatile char stop_reason; /* M=버튼 F=만재 S=RFID A=알람 E=ESTOP */
static int manjae_mask, last_manjae;
static uint8_t start_count, stop_count;
static int start_pressed, stop_pressed, estop_input;
static int tcp_mode[2], tcp_left[2], tcp_on;

/* ===== 만재 ===== */
static int manjae_now(void) {
    return (HAL_GPIO_ReadPin(F1_GPIO_Port, F1_Pin) ? 0 : 1)
         | (HAL_GPIO_ReadPin(F2_GPIO_Port, F2_Pin) ? 0 : 2)
         | (HAL_GPIO_ReadPin(F3_GPIO_Port, F3_Pin) ? 0 : 4)
         | (HAL_GPIO_ReadPin(F4_GPIO_Port, F4_Pin) ? 0 : 8);
}

/* 만재가 바뀔 때만 FF 1회 */
static void manjae_event_run(void) {
    int manjae = manjae_now();
    char b[32];

    if (manjae == last_manjae) return;

    last_manjae = manjae;
    snprintf(b, sizeof(b), "%02dFF_1_1_%d%d%d%d", event_number,
            !!(manjae & 1), !!(manjae & 2),
            !!(manjae & 4), !!(manjae & 8));
    send_to_tcp_queue(b);
}

/* ===== 버튼 읽기 ===== */
void button_50ms(GPIO_TypeDef *port, uint16_t pin, uint8_t *count) {
    if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) {
        *count = 0;
        return;
    }
    if (*count < BUTTON_COUNT + 1) (*count)++;
}

static void button_read(void) {
	// 버튼 누르면 50ms 이상 눌러야 인정 되는 기능
    button_50ms(MOTOR_START_btn_GPIO_Port, MOTOR_START_btn_Pin, &start_count);
    button_50ms(STOP_btn_GPIO_Port, STOP_btn_Pin, &stop_count);

    // 알람 발생시 긴급정지 누르면 시작이랑 일시정지 둘다 누를때 인식이 되도록
    start_pressed = start_count == BUTTON_COUNT;
    stop_pressed = stop_count == BUTTON_COUNT;
    estop_input = HAL_GPIO_ReadPin(ESTOP_btn_GPIO_Port, ESTOP_btn_Pin);
}

/* ===== 램프 ===== */
void lamp_ready_start(void) {
    HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_RESET);
}

/* ESTOP 해제 후 PG1 대기 */
static void lamp_wait_start(void) {
    HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_RESET);
}

/* RFID 없음: START 전체 주기 약 240ms */
void lamp_no_rfid(void) {
    GPIO_PinState on = ((HAL_GetTick() / 120) % 2)
            ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, on);
}

/* STOP 또는 만재: STOP 램프 전체 주기 약 500ms */
static void lamp_stop(void) {
    GPIO_PinState on = ((HAL_GetTick() / 250) % 2)
            ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, on);
}

/* ESTOP/HOME/ALARM 공용 점멸 */
void lamp_blink(char mode, int tick, int motor) {
    GPIO_PinState on = ((HAL_GetTick() / tick) % 2)
            ? GPIO_PIN_SET : GPIO_PIN_RESET;
    (void)mode;

    HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, on);
    HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, on);
    HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, on);
    HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, on);
    HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin,
            motor ? on : GPIO_PIN_SET);
}

void lamp_estop(void) { lamp_blink('E', 120, 0); }
void lamp_home(void)  { lamp_blink('I', 250, 0); }
void lamp_alarm(void) { lamp_blink('A', 120, 1); }

/* TCP RED/GREEN 램프 */
static void lamp_tcp(void) {
    int i;
    int blink = ((HAL_GetTick() / 250) % 2);
    GPIO_PinState red = GPIO_PIN_RESET;
    GPIO_PinState green = GPIO_PIN_RESET;

    for (i = 0; i < 2; i++)
        if (tcp_left[i] > 0 && --tcp_left[i] == 0)
            tcp_mode[i] = 0;

    if (tcp_mode[0] == 1 || (tcp_mode[0] == 2 && blink))
        red = GPIO_PIN_SET;
    if (tcp_mode[1] == 1 || (tcp_mode[1] == 2 && blink))
        green = GPIO_PIN_SET;

    HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, red);
    HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, green);
    tcp_on = tcp_mode[0] || tcp_mode[1];
}

void lamp_cmd(char *s) {
    int c1,id1,m1,t1,c2,id2,m2,t2,p1,p2;
    char b[64];

    if (sscanf(s, "01LL_%d_%d_%1d%d&L_%d_%d_%1d%d",
            &c1,&id1,&m1,&t1,&c2,&id2,&m2,&t2) != 8
            || c1 != 1 || c2 != 1 || id1 < 1 || id1 > 9 || id2 < 1 || id2 > 9
            || ((id1 & 1) == (id2 & 1)) || m1 < 0 || m1 > 2 || m2 < 0 || m2 > 2
            || t1 < 0 || t1 > 60 || t2 < 0 || t2 > 60) {
        snprintf(b, sizeof(b), "01%cL_bad_data", NAK);
        reply(b);
        return;
    }

    p1 = (id1 & 1) ? 0 : 1;
    p2 = (id2 & 1) ? 0 : 1;
    tcp_mode[p1] = m1;
    tcp_mode[p2] = m2;
    tcp_left[p1] = m1 ? (t1 ? t1 * 100 : -1) : 0;
    tcp_left[p2] = m2 ? (t2 ? t2 * 100 : -1) : 0;
    tcp_on = tcp_mode[0] || tcp_mode[1];

    snprintf(b, sizeof(b), "01%c%s", ACK, s + 2);
    reply(b);
}

/* 램프 상태 선택 */
void lamp_run(char state, int card) {
    /* ESTOP 해제 후 복구 버튼 대기 */
    if (estop == 0 && run == 0 && stop_reason == 'E') {
        if (motor_alarm_on()) lamp_alarm();
        else lamp_wait_start();
        return;
    }

    if (estop) {
        lamp_estop();
        return;
    }

    if (state == 'A') {
        lamp_alarm();
        return;
    }

    if (state == 'I') {
        lamp_home();
        return;
    }

    lamp_ready_start();

    if (card == 0)
        lamp_no_rfid();

    if (pause || manjae_now())
        lamp_stop();

    if (tcp_on)
        lamp_tcp();
}

/* ===== STOP ===== */
static void stop_on(char why) {
    pause = 1;
    run = 0;
    stop_reason = why;
    pause_msg(why);
}

static void stop_off(void) {
    pause = 0;
    run = 1;
    stop_reason = 0;
    manjae_mask = 0;
    pause_msg(0);
}

/* PO에서만 만재/RFID를 실제 STOP */
static void stop_po(int mask) {
    manjae_mask = mask;

    if (manjae_now() & mask) {
        stop_on('F');
        return;
    }

    if (card_ok == 0)
        stop_on('S');
}

/* PO가 만재/RFID 때문에 멈췄을 때 해제 확인 */
static int stop_sensor_run(void) {
    /* 만재 때문에 멈춤 */
    if (stop_reason == 'F') {
        if (manjae_now() & manjae_mask)
            return 1;

        if (card_ok == 0) {
            stop_reason = 'S';
            pause_msg('S');
            return 1;
        }

        stop_off();
        return 1;
    }

    /* RFID 때문에 멈춤 */
    if (stop_reason == 'S') {
        if (card_ok == 0)
            return 1;

        if (manjae_now() & manjae_mask) {
            stop_reason = 'F';
            pause_msg('F');
            return 1;
        }

        stop_off();
        return 1;
    }

    return 0;
}

/* 일반 STOP 버튼: PF9 정지, PG1 해제 */
static void stop_button_run(void) {
    if (stop_reason == 'M') {
        if (start_pressed) {
            stop_off();
            print("START\r\n");
        }
        return;
    }

    if (pause == 0 && stop_pressed && start_pressed == 0) {
        stop_on('M');
        print("PAUSE\r\n");
    }
}

/* ===== ESTOP / ALARM ===== */
static int motor_alarm_on(void) {
    int alarm = alarm_get();

    if (alarm == 0) return 0;
    if (alarm == 9) return 0;
    return 1;
}

/* ESTOP 눌림, 해제, 원점복귀 */
static int estop_run(void) {
    /* ESTOP 눌림 */
    if (estop_input == 1 && estop == 0) {
        estop = 1;
        run = 0;
        pause = 0;
        pause_msg(0);

        if (motor_alarm_on()) {
            stop_reason = 'A';
            print("ALARM ESTOP\r\n");
        } else {
            stop_reason = 'E';
            alarm_set(9);
            print("ESTOP\r\n");
        }

        lamp_estop();
        return 1;
    }

    /* ESTOP 해제 */
    if (estop_input == 0 && estop == 1) {
        estop = 0;
        run = 0;
        pause = 0;
        pause_msg(0);
        stop_reason = 'E';

        if (motor_alarm_on()) {
            lamp_alarm();
            print("ALARM ESTOP CLEAR READY\r\n");
        } else {
            lamp_wait_start();
            print("ESTOP CLEAR READY\r\n");
        }
        return 1;
    }

    /* ESTOP 누른 상태에서는 다른 버튼 처리 안 함 */
    if (estop)
        return 1;

    /* ESTOP 해제 후 원점복귀 대기 */
    if (stop_reason == 'E') {
        /* 알람 중 ESTOP이면 alarm_run에서 PG1 + PF9 처리 */
        if (motor_alarm_on())
            return 0;

        /* 일반 ESTOP이면 PG1 */
        if (start_pressed) {
            run = 1;
            pause = 0;
            stop_reason = 0;
            lamp_home();
            net_cmd("00I");
            print("ESTOP CLEAR HOME\r\n");
        }
        return 1;
    }

    return 0;
}

/* 알람 복구 */
static int alarm_run(void)
{
    /* 알람 상태에서 ESTOP까지 했다가 해제한 경우 */
	/* 알람 상태에서 ESTOP까지 했다가 해제한 경우 */
	if (stop_reason == 'E' && motor_alarm_on()) {

	    /* PG1을 누르고 있는 상태에서 PF9를 새로 누르면 HOME */
	    if (start_count >= BUTTON_COUNT && stop_pressed) {

	        run = 1;
	        pause = 0;
	        stop_reason = 0;

	        lamp_home();
	        net_cmd("00I");

	        print("ALARM ESTOP CLEAR HOME\r\n");
	    }

	    return 1;
	}

    /* 모든 일반 알람 */
    if (motor_alarm_on()) {
        stop_reason = 'A';
        run = 0;
        pause = 0;
        lamp_alarm();

        /* 알람 상태에서는 PG1 누르면 원점복귀 */
        if (start_pressed) {
            run = 1;
            pause = 0;
            stop_reason = 0;

            lamp_home();
            net_cmd("00I");

            print("ALARM CLEAR HOME\r\n");
        }

        return 1;
    }

    return 0;
}

/* move.c에서 이동 중 ESTOP/ALARM 확인용 */
int button_stop_requested(void) {
    if (estop) return 1;
    if (alarm_get()) return 1;
    return 0;
}

/* ===== 시작 ===== */
void button_init(void) {
    run = 1;
    pause = 0;
    manjae_mask = 0;
    last_manjae = 0;
    stop_reason = 0;

    // 전원 켰을때 긴급정지가 눌려 있을 수 있으니까
    estop = HAL_GPIO_ReadPin(ESTOP_btn_GPIO_Port, ESTOP_btn_Pin);
    if (estop) {
        stop_reason = 'E';
        alarm_set(9);
        lamp_estop();
        return;
    }

    lamp_ready_start();
}

void button_run(void) {
    button_read();
    manjae_event_run();

    if (estop_run()) return;
    if (alarm_run()) return;
    if (stop_sensor_run()) return;

    stop_button_run();
}

/* ===== 기존 move.c / net.c 호출명 호환 ===== */
int full_read(void) { return manjae_now(); }
int full_get(void)  { return manjae_now(); }

void lamp_pause(void) { lamp_stop(); }

void pause_on(char why) { stop_on(why); }
void pause_full(int mask) {
    manjae_mask = mask;
    stop_on('F');
}
void pause_po(int mask) { stop_po(mask); }
