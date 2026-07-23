/*
 * button.c
 *
 *  Created on: Jul 22, 2026
 *      Author: kotec
 */
#include "button.h"
#include "save.h"
#include "item.h"
#include "motor.h"

#define DEBOUNCE 200U

volatile int run;
volatile int pause;
volatile int estop;

static volatile int start_req;
static volatile int pause_req;
static volatile int estop_req;


static void lamp_idle(void)
{
    HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_SET);
}

static void lamp_run(void)
{
    HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_SET);
}

static void lamp_pause(void)
{
    HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_RESET);
}

static void lamp_estop(void)
{
    HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_RESET);
}

void button_init(void)
{
    run = 0;
    pause = 0;
    estop = HAL_GPIO_ReadPin(ESTOP_btn_GPIO_Port, ESTOP_btn_Pin);
    start_req = pause_req = estop_req = 0;

    HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET);
    lamp_idle();
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    static uint32_t last[3];
    uint32_t now = HAL_GetTick();

    if (pin == MOTOR_START_btn_Pin) {
        if (now - last[0] < DEBOUNCE) return;
        last[0] = now;
        if (!estop) start_req = 1;
        return;
    }

    if (pin == STOP_btn_Pin) {
        if (now - last[1] < DEBOUNCE) return;
        last[1] = now;
        pause = 1;
        run = 0;
        pause_req = 1;
        lamp_pause();
        return;
    }

    if (pin == ESTOP_btn_Pin) {
        if (now - last[2] < DEBOUNCE) return;
        last[2] = now;
        estop = HAL_GPIO_ReadPin(ESTOP_btn_GPIO_Port, ESTOP_btn_Pin);

        if (estop) {
            run = 0;
            pause = 0;
            estop_req = 1;
            lamp_estop();
        } else {
            lamp_idle();
        }
    }
}

/* 블로킹 홈 동작에서도 STOP/ESTOP을 확인한다 */
int button_stop_requested(void)
{
    return pause || estop;
}

void button_run(void)
{
    if (estop_req) {
        estop_req = 0;
        item_auto_stop();
        save_abort();
        print("ESTOP\r\n");
        return;
    }

    if (estop) return;

    if (run && !item_auto_on() && !save_busy()) {
        run = 0;
        lamp_idle();
    }

    if (pause_req) {
        pause_req = 0;
        if (save_pause()) print("BUTTON PAUSE\r\n");
        return;
    }

    if (!start_req) return;
    start_req = 0;

    if (pause || save_paused()) {
        if (save_resume()) {
            pause = 0;
            run = 1;
            lamp_run();
            print("BUTTON RESUME\r\n");
            return;
        }
        /* 홈 도중 멈춘 경우에는 홈부터 다시 시작한다 */
        pause = 0;
    }

    /* 첫 시작 때 X/Y가 홈이 아니면 각 센서로 원점부터 잡는다 */
    if (!save_ready() && !save_home()) {
        run = 0;
        lamp_pause();
        print("BUTTON HOME FAIL\r\n");
        return;
    }

    /* TCP의 auto 명령과 같은 자동 랜덤 운전을 시작한다 */
    if (!item_auto_start()) {
        run = 0;
        lamp_pause();
        print("BUTTON AUTO FAIL\r\n");
        return;
    }

    pause = 0;
    run = 1;
    lamp_run();
    print("BUTTON AUTO START\r\n");
}

