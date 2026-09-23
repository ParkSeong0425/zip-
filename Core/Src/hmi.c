/*
 * hmi.c
 */

#include "hmi.h"
#include "run.h"
#include "cli.h"
#include "usart.h"
#include <stdint.h>

#define STX     0x02
#define ETX     0x03


static void HMI_Command(int button)
{
    switch (button)
    {
    case BTN_RUN:
        CLI_Print("HMI RUN\r\n");
        break;

    case BTN_STOP:
        S();
        CLI_Print("HMI STOP\r\n");
        break;

    case BTN_ESTOP:
        CLI_Print("HMI ESTOP\r\n");
        break;

    case BTN_TILT_Lift:
        CLI_Print("HMI TILT LEFT\r\n");
        break;

    case BTN_TILT_Right:
        CLI_Print("HMI TILT RIGHT\r\n");
        break;

    case BTN_LIFT_UP:
        CLI_Print("HMI LIFT UP\r\n");
        break;

    case BTN_LIFT_Down:
        CLI_Print("HMI LIFT DOWN\r\n");
        break;

    case BTN_TRAVEL_Forward:
        CLI_Print("HMI TRAVEL FORWARD\r\n");
        break;

    case BTN_TRAVEL_Back:
        CLI_Print("HMI TRAVEL BACK\r\n");
        break;
    }
}


void HMI_Run(void)
{
    uint8_t tx[27];
    uint8_t rx[4];
    int i;

    tx[0] = STX;
    tx[1] = '1';
    tx[2] = 'C';

    tx[3] = '1';     /* WAIT */
    tx[4] = '0';     /* Pause */
    tx[5] = '0';     /* Alarm */

    for (i = 6; i < 26; i++)
        tx[i] = '0';

    tx[26] = ETX;

    /* STM -> HMI */
    HAL_UART_Transmit(&huart4, tx, 27, 100);

    /* HMI -> STM */
    if (HAL_UART_Receive(&huart4, rx, 4, 100) != HAL_OK)
        return;

    if (rx[0] != STX ||
        rx[1] != '1' ||
        rx[3] != ETX)
        return;

    HMI_Command(rx[2] - 0x30);
}
