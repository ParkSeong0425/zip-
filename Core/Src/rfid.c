#include <stdio.h>
#include "rfid.h"
#include "can.h"
#include "net.h"

#define RFID_CAN_ID 201

static volatile uint8_t can_check;

volatile uint8_t rfid_check; // RFID에 카드가 읽혔는지 확인

/* 02R을 받으면 RFID 보드에 현재 UID 요청 */
void Rfid_Request(void)
{
    uint8_t cmd = 'C';

    osMessageQueueReset(CanQueue);
    can_check = 1;
    CAN1_Tx_Data(RFID_CAN_ID, &cmd, 1);
}

/* 02R 요청 후 최대 5초 동안 UID 확인 */
void Rfid_Run(void)
{
    CanFrame frame;
    char b[32];
    int i;

    for (;;) {
        if (!can_check) {
            osDelay(10);
            continue;
        }

        /* 100ms × 50회 = 최대 약 5초 */
        for (i = 0; i < 50; i++) {
            if (osMessageQueueGet(
                    CanQueue, &frame, NULL, 100) != osOK)
                continue;

            if (frame.Id != RFID_CAN_ID || frame.Len < 4)
                continue;

            if (frame.Data[0] || frame.Data[1]
                    || frame.Data[2] || frame.Data[3]) {
                snprintf(b, sizeof(b),
                        "02UT_1_%02X%02X%02X%02X\r\n",
                        frame.Data[0], frame.Data[1],
                        frame.Data[2], frame.Data[3]);

                reply(b);
                break;
            }
        }

        /* UID 결과 저장 */
        rfid_check = i < 50;

        if (!rfid_check)
            reply("02UU_1_00000000\r\n");

        /* 이번 02R 처리 종료 */
        can_check = 0;
    }
}
