#include <stdio.h>
#include <string.h>
#include "rfid.h"
#include "can.h"
#include "net.h"

#define RFID_CAN_ID 201
#define POLL_MS     500   /* 카드 확인 주기 */
#define WAIT_MS     200   /* 응답 대기 */

volatile int card_ok;          /* 카드가 있으면 1 */

static volatile int show_req;  /* 02R 이 오면 UID 를 한 번 보낸다 */
static uint8_t uid[4];

/* 02R: 지금 읽힌 UID 를 보여준다 */
void Rfid_Request(void)
{
    show_req = 1;
}

/* RFID 보드에 UID 를 물어본다. 카드가 있으면 1 */
static int ask(void)
{
    CanFrame frame;
    uint8_t cmd = 'C';

    osMessageQueueReset(CanQueue);
    CAN1_Tx_Data(RFID_CAN_ID, &cmd, 1);

    if (osMessageQueueGet(CanQueue, &frame, NULL, WAIT_MS) != osOK)
        return 0;

    if (frame.Id != RFID_CAN_ID || frame.Len < 4)
        return 0;

    if (!frame.Data[0] && !frame.Data[1]
            && !frame.Data[2] && !frame.Data[3])
        return 0;

    memcpy(uid, frame.Data, 4);
    return 1;
}

/* 카드가 있는지 계속 확인한다 */
void Rfid_Run(void)
{
    char b[32];

    for (;;) {
        card_ok = ask();

        if (show_req) {
            show_req = 0;

            if (card_ok)
                snprintf(b, sizeof(b),
                        "02UT_1_%02X%02X%02X%02X\r\n",
                        uid[0], uid[1], uid[2], uid[3]);
            else
                snprintf(b, sizeof(b), "02UU_1_00000000\r\n");

            reply(b);
        }

        osDelay(POLL_MS);
    }
}
