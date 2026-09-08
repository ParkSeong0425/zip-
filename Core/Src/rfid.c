#include <stdio.h>
#include <string.h>
#include "rfid.h"
#include "can.h"
#include "net.h"

#define RFID_CAN_ID 201
#define POLL_MS     200
#define WAIT_MS     200

volatile int card_ok;
static volatile int show_req;
static uint8_t uid[4];
extern volatile int event_number;

static void send_event(int on)
{
    char b[32];
    if (on)
        snprintf(b, sizeof(b), "%02dUT_1_%02X%02X%02X%02X", event_number,
                uid[0], uid[1], uid[2], uid[3]);
    else
        snprintf(b, sizeof(b), "%02dUU_1_00000000", event_number);
    send_to_tcp_queue(b);
}

/* R 명령: 현재 UID를 한 번 응답한다 */
void Rfid_Request(void)
{
    show_req = 1;
}

/* RFID 보드에 UID를 물어본다 */
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
    if (!frame.Data[0] && !frame.Data[1] && !frame.Data[2] && !frame.Data[3])
        return 0;

    memcpy(uid, frame.Data, 4);
    return 1;
}

/* 카드 변화는 자동 이벤트 1회, R 요청은 요청 번호로 1회 응답 */
void Rfid_Run(void)
{
    char b[32];
    int seen, miss = 0;

    for (;;) {
        seen = ask();

        if (seen) {
            miss = 0;
            if (!card_ok) {
                card_ok = 1;
                send_event(1);
            }
        } else if (++miss >= 3 && card_ok) {
            card_ok = 0;
            send_event(0);
        }

        if (show_req) {
            show_req = 0;
            if (card_ok)
                snprintf(b, sizeof(b), "01UT_1_%02X%02X%02X%02X\r\n",
                        uid[0], uid[1], uid[2], uid[3]);
            else
                snprintf(b, sizeof(b), "01UU_1_00000000\r\n");
            reply(b);
        }

        osDelay(POLL_MS);
    }
}
