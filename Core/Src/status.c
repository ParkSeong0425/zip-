/*
 * status.c
 *
 *  Created on: Sep 17, 2026
 *      Author: HWNOT
 */
#include "status.h"

/* 전원 켜지면 둘 다 대기 */
char status_1 = WAIT;
char status_2 = WAIT;

/* 만재 : 아직 미연결 */
int full_1_1 = 0;
int full_1_2 = 0;
int full_1_3 = 0;
int full_1_4 = 0;

int full_2_1 = 0;
int full_2_2 = 0;
int full_2_3 = 0;
int full_2_4 = 0;


/* 명령 시작 : 해당 렉 운전 */
void Status_Start(int rack)
{
    if (rack == 1) status_1 = RUN;
    else           status_2 = RUN;
}


/* 명령 끝 : MO 는 도착해도 운전 유지, MI / PO 는 대기 */
void Status_End(int command, int rack)
{
    if (command == CMD_MO)
        return;

    if (rack == 1) status_1 = WAIT;
    else           status_2 = WAIT;
}
