#include "save.h"
#include "fram.h"
#include <string.h>

#define SAVE_ADDR       0
#define SAVE_MAGIC      0x0425
#define SAVE_VERSION    5

SAVE_Data save;


/* 네트워크 기본값 */
static void Network_Default(void)
{
    save.fi.ip[0] = 172;
    save.fi.ip[1] = 20;
    save.fi.ip[2] = 0;
    save.fi.ip[3] = 1;

    save.fi.sn[0] = 255;
    save.fi.sn[1] = 255;
    save.fi.sn[2] = 255;
    save.fi.sn[3] = 0;

    save.fi.gw[0] = 0;
    save.fi.gw[1] = 0;
    save.fi.gw[2] = 0;
    save.fi.gw[3] = 0;

    save.fi.dip[0] = 172;
    save.fi.dip[1] = 20;
    save.fi.dip[2] = 0;
    save.fi.dip[3] = 100;

    save.fi.port = 2500;
    save.fi.mode = 0;
    save.fi.timeout = 20000;
}

/* 전원 켤 때 한번 실행 */
int Save_Init(void)
{
    /* FRAM에 저장된 값을 가져온다 */
    if (Bring_FRAM(SAVE_ADDR, &save, sizeof(save))) {
        if (save.magic == SAVE_MAGIC &&
            save.version == SAVE_VERSION)
            return 1;
    }

    /* 정상 저장값이 없을 때만 기본값 설정 */
    memset(&save, 0, sizeof(save));

    save.magic = SAVE_MAGIC;
    save.version = SAVE_VERSION;

    Network_Default();

    return Send_FRAM(SAVE_ADDR, &save, sizeof(save));
}

/* 장비 / 위치 개수 저장 */
int FS(int rack, char type, int rack_channel,
       int in_x, int in_y,
       int left_x, int left_y,
       int right_x, int right_y)
{
    if (rack != 1)
        return 0;

    if (type != 'S' && type != 'I')
        return 0;

    if (rack_channel < 1 || rack_channel > 4)
        return 0;

    if (in_x != 1)
        return 0;

    if (in_y < 1 || in_y > 5)
        return 0;

    if (left_x < 1 || left_x > 9)
        return 0;

    if (left_y < 1 || left_y > 9)
        return 0;

    if (right_x < 1 || right_x > 9)
        return 0;

    if (right_y < 1 || right_y > 9)
        return 0;

    save.fs.rack = rack;
    save.fs.type = type;
    save.fs.rack_channel = rack_channel;

    save.fs.in_x = in_x;
    save.fs.in_y = in_y;

    save.fs.left_x = left_x;
    save.fs.left_y = left_y;

    save.fs.right_x = right_x;
    save.fs.right_y = right_y;

    /* FS에서 사용하지 않는 위치는 0 */
    if (in_y < 5) save.fp.in_y5 = 0;
    if (in_y < 4) save.fp.in_y4 = 0;
    if (in_y < 3) save.fp.in_y3 = 0;
    if (in_y < 2) save.fp.in_y2 = 0;

    if (left_x < 9) save.fp.left_x9 = 0;
    if (left_x < 8) save.fp.left_x8 = 0;
    if (left_x < 7) save.fp.left_x7 = 0;
    if (left_x < 6) save.fp.left_x6 = 0;
    if (left_x < 5) save.fp.left_x5 = 0;
    if (left_x < 4) save.fp.left_x4 = 0;
    if (left_x < 3) save.fp.left_x3 = 0;
    if (left_x < 2) save.fp.left_x2 = 0;

    if (left_y < 9) save.fp.left_y9 = 0;
    if (left_y < 8) save.fp.left_y8 = 0;
    if (left_y < 7) save.fp.left_y7 = 0;
    if (left_y < 6) save.fp.left_y6 = 0;
    if (left_y < 5) save.fp.left_y5 = 0;
    if (left_y < 4) save.fp.left_y4 = 0;
    if (left_y < 3) save.fp.left_y3 = 0;
    if (left_y < 2) save.fp.left_y2 = 0;

    if (right_x < 9) save.fp.right_x9 = 0;
    if (right_x < 8) save.fp.right_x8 = 0;
    if (right_x < 7) save.fp.right_x7 = 0;
    if (right_x < 6) save.fp.right_x6 = 0;
    if (right_x < 5) save.fp.right_x5 = 0;
    if (right_x < 4) save.fp.right_x4 = 0;
    if (right_x < 3) save.fp.right_x3 = 0;
    if (right_x < 2) save.fp.right_x2 = 0;

    if (right_y < 9) save.fp.right_y9 = 0;
    if (right_y < 8) save.fp.right_y8 = 0;
    if (right_y < 7) save.fp.right_y7 = 0;
    if (right_y < 6) save.fp.right_y6 = 0;
    if (right_y < 5) save.fp.right_y5 = 0;
    if (right_y < 4) save.fp.right_y4 = 0;
    if (right_y < 3) save.fp.right_y3 = 0;
    if (right_y < 2) save.fp.right_y2 = 0;

    return Send_FRAM(SAVE_ADDR, &save, sizeof(save));
}

/* 위치값 저장 */
int FP(int rack, int no, int value[], int count)
{
    rack = 1;

    switch (no)
    {
    case 1:     /* 입고 X */
        if (count != 1)
            return 0;

        save.fp.in_x = value[0];
        break;

    case 2:     /* 입고 Y */
        if (count != save.fs.in_y)
            return 0;

        save.fp.in_y1 = 0;
        save.fp.in_y2 = 0;
        save.fp.in_y3 = 0;
        save.fp.in_y4 = 0;
        save.fp.in_y5 = 0;

        if (count >= 1) save.fp.in_y1 = value[0];
        if (count >= 2) save.fp.in_y2 = value[1];
        if (count >= 3) save.fp.in_y3 = value[2];
        if (count >= 4) save.fp.in_y4 = value[3];
        if (count >= 5) save.fp.in_y5 = value[4];
        break;

    case 3:     /* Left X */
        if (count != save.fs.left_x)
            return 0;

        save.fp.left_x1 = 0;
        save.fp.left_x2 = 0;
        save.fp.left_x3 = 0;
        save.fp.left_x4 = 0;
        save.fp.left_x5 = 0;
        save.fp.left_x6 = 0;
        save.fp.left_x7 = 0;
        save.fp.left_x8 = 0;
        save.fp.left_x9 = 0;

        if (count >= 1) save.fp.left_x1 = value[0];
        if (count >= 2) save.fp.left_x2 = value[1];
        if (count >= 3) save.fp.left_x3 = value[2];
        if (count >= 4) save.fp.left_x4 = value[3];
        if (count >= 5) save.fp.left_x5 = value[4];
        if (count >= 6) save.fp.left_x6 = value[5];
        if (count >= 7) save.fp.left_x7 = value[6];
        if (count >= 8) save.fp.left_x8 = value[7];
        if (count >= 9) save.fp.left_x9 = value[8];
        break;

    case 4:     /* Left Y */
        if (count != save.fs.left_y)
            return 0;

        save.fp.left_y1 = 0;
        save.fp.left_y2 = 0;
        save.fp.left_y3 = 0;
        save.fp.left_y4 = 0;
        save.fp.left_y5 = 0;
        save.fp.left_y6 = 0;
        save.fp.left_y7 = 0;
        save.fp.left_y8 = 0;
        save.fp.left_y9 = 0;

        if (count >= 1) save.fp.left_y1 = value[0];
        if (count >= 2) save.fp.left_y2 = value[1];
        if (count >= 3) save.fp.left_y3 = value[2];
        if (count >= 4) save.fp.left_y4 = value[3];
        if (count >= 5) save.fp.left_y5 = value[4];
        if (count >= 6) save.fp.left_y6 = value[5];
        if (count >= 7) save.fp.left_y7 = value[6];
        if (count >= 8) save.fp.left_y8 = value[7];
        if (count >= 9) save.fp.left_y9 = value[8];
        break;

    case 5:     /* Right X */
        if (count != save.fs.right_x)
            return 0;

        save.fp.right_x1 = 0;
        save.fp.right_x2 = 0;
        save.fp.right_x3 = 0;
        save.fp.right_x4 = 0;
        save.fp.right_x5 = 0;
        save.fp.right_x6 = 0;
        save.fp.right_x7 = 0;
        save.fp.right_x8 = 0;
        save.fp.right_x9 = 0;

        if (count >= 1) save.fp.right_x1 = value[0];
        if (count >= 2) save.fp.right_x2 = value[1];
        if (count >= 3) save.fp.right_x3 = value[2];
        if (count >= 4) save.fp.right_x4 = value[3];
        if (count >= 5) save.fp.right_x5 = value[4];
        if (count >= 6) save.fp.right_x6 = value[5];
        if (count >= 7) save.fp.right_x7 = value[6];
        if (count >= 8) save.fp.right_x8 = value[7];
        if (count >= 9) save.fp.right_x9 = value[8];
        break;

    case 6:     /* Right Y */
        if (count != save.fs.right_y)
            return 0;

        save.fp.right_y1 = 0;
        save.fp.right_y2 = 0;
        save.fp.right_y3 = 0;
        save.fp.right_y4 = 0;
        save.fp.right_y5 = 0;
        save.fp.right_y6 = 0;
        save.fp.right_y7 = 0;
        save.fp.right_y8 = 0;
        save.fp.right_y9 = 0;

        if (count >= 1) save.fp.right_y1 = value[0];
        if (count >= 2) save.fp.right_y2 = value[1];
        if (count >= 3) save.fp.right_y3 = value[2];
        if (count >= 4) save.fp.right_y4 = value[3];
        if (count >= 5) save.fp.right_y5 = value[4];
        if (count >= 6) save.fp.right_y6 = value[5];
        if (count >= 7) save.fp.right_y7 = value[6];
        if (count >= 8) save.fp.right_y8 = value[7];
        if (count >= 9) save.fp.right_y9 = value[8];
        break;

    case 7:     /* Lift / Tilt */
        if (count != 4)
            return 0;

        save.fp.lift_y_offset   = value[0];
        save.fp.tilt_angle      = value[1];
        save.fp.tilting_y       = value[2];
        save.fp.tilting_y_speed = value[3];
        break;

    case 8:     /* Y offset */
        if (count != 1)
            return 0;

        save.fp.y_offset += value[0];

        if (save.fs.in_y >= 1) save.fp.in_y1 += value[0];
        if (save.fs.in_y >= 2) save.fp.in_y2 += value[0];
        if (save.fs.in_y >= 3) save.fp.in_y3 += value[0];
        if (save.fs.in_y >= 4) save.fp.in_y4 += value[0];
        if (save.fs.in_y >= 5) save.fp.in_y5 += value[0];

        if (save.fs.left_y >= 1) save.fp.left_y1 += value[0];
        if (save.fs.left_y >= 2) save.fp.left_y2 += value[0];
        if (save.fs.left_y >= 3) save.fp.left_y3 += value[0];
        if (save.fs.left_y >= 4) save.fp.left_y4 += value[0];
        if (save.fs.left_y >= 5) save.fp.left_y5 += value[0];
        if (save.fs.left_y >= 6) save.fp.left_y6 += value[0];
        if (save.fs.left_y >= 7) save.fp.left_y7 += value[0];
        if (save.fs.left_y >= 8) save.fp.left_y8 += value[0];
        if (save.fs.left_y >= 9) save.fp.left_y9 += value[0];

        if (save.fs.right_y >= 1) save.fp.right_y1 += value[0];
        if (save.fs.right_y >= 2) save.fp.right_y2 += value[0];
        if (save.fs.right_y >= 3) save.fp.right_y3 += value[0];
        if (save.fs.right_y >= 4) save.fp.right_y4 += value[0];
        if (save.fs.right_y >= 5) save.fp.right_y5 += value[0];
        if (save.fs.right_y >= 6) save.fp.right_y6 += value[0];
        if (save.fs.right_y >= 7) save.fp.right_y7 += value[0];
        if (save.fs.right_y >= 8) save.fp.right_y8 += value[0];
        if (save.fs.right_y >= 9) save.fp.right_y9 += value[0];
        break;

    case 9:     /* Tilt offset */
        if (count != 1)
            return 0;

        save.fp.tilt_offset += value[0];
        save.fp.tilt_angle += value[0];
        break;

    case 10:    /* X offset */
        if (count != 1)
            return 0;

        save.fp.x_offset += value[0];
        save.fp.in_x += value[0];

        if (save.fs.left_x >= 1) save.fp.left_x1 += value[0];
        if (save.fs.left_x >= 2) save.fp.left_x2 += value[0];
        if (save.fs.left_x >= 3) save.fp.left_x3 += value[0];
        if (save.fs.left_x >= 4) save.fp.left_x4 += value[0];
        if (save.fs.left_x >= 5) save.fp.left_x5 += value[0];
        if (save.fs.left_x >= 6) save.fp.left_x6 += value[0];
        if (save.fs.left_x >= 7) save.fp.left_x7 += value[0];
        if (save.fs.left_x >= 8) save.fp.left_x8 += value[0];
        if (save.fs.left_x >= 9) save.fp.left_x9 += value[0];

        if (save.fs.right_x >= 1) save.fp.right_x1 += value[0];
        if (save.fs.right_x >= 2) save.fp.right_x2 += value[0];
        if (save.fs.right_x >= 3) save.fp.right_x3 += value[0];
        if (save.fs.right_x >= 4) save.fp.right_x4 += value[0];
        if (save.fs.right_x >= 5) save.fp.right_x5 += value[0];
        if (save.fs.right_x >= 6) save.fp.right_x6 += value[0];
        if (save.fs.right_x >= 7) save.fp.right_x7 += value[0];
        if (save.fs.right_x >= 8) save.fp.right_x8 += value[0];
        if (save.fs.right_x >= 9) save.fp.right_x9 += value[0];
        break;

    case 11:    /* Speed : X, Tilt, Y */
        if (count != 3)
            return 0;

        save.fp.speed_x    = value[0];
        save.fp.speed_tilt = value[1];
        save.fp.speed_y    = value[2];
        break;

    case 12:    /* Acc/Dec : X, Tilt, Y */
        if (count != 3)
            return 0;

        save.fp.accdec_x    = value[0];
        save.fp.accdec_tilt = value[1];
        save.fp.accdec_y    = value[2];
        break;

    case 13:    /* 안전위치 : X, Y */
        if (count != 2)
            return 0;

        save.fp.safe_x = value[0];
        save.fp.safe_y = value[1];
        break;

    default:
        return 0;
    }

    return Send_FRAM(SAVE_ADDR, &save, sizeof(save));
}

/* 네트워크 설정 저장 */
int FI(int no, int value[], int count)
{
    switch (no)
    {
    case 1:     /* IP */

        if (value[0] < 0 || value[0] > 255 ||
            value[1] < 0 || value[1] > 255 ||
            value[2] < 0 || value[2] > 255 ||
            value[3] < 0 || value[3] > 255)
            return 0;

        save.fi.ip[0] = value[0];
        save.fi.ip[1] = value[1];
        save.fi.ip[2] = value[2];
        save.fi.ip[3] = value[3];

        break;


    case 2:     /* Subnet Mask */

        if (value[0] < 0 || value[0] > 255 ||
            value[1] < 0 || value[1] > 255 ||
            value[2] < 0 || value[2] > 255 ||
            value[3] < 0 || value[3] > 255)
            return 0;

        save.fi.sn[0] = value[0];
        save.fi.sn[1] = value[1];
        save.fi.sn[2] = value[2];
        save.fi.sn[3] = value[3];

        break;


    case 3:     /* Gateway */

        if (value[0] < 0 || value[0] > 255 ||
            value[1] < 0 || value[1] > 255 ||
            value[2] < 0 || value[2] > 255 ||
            value[3] < 0 || value[3] > 255)
            return 0;

        save.fi.gw[0] = value[0];
        save.fi.gw[1] = value[1];
        save.fi.gw[2] = value[2];
        save.fi.gw[3] = value[3];

        break;


    case 4:     /* Destination IP */

        if (value[0] < 0 || value[0] > 255 ||
            value[1] < 0 || value[1] > 255 ||
            value[2] < 0 || value[2] > 255 ||
            value[3] < 0 || value[3] > 255)
            return 0;

        save.fi.dip[0] = value[0];
        save.fi.dip[1] = value[1];
        save.fi.dip[2] = value[2];
        save.fi.dip[3] = value[3];

        break;

    case 5:     /* PORT */

        if (value[0] < 1 || value[0] > 65535)
            return 0;

        save.fi.port = value[0];

        break;

    case 6:     /* MODE : 0=Server, 1=Client */

        if (value[0] < 0 || value[0] > 1)
            return 0;

        save.fi.mode = value[0];

        break;

    case 7:     /* Timeout : 10000~60000ms */

        if (value[0] < 10000 || value[0] > 60000)
            return 0;

        save.fi.timeout = value[0];

        break;


    default:
        return 0;
    }


    return Send_FRAM(SAVE_ADDR, &save, sizeof(save));
}

/* 네트워크 기본값 복구 */
int FD(void)
{
	Network_Default();

    return Send_FRAM(SAVE_ADDR, &save, sizeof(save));
}
