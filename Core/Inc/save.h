/*
 * save.h
 *
 *  Created on: Sep 15, 2026
 *      Author: HWNOT
 */
#ifndef INC_SAVE_H_
#define INC_SAVE_H_

#include "main.h"

/* 장비 / 위치 개수 */
typedef struct
{
    uint8_t rack;             /* 1 고정 */
    char type;                /* S / I */
    uint8_t rack_channel;     /* 1~4 */

    uint8_t in_x;             /* 1 고정 */
    uint8_t in_y;             /* 1~5 */

    uint8_t left_x;           /* 1~9 */
    uint8_t left_y;           /* 1~9 */

    uint8_t right_x;          /* 1~9 */
    uint8_t right_y;          /* 1~9 */

} FS_Data;


/* 위치 설정 */
typedef struct
{
    /* FP 1 : 입고 X */
    int32_t in_x;

    /* FP 2 : 입고 Y */
    int32_t in_y1;
    int32_t in_y2;
    int32_t in_y3;
    int32_t in_y4;
    int32_t in_y5;

    /* FP 3 : Left X */
    int32_t left_x1;
    int32_t left_x2;
    int32_t left_x3;
    int32_t left_x4;
    int32_t left_x5;
    int32_t left_x6;
    int32_t left_x7;
    int32_t left_x8;
    int32_t left_x9;

    /* FP 4 : Left Y */
    int32_t left_y1;
    int32_t left_y2;
    int32_t left_y3;
    int32_t left_y4;
    int32_t left_y5;
    int32_t left_y6;
    int32_t left_y7;
    int32_t left_y8;
    int32_t left_y9;

    /* FP 5 : Right X */
    int32_t right_x1;
    int32_t right_x2;
    int32_t right_x3;
    int32_t right_x4;
    int32_t right_x5;
    int32_t right_x6;
    int32_t right_x7;
    int32_t right_x8;
    int32_t right_x9;

    /* FP 6 : Right Y */
    int32_t right_y1;
    int32_t right_y2;
    int32_t right_y3;
    int32_t right_y4;
    int32_t right_y5;
    int32_t right_y6;
    int32_t right_y7;
    int32_t right_y8;
    int32_t right_y9;

    /* FP 7 */
    int32_t lift_y_offset;
    int32_t tilt_angle;
    int32_t tilting_y;
    int32_t tilting_y_speed;

    /* FP 8 */
    int32_t y_offset;

    /* FP 9 */
    int32_t tilt_offset;

    /* FP 10 */
    int32_t x_offset;

    /* FP 11 */
    int32_t speed_x;
    int32_t speed_tilt;
    int32_t speed_y;

    /* FP 12 */
    int32_t accdec_x;
    int32_t accdec_tilt;
    int32_t accdec_y;

    /* FP 13 */
    int32_t safe_x;
    int32_t safe_y;

} FP_Data;


/* 네트워크 설정 */
typedef struct
{
    uint8_t ip[4];
    uint8_t sn[4];
    uint8_t gw[4];
    uint8_t dip[4];

    uint16_t port;
    uint8_t mode;
    uint32_t timeout;

} FI_Data;


/* FRAM에 저장할 전체 데이터 */
typedef struct
{
    uint16_t magic;
    uint16_t version;

    FS_Data fs;
    FP_Data fp;
    FI_Data fi;

} SAVE_Data;

extern SAVE_Data save;

int Save_Init(void);
int FS(int rack, char type, int rack_channel,
       int in_x, int in_y,
       int left_x, int left_y,
       int right_x, int right_y);
int FP(int rack, int no, int value[], int count);
int FI(int no, int value[], int count);
int FD(void);

#endif
