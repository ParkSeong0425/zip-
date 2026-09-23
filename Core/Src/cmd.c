/*
 * cmd.c
 *
 *  Created on: Sep 15, 2026
 *      Author: HWNOT
 */
#include "cmd.h"
#include "save.h"
#include "socket.h"
#include "run.h"
#include "status.h"
#include "cmsis_os2.h"

extern osMessageQueueId_t MotorQueueHandle;

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define STX       0x02
#define ETX       0x03
#define ACK       0x06
#define NAK       0x15

#define CMD_FS  (('F' << 8) | 'S')
#define CMD_FP  (('F' << 8) | 'P')
#define CMD_FI  (('F' << 8) | 'I')
#define CMD_C   ('C' << 8)
#define CMD_FD  (('F' << 8) | 'D')

#define TCP_SOCK  0


static void ack(char *s)
{
    char tx[160];

    int len = snprintf(tx, sizeof(tx),
            "%c%c%c%c%s%c",
            0x02,
            s[0], s[1],
            0x06,
            s + 2,
            0x03);

    send(TCP_SOCK, (uint8_t *)tx, len);
}


static void nak(char *s, char *error)
{
    char tx[80];

    int len = snprintf(tx, sizeof(tx),
            "%c%c%c%c%s%c",
            0x02,
            s[0], s[1],
            0x15,
            error,
            0x03);

    send(TCP_SOCK, (uint8_t *)tx, len);
}

/* 완료응답 : MI→AI, MO→AO, PO→EO */
void CMD_Done(MotorCommand cmd)
{
    char tx[40];
    int len = 0;

    if (cmd.command == CMD_MI)
        len = snprintf(tx, sizeof(tx), "%c%02dAI_%d_%d_%d%c",
                0x02, cmd.serial, cmd.rack, cmd.x_no, cmd.y_no, 0x03);
    else if (cmd.command == CMD_MO)
        len = snprintf(tx, sizeof(tx), "%c%02dAO_%d_%d_%d%c",
                0x02, cmd.serial, cmd.rack, cmd.x_no, cmd.y_no, 0x03);
    else if (cmd.command == CMD_PO)
        len = snprintf(tx, sizeof(tx), "%c%02dEO_%d_%d_%d%c",
                0x02, cmd.serial, cmd.rack, cmd.x_no, cmd.y_no, 0x03);

    if (len > 0)
        send(TCP_SOCK, (uint8_t *)tx, len);
}

/* 명령 처리 */
void CMD_Run(char *s)
{
    int cmd;
    MotorCommand motor = {0};

    int rack;
    char type;
    int rack_channel;

    int in_x, in_y;
    int left_x, left_y;
    int right_x, right_y;

    int x_no;
    int y_no;
    int xy_speed;
    int tilt_speed;
    int move_delay;
    int return_delay;

    int no;
    int value[9];
    int count;

    cmd = (s[2] << 8) | s[3];

    /* 완료응답용 : 받은 일련번호 / 렉 / 단 / 열 */
    sscanf(s, "%2d", &motor.serial);
    sscanf(s + 5, "%d_%d_%d", &motor.rack, &motor.x_no, &motor.y_no);
    switch (cmd)

    {
    /* FS */
    case CMD_FS:

        if (sscanf(s + 2,
                "FS_%d_%c_%d_%d_%d_%d_%d_%d_%d",
                &rack,
                &type,
                &rack_channel,
                &in_x,
                &in_y,
                &left_x,
                &left_y,
                &right_x,
                &right_y) != 9)
        {
            nak(s, "bad_data");
            break;
        }

        if (FS(rack, type, rack_channel,
               in_x, in_y,
               left_x, left_y,
               right_x, right_y))
            ack(s);
        else
            nak(s, "bad_data");

        break;


    /* FP */
    case CMD_FP:

        count = sscanf(s + 2,
                "FP_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d",
                &rack,
                &no,
                &value[0],
                &value[1],
                &value[2],
                &value[3],
                &value[4],
                &value[5],
                &value[6],
                &value[7],
                &value[8]);

        count -= 2;

        if (count < 1)
        {
            nak(s, "bad_data");
            break;
        }

        if (FP(rack, no, value, count))
            ack(s);
        else
            nak(s, "bad_data");

        break;


    /* FI */
    case CMD_FI:

        /* FI 1~4 : IP 형식 */
        if (sscanf(s + 2,
                "FI_1_%d_%d.%d.%d.%d",
                &no,
                &value[0],
                &value[1],
                &value[2],
                &value[3]) == 5)
        {
            if (FI(no, value, 4))
                ack(s);
            else
                nak(s, "bad_data");

            break;
        }

        /* FI 5~7 : 숫자 하나 */
        if (sscanf(s + 2,
                "FI_1_%d_%d",
                &no,
                &value[0]) == 2)
        {
            if (FI(no, value, 1))
                ack(s);
            else
                nak(s, "bad_data");

            break;
        }

        nak(s, "bad_data");
        break;


    /* FD */
    case CMD_FD:

        if (strcmp(s + 2, "FD_1") != 0)
        {
            nak(s, "bad_data");
            break;
        }

        if (FD())
            ack(s);
        else
            nak(s, "fram_error");

        break;


    /* 입고 */
    case CMD_MI:

        if (sscanf(s + 5, "%d_%d_%d_%d_%d_%d_%d",
                &rack,
                &x_no,
                &y_no,
                &xy_speed,
                &tilt_speed,
                &move_delay,
                &return_delay) != 7)
        {
            nak(s, "bad_data");
            break;
        }

        if (x_no != 1 || y_no < 1 || y_no > save.fs.in_y)
        {
            nak(s, "bad_data");
            break;
        }

        motor.command = CMD_MI;
        motor.rack = rack;
        motor.x = save.fp.in_x;

        if (y_no == 1) motor.y = save.fp.in_y1;
        else if (y_no == 2) motor.y = save.fp.in_y2;
        else if (y_no == 3) motor.y = save.fp.in_y3;
        else if (y_no == 4) motor.y = save.fp.in_y4;
        else if (y_no == 5) motor.y = save.fp.in_y5;

        motor.xy_speed = xy_speed;
        motor.tilt_speed = tilt_speed;
        motor.move_delay = move_delay;
        motor.return_delay = return_delay;

        osMessageQueuePut(MotorQueueHandle, &motor, 0, osWaitForever);
        ack(s);
        break;


    /* 출고 */
    case CMD_MO:

        if (sscanf(s + 5, "%d_%d_%d_%d_%d_%d_%d",
                &rack,
                &x_no,
                &y_no,
                &xy_speed,
                &tilt_speed,
                &move_delay,
                &return_delay) != 7)
        {
            nak(s, "bad_data");
            break;
        }

        motor.command = CMD_MO;
        motor.rack = rack;

        if (rack == 1)
        {
            if (x_no < 1 || x_no > save.fs.right_x ||
                y_no < 1 || y_no > save.fs.right_y)
            {
                nak(s, "bad_data");
                break;
            }

            if (x_no == 1) motor.x = save.fp.right_x1;
            else if (x_no == 2) motor.x = save.fp.right_x2;
            else if (x_no == 3) motor.x = save.fp.right_x3;
            else if (x_no == 4) motor.x = save.fp.right_x4;
            else if (x_no == 5) motor.x = save.fp.right_x5;
            else if (x_no == 6) motor.x = save.fp.right_x6;
            else if (x_no == 7) motor.x = save.fp.right_x7;
            else if (x_no == 8) motor.x = save.fp.right_x8;
            else if (x_no == 9) motor.x = save.fp.right_x9;

            if (y_no == 1) motor.y = save.fp.right_y1;
            else if (y_no == 2) motor.y = save.fp.right_y2;
            else if (y_no == 3) motor.y = save.fp.right_y3;
            else if (y_no == 4) motor.y = save.fp.right_y4;
            else if (y_no == 5) motor.y = save.fp.right_y5;
            else if (y_no == 6) motor.y = save.fp.right_y6;
            else if (y_no == 7) motor.y = save.fp.right_y7;
            else if (y_no == 8) motor.y = save.fp.right_y8;
            else if (y_no == 9) motor.y = save.fp.right_y9;
        }
        else
        {
            if (x_no < 1 || x_no > save.fs.left_x ||
                y_no < 1 || y_no > save.fs.left_y)
            {
                nak(s, "bad_data");
                break;
            }

            if (x_no == 1) motor.x = save.fp.left_x1;
            else if (x_no == 2) motor.x = save.fp.left_x2;
            else if (x_no == 3) motor.x = save.fp.left_x3;
            else if (x_no == 4) motor.x = save.fp.left_x4;
            else if (x_no == 5) motor.x = save.fp.left_x5;
            else if (x_no == 6) motor.x = save.fp.left_x6;
            else if (x_no == 7) motor.x = save.fp.left_x7;
            else if (x_no == 8) motor.x = save.fp.left_x8;
            else if (x_no == 9) motor.x = save.fp.left_x9;

            if (y_no == 1) motor.y = save.fp.left_y1;
            else if (y_no == 2) motor.y = save.fp.left_y2;
            else if (y_no == 3) motor.y = save.fp.left_y3;
            else if (y_no == 4) motor.y = save.fp.left_y4;
            else if (y_no == 5) motor.y = save.fp.left_y5;
            else if (y_no == 6) motor.y = save.fp.left_y6;
            else if (y_no == 7) motor.y = save.fp.left_y7;
            else if (y_no == 8) motor.y = save.fp.left_y8;
            else if (y_no == 9) motor.y = save.fp.left_y9;
        }

        motor.xy_speed = xy_speed;
        motor.tilt_speed = tilt_speed;
        motor.move_delay = move_delay;
        motor.return_delay = return_delay;

        osMessageQueuePut(MotorQueueHandle, &motor, 0, osWaitForever);
        ack(s);
        break;


    /* 분배 */
    case CMD_PO:

        if (sscanf(s + 5, "%d_%d_%d_%d_%d_%d_%d",
                &rack,
                &x_no,
                &y_no,
                &xy_speed,
                &tilt_speed,
                &move_delay,
                &return_delay) != 7)
        {
            nak(s, "bad_data");
            break;
        }

        motor.command = CMD_PO;
        motor.rack = rack;

        if (rack == 1)
        {
            if (x_no < 1 || x_no > save.fs.right_x ||
                y_no < 1 || y_no > save.fs.right_y)
            {
                nak(s, "bad_data");
                break;
            }

            if (x_no == 1) motor.x = save.fp.right_x1;
            else if (x_no == 2) motor.x = save.fp.right_x2;
            else if (x_no == 3) motor.x = save.fp.right_x3;
            else if (x_no == 4) motor.x = save.fp.right_x4;
            else if (x_no == 5) motor.x = save.fp.right_x5;
            else if (x_no == 6) motor.x = save.fp.right_x6;
            else if (x_no == 7) motor.x = save.fp.right_x7;
            else if (x_no == 8) motor.x = save.fp.right_x8;
            else if (x_no == 9) motor.x = save.fp.right_x9;

            if (y_no == 1) motor.y = save.fp.right_y1;
            else if (y_no == 2) motor.y = save.fp.right_y2;
            else if (y_no == 3) motor.y = save.fp.right_y3;
            else if (y_no == 4) motor.y = save.fp.right_y4;
            else if (y_no == 5) motor.y = save.fp.right_y5;
            else if (y_no == 6) motor.y = save.fp.right_y6;
            else if (y_no == 7) motor.y = save.fp.right_y7;
            else if (y_no == 8) motor.y = save.fp.right_y8;
            else if (y_no == 9) motor.y = save.fp.right_y9;
        }
        else
        {
            if (x_no < 1 || x_no > save.fs.left_x ||
                y_no < 1 || y_no > save.fs.left_y)
            {
                nak(s, "bad_data");
                break;
            }

            if (x_no == 1) motor.x = save.fp.left_x1;
            else if (x_no == 2) motor.x = save.fp.left_x2;
            else if (x_no == 3) motor.x = save.fp.left_x3;
            else if (x_no == 4) motor.x = save.fp.left_x4;
            else if (x_no == 5) motor.x = save.fp.left_x5;
            else if (x_no == 6) motor.x = save.fp.left_x6;
            else if (x_no == 7) motor.x = save.fp.left_x7;
            else if (x_no == 8) motor.x = save.fp.left_x8;
            else if (x_no == 9) motor.x = save.fp.left_x9;

            if (y_no == 1) motor.y = save.fp.left_y1;
            else if (y_no == 2) motor.y = save.fp.left_y2;
            else if (y_no == 3) motor.y = save.fp.left_y3;
            else if (y_no == 4) motor.y = save.fp.left_y4;
            else if (y_no == 5) motor.y = save.fp.left_y5;
            else if (y_no == 6) motor.y = save.fp.left_y6;
            else if (y_no == 7) motor.y = save.fp.left_y7;
            else if (y_no == 8) motor.y = save.fp.left_y8;
            else if (y_no == 9) motor.y = save.fp.left_y9;
        }

        motor.xy_speed = xy_speed;
        motor.tilt_speed = tilt_speed;
        motor.move_delay = move_delay;
        motor.return_delay = return_delay;

        motor.tilt_angle = save.fp.tilt_angle;
        motor.tilting_y = save.fp.tilting_y;
        motor.tilting_y_speed = save.fp.tilting_y_speed;

        osMessageQueuePut(MotorQueueHandle, &motor, 0, osWaitForever);
        ack(s);
        break;
        /* 상태 조회 */
        case CMD_C:

            char tx[80];

            int len = snprintf(tx, sizeof(tx),
                    "%c%c%c%cS_1_%c&S_2_%c&F_1_%d%d%d%d&F_2_%d%d%d%d%c",
                    0x02,
                    s[0], s[1],
                    0x06,
                    status_1, status_2,
                    full_1_1, full_1_2, full_1_3, full_1_4,
                    full_2_1, full_2_2, full_2_3, full_2_4,
                    0x03);

            send(TCP_SOCK, (uint8_t *)tx, len);
            break;

        /* stop */
        case CMD_S:

            if (strcmp(s + 2, "S_1") != 0)
            {
                nak(s, "bad_data");
                break;
            }

            if (S())
                ack(s);
            else
                nak(s, "Stop_errer");

            break;

      /* home */
        case CMD_I:

            if (strcmp(s + 2, "I") != 0)
            {
                nak(s, "bad_data");
                break;
            }

            motor.command = CMD_I;

            osMessageQueuePut(MotorQueueHandle,
                              &motor, 0, osWaitForever);

            ack(s);
            break;


        default:
        nak(s, "bad_command");
        break;
    }
}
