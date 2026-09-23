#include "cli.h"
#include "run.h"
#include "net.h"
#include "i_motor.h"
#include "a_motor.h"
#include "t_motor.h"
#include "save.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>

#define CLI_SIZE 32

extern SAVE_Data save;
extern osMessageQueueId_t MotorQueueHandle;


/* UART1 출력 */
void CLI_Print(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), 100);
}


/* 시작 메시지 */
void CLI_Start(int ok)
{
    if (ok)
    {
        CLI_Print(
            "\r\nMOTOR INIT OK\r\n"
            "X ID=3\r\n"
            "Y ID=1\r\n"
            "TILT ID=2\r\n"
        );
    }
    else
    {
        CLI_Print(
            "\r\nMOTOR INIT ERR\r\n"
            "Type INIT to retry\r\n"
        );
    }

    CLI_Print(
        "\r\nX pulse percent\r\n"
        "Y pulse percent\r\n"
        "C percent\r\n"
        "R deg percent\r\n"
        "L deg percent\r\n"
        "S / H / INIT / IP / FRAM\r\n> "
    );
}


/* 모터 실행 결과 출력 */
void CLI_Result(char command, int ok)
{
    char text[160];

    if (command == 'X')
        snprintf(text, sizeof(text),
                 "X_MOVE %s\r\n> ", ok ? "OK" : "ERR");

    else if (command == 'Y')
        snprintf(text, sizeof(text),
                 "Y_MOVE %s\r\n> ", ok ? "OK" : "ERR");

    else if (command == 'C')
        snprintf(text, sizeof(text),
                 "CENTER %s\r\n> ", ok ? "OK" : "ERR");

    else if (command == 'R')
        snprintf(text, sizeof(text),
                 "RIGHT %s\r\n> ", ok ? "OK" : "ERR");

    else if (command == 'L')
        snprintf(text, sizeof(text),
                 "LEFT %s\r\n> ", ok ? "OK" : "ERR");

    else if (command == 'S')
        snprintf(text, sizeof(text),
                 "STOP %s\r\n> ", ok ? "OK" : "ERR");

    else if (command == 'I')
        snprintf(text, sizeof(text),
                 "HOME %s\r\n> ", ok ? "OK" : "ERR");
    else
        return;

    CLI_Print(text);
}


/* FRAM 배열 출력 */
static void CLI_List(const char *name, int32_t *data, int count, int max)
{
    char buf[32];

    CLI_Print(name);

    if (count > max)
        count = max;

    for (int i = 0; i < count; i++)
    {
        snprintf(buf, sizeof(buf), "%ld ", (long)data[i]);
        CLI_Print(buf);
    }

    CLI_Print("\r\n");
}


/* FRAM 저장값 출력 */
static void CLI_FRAM(void)
{
    char buf[500];

    int32_t in_y[5] = {
        save.fp.in_y1, save.fp.in_y2, save.fp.in_y3,
        save.fp.in_y4, save.fp.in_y5
    };

    int32_t left_x[9] = {
        save.fp.left_x1, save.fp.left_x2, save.fp.left_x3,
        save.fp.left_x4, save.fp.left_x5, save.fp.left_x6,
        save.fp.left_x7, save.fp.left_x8, save.fp.left_x9
    };

    int32_t left_y[9] = {
        save.fp.left_y1, save.fp.left_y2, save.fp.left_y3,
        save.fp.left_y4, save.fp.left_y5, save.fp.left_y6,
        save.fp.left_y7, save.fp.left_y8, save.fp.left_y9
    };

    int32_t right_x[9] = {
        save.fp.right_x1, save.fp.right_x2, save.fp.right_x3,
        save.fp.right_x4, save.fp.right_x5, save.fp.right_x6,
        save.fp.right_x7, save.fp.right_x8, save.fp.right_x9
    };

    int32_t right_y[9] = {
        save.fp.right_y1, save.fp.right_y2, save.fp.right_y3,
        save.fp.right_y4, save.fp.right_y5, save.fp.right_y6,
        save.fp.right_y7, save.fp.right_y8, save.fp.right_y9
    };

    snprintf(buf, sizeof(buf),
             "\r\n[FS]\r\n"
             "RACK         = %d\r\n"
             "TYPE         = %c\r\n"
             "CHANNEL      = %d\r\n"
             "IN_X         = %d\r\n"
             "IN_Y         = %d\r\n"
             "LEFT_X       = %d\r\n"
             "LEFT_Y       = %d\r\n"
             "RIGHT_X      = %d\r\n"
             "RIGHT_Y      = %d\r\n"
             "\r\n[FP]\r\n"
             "IN_X = %ld\r\n",
             save.fs.rack,
             save.fs.type,
             save.fs.rack_channel,
             save.fs.in_x,
             save.fs.in_y,
             save.fs.left_x,
             save.fs.left_y,
             save.fs.right_x,
             save.fs.right_y,
             (long)save.fp.in_x);

    CLI_Print(buf);

    CLI_List("IN_Y = ", in_y, save.fs.in_y, 5);
    CLI_List("LEFT_X = ", left_x, save.fs.left_x, 9);
    CLI_List("LEFT_Y = ", left_y, save.fs.left_y, 9);
    CLI_List("RIGHT_X = ", right_x, save.fs.right_x, 9);
    CLI_List("RIGHT_Y = ", right_y, save.fs.right_y, 9);

    snprintf(buf, sizeof(buf),
             "\r\n"
             "LIFT Y OFFSET = %ld\r\n"
             "TILT ANGLE    = %ld\r\n"
             "TILTING Y     = %ld\r\n"
             "TILTING SPEED = %ld\r\n"
             "Y OFFSET      = %ld\r\n"
             "TILT OFFSET   = %ld\r\n"
             "X OFFSET      = %ld\r\n"
             "SPEED X       = %ld\r\n"
             "SPEED TILT    = %ld\r\n"
             "SPEED Y       = %ld\r\n"
             "ACCDEC X      = %ld\r\n"
             "ACCDEC TILT   = %ld\r\n"
             "ACCDEC Y      = %ld\r\n"
             "SAFE X        = %ld\r\n"
             "SAFE Y        = %ld\r\n> ",
             (long)save.fp.lift_y_offset,
             (long)save.fp.tilt_angle,
             (long)save.fp.tilting_y,
             (long)save.fp.tilting_y_speed,
             (long)save.fp.y_offset,
             (long)save.fp.tilt_offset,
             (long)save.fp.x_offset,
             (long)save.fp.speed_x,
             (long)save.fp.speed_tilt,
             (long)save.fp.speed_y,
             (long)save.fp.accdec_x,
             (long)save.fp.accdec_tilt,
             (long)save.fp.accdec_y,
             (long)save.fp.safe_x,
             (long)save.fp.safe_y);

    CLI_Print(buf);
}


/* 문자열 명령 처리 */
static void CLI_Command(char *text)
{
    MotorCommand cmd = {0};
    long pulse;
    int persent;
    char buf[300];

    /* 네트워크 값 */
    if (!strcmp(text, "IP"))
    {
        snprintf(buf, sizeof(buf),
                 "\r\n[IP]\r\n"
                 "IP      = %d.%d.%d.%d\r\n"
                 "SUBNET  = %d.%d.%d.%d\r\n"
                 "GATEWAY = %d.%d.%d.%d\r\n"
                 "DEST IP = %d.%d.%d.%d\r\n"
                 "PORT    = %d\r\n"
                 "MODE    = %d\r\n"
                 "TIMEOUT = %lu\r\n> ",
                 save.fi.ip[0], save.fi.ip[1],
                 save.fi.ip[2], save.fi.ip[3],
                 save.fi.sn[0], save.fi.sn[1],
                 save.fi.sn[2], save.fi.sn[3],
                 save.fi.gw[0], save.fi.gw[1],
                 save.fi.gw[2], save.fi.gw[3],
                 save.fi.dip[0], save.fi.dip[1],
                 save.fi.dip[2], save.fi.dip[3],
                 save.fi.port, save.fi.mode,
                 (unsigned long)save.fi.timeout);

        CLI_Print(buf);
        return;
    }

    /* FRAM 값 */
    if (!strcmp(text, "FRAM"))
    {
        CLI_FRAM();
        return;
    }

    /* 모터 초기화 재확인 */
    if (!strcmp(text, "INIT"))
    {
        cmd.command = 'I';
    }

    /* X 모터 */
    else if (sscanf(text, "X %ld %d", &pulse, &persent) == 2)
    {
        cmd.command = 'X';
        cmd.pulse = (int32_t)pulse;
        cmd.rpm = persent;
    }

    /* Y 모터 */
    else if (sscanf(text, "Y %ld %d", &pulse, &persent) == 2)
    {
        cmd.command = 'Y';
        cmd.pulse = (int32_t)pulse;
        cmd.rpm = persent;
    }

    /* Tilt 중앙 */
    else if (sscanf(text, "C %d", &persent) == 1)
    {
        cmd.command = 'C';
        cmd.rpm = persent;
    }

    /* Tilt 오른쪽 */
    else if (sscanf(text, "R %ld %d", &pulse, &persent) == 2)
    {
        cmd.command = 'R';
        cmd.pulse = (int32_t)pulse;
        cmd.rpm = persent;
    }

    /* Tilt 왼쪽 */
    else if (sscanf(text, "L %ld %d", &pulse, &persent) == 2)
    {
        cmd.command = 'L';
        cmd.pulse = (int32_t)pulse;
        cmd.rpm = persent;
    }

    /* 정지 */
    else if (!strcmp(text, "S"))
    {
        cmd.command = 'S';
    }

    /* 원점복귀 */
    else if (!strcmp(text, "I"))
    {
        cmd.command = 'I';
    }

    else
    {
        CLI_Print("BAD COMMAND\r\n> ");
        return;
    }

    osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
}


/* UART1 CLI */
void CLI_Run(void)
{
    static char line[CLI_SIZE];
    static uint8_t index = 0;
    uint8_t ch;

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) &&
        HAL_UART_Receive(&huart1, &ch, 1, 1) == HAL_OK)
    {
        if (ch == '\r' || ch == '\n')
        {
            if (index > 0)
            {
                line[index] = 0;
                CLI_Print("\r\n");
                CLI_Command(line);
                index = 0;
            }
        }
        else if (index < CLI_SIZE - 1)
        {
            line[index++] = (char)ch;
        }
    }
}
