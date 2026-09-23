#ifndef INC_RUN_H_
#define INC_RUN_H_

#include <stdint.h>
#define CMD_I   ('I' << 8)
#define CMD_MI  (('M' << 8) | 'I')
#define CMD_MO  (('M' << 8) | 'O')
#define CMD_PO  (('P' << 8) | 'O')
#define CMD_S   (('S' << 8) | '_')

typedef struct
{
    int command;

    /* 기존 CLI 모터 시험 */
    int32_t pulse;
    int rpm;

    /* MI / MO / PO */
    /* MI / MO / PO */
    int serial;
    int rack;
    int x_no;
    int y_no;
    int x;
    int y;
    int xy_speed;
    int tilt_speed;
    int move_delay;
    int return_delay;

    /* PO 틸팅 값 */
    int tilt_angle;
    int tilting_y;
    int tilting_y_speed;

} MotorCommand;

int motor_basic_init(void);
int motor_basic_run(MotorCommand cmd);

int MI(int x, int y, int xy_speed, int tilt_speed);
int MO(int x, int y, int xy_speed, int tilt_speed);
int PO(int rack, int x, int y,
       int xy_speed, int tilt_speed,
       int move_delay, int return_delay,
       int tilt_angle, int tilting_y, int tilting_y_speed);
int S (void);
int I (void);

void Motor_Run(void);

#endif
