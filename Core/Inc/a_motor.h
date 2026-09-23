#ifndef INC_A_MOTOR_H_
#define INC_A_MOTOR_H_

#include "main.h"

#define Y_PULSE            1000
#define Y_GEAR             5
#define Y_PI               160


/* Y axis AIMotor */
#define AM_ID               1
#define AM_BAUD             115200
#define AM_MAX_RPM          3000

/* 1구간 절대위치 이동 */
#define AM_ACC_DEC_MS       100
#define AM_WAIT_MS          0
#define AM_DONE_GAP         10

/* Reverse mechanical torque home */
#define AM_HOME_MODE        11      /* H05_31 : Reverse mechanical stop */
#define AM_HOME_HI_RPM      50     /* H05_32 */
#define AM_HOME_LO_RPM      10      /* H05_33 */
#define AM_HOME_ACC_MS      200     /* H05_34 */
#define AM_HOME_TIMEOUT_MS  60000   /* H05_35 */
#define AM_HOME_TORQUE      20     /* H05_58 : 0.1%, 100 = 10.0% */

/* 기본 기능 */
int AM_Init(void);
int AM_Move(int32_t mm, int percent);
int AM_Done(void);
int AM_Home(void);
int AM_Stop(void);
int AM_EStop(int on);
int AM_Servo(int on);
int AM_Zero(void);
/* 상태 읽기 */
int AM_Pos(int32_t *mm);
int AM_Speed(int *rpm);
int AM_Torque(int *torque);
int AM_AlarmRead(uint16_t *alarm);
int AM_ID_COMM(void);
int AM_HomeDone(void);
uint8_t AM_GetID(void);

/* 보조 기능 */
int AM_AlarmReset(void);

/* 디버깅용 */
int AM_Read(uint16_t reg, uint16_t *val);

/* 보드레이트 변경 */
int AM_BaudInit(void);

#endif /* INC_A_MOTOR_H_ */
