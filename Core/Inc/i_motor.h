#ifndef INC_I_MOTOR_H_
#define INC_I_MOTOR_H_

#include "main.h"

/* PR0 절대위치 운전 설정 */
#define IM_MAX_RPM        3000
#define IM_ACC_MS         200
#define IM_DEC_MS         200
#define X_PULSE           10000    /* 데이터시트 : 1회전 10000펄스 */
#define X_PI              160      /* 폴리 : 1회전 160mm */
#define X_GEAR            10       /* 드라이브가 지령을 1/10로 받는다 (실측) */

/* 토크 원점복귀 설정 */
#define IM_HOME_DIR       0          /* 0x600A Bit0 : 방향 (실측 확인) */
#define IM_HOME_TORQUE    (3 << 2)   /* 0x600A Bit2~ : 3 = 토크 원점 */
#define IM_HOME_HI_RPM    30
#define IM_HOME_LO_RPM    10
#define IM_HOME_ACC_MS    300
#define IM_HOME_DEC_MS    300
#define IM_HOME_TIME_MS   200
#define IM_HOME_FORCE     20
#define IM_HOME_LIMIT     20000

/* 0x600F ~ 0x6015 연속 홈 설정값 */
#define IM_HOME_COUNT     7
static const uint16_t IM_HOME_DATA[IM_HOME_COUNT] = {
    IM_HOME_HI_RPM,      /* 0x600F Homing high velocity */
    IM_HOME_LO_RPM,      /* 0x6010 Homing low velocity */
    IM_HOME_ACC_MS,      /* 0x6011 Homing Acc */
    IM_HOME_DEC_MS,      /* 0x6012 Homing Dec */
    IM_HOME_TIME_MS,     /* 0x6013 Torque zero return time */
    IM_HOME_FORCE,       /* 0x6014 Torque zero return value */
    IM_HOME_LIMIT        /* 0x6015 Homing over-travel */
};

/* 0x1003 Motion state */
#define IM_PATH_DONE      (1 << 5)
#define IM_HOMED          (1 << 6)

/* Mapping으로 한번에 읽는 상시 상태 */
typedef struct {
    uint16_t state;       /* 0x1003 Motion state */
    int32_t  pos;         /* 0x1014, 0x1015 Feedback position */
    uint16_t alarm;       /* 0x2203 Current alarm */
    uint16_t warn;        /* 0x601D PR/Home warning */
} IM_Info;

/* 기본 기능 */
int IM_Move(int32_t mm, int percent);
int IM_Done(void);
int IM_Home(void);
int IM_Stop(void);

/* 상태 읽기 */
uint16_t IM_State(void);
int IM_Pos(int32_t *mm);
int IM_AlarmRead(uint16_t *alarm);
int IM_DIPRead(uint16_t *dip);

/* Register Mapping */
int IM_MapInit(void);
int IM_MapRead(IM_Info *info);

/* 디버깅용 */
int IM_Read(uint16_t reg, uint16_t *val);

#endif /* INC_I_MOTOR_H_ */
