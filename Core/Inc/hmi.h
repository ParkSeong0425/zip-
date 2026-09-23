/*
 * hmi.h
 *
 *  Created on: Sep 18, 2026
 *      Author: HWNOT
 */
#ifndef INC_HMI_H_
#define INC_HMI_H_

/* HMI 버튼 번호 : 기존 HMI ZIP 그대로 */

/* 메인 */
#define BTN_RUN            11
#define BTN_STOP           12
#define BTN_ESTOP          13

/* 수동 */
#define BTN_TILT_Lift       21
#define BTN_TILT_Right      22
#define BTN_LIFT_UP         23
#define BTN_LIFT_Down       24
#define BTN_TRAVEL_Forward  25
#define BTN_TRAVEL_Back     26

/* 설정 */
#define BTN_SOUND_OFF       31
#define BTN_RESET           32
#define BTN_MANUAL          33
#define BTN_ORIGIN          34

void HMI_Run(void);

#endif
