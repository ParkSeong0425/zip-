/*
 * move.h
 *
 *  Created on: Sep 1, 2026
 *      Author: HWNOT
 */
#ifndef INC_MOVE_H_
#define INC_MOVE_H_

#ifdef __cplusplus
extern "C" {
#endif

extern volatile char status;
extern volatile int alarm;
extern int rack_now;
extern char motor_line[64];
extern volatile int motor_ready;
extern volatile int motor_busy;

void MOTOR_TaskRun(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOVE_H_ */
