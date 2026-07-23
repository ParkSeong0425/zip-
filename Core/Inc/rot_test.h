#ifndef INC_ROT_TEST_H_
#define INC_ROT_TEST_H_

#include "main.h"

/* 전체 실행 */
void rot_test_init(void);
void rot_test_run(void);

/* MKS 회전축: UART5, RS4852, ID 2, Modbus-RTU */
int mks_init(void);
int mks_check(void);
int mks_pos(int *out);
int mks_raw(int *out);
int mks_io(uint8_t *out);
int mks_state(uint8_t *out);
int mks_zero(void);
int mks_home(void);
int mks_move_abs(int rpm, int target);
int mks_move_rel(int rpm, int delta);
int mks_stop(void);

#endif
