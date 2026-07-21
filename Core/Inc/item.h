/*
 * item.h
 *
 *  Created on: Jul 21, 2026
 *      Author: kotec
 */
#ifndef INC_ITEM_H_
#define INC_ITEM_H_

#include "main.h"

void item_init(void);

/* 매 루프마다 부른다. 사이클이 끝나면 다음 예약을 시작한다 */
void item_run(void);

/* 명령을 처리했으면 1, 모르는 명령이면 0 */
int item_cmd(char *cmd);

#endif
