/*
 * item.h
 *
 *  Created on: Jul 21, 2026
 *      Author: kotec
 */

#ifndef ITEM_H
#define ITEM_H

#include "main.h"

/* 제품 관리 */
void item_init(void);
void item_run(void);
int item_cmd(char *cmd);

/* 자동 운전 */
void item_auto_stop(void);
int item_auto_on(void);
int item_auto_start(void);

#endif
