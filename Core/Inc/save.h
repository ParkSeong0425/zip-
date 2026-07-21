#ifndef INC_SAVE_H_
#define INC_SAVE_H_

#include "main.h"

void save_init(void);
void save_run(void);
void save_cmd(char *cmd);

int save_home(void);
int save_go(int col, int dan);
void save_stop(void);
int save_busy(void);

/* item.c가 쓴다. save_result: 0=진행중 1=성공 -1=실패 */
int save_result(void);
int save_grid_x(void);
int save_grid_y(void);

#endif
