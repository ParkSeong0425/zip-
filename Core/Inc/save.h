#ifndef INC_SAVE_H_
#define INC_SAVE_H_

#include "main.h"

void save_init(void);
void save_run(void);
void save_cmd(char *cmd);

int save_home(void);
int save_go(int col, int dan);
int save_auto(void);
void save_stop(void);
int save_busy(void);

#endif
