/*
 * cli.h
 */
#ifndef CLI_H
#define CLI_H

void cli_start(void);
void cli_poll(void);
void cli_exec(char *s);
int cli_key(void);

#endif
