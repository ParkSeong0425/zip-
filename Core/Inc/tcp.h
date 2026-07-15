/*
 * tcp.h
 *
 *  Created on: Jul 14, 2026
 *      Author: kotec
 */
#ifndef INC_TCP_H_
#define INC_TCP_H_

#include "main.h"

void tcp_init(void);
void tcp_run(void);
void tcp_reply(const char *s);

#endif
