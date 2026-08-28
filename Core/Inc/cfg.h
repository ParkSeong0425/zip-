/*
 * cfg.h
 *
 *  Created on: Aug 26, 2026
 *      Author: HWNOT
 */

#ifndef CFG_H
#define CFG_H


void ack(const char *s);
void nak(const char *s);
void cfg_init(void);
int  cfg_net(int no);   /* 1 SIP 2 SN 3 GW 4 DIP 5 PORT 6 MODE */
void cfg_show(void);
void cfg_cmd(char *s);

#endif
