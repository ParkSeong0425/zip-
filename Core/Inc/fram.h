/*
 * fram.h
 *
 *  Created on: Sep 15, 2026
 *      Author: HWNOT
 */
#ifndef INC_FRAM_H_
#define INC_FRAM_H_

#include "main.h"

int Send_FRAM(uint16_t addr, const void *data, uint16_t len);
int Bring_FRAM(uint16_t addr, void *data, uint16_t len);

#endif
