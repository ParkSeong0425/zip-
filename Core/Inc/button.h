/*
 * button.h
 *
 *  Created on: Jul 22, 2026
 *      Author: kotec
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"

extern volatile int pause;
extern volatile int estop;

void button_init(void);
void HAL_GPIO_EXTI_Callback(uint16_t pin);
void button_run(void);

int full_get(void);
int full_clear(void);
int button_stop_requested(void);

#endif /* INC_BUTTON_H_ */
