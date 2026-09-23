/*
 * cli.h
 *
 *  Created on: Sep 11, 2026
 *      Author: HWNOT
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_

#include <stdint.h>

void CLI_Run(void);
void CLI_Print(const char *text);
void CLI_Start(int ok);
void CLI_Result(char command, int ok);
void CLI_Baud(int ok);

#endif /* INC_CLI_H_ */
