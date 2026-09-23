/*
 * eeprom.c
 *
 *  Created on: Sep 22, 2026
 *      Author: HWNOT
 */
#include "eeprom.h"
#include "i2c.h"

#define EEPROM_ADDR   (0x50 << 1)
#define MAC_ADDR      0xFA

/* PCB MAC 읽기 */
int EEPROM_MAC(uint8_t *mac)
{
    return HAL_I2C_Mem_Read(&hi2c3,
                            EEPROM_ADDR,
                            MAC_ADDR,
                            I2C_MEMADD_SIZE_8BIT,
                            mac,
                            6,
                            100) == HAL_OK;
}
