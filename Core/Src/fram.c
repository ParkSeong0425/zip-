/*
 * fram.c
 *
 *  Created on: Jul 10, 2026
 *      Author: kotec
 */
#include "fram.h"

extern SPI_HandleTypeDef hspi3;

// MB85RS64PNF ���ɾ�
#define WREN  0x06   // ���� ���
#define WRITE 0x02   // ������ ����
#define READ  0x03   // ������ �б�

// CS �� (CubeMX���� ���� FRAM_CS�� ����, �ٸ��� ���⸸ ����)
#define CS_LOW()  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET)

void fram_read(uint16_t addr, void *buf, uint16_t len) {
	uint8_t cmd[3] = { READ, addr >> 8, addr & 0xFF };
	CS_LOW();
	HAL_SPI_Transmit(&hspi3, cmd, 3, 100);
	HAL_SPI_Receive(&hspi3, buf, len, 100);
	CS_HIGH();
}

void fram_write(uint16_t addr, void *buf, uint16_t len) {
	uint8_t wren = WREN;
	uint8_t cmd[3] = { WRITE, addr >> 8, addr & 0xFF };

	CS_LOW();                                   // WREN�� �ܵ� ������
	HAL_SPI_Transmit(&hspi3, &wren, 1, 100);
	CS_HIGH();

	CS_LOW();
	HAL_SPI_Transmit(&hspi3, cmd, 3, 100);
	HAL_SPI_Transmit(&hspi3, buf, len, 100);
	CS_HIGH();
}

