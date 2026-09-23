#include "fram.h"
#include "spi.h"

#define WREN   0x06
#define WRITE  0x02
#define READ   0x03


/* FRAM에 값 저장 */
int Send_FRAM(uint16_t addr, const void *data, uint16_t len)
{
    uint8_t wren = WREN;
    uint8_t cmd[3] = {
        WRITE,
        (uint8_t)(addr >> 8),
        (uint8_t)addr
    };

    /* 쓰기 허용 */
    __HAL_SPI_ENABLE(&hspi1);

    if (HAL_SPI_Transmit(&hspi1, &wren, 1, 100) != HAL_OK)
        return 0;

    while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY));
    __HAL_SPI_DISABLE(&hspi1);


    /* 주소 + 데이터 저장 */
    __HAL_SPI_ENABLE(&hspi1);

    if (HAL_SPI_Transmit(&hspi1, cmd, 3, 100) != HAL_OK)
        return 0;

    if (HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 100) != HAL_OK)
        return 0;

    while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY));
    __HAL_SPI_DISABLE(&hspi1);

    return 1;
}


/* FRAM에 저장된 값 가져오기 */
int Bring_FRAM(uint16_t addr, void *data, uint16_t len)
{
    uint8_t cmd[3] = {
        READ,
        (uint8_t)(addr >> 8),
        (uint8_t)addr
    };

    __HAL_SPI_ENABLE(&hspi1);

    if (HAL_SPI_Transmit(&hspi1, cmd, 3, 100) != HAL_OK)
        return 0;

    if (HAL_SPI_Receive(&hspi1, (uint8_t *)data, len, 100) != HAL_OK)
        return 0;

    while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY));
    __HAL_SPI_DISABLE(&hspi1);

    return 1;
}
