/* net.c : W6100 Ethernet only. Command parsing is not handled here. */
#include "net.h"
#include "save.h"
#include "cmd.h"
#include "spi.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "cmsis_os2.h"

#define NET_SPI     hspi4
#define TCP_SOCK    0

static uint16_t net_port;
static uint8_t net_mode;

/* Temporary network values for first ping test.
 * MAC will later be replaced by 24AA02E48.
 * IP settings will later be replaced by FRAM values.
 */
static wiz_NetInfo net_info = {
    .mac = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01},
    .dns = {0, 0, 0, 0},
    .ipmode = NETINFO_STATIC_V4
};

/* Hardware NSS: SPI enable -> NSS LOW */
static void cs_on(void)
{
    __HAL_SPI_ENABLE(&NET_SPI);
}

/* Hardware NSS: SPI disable -> NSS released HIGH by pull-up */
static void cs_off(void)
{
    while (__HAL_SPI_GET_FLAG(&NET_SPI, SPI_FLAG_BSY));
    __HAL_SPI_DISABLE(&NET_SPI);
}

/* SPI 1 byte read */
static uint8_t spi_rb(void)
{
    uint8_t tx = 0xFF;
    uint8_t rx = 0;

    HAL_SPI_TransmitReceive(&NET_SPI, &tx, &rx, 1, 100);
    return rx;
}

/* SPI 1 byte write */
static void spi_wb(uint8_t data)
{
    HAL_SPI_Transmit(&NET_SPI, &data, 1, 100);
}

/* SPI buffer read */
static void spi_rbuf(uint8_t *buf, datasize_t len)
{
    for (datasize_t i = 0; i < len; i++)
        buf[i] = spi_rb();
}

/* SPI buffer write */
static void spi_wbuf(uint8_t *buf, datasize_t len)
{
    if (len > 0)
        HAL_SPI_Transmit(&NET_SPI, buf, (uint16_t)len, 1000);
}

/* W6100 hardware reset */
static void w610_reset(void)
{
    HAL_GPIO_WritePin(W610_RST_GPIO_Port, W610_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    HAL_GPIO_WritePin(W610_RST_GPIO_Port, W610_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

/* Ethernet initialization for first ping test */
int NET_Init(void)
{
    uint8_t size[8] = {2, 2, 2, 2, 2, 2, 2, 2};

    /* 저장된 네트워크 값 적용 */
    net_info.ip[0] = save.fi.ip[0];
    net_info.ip[1] = save.fi.ip[1];
    net_info.ip[2] = save.fi.ip[2];
    net_info.ip[3] = save.fi.ip[3];

    net_info.sn[0] = save.fi.sn[0];
    net_info.sn[1] = save.fi.sn[1];
    net_info.sn[2] = save.fi.sn[2];
    net_info.sn[3] = save.fi.sn[3];

    net_info.gw[0] = save.fi.gw[0];
    net_info.gw[1] = save.fi.gw[1];
    net_info.gw[2] = save.fi.gw[2];
    net_info.gw[3] = save.fi.gw[3];

    net_port = save.fi.port;
    net_mode = save.fi.mode;
    reg_wizchip_cs_cbfunc(cs_on, cs_off);
    reg_wizchip_spi_cbfunc(spi_rb, spi_wb, spi_rbuf, spi_wbuf);

    w610_reset();

    if (getCIDR() != 0x6100)
        return -1;

    if (getVER() != 0x4661)
        return -2;

    if (wizchip_init(size, size) != 0)
        return -3;

    NETUNLOCK();

    wizchip_setnetinfo(&net_info);

    return 1;
}

/* Ethernet cable/link status */
int NET_Link(void)
{
    return wizphy_getphylink() == PHY_LINK_ON;
}

/* TCP Server */
void NET_Run(void)
{
    /* 현재는 Server만 처리 */
    if (net_mode != 0)
        return;

    switch (getSn_SR(TCP_SOCK))
    {
    case SOCK_CLOSED:

        socket(TCP_SOCK, Sn_MR_TCP4, net_port, 0);

        break;


    case SOCK_INIT:

        listen(TCP_SOCK);

        break;


    case SOCK_LISTEN:

        /* PC 접속 대기 */

        break;


    case SOCK_ESTABLISHED:
    {
        static char rx[160];
        static int index = 0;

        uint8_t ch;
        uint16_t len = getSn_RX_RSR(TCP_SOCK);

        while (len > 0)
        {
            if (recv(TCP_SOCK, &ch, 1) != 1)
                break;

            len--;

            /* STX */
            if (ch == 0x02)
            {
                index = 0;
            }

            /* ETX */
            else if (ch == 0x03)
            {
                rx[index] = 0;

                CMD_Run(rx);

                index = 0;
            }

            else
            {
                if (index < sizeof(rx) - 1)
                {
                    rx[index] = ch;
                    index++;
                }
            }
        }

        break;
    }
   }
}
