#include "tcp.h"
#include "save.h"
#include "motor.h"
#include "wizchip_conf.h"
#include "socket.h"

#include <stdint.h>
#include <string.h>

extern SPI_HandleTypeDef hspi1;

/* ===================== W6100 ===================== */

#define CS_PORT   GPIOA
#define CS_PIN    GPIO_PIN_0
#define RST_PORT  GPIOE
#define RST_PIN   GPIO_PIN_0

#define SOCK_NUM    0
#define LOCAL_PORT  2500
#define BUF_SIZE    64

static uint8_t rx_buf[BUF_SIZE + 1];
static char line_buf[BUF_SIZE + 1];
static int line_len = 0;

static uint8_t sock_status = SOCK_CLOSED;

/* ===================== W6100 SPI ===================== */

static void w6100_cs_sel(void)
{
	HAL_GPIO_WritePin(
			CS_PORT,
			CS_PIN,
			GPIO_PIN_RESET);
}

static void w6100_cs_desel(void)
{
	HAL_GPIO_WritePin(
			CS_PORT,
			CS_PIN,
			GPIO_PIN_SET);
}

static uint8_t w6100_spi_rb(void)
{
	uint8_t tx = 0xFF;
	uint8_t rx = 0;

	HAL_SPI_TransmitReceive(
			&hspi1,
			&tx,
			&rx,
			1,
			100);

	return rx;
}

static void w6100_spi_wb(uint8_t b)
{
	HAL_SPI_Transmit(
			&hspi1,
			&b,
			1,
			100);
}

static void w6100_spi_rbuf(
		uint8_t *buf,
		datasize_t len)
{
	uint8_t tx = 0xFF;

	for (datasize_t i = 0; i < len; i++) {
		HAL_SPI_TransmitReceive(
				&hspi1,
				&tx,
				&buf[i],
				1,
				100);
	}
}

static void w6100_spi_wbuf(
		uint8_t *buf,
		datasize_t len)
{
	HAL_SPI_Transmit(
			&hspi1,
			buf,
			(uint16_t)len,
			100);
}

static void w6100_reset(void)
{
	HAL_GPIO_WritePin(
			RST_PORT,
			RST_PIN,
			GPIO_PIN_RESET);

	HAL_Delay(1);

	HAL_GPIO_WritePin(
			RST_PORT,
			RST_PIN,
			GPIO_PIN_SET);

	HAL_Delay(65);
}

/* ===================== 네트워크 초기화 ===================== */

static void net_init(void)
{
	uint8_t mem[16] = {
			2, 2, 2, 2,
			2, 2, 2, 2,
			2, 2, 2, 2,
			2, 2, 2, 2
	};

	wiz_NetInfo net;

	net.mac[0] = 0x00;
	net.mac[1] = 0x08;
	net.mac[2] = 0xDC;
	net.mac[3] = 0x11;
	net.mac[4] = 0x11;
	net.mac[5] = 0x13;

	net.ip[0] = 172;
	net.ip[1] = 20;
	net.ip[2] = 0;
	net.ip[3] = 101;

	net.sn[0] = 255;
	net.sn[1] = 255;
	net.sn[2] = 0;
	net.sn[3] = 0;

	net.gw[0] = 172;
	net.gw[1] = 20;
	net.gw[2] = 0;
	net.gw[3] = 1;

	net.dns[0] = 8;
	net.dns[1] = 8;
	net.dns[2] = 8;
	net.dns[3] = 8;

	net.ipmode = NETINFO_STATIC_V4;

	w6100_cs_desel();

	HAL_GPIO_WritePin(
			RST_PORT,
			RST_PIN,
			GPIO_PIN_SET);

	reg_wizchip_cs_cbfunc(
			w6100_cs_sel,
			w6100_cs_desel);

	reg_wizchip_spi_cbfunc(
			w6100_spi_rb,
			w6100_spi_wb,
			w6100_spi_rbuf,
			w6100_spi_wbuf);

	w6100_reset();

	ctlwizchip(
			CW_INIT_WIZCHIP,
			mem);

	{
		uint8_t lock = SYS_NET_LOCK;

		ctlwizchip(
				CW_SYS_UNLOCK,
				&lock);
	}

	ctlnetwork(
			CN_SET_NETINFO,
			&net);
}

/* ===================== TCP 응답 ===================== */

void tcp_reply(const char *s)
{
	/* 기존 STM 콘솔 출력 유지 */
	print(s);

	if (sock_status == SOCK_ESTABLISHED) {
		send(
				SOCK_NUM,
				(uint8_t *)s,
				(uint16_t)strlen(s));
	}
}

/* ===================== 초기화 ===================== */

void tcp_init(void)
{
	net_init();
}

/* ===================== TCP 실행 ===================== */

void tcp_run(void)
{
	datasize_t received_size;

	getsockopt(
			SOCK_NUM,
			SO_STATUS,
			&sock_status);

	switch (sock_status) {
	case SOCK_ESTABLISHED:

		getsockopt(
				SOCK_NUM,
				SO_RECVBUF,
				&received_size);

		if (received_size > 0) {
			int32_t n;

			if (received_size > BUF_SIZE) {
				received_size = BUF_SIZE;
			}

			n = recv(
					SOCK_NUM,
					rx_buf,
					received_size);

			if (n <= 0) {
				break;
			}

			for (int32_t i = 0; i < n; i++) {
				char c = (char)rx_buf[i];

				if (c == '\r' ||
					c == '\n') {

					if (line_len > 0) {
						line_buf[line_len] = '\0';

						save_cmd(line_buf);

						line_len = 0;
					}
				}
				else if (c == 0x08 ||
						 c == 0x7F) {

					if (line_len > 0) {
						line_len--;
					}
				}
				else if (line_len < BUF_SIZE) {
					line_buf[line_len++] = c;
				}
			}
		}
		break;

	case SOCK_CLOSE_WAIT:

		disconnect(SOCK_NUM);
		line_len = 0;
		break;

	case SOCK_INIT:

		listen(SOCK_NUM);
		break;

	case SOCK_CLOSED:

		socket(
				SOCK_NUM,
				Sn_MR_TCP4,
				LOCAL_PORT,
				SOCK_IO_NONBLOCK);

		line_len = 0;
		break;

	default:
		break;
	}
}
