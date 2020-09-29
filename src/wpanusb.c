/*
 * Copyright (c) 2016-2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_USB_DEVICE_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(wpanusb_bc, LOG_LEVEL_DBG);

// #include <usb/usb_device.h>
// #include <usb/usb_common.h>
// #include <usb_descriptor.h>

#include <net/buf.h>
#include <net/ieee802154_radio.h>
#include <ieee802154/ieee802154_frame.h>
#include <net_private.h>
#include <sys/ring_buffer.h>
#include <sys/crc.h>
#include <drivers/uart.h>
#include <drivers/console/uart_mux.h>

#include "wpanusb.h"

#define UART_BUF_LEN CONFIG_WPANUSB_UART_BUF_LEN

#define PPP_WORKQ_PRIORITY CONFIG_WPANUSB_RX_PRIORITY
#define PPP_WORKQ_STACK_SIZE CONFIG_WPANUSB_RX_STACK_SIZE

K_KERNEL_STACK_DEFINE(ppp_workq, PPP_WORKQ_STACK_SIZE);

struct wpan_driver_context {
	const struct device *dev;
	struct net_if *iface;

	/* This net_pkt contains pkt that is being read */
	struct net_pkt *pkt;

	/* How much free space we have in the net_pkt */
	size_t available;

	/* ppp data is read into this buf */
	uint8_t buf[UART_BUF_LEN];

	// /* ppp buf use when sending data */
	// uint8_t send_buf[UART_BUF_LEN];

	// uint8_t mac_addr[6];
	// struct net_linkaddr ll_addr;

	// /* Flag that tells whether this instance is initialized or not */
	// atomic_t modem_init_done;

	/* Incoming data is routed via ring buffer */
	struct ring_buf rx_ringbuf;
	uint8_t rx_buf[CONFIG_WPAN_RINGBUF_SIZE];

	/* ISR function callback worker */
	struct k_work cb_work;
	struct k_work_q cb_workq;

// #if defined(CONFIG_NET_STATISTICS_PPP)
// 	struct net_stats_ppp stats;
// #endif
// 	enum ppp_driver_state state;

// #if defined(CONFIG_PPP_CLIENT_CLIENTSERVER)
// 	/* correctly received CLIENT bytes */
// 	uint8_t client_index;
// #endif

// 	uint8_t init_done : 1;
	uint8_t next_escaped;
};

#define ADDRESS_CTRL 0x01
#define ADDRESS_WPAN 0x03
#define ADDRESS_CDC  0x05

#define HDLC_FRAME      0x7E
#define HDLC_ESC        0x7D
#define HDLC_ESC_FRAME  0x5E
#define HDLC_ESC_ESC    0x5D

// #define WPANUSB_SUBCLASS	0
// #define WPANUSB_PROTOCOL	0

/* Max packet size for endpoints */
// #define WPANUSB_BULK_EP_MPS		64

// #define WPANUSB_IN_EP_IDX		0

static struct ieee802154_radio_api *radio_api;
static const struct device *ieee802154_dev;

static struct k_fifo tx_queue;

/* IEEE802.15.4 frame + 1 byte len + 1 byte LQI */
uint8_t tx_buf[IEEE802154_MTU + 1 + 1];


const static struct device *uart_dev;
// #define UART_RX_BUF_SIZE 128
// uint8_t uart_rx_buf[UART_RX_BUF_SIZE];

/**
 * Stack for the tx thread.
 */
static K_THREAD_STACK_DEFINE(tx_stack, 1024);
static struct k_thread tx_thread_data;

/**
 * Vendor handler is executed in the ISR context, queue data for
 * later processing
 */
// static int wpanusb_vendor_handler(struct usb_setup_packet *setup,
// 				  int32_t *len, uint8_t **data)
// {
// 	struct net_pkt *pkt;

// 	/* Maximum 2 bytes are added to the len */
// 	pkt = net_pkt_alloc_with_buffer(NULL, *len + 2, AF_UNSPEC, 0,
// 					K_NO_WAIT);
// 	if (!pkt) {
// 		return -ENOMEM;
// 	}

// 	net_pkt_write_u8(pkt, setup->bRequest);

// 	/* Add seq to TX */
// 	if (setup->bRequest == TX) {
// 		net_pkt_write_u8(pkt, setup->wIndex);
// 	}

// 	net_pkt_write(pkt, *data, *len);

// 	LOG_DBG("pkt %p len %u seq %u", pkt, *len, setup->wIndex);

// 	k_fifo_put(&tx_queue, pkt);

// 	return 0;
// }

static void write_crc_byte(uint8_t byte, uint16_t *crc)
{
	uart_poll_out(uart_dev, byte);
	*crc = crc16_ccitt(*crc, &byte, 1);
}

static void write_escaped_byte(uint8_t byte, uint16_t *crc)
{
	if(byte == HDLC_FRAME || byte == HDLC_ESC) {
		write_crc_byte(HDLC_ESC, crc);
		write_crc_byte(byte ^ 0x20, crc);
	} else {
		write_crc_byte(byte, crc);
	}
}

static void write_packet(const uint8_t* buf, size_t len, uint8_t address)
{
	uint16_t crc = 0xffff;

	uart_poll_out(uart_dev, HDLC_FRAME);
	write_escaped_byte(address, &crc);
	write_escaped_byte(0x03, &crc);
	for(int i=0; i<len; i++) {
		write_escaped_byte(buf[i], &crc);
	}

	uint16_t crc_calc = crc ^ 0xffff;
	write_escaped_byte(crc_calc, &crc);
	write_escaped_byte(crc_calc >> 8, &crc);
	uart_poll_out(uart_dev, HDLC_FRAME);

	LOG_DBG("Out CRC:%04x Check:%04x", crc_calc, crc);
	//assert crc_calc == 0xf0b8???
}

static void uart_recv_handler(const struct device *dev, void *user_data)
{
	static uint8_t inEsc = false;
	static struct net_pkt *pkt = NULL;//net_pkt_alloc_with_buffer(NULL, 128, AF_UNSPEC, 0, K_NO_WAIT);
	static uint16_t crc = 0xFFFF;

	struct net_buf *buf;
	uint8_t packet_address;
	uint8_t packet_type;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		uint8_t c;

		if (!uart_irq_rx_ready(dev)) {
			continue;
		}

		//uart_poll_in(dev, &c)????
		while (uart_fifo_read(dev, &c, sizeof(c))) {
			if(c == HDLC_FRAME) {
				if(pkt && crc == 0xf0b8) {
					buf = net_buf_frag_last(pkt->buffer);
					//net_pkt_hexdump(pkt, "<");
					packet_address = net_buf_pull_u8(buf);
					packet_type = net_buf_pull_u8(buf);
					buf->len -= 2; //drop CRC

					if(packet_address == ADDRESS_CTRL) {
						k_fifo_put(&tx_queue, pkt);
					} else {
						LOG_ERR("Dropped HDLC addr:%x type:%x", packet_address, packet_type);
						LOG_ERR("Packet length:%u", buf->len);
						net_pkt_hexdump(pkt, "<");
						net_pkt_unref(pkt);
						pkt = NULL;
					}
				} else if(crc != 0xffff) {
					//discard
					LOG_ERR("Dropped HDLC pkt:%x, crc:%x", (unsigned int)pkt, crc);
					if(pkt) {
						buf = net_buf_frag_last(pkt->buffer);
						LOG_ERR("Packet length:%u", buf->len);
						net_pkt_hexdump(pkt, "<");
					}
					net_pkt_unref(pkt);
					pkt = NULL;
				}
				crc = 0xffff;
			} else if(c == HDLC_ESC) {
				inEsc = true;
			} else {
				crc = crc16_ccitt(crc, &c, 1);

				if(inEsc) {
					//TODO assert c != HDLC_FRAME
					c ^= 0x20;
					inEsc = false;
				} else {
					// //TODO check buffer full
					// pCurrentBuffer[currentOffset] = c;
					// currentOffset++;
					if(!pkt) {
						pkt = net_pkt_alloc_with_buffer(NULL, 128, AF_UNSPEC, 0, K_NO_WAIT);
					}
					//LOG_DBG("0x%02x", c);
					net_pkt_write_u8(pkt, c);
				}
			}
		}
	}

	// ////////////////////////////////////////////////////////////
	// struct net_pkt *pkt;

	// pkt = net_pkt_alloc_with_buffer(NULL, *off + 2, AF_UNSPEC, 0,
	// 				K_NO_WAIT);
	// if (!pkt) {
	// 	return -ENOMEM;
	// }

	// // net_pkt_write_u8(pkt, setup->bRequest);

	// // /* Add seq to TX */
	// // if (setup->bRequest == TX) {
	// // 	net_pkt_write_u8(pkt, setup->wIndex);
	// // }

	// //looking for HDLC FRAME

	// net_pkt_write(pkt, *data, *len);

	// //LOG_DBG("pkt %p len %u seq %u", pkt, *len, setup->wIndex);

	// k_fifo_put(&tx_queue, pkt);
}


/* Decode wpanusb commands */

static int set_channel(void *data, int len)
{
	struct set_channel *req = data;

	LOG_DBG("page %u channel %u", req->page, req->channel);

	return radio_api->set_channel(ieee802154_dev, req->channel);
}

static int set_ieee_addr(void *data, int len)
{
	struct set_ieee_addr *req = data;

	LOG_DBG("len %u", len);

	if (IEEE802154_HW_FILTER &
		radio_api->get_capabilities(ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.ieee_addr = (uint8_t *)&req->ieee_addr;

		return radio_api->filter(ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_IEEE_ADDR,
					 &filter);
	}

	return 0;
}

static int set_short_addr(void *data, int len)
{
	struct set_short_addr *req = data;

	LOG_DBG("len %u", len);


	if (IEEE802154_HW_FILTER &
		radio_api->get_capabilities(ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.short_addr = req->short_addr;

		return radio_api->filter(ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_SHORT_ADDR,
					 &filter);
	}

	return 0;
}

static int set_pan_id(void *data, int len)
{
	struct set_pan_id *req = data;

	LOG_DBG("len %u", len);

	if (IEEE802154_HW_FILTER &
		radio_api->get_capabilities(ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.pan_id = req->pan_id;

		return radio_api->filter(ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_PAN_ID,
					 &filter);
	}

	return 0;
}

static int start(void)
{
	LOG_INF("Start IEEE 802.15.4 device");

	return radio_api->start(ieee802154_dev);
}

static int stop(void)
{
	LOG_INF("Stop IEEE 802.15.4 device");

	return radio_api->stop(ieee802154_dev);
}

static int tx(struct net_pkt *pkt, struct net_buf *buf, uint8_t seq)
{
	//uint8_t ep = wpanusb_config.endpoint[WPANUSB_IN_EP_IDX].ep_addr;
	//struct net_buf *buf = net_buf_frag_last(pkt->buffer);
	int retries = 3;
	int ret;

	LOG_DBG("len %d seq %u", buf->len, seq);

	do {
		ret = radio_api->tx(ieee802154_dev, IEEE802154_TX_MODE_CSMA_CA,
					pkt, buf);
	} while (ret && retries--);

	if (ret) {
		LOG_ERR("Error sending data, seq %u", seq);
		/* Send seq = 0 for unsuccessful send */
		seq = 0U;
	} 

	write_packet(&seq, sizeof(seq), ADDRESS_WPAN);

	// ret = usb_transfer_sync(ep, &seq, sizeof(seq), USB_TRANS_WRITE);
	// if (ret != sizeof(seq)) {
	// 	LOG_ERR("Error sending seq");
	// 	ret = -EINVAL;
	// } else {
	// 	ret = 0;
	// }



	return ret;
}

static void tx_thread(void)
{
	LOG_DBG("Tx thread started");

	while (1) {
		uint8_t cmd;
		uint16_t value;
		uint16_t index;
		uint16_t length;

		struct net_pkt *pkt;
		struct net_buf *buf;

		pkt = k_fifo_get(&tx_queue, K_FOREVER);
		buf = net_buf_frag_last(pkt->buffer);

		//net_pkt_hexdump(pkt, "tx_thread");

		net_buf_pull_u8(buf); //skip requestType
		cmd = net_buf_pull_u8(buf);
		value = net_buf_pull_le16(buf);
		index = net_buf_pull_le16(buf);
		length = net_buf_pull_le16(buf);

		LOG_DBG("cmd: %x v:%x i:%x l:%x", cmd, value, index, length);

		switch (cmd) {
		case RESET:
			LOG_DBG("Reset device");
			break;
		case TX:
			tx(pkt, buf, (uint8_t)index);
			break;
		case START:
			start();
			break;
		case STOP:
			stop();
			break;
		case SET_CHANNEL:
			set_channel(buf->data, buf->len);
			break;
		case SET_IEEE_ADDR:
			set_ieee_addr(buf->data, buf->len);
			break;
		case SET_SHORT_ADDR:
			set_short_addr(buf->data, buf->len);
			break;
		case SET_PAN_ID:
			set_pan_id(buf->data, buf->len);
			break;
		default:
			LOG_ERR("%x: Not handled for now", cmd);
			break;
		}

		net_pkt_unref(pkt);

		k_yield();
	}
}

static void init_tx_queue(void)
{
	/* Transmit queue init */
	k_fifo_init(&tx_queue);

	k_thread_create(&tx_thread_data, tx_stack,
			K_THREAD_STACK_SIZEOF(tx_stack),
			(k_thread_entry_t)tx_thread,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
}

/**
 * Interface to the network stack, will be called when the packet is
 * received
 */
int net_recv_data(struct net_if *iface, struct net_pkt *pkt)
{
	size_t len = net_pkt_get_len(pkt);
	uint8_t *p = tx_buf;
	int ret;
	//uint8_t ep;

	LOG_DBG("Got data, pkt %p, len %d", pkt, len);

	//net_pkt_hexdump(pkt, "<");

	if (len > (sizeof(tx_buf) - 2)) {
		LOG_ERR("Too large packet");
		ret = -ENOMEM;
		goto out;
	}

	/**
	 * Add length 1 byte
	 */
	*p++ = (uint8_t)len;

	/* This is needed to work with pkt */
	net_pkt_cursor_init(pkt);

	ret = net_pkt_read(pkt, p, len);
	if (ret < 0) {
		LOG_ERR("Cannot read pkt");
		goto out;
	}

	p += len;

	/**
	 * Add LQI at the end of the packet
	 */
	*p = net_pkt_ieee802154_lqi(pkt);

	// ep = wpanusb_config.endpoint[WPANUSB_IN_EP_IDX].ep_addr;

	// ret = usb_transfer_sync(ep, tx_buf, len + 2,
	// 			USB_TRANS_WRITE | USB_TRANS_NO_ZLP);
	// if (ret != len + 2) {
	// 	LOG_ERR("Transfer failure");
	// 	ret = -EINVAL;
	// }

	write_packet(tx_buf, len+2, ADDRESS_WPAN);

out:
	net_pkt_unref(pkt);

	return ret;
}

void main(void)
{
	//int ret;
	LOG_INF("Starting wpanusb");

	ieee802154_dev = device_get_binding(CONFIG_NET_CONFIG_IEEE802154_DEV_NAME);
	if (!ieee802154_dev) {
		LOG_ERR("Cannot get IEEE802.15.4 device");
		return;
	}

	/* Initialize net_pkt */
	net_pkt_init();

	/* Initialize transmit queue */
	init_tx_queue();

	radio_api = (struct ieee802154_radio_api *)ieee802154_dev->api;

	// ret = usb_enable(NULL);
	// if (ret != 0) {
	// 	LOG_ERR("Failed to enable USB");
	// 	return;
	// }
	/* TODO: Initialize more */
	//uart_pipe_register(uart_rx_buf, UART_RX_BUF_SIZE, uart_recv_cb);
	uart_dev = device_get_binding("UART_1");

	uart_irq_callback_set(uart_dev, uart_recv_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(uart_dev);

	LOG_DBG("radio_api %p initialized", radio_api);
}
