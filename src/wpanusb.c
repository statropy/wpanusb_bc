/*
 * Copyright (c) 2016-2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_USB_DEVICE_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(wpanusb_bc, LOG_LEVEL_DBG);

#include <net/buf.h>
#include <net/ieee802154_radio.h>
#include <ieee802154/ieee802154_frame.h>
#include <net_private.h>
#include <sys/ring_buffer.h>
#include <sys/crc.h>
#include <drivers/uart.h>
#include <drivers/console/uart_mux.h>

#include "wpanusb.h"

#define ADDRESS_CTRL 0x01
#define ADDRESS_WPAN 0x03
#define ADDRESS_CDC  0x05

#define HDLC_FRAME      0x7E
#define HDLC_ESC        0x7D
#define HDLC_ESC_FRAME  0x5E
#define HDLC_ESC_ESC    0x5D

#ifndef CONFIG_WPAN_TX_STACK_SIZE
#define CONFIG_WPAN_TX_STACK_SIZE 1024
#endif

#ifndef CONFIG_WPANUSB_UART_BUF_LEN
#define CONFIG_WPANUSB_UART_BUF_LEN 8
#endif

#ifndef CONFIG_WPANUSB_RINGBUF_SIZE
#define CONFIG_WPANUSB_RINGBUF_SIZE 256
#endif

#ifndef CONFIG_WPANUSB_RX_PRIORITY
#define CONFIG_WPANUSB_RX_PRIORITY 7
#endif

#ifndef CONFIG_WPANUSB_RX_STACK_SIZE
#define CONFIG_WPANUSB_RX_STACK_SIZE 768
#endif

#ifndef CONFIG_WPANUSB_NET_BUF_SIZE
#define CONFIG_WPANUSB_NET_BUF_SIZE 100
#endif

#ifndef CONFIG_WPANUSB_UART_NAME
#define CONFIG_WPANUSB_UART_NAME "UART_1"
#endif

#define UART_BUF_LEN CONFIG_WPANUSB_UART_BUF_LEN

#define WPAN_WORKQ_PRIORITY CONFIG_WPANUSB_RX_PRIORITY
#define WPAN_WORKQ_STACK_SIZE CONFIG_WPANUSB_RX_STACK_SIZE

K_KERNEL_STACK_DEFINE(wpan_workq, WPAN_WORKQ_STACK_SIZE);
K_THREAD_STACK_DEFINE(tx_stack, CONFIG_WPAN_TX_STACK_SIZE);

struct wpan_driver_context {
	const struct device *uart_dev;
	//struct net_if *iface;

	struct ieee802154_radio_api *radio_api;
	const struct device *ieee802154_dev;

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
	uint8_t rx_buf[CONFIG_WPANUSB_RINGBUF_SIZE];

	/* ISR function callback worker */
	struct k_work cb_work;
	struct k_work_q cb_workq;

	struct k_thread tx_thread_data;
	struct k_fifo tx_queue;
	uint8_t tx_buf[IEEE802154_MTU + 1 + 1];

	uint16_t crc;

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

static struct wpan_driver_context wpan_context_data;

// #define WPANUSB_SUBCLASS	0
// #define WPANUSB_PROTOCOL	0

/* Max packet size for endpoints */
// #define WPANUSB_BULK_EP_MPS		64

// #define WPANUSB_IN_EP_IDX		0

// static struct ieee802154_radio_api *radio_api;
// static const struct device *ieee802154_dev;

//static struct k_fifo tx_queue;

/* IEEE802.15.4 frame + 1 byte len + 1 byte LQI */
// uint8_t tx_buf[IEEE802154_MTU + 1 + 1];


//const static struct device *uart_dev;
// #define UART_RX_BUF_SIZE 128
// uint8_t uart_rx_buf[UART_RX_BUF_SIZE];

/**
 * Stack for the tx thread.
 */
//static K_THREAD_STACK_DEFINE(tx_stack, 1024);
// static struct k_thread tx_thread_data;

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

// static void write_crc_byte(uint8_t byte, uint16_t *crc)
// {
// 	uart_poll_out(uart_dev, byte);
// 	*crc = crc16_ccitt(*crc, &byte, 1);
// }

// static void write_escaped_byte(uint8_t byte, uint16_t *crc)
// {
// 	if(byte == HDLC_FRAME || byte == HDLC_ESC) {
// 		write_crc_byte(HDLC_ESC, crc);
// 		write_crc_byte(byte ^ 0x20, crc);
// 	} else {
// 		write_crc_byte(byte, crc);
// 	}
// }

// static void write_packet(const uint8_t* buf, size_t len, uint8_t address)
// {
// 	uint16_t crc = 0xffff;

// 	uart_poll_out(uart_dev, HDLC_FRAME);
// 	write_escaped_byte(address, &crc);
// 	write_escaped_byte(0x03, &crc);
// 	for(int i=0; i<len; i++) {
// 		write_escaped_byte(buf[i], &crc);
// 	}

// 	uint16_t crc_calc = crc ^ 0xffff;
// 	write_escaped_byte(crc_calc, &crc);
// 	write_escaped_byte(crc_calc >> 8, &crc);
// 	uart_poll_out(uart_dev, HDLC_FRAME);

// 	LOG_DBG("Out CRC:%04x Check:%04x", crc_calc, crc);
// 	//assert crc_calc == 0xf0b8???
// }

// static void uart_recv_handler(const struct device *dev, void *user_data)
// {
// 	while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
// 		int rx, ret;

// 		rx = uart_fifo_read(uart, context->buf, sizeof(context->buf));
// 		if (rx <= 0) {
// 			continue;
// 		}

// 		ret = ring_buf_put(&context->rx_ringbuf, context->buf, rx);
// 		if (ret < rx) {
// 			LOG_ERR("Rx buffer doesn't have enough space. "
// 				"Bytes pending: %d, written: %d",
// 				rx, ret);
// 			break;
// 		}

// 		k_work_submit_to_queue(&context->cb_workq, &context->cb_work);
// 	}
// }

// static void uart_recv_process()
// {
// 	static uint8_t inEsc = false;
// 	static struct net_pkt *pkt = NULL;//net_pkt_alloc_with_buffer(NULL, 128, AF_UNSPEC, 0, K_NO_WAIT);
// 	static uint16_t crc = 0xFFFF;

// 	struct net_buf *buf;
// 	uint8_t packet_address;
// 	uint8_t packet_type;

// 		//uart_poll_in(dev, &c)????
// 		while (uart_fifo_read(dev, &c, sizeof(c))) {
// 			if(c == HDLC_FRAME) {
// 				if(pkt && crc == 0xf0b8) {
// 					buf = net_buf_frag_last(pkt->buffer);
// 					//net_pkt_hexdump(pkt, "<");
// 					packet_address = net_buf_pull_u8(buf);
// 					packet_type = net_buf_pull_u8(buf);
// 					buf->len -= 2; //drop CRC

// 					if(packet_address == ADDRESS_CTRL) {
// 						k_fifo_put(&tx_queue, pkt);
// 					} else {
// 						LOG_ERR("Dropped HDLC addr:%x type:%x", packet_address, packet_type);
// 						LOG_ERR("Packet length:%u", buf->len);
// 						net_pkt_hexdump(pkt, "<");
// 						net_pkt_unref(pkt);
// 						pkt = NULL;
// 					}
// 				} else if(crc != 0xffff) {
// 					//discard
// 					LOG_ERR("Dropped HDLC pkt:%x, crc:%x", (unsigned int)pkt, crc);
// 					if(pkt) {
// 						buf = net_buf_frag_last(pkt->buffer);
// 						LOG_ERR("Packet length:%u", buf->len);
// 						net_pkt_hexdump(pkt, "<");
// 					}
// 					net_pkt_unref(pkt);
// 					pkt = NULL;
// 				}
// 				crc = 0xffff;
// 			} else if(c == HDLC_ESC) {
// 				inEsc = true;
// 			} else {
// 				crc = crc16_ccitt(crc, &c, 1);

// 				if(inEsc) {
// 					//TODO assert c != HDLC_FRAME
// 					c ^= 0x20;
// 					inEsc = false;
// 				} else {
// 					// //TODO check buffer full
// 					// pCurrentBuffer[currentOffset] = c;
// 					// currentOffset++;
// 					if(!pkt) {
// 						pkt = net_pkt_alloc_with_buffer(NULL, 128, AF_UNSPEC, 0, K_NO_WAIT);
// 					}
// 					//LOG_DBG("0x%02x", c);
// 					net_pkt_write_u8(pkt, c);
// 				}
// 			}
// 		}
// 	}

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
// }


/* Decode wpanusb commands */

static int set_channel(struct wpan_driver_context *context)
{
	struct set_channel *req = net_buf_pull_mem(context->pkt->buffer, sizeof(struct set_channel));

	LOG_DBG("page %u channel %u", req->page, req->channel);

	return context->radio_api->set_channel(context->ieee802154_dev, req->channel);
}

static int set_ieee_addr(struct wpan_driver_context *context)
{
	struct set_ieee_addr *req = net_buf_pull_mem(context->pkt->buffer, sizeof(struct set_ieee_addr));

	LOG_DBG("%08llX", req->ieee_addr);

	if (IEEE802154_HW_FILTER &
		context->radio_api->get_capabilities(context->ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.ieee_addr = (uint8_t *)&req->ieee_addr;

		return context->radio_api->filter(context->ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_IEEE_ADDR,
					 &filter);
	}

	return 0;
}

static int set_short_addr(struct wpan_driver_context *context)
{
	struct set_short_addr *req = net_buf_pull_mem(context->pkt->buffer, sizeof(struct set_short_addr));

	LOG_DBG("%04X", req->short_addr);

	if (IEEE802154_HW_FILTER &
		context->radio_api->get_capabilities(context->ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.short_addr = req->short_addr;

		return context->radio_api->filter(context->ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_SHORT_ADDR,
					 &filter);
	}

	return 0;
}

static int set_pan_id(struct wpan_driver_context *context)
{
	struct set_pan_id *req = net_buf_pull_mem(context->pkt->buffer, sizeof(struct set_pan_id));

	LOG_DBG("%04X", req->pan_id);

	if (IEEE802154_HW_FILTER &
		context->radio_api->get_capabilities(context->ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.pan_id = req->pan_id;

		return context->radio_api->filter(context->ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_PAN_ID,
					 &filter);
	}

	return 0;
}

static int start(struct wpan_driver_context *context)
{
	LOG_INF("Start IEEE 802.15.4 device");

	return context->radio_api->start(context->ieee802154_dev);
}

static int stop(struct wpan_driver_context *context)
{
	LOG_INF("Stop IEEE 802.15.4 device");

	return context->radio_api->stop(context->ieee802154_dev);
}

static void wpan_send_packet(struct wpan_driver_context *context, struct net_pkt *pkt, uint8_t address)
{
	LOG_DBG("Send %d byte packet to %02x (%d avail) (%d rem)", net_pkt_get_len(pkt), address, net_pkt_available_buffer(pkt), net_pkt_remaining_data(pkt));
	net_pkt_hexdump(context->pkt, "send WPAN");
	net_pkt_unref(pkt);
}

static int tx(struct wpan_driver_context *context, uint8_t seq, uint16_t len)
{
	int retries = 3;
	int ret;

	LOG_DBG("len %d seq %u plen: %d rem: %d", 
		len, seq, net_pkt_get_len(context->pkt), net_pkt_remaining_data(context->pkt));

	net_pkt_hexdump(context->pkt, "tx");

	do {
		ret = context->radio_api->tx(context->ieee802154_dev, IEEE802154_TX_MODE_CSMA_CA, context->pkt, context->pkt->buffer);
	} while (ret && retries--);

	if (ret) {
		LOG_ERR("Error sending data, seq %u", seq);
		/* Send seq = 0 for unsuccessful send */
		//seq = 0U; //skip failures for now
	}

	net_pkt_set_overwrite(context->pkt, true); 
	net_pkt_cursor_init(context->pkt);
	net_pkt_write_u8(context->pkt, seq);
	net_pkt_update_length(context->pkt, 1);
	net_pkt_cursor_init(context->pkt);

	wpan_send_packet(context, context->pkt, ADDRESS_WPAN);

	return ret;
}

// static void tx_thread(void)
// {
// 	LOG_DBG("Tx thread started");

// 	while (1) {
// 		uint8_t cmd;
// 		uint16_t value;
// 		uint16_t index;
// 		uint16_t length;

// 		struct net_pkt *pkt;
// 		struct net_buf *buf;

// 		pkt = k_fifo_get(&tx_queue, K_FOREVER);
// 		buf = net_buf_frag_last(pkt->buffer);

// 		//net_pkt_hexdump(pkt, "tx_thread");

// 		net_buf_pull_u8(buf); //skip requestType
// 		cmd = net_buf_pull_u8(buf);
// 		value = net_buf_pull_le16(buf);
// 		index = net_buf_pull_le16(buf);
// 		length = net_buf_pull_le16(buf);

// 		LOG_DBG("cmd: %x v:%x i:%x l:%x", cmd, value, index, length);

// 		switch (cmd) {
// 		case RESET:
// 			LOG_DBG("Reset device");
// 			break;
// 		case TX:
// 			tx(pkt, buf, (uint8_t)index);
// 			break;
// 		case START:
// 			start();
// 			break;
// 		case STOP:
// 			stop();
// 			break;
// 		case SET_CHANNEL:
// 			set_channel(buf->data, buf->len);
// 			break;
// 		case SET_IEEE_ADDR:
// 			set_ieee_addr(buf->data, buf->len);
// 			break;
// 		case SET_SHORT_ADDR:
// 			set_short_addr(buf->data, buf->len);
// 			break;
// 		case SET_PAN_ID:
// 			set_pan_id(buf->data, buf->len);
// 			break;
// 		default:
// 			LOG_ERR("%x: Not handled for now", cmd);
// 			break;
// 		}

// 		net_pkt_unref(pkt);

// 		k_yield();
// 	}
// }

// static void init_tx_queue(void)
// {
// 	/* Transmit queue init */
// 	k_fifo_init(&tx_queue);

// 	k_thread_create(&tx_thread_data, tx_stack,
// 			K_THREAD_STACK_SIZEOF(tx_stack),
// 			(k_thread_entry_t)tx_thread,
// 			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
// }

/**
 * Interface to the network stack, will be called when the packet is
 * received
 */
int net_recv_data(struct net_if *iface, struct net_pkt *pkt)
{
// 	size_t len = net_pkt_get_len(pkt);
// 	uint8_t *p = tx_buf;
// 	int ret;
// 	//uint8_t ep;

// 	LOG_DBG("Got data, pkt %p, len %d, avail: %d", pkt, len, net_pkt_available_buffer(pkt));

// 	//net_pkt_hexdump(pkt, "<");

// 	if (len > (sizeof(tx_buf) - 2)) {
// 		LOG_ERR("Too large packet");
// 		ret = -ENOMEM;
// 		goto out;
// 	}

// 	/**
// 	 * Add length 1 byte
// 	 */
// 	*p++ = (uint8_t)len;

// 	/* This is needed to work with pkt */
// 	net_pkt_cursor_init(pkt);

// 	ret = net_pkt_read(pkt, p, len);
// 	if (ret < 0) {
// 		LOG_ERR("Cannot read pkt");
// 		goto out;
// 	}

// 	p += len;

// 	/**
// 	 * Add LQI at the end of the packet
// 	 */
// 	*p = net_pkt_ieee802154_lqi(pkt);

// 	// ep = wpanusb_config.endpoint[WPANUSB_IN_EP_IDX].ep_addr;

// 	// ret = usb_transfer_sync(ep, tx_buf, len + 2,
// 	// 			USB_TRANS_WRITE | USB_TRANS_NO_ZLP);
// 	// if (ret != len + 2) {
// 	// 	LOG_ERR("Transfer failure");
// 	// 	ret = -EINVAL;
// 	// }

// 	write_packet(tx_buf, len+2, ADDRESS_WPAN);

// out:
// 	net_pkt_unref(pkt);

//	return ret;

	wpan_send_packet(&wpan_context_data, pkt, ADDRESS_WPAN);
	return 0;
}

static void wpan_process_ctrl_frame(struct wpan_driver_context *context)
{
	uint8_t cmd;
	uint16_t value;
	uint16_t index;
	uint16_t length;

	//net_pkt_skip(context->pkt, 1); //skip requestType
	net_buf_pull_u8(context->pkt->buffer);
	cmd = net_buf_pull_u8(context->pkt->buffer);
	value = net_buf_pull_le16(context->pkt->buffer);
	index = net_buf_pull_le16(context->pkt->buffer);
	length = net_buf_pull_le16(context->pkt->buffer);

	//LOG_DBG("cmd: %x v:%x i:%x l:%x", cmd, value, index, length);
	//net_pkt_hexdump(context->pkt, "<");

	net_pkt_cursor_init(context->pkt);
	//net_pkt_set_overwrite(context->pkt, true);

	switch (cmd) {
	case RESET:
		LOG_DBG("Reset device");
		break;
	case TX:
		tx(context, (uint8_t)index, length);
		return; //DON'T UNREF PKT! Reuse for output
	case START:
		start(context);
		break;
	case STOP:
		stop(context);
		break;
	case SET_CHANNEL:
		//set_channel(buf->data, buf->len);
		set_channel(context);
		break;
	case SET_IEEE_ADDR:
		//set_ieee_addr(buf->data, buf->len);
		set_ieee_addr(context);
		break;
	case SET_SHORT_ADDR:
		//set_short_addr(buf->data, buf->len);
		set_short_addr(context);
		break;
	case SET_PAN_ID:
		set_pan_id(context);
		break;
	default:
		LOG_ERR("%x: Not handled for now", cmd);
		break;
	}

	net_pkt_unref(context->pkt);
}

static void wpan_process_frame(struct wpan_driver_context *context)
{
	if(context->pkt) {
		if(net_pkt_get_len(context->pkt) > 3 && context->crc == 0xf0b8) {
			uint8_t address = net_buf_pull_u8(context->pkt->buffer);
			uint8_t ctrl = net_buf_pull_u8(context->pkt->buffer);
			//net_buf_frag_last(context->pkt->buffer)->len -= 2; //drop crc, only works with single buf
			net_pkt_update_length(context->pkt, net_pkt_get_len(context->pkt)-2);
			net_pkt_cursor_init(context->pkt);
			// buf = net_buf_frag_last(pkt->buffer);
			// //net_pkt_hexdump(pkt, "<");
			// packet_address = net_buf_pull_u8(buf);
			// packet_type = net_buf_pull_u8(buf);
			//buf->len -= 2; //drop CRC

			//net_pkt_cursor_init(context->pkt);
			//net_pkt_set_overwrite(context->pkt, true);

			if(address == ADDRESS_CTRL && net_pkt_get_len(context->pkt) > 7) {
				//k_fifo_put(&tx_queue, pkt);
				wpan_process_ctrl_frame(context);
			} else {
				LOG_ERR("Dropped HDLC addr:%x ctrl:%x", address, ctrl);
				net_pkt_hexdump(context->pkt, "<");
				net_pkt_unref(context->pkt);
			}
		} else {
			//discard
			LOG_ERR("Dropped HDLC crc:%04x len:%d", context->crc, net_pkt_get_len(context->pkt));
			net_pkt_unref(context->pkt);
		}
	}
	context->crc = 0xffff;
	context->pkt = NULL;
}


static int wpan_save_byte(struct wpan_driver_context *context, uint8_t byte)
{
	int ret;

	if (!context->pkt) {
		context->pkt = net_pkt_rx_alloc_with_buffer(
			NULL,
			CONFIG_WPANUSB_NET_BUF_SIZE,
			AF_UNSPEC, 0, K_NO_WAIT);
		if (!context->pkt) {
			LOG_ERR("[%p] cannot allocate pkt", context);
			return -ENOMEM;
		}

		net_pkt_cursor_init(context->pkt);

		context->available = net_pkt_available_buffer(context->pkt);
	}

	/* Extra debugging can be enabled separately if really
	 * needed. Normally it would just print too much data.
	 */
	if (0) {
		LOG_DBG("Saving byte %02x", byte);
	}

	/* This is not very intuitive but we must allocate new buffer
	 * before we write a byte to last available cursor position.
	 */
	if (context->available == 1) {
		ret = net_pkt_alloc_buffer(context->pkt,
					   CONFIG_WPANUSB_NET_BUF_SIZE,
					   AF_UNSPEC, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("[%p] cannot allocate new data buffer", context);
			goto out_of_mem;
		}

		context->available = net_pkt_available_buffer(context->pkt);
	}

	if (context->available) {
		ret = net_pkt_write_u8(context->pkt, byte);
		if (ret < 0) {
			LOG_ERR("[%p] Cannot write to pkt %p (%d)",
				context, context->pkt, ret);
			goto out_of_mem;
		}

		context->available--;
	}

	return 0;

out_of_mem:
	net_pkt_unref(context->pkt);
	context->pkt = NULL;
	return -ENOMEM;
}

static int wpan_input_byte(struct wpan_driver_context *context, uint8_t byte)
{
	if(byte == HDLC_FRAME) {
		wpan_process_frame(context);
	} else {
		context->crc = crc16_ccitt(context->crc, &byte, 1);
		if(byte == HDLC_ESC) {
			context->next_escaped = true;
		} else {
			if(context->next_escaped) {
				//TODO assert byte != HDLC_FRAME
				byte ^= 0x20;
				context->next_escaped = false;
			}
			wpan_save_byte(context, byte);
		}
	}
	return 0;
	// static uint8_t inEsc = false;
	// static struct net_pkt *pkt = NULL;//net_pkt_alloc_with_buffer(NULL, 128, AF_UNSPEC, 0, K_NO_WAIT);
	// static uint16_t crc = 0xFFFF;

	// struct net_buf *buf;
	// uint8_t packet_address;
	// uint8_t packet_type;

	// if(byte == HDLC_FRAME) {
	// 	wpan_process_frame(context);
		// if(pkt && crc == 0xf0b8) {
		// 	buf = net_buf_frag_last(pkt->buffer);
		// 	//net_pkt_hexdump(pkt, "<");
		// 	packet_address = net_buf_pull_u8(buf);
		// 	packet_type = net_buf_pull_u8(buf);
		// 	buf->len -= 2; //drop CRC

		// 	if(packet_address == ADDRESS_CTRL) {
		// 		k_fifo_put(&tx_queue, pkt);
		// 	} else {
		// 		LOG_ERR("Dropped HDLC addr:%x type:%x", packet_address, packet_type);
		// 		LOG_ERR("Packet length:%u", buf->len);
		// 		net_pkt_hexdump(pkt, "<");
		// 		net_pkt_unref(pkt);
		// 		pkt = NULL;
		// 	}
		// } else if(crc != 0xffff) {
		// 	//discard
		// 	LOG_ERR("Dropped HDLC pkt:%x, crc:%x", (unsigned int)pkt, crc);
		// 	if(pkt) {
		// 		buf = net_buf_frag_last(pkt->buffer);
		// 		LOG_ERR("Packet length:%u", buf->len);
		// 		net_pkt_hexdump(pkt, "<");
		// 	}
		// 	net_pkt_unref(pkt);
		// 	pkt = NULL;
		// }
		//context->crc = 0xffff;
	// } else if(byte == HDLC_ESC) {
	// 	context->next_escaped = true;
	// } else {
	// 	crc = crc16_ccitt(crc, &byte, 1);

	// 	if(context->next_escaped) {
	// 		//TODO assert byte != HDLC_FRAME
	// 		byte ^= 0x20;
	// 		context->next_escaped = false;
	// 	} else {
	// 		if(!pkt) {
	// 			pkt = net_pkt_alloc_with_buffer(NULL, 128, AF_UNSPEC, 0, K_NO_WAIT);
	// 		}
	// 		//LOG_DBG("0x%02x", byte);
	// 		net_pkt_write_u8(pkt, byte);
	// 	}
	// }
}

static int wpan_consume_ringbuf(struct wpan_driver_context *context)
{
	uint8_t *data;
	size_t len, tmp;
	int ret;

	len = ring_buf_get_claim(&context->rx_ringbuf, &data,
				 CONFIG_WPANUSB_RINGBUF_SIZE);
	if (len == 0) {
		//LOG_DBG("Ringbuf %p is empty!", &context->rx_ringbuf);
		return 0;
	}

	/* This will print too much data, enable only if really needed */
	if (0) {
		LOG_HEXDUMP_DBG(data, len, context->uart_dev->name);
	}

	tmp = len;

	do {
		// if (wpan_input_byte(context, *data++) == 0) {
		// 	/* Ignore empty or too short frames */
		// 	if (context->pkt && net_pkt_get_len(context->pkt) > 3) {
		// 		ppp_process_msg(context);
		// 	}
		// }
		wpan_input_byte(context, *data++);
	} while (--tmp);

	ret = ring_buf_get_finish(&context->rx_ringbuf, len);
	if (ret < 0) {
		LOG_DBG("Cannot flush ring buffer (%d)", ret);
	}

	return -EAGAIN;
}

static void wpan_isr_cb_work(struct k_work *work)
{
	struct wpan_driver_context *context =
		CONTAINER_OF(work, struct wpan_driver_context, cb_work);
	int ret = -EAGAIN;

	//LOG_DBG("Worker ISR!");

	while (ret == -EAGAIN) {
		ret = wpan_consume_ringbuf(context);
	}
}

static void wpan_isr_uart(const struct device *uart, void *user_data)
{
	struct wpan_driver_context *context = user_data;
	int rx = 0, ret;

	/* get all of the data off UART as fast as we can */
	while (uart_irq_update(uart) && uart_irq_rx_ready(uart)) {
		rx = uart_fifo_read(uart, context->buf, sizeof(context->buf));
		if (rx <= 0) {
			continue;
		}

		ret = ring_buf_put(&context->rx_ringbuf, context->buf, rx);
		if (ret < rx) {
			LOG_ERR("Rx buffer doesn't have enough space. "
				"Bytes pending: %d, written: %d",
				rx, ret);
			break;
		}

		k_work_submit_to_queue(&context->cb_workq, &context->cb_work);
	}
}

void main(void)
{
	struct wpan_driver_context *context = &wpan_context_data;

	LOG_INF("Starting wpanusb");

	context->ieee802154_dev = device_get_binding(CONFIG_NET_CONFIG_IEEE802154_DEV_NAME);
	if (!context->ieee802154_dev) {
		LOG_ERR("Cannot get IEEE802.15.4 device");
		return;
	}

	/* Initialize net_pkt */
	net_pkt_init();

	/* Initialize transmit queue */
	//init_tx_queue();

	context->radio_api = (struct ieee802154_radio_api *)context->ieee802154_dev->api;


	ring_buf_init(&context->rx_ringbuf, sizeof(context->rx_buf), context->rx_buf);
	k_work_init(&context->cb_work, wpan_isr_cb_work);

	k_work_q_start(&context->cb_workq, wpan_workq,
				K_KERNEL_STACK_SIZEOF(wpan_workq),
				K_PRIO_COOP(WPAN_WORKQ_PRIORITY));
	k_thread_name_set(&context->cb_workq.thread, "wpan_workq");

	context->pkt = NULL;

	context->uart_dev = device_get_binding(CONFIG_WPANUSB_UART_NAME); //UART_1
	uart_irq_callback_user_data_set(context->uart_dev, wpan_isr_uart, context);
	uart_irq_rx_enable(context->uart_dev);

	LOG_DBG("radio_api %p initialized", context->radio_api);
}
