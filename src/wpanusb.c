/*
 * Copyright (c) 2016-2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 * Modifications Copyright (c) 2020 Erik Larson
 */

#define LOG_LEVEL CONFIG_USB_DEVICE_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(wpanusb_bc, LOG_LEVEL_INF);

#include <net/buf.h>
#include <net/ieee802154_radio.h>
#include <ieee802154/ieee802154_frame.h>
#include <net_private.h>
#include <sys/ring_buffer.h>
#include <sys/crc.h>
#include <drivers/uart.h>
#include <drivers/console/uart_mux.h>

#include "wpanusb.h"

#define ADDRESS_CTRL    0x01
#define ADDRESS_WPAN    0x03
#define ADDRESS_CDC     0x05

#define HDLC_FRAME      0x7E
#define HDLC_ESC        0x7D
#define HDLC_ESC_FRAME  0x5E
#define HDLC_ESC_ESC    0x5D

K_KERNEL_STACK_DEFINE(wpan_workq, CONFIG_WPANUSB_RX_STACK_SIZE);
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
	uint8_t buf[CONFIG_WPANUSB_UART_BUF_LEN];

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

	struct set_channel channel;

	uint8_t next_escaped;
};

static struct wpan_driver_context wpan_context_data;

/* Decode wpanusb commands */

static int set_channel(struct wpan_driver_context *wpan)
{
	struct set_channel *req = net_buf_pull_mem(wpan->pkt->buffer, sizeof(struct set_channel));

	wpan->channel.page = req->page;
	wpan->channel.channel = req->channel;
	LOG_DBG("page %u channel %u", wpan->channel.page, wpan->channel.channel);

	return wpan->radio_api->set_channel(wpan->ieee802154_dev, req->channel);
}

static int set_ieee_addr(struct wpan_driver_context *wpan)
{
	struct set_ieee_addr *req = net_buf_pull_mem(wpan->pkt->buffer, sizeof(struct set_ieee_addr));

	if (IEEE802154_HW_FILTER &
		wpan->radio_api->get_capabilities(wpan->ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.ieee_addr = (uint8_t *)&req->ieee_addr;

		LOG_DBG("%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
			filter.ieee_addr[0], filter.ieee_addr[1], filter.ieee_addr[2], filter.ieee_addr[3], 
			filter.ieee_addr[4], filter.ieee_addr[5], filter.ieee_addr[6], filter.ieee_addr[7]);

		return wpan->radio_api->filter(wpan->ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_IEEE_ADDR,
					 &filter);
	}

	return 0;
}

static int set_short_addr(struct wpan_driver_context *wpan)
{
	struct set_short_addr *req = net_buf_pull_mem(wpan->pkt->buffer, sizeof(struct set_short_addr));

	LOG_DBG("%04X", req->short_addr);

	if (IEEE802154_HW_FILTER &
		wpan->radio_api->get_capabilities(wpan->ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.short_addr = req->short_addr;

		return wpan->radio_api->filter(wpan->ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_SHORT_ADDR,
					 &filter);
	}

	return 0;
}

static int set_pan_id(struct wpan_driver_context *wpan)
{
	struct set_pan_id *req = net_buf_pull_mem(wpan->pkt->buffer, sizeof(struct set_pan_id));

	LOG_DBG("%04X", req->pan_id);

	if (IEEE802154_HW_FILTER &
		wpan->radio_api->get_capabilities(wpan->ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.pan_id = req->pan_id;

		return wpan->radio_api->filter(wpan->ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_PAN_ID,
					 &filter);
	}

	return 0;
}

static int start(struct wpan_driver_context *wpan)
{
	LOG_INF("Start IEEE 802.15.4 device");
	int ret = wpan->radio_api->set_channel(wpan->ieee802154_dev, wpan->channel.channel);
	if(ret == 0) {
		return wpan->radio_api->start(wpan->ieee802154_dev);
	}
	return ret;
}

static int stop(struct wpan_driver_context *wpan)
{
	LOG_INF("Stop IEEE 802.15.4 device");

	return wpan->radio_api->stop(wpan->ieee802154_dev);
}

static void uart_poll_out_crc(const struct device *dev, uint8_t byte, uint16_t *crc)
{
	*crc = crc16_ccitt(*crc, &byte, 1);
	if(byte == HDLC_FRAME || byte == HDLC_ESC) {
		uart_poll_out(dev, HDLC_ESC);
		byte ^= 0x20;
	}
	uart_poll_out(dev, byte);
}

static void tx_thread(void *p1)
{
	struct wpan_driver_context *wpan = p1;
	struct net_pkt *pkt;
	uint8_t address;

	while(1) {
		pkt = k_fifo_get(&wpan->tx_queue, K_FOREVER);
		LOG_DBG("%d byte (%d avail) (%d rem)", net_pkt_get_len(pkt), net_pkt_available_buffer(pkt), net_pkt_remaining_data(pkt));
		//net_pkt_hexdump(pkt, "tx thread");

#ifdef WPAN_APPEND_ADDRESS
		net_pkt_set_overwrite(pkt, true); //overwrite must be true otherwise the buffer length is doubled by skip??????
		net_pkt_skip(pkt, net_pkt_get_len(pkt)-1); //Skip to last byte in buffer
		net_pkt_set_overwrite(pkt, false); //overwrite must be false here, otherwise data doesn't get written
		net_pkt_read_u8(pkt, &address); //write to end of packet
		net_pkt_update_length(pkt, net_pkt_get_len(pkt)-1);
		net_pkt_cursor_init(pkt); //reset cursor
#else
		address = pkt->priority;
#endif

		uint16_t crc = 0xffff;

		uart_poll_out(wpan->uart_dev, HDLC_FRAME);
		uart_poll_out_crc(wpan->uart_dev, address, &crc);
		uart_poll_out_crc(wpan->uart_dev, 0x03, &crc);
		for(int i=0; i<net_pkt_get_len(pkt); i++) {
			uint8_t byte;
			net_pkt_read_u8(pkt, &byte);
			uart_poll_out_crc(wpan->uart_dev, byte, &crc);
		}

		uint16_t crc_calc = crc ^ 0xffff;
		uart_poll_out_crc(wpan->uart_dev, crc_calc, &crc);
		uart_poll_out_crc(wpan->uart_dev, crc_calc >> 8, &crc);
		uart_poll_out(wpan->uart_dev, HDLC_FRAME);

		LOG_DBG("CRC:%04x Check:%04x", crc_calc, crc);
		//assert crc_calc == 0xf0b8?

		net_pkt_unref(pkt);
	}
}

static void wpan_send_packet(struct wpan_driver_context *wpan, struct net_pkt *pkt, uint8_t address)
{
	LOG_DBG("%d byte packet to %02x (%d avail) (%d rem)", net_pkt_get_len(pkt), address, net_pkt_available_buffer(pkt), net_pkt_remaining_data(pkt));
	//net_pkt_hexdump(pkt, "send WPAN");

#ifdef WPAN_APPEND_ADDRESS
	net_pkt_set_overwrite(pkt, true); //overwrite must be true otherwise the buffer length is doubled by skip??????
	net_pkt_skip(pkt, net_pkt_get_len(pkt)); //Skip to end-of-buffer
	net_pkt_set_overwrite(pkt, false); //overwrite must be false here, otherwise data doesn't get written
	net_pkt_write_u8(pkt, address); //write to end of packet
	net_pkt_cursor_init(pkt); //reset cursor
#else
	pkt->priority = address;
#endif

	k_fifo_put(&wpan->tx_queue, pkt);
}

static int tx(struct wpan_driver_context *wpan, uint8_t seq, uint16_t len)
{
	int retries = 3;
	int ret;
	struct net_buf *frag = wpan->pkt->buffer;
	struct net_buf frame_buf = {
		.data = wpan->tx_buf,
		.size = IEEE802154_MTU + 2,
		.frags = NULL,
		.len = len,
		.__buf = wpan->tx_buf,
	};

	LOG_DBG("len %d seq %u plen: %d rem: %d", 
		len, seq, net_pkt_get_len(wpan->pkt), net_pkt_remaining_data(wpan->pkt));

	//net_pkt_print_buffer_info(wpan->pkt, "tx frags");

	net_pkt_hexdump(wpan->pkt, "tx");

	if(wpan->pkt->buffer->len < len) {
		LOG_DBG("BUFFER USES MULTIPLE FRAGMENTS!!!");

		for(int i=0;i<len;i++) {
			net_pkt_read_u8(wpan->pkt, &wpan->tx_buf[i]);
		}
		frag = &frame_buf;
	}

	do {
		ret = wpan->radio_api->tx(wpan->ieee802154_dev, IEEE802154_TX_MODE_CSMA_CA, wpan->pkt, frag);
	} while (ret && retries--);

	if (ret) {
		LOG_ERR("Error sending data, seq %u", seq);
		/* Send seq = 0 for unsuccessful send */
		seq = 0U;
	}
	net_pkt_update_length(wpan->pkt, 0);
	net_pkt_cursor_init(wpan->pkt);
	net_pkt_write_u8(wpan->pkt, seq);
	net_pkt_cursor_init(wpan->pkt);

	wpan_send_packet(wpan, wpan->pkt, ADDRESS_WPAN);

	return ret;
}

static void init_tx_queue(struct wpan_driver_context *wpan)
{
	/* Transmit queue init */
	k_fifo_init(&wpan->tx_queue);

	k_thread_create(&wpan->tx_thread_data, tx_stack,
			K_THREAD_STACK_SIZEOF(tx_stack),
			(k_thread_entry_t)tx_thread,
			wpan, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
}

/**
 * Interface to the network stack, will be called when the packet is
 * received
 */
int net_recv_data(struct net_if *iface, struct net_pkt *rx_pkt)
{
	struct net_pkt *pkt;
	size_t len = net_pkt_get_len(rx_pkt);

	LOG_DBG("Got data, pkt %p, len %d, avail: %d", rx_pkt, len, net_pkt_available_buffer(rx_pkt));

	//net_pkt_print_buffer_info(rx_pkt, "rx frags");
	net_pkt_hexdump(rx_pkt, "rx");

	pkt = net_pkt_rx_alloc_with_buffer(NULL, CONFIG_WPANUSB_NET_BUF_SIZE, AF_UNSPEC, 0, K_NO_WAIT);
	if (!pkt) {
		LOG_ERR("cannot allocate pkt");
		net_pkt_unref(rx_pkt);
		return -ENOMEM;
	}

	//pre-pend packet length, post-pend LQI
	net_pkt_write_u8(pkt, len);
	for(int i=0;i<len;i++) {
		uint8_t byte = net_buf_pull_u8(rx_pkt->buffer);
		net_pkt_write_u8(pkt, byte);
	}

	net_pkt_write_u8(pkt, net_pkt_ieee802154_lqi(rx_pkt));
	net_pkt_cursor_init(pkt);

	net_pkt_unref(rx_pkt);

	wpan_send_packet(&wpan_context_data, pkt, ADDRESS_WPAN);
	return 0;
}

static void wpan_process_ctrl_frame(struct wpan_driver_context *wpan)
{
	int ret = 0;
	uint8_t cmd;
	uint16_t value;
	uint16_t index;
	uint16_t length;

	net_buf_pull_u8(wpan->pkt->buffer);
	cmd = net_buf_pull_u8(wpan->pkt->buffer);
	value = net_buf_pull_le16(wpan->pkt->buffer);
	index = net_buf_pull_le16(wpan->pkt->buffer);
	length = net_buf_pull_le16(wpan->pkt->buffer);

	//LOG_DBG("cmd: %x v:%x i:%x l:%x", cmd, value, index, length);
	//net_pkt_hexdump(wpan->pkt, "<");

	net_pkt_cursor_init(wpan->pkt);

	switch (cmd) {
	case RESET:
		LOG_DBG("Reset device");
		break;
	case TX:
		tx(wpan, (uint8_t)index, length);
		return; //DON'T UNREF PKT! Reuse for output
	case START:
		ret = start(wpan);
		break;
	case STOP:
		ret = stop(wpan);
		break;
	case SET_CHANNEL:
		ret = set_channel(wpan);
		break;
	case SET_IEEE_ADDR:
		ret = set_ieee_addr(wpan);
		break;
	case SET_SHORT_ADDR:
		ret = set_short_addr(wpan);
		break;
	case SET_PAN_ID:
		ret = set_pan_id(wpan);
		break;
	default:
		LOG_ERR("%x: Not handled for now", cmd);
		break;
	}

	if(ret != 0) {
		LOG_ERR("Command 0x%02x failed: (%d)", cmd, ret);
	}

	net_pkt_unref(wpan->pkt);
}

static void wpan_process_frame(struct wpan_driver_context *wpan)
{
	if(wpan->pkt) {
		if(net_pkt_get_len(wpan->pkt) > 3 && wpan->crc == 0xf0b8) {
			uint8_t address = net_buf_pull_u8(wpan->pkt->buffer);
			uint8_t ctrl = net_buf_pull_u8(wpan->pkt->buffer);
			net_pkt_update_length(wpan->pkt, net_pkt_get_len(wpan->pkt)-2);
			net_pkt_cursor_init(wpan->pkt);

			if(address == ADDRESS_CTRL && net_pkt_get_len(wpan->pkt) > 7) {
				wpan_process_ctrl_frame(wpan);
			} else {
				LOG_ERR("Dropped HDLC addr:%x ctrl:%x", address, ctrl);
				net_pkt_hexdump(wpan->pkt, "<");
				net_pkt_unref(wpan->pkt);
			}
		} else {
			//discard
			LOG_ERR("Dropped HDLC crc:%04x len:%d", wpan->crc, net_pkt_get_len(wpan->pkt));
			net_pkt_hexdump(wpan->pkt, "err");
			net_pkt_unref(wpan->pkt);
		}
	}
	wpan->crc = 0xffff;
	wpan->pkt = NULL;
}


static int wpan_save_byte(struct wpan_driver_context *wpan, uint8_t byte)
{
	int ret;

	if (!wpan->pkt) {
		wpan->pkt = net_pkt_rx_alloc_with_buffer(
			NULL,
			CONFIG_WPANUSB_NET_BUF_SIZE,
			AF_UNSPEC, 0, K_NO_WAIT);
		if (!wpan->pkt) {
			LOG_ERR("[%p] cannot allocate pkt", wpan);
			return -ENOMEM;
		}

		net_pkt_cursor_init(wpan->pkt);

		wpan->available = net_pkt_available_buffer(wpan->pkt);
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
	if (wpan->available == 1) {
		ret = net_pkt_alloc_buffer(wpan->pkt,
					   CONFIG_WPANUSB_NET_BUF_SIZE,
					   AF_UNSPEC, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("[%p] cannot allocate new data buffer", wpan);
			goto out_of_mem;
		}

		wpan->available = net_pkt_available_buffer(wpan->pkt);
	}

	if (wpan->available) {
		ret = net_pkt_write_u8(wpan->pkt, byte);
		if (ret < 0) {
			LOG_ERR("[%p] Cannot write to pkt %p (%d)",
				wpan, wpan->pkt, ret);
			goto out_of_mem;
		}

		wpan->available--;
	}

	return 0;

out_of_mem:
	net_pkt_unref(wpan->pkt);
	wpan->pkt = NULL;
	return -ENOMEM;
}

static void wpan_input_byte(struct wpan_driver_context *wpan, uint8_t byte)
{
	if(byte == HDLC_FRAME) {
		wpan_process_frame(wpan);
	} else {
		if(byte == HDLC_ESC) {
			wpan->next_escaped = true;
		} else {
			if(wpan->next_escaped) {
				//TODO assert byte != HDLC_FRAME
				LOG_DBG("ESC: 0x%02X->", byte);
				byte ^= 0x20;
				LOG_DBG("0x%02X", byte);
				wpan->next_escaped = false;
			}
			wpan->crc = crc16_ccitt(wpan->crc, &byte, 1);
			wpan_save_byte(wpan, byte);
		}
	}
}

static int wpan_consume_ringbuf(struct wpan_driver_context *wpan)
{
	uint8_t *data;
	size_t len, tmp;
	int ret;

	len = ring_buf_get_claim(&wpan->rx_ringbuf, &data,
				 CONFIG_WPANUSB_RINGBUF_SIZE);
	if (len == 0) {
		return 0;
	}

	/* This will print too much data, enable only if really needed */
	if (0) {
		LOG_HEXDUMP_DBG(data, len, wpan->uart_dev->name);
	}

	tmp = len;

	do {
		wpan_input_byte(wpan, *data++);
	} while (--tmp);

	ret = ring_buf_get_finish(&wpan->rx_ringbuf, len);
	if (ret < 0) {
		LOG_DBG("Cannot flush ring buffer (%d)", ret);
	}

	return -EAGAIN;
}

static void wpan_isr_cb_work(struct k_work *work)
{
	struct wpan_driver_context *wpan =
		CONTAINER_OF(work, struct wpan_driver_context, cb_work);
	int ret = -EAGAIN;

	while (ret == -EAGAIN) {
		ret = wpan_consume_ringbuf(wpan);
	}
}

static void wpan_isr_uart(const struct device *uart, void *user_data)
{
	struct wpan_driver_context *wpan = user_data;
	int rx = 0, ret;

	/* get all of the data off UART as fast as we can */
	while (uart_irq_update(uart) && uart_irq_rx_ready(uart)) {
		rx = uart_fifo_read(uart, wpan->buf, sizeof(wpan->buf));
		if (rx <= 0) {
			continue;
		}

		ret = ring_buf_put(&wpan->rx_ringbuf, wpan->buf, rx);
		if (ret < rx) {
			LOG_ERR("Rx buffer doesn't have enough space. "
				"Bytes pending: %d, written: %d",
				rx, ret);
			break;
		}

		k_work_submit_to_queue(&wpan->cb_workq, &wpan->cb_work);
	}
}

void main(void)
{
	struct wpan_driver_context *wpan = &wpan_context_data;

	LOG_INF("Starting wpanusb");

	wpan->ieee802154_dev = device_get_binding(CONFIG_NET_CONFIG_IEEE802154_DEV_NAME);
	if (!wpan->ieee802154_dev) {
		LOG_ERR("Cannot get IEEE802.15.4 device");
		return;
	}

	/* Initialize net_pkt */
	net_pkt_init();

	/* Initialize transmit queue */
	init_tx_queue(wpan);

	wpan->radio_api = (struct ieee802154_radio_api *)wpan->ieee802154_dev->api;


	ring_buf_init(&wpan->rx_ringbuf, sizeof(wpan->rx_buf), wpan->rx_buf);
	k_work_init(&wpan->cb_work, wpan_isr_cb_work);

	k_work_q_start(&wpan->cb_workq, wpan_workq,
				K_KERNEL_STACK_SIZEOF(wpan_workq),
				K_PRIO_COOP(CONFIG_WPANUSB_RX_PRIORITY));
	k_thread_name_set(&wpan->cb_workq.thread, "wpan_workq");

	wpan->pkt = NULL;

	wpan->uart_dev = device_get_binding(CONFIG_WPANUSB_UART_NAME); //UART_1
	uart_irq_callback_user_data_set(wpan->uart_dev, wpan_isr_uart, wpan);
	uart_irq_rx_enable(wpan->uart_dev);

	LOG_DBG("radio_api %p initialized", wpan->radio_api);
}
