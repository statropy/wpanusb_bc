/*
 * Copyright (c) 2016-2019 Intel Corporation
 * Copyright (c) 2020 Statropy Software LLC
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <device.h>
#include <init.h>
#include <drivers/gpio.h>

#include <stdio.h>

#define LED0_NODE DT_ALIAS(led0)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN0	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS0	DT_GPIO_FLAGS(LED0_NODE, gpios)

#define LED1_NODE DT_ALIAS(led1)
#define LED1	DT_GPIO_LABEL(LED1_NODE, gpios)
#define PIN1	DT_GPIO_PIN(LED1_NODE, gpios)
#define FLAGS1	DT_GPIO_FLAGS(LED1_NODE, gpios)

#define GPIO15_NODE	DT_ALIAS(gpio15)
#define GPIO15		DT_GPIO_LABEL(GPIO15_NODE, gpios)
#define PIN15		DT_GPIO_PIN(GPIO15_NODE, gpios)
#define FLAGS15		DT_GPIO_FLAGS(GPIO15_NODE, gpios)

static const struct device *dev_led0;
static const struct device *dev_led1;
static const struct device *dev_gpio15;

#include "wpanusb.h"

#define ADDRESS_CTRL    0x01
#define ADDRESS_WPAN    0x03
#define ADDRESS_CDC     0x05
#define ADDRESS_HW      0x41

#define HDLC_FRAME      0x7E
#define HDLC_ESC        0x7D
#define HDLC_ESC_FRAME  0x5E
#define HDLC_ESC_ESC    0x5D

#define HDLC_BUFFER_SIZE 140

struct hdlc_block {
	void *fifo_reserved;
	uint8_t address;
	uint8_t ctrl;
	uint8_t length;
	uint8_t buffer[HDLC_BUFFER_SIZE];
};

K_KERNEL_STACK_DEFINE(wpan_workq, CONFIG_WPANUSB_RX_STACK_SIZE);
K_THREAD_STACK_DEFINE(tx_stack, CONFIG_WPAN_TX_STACK_SIZE);
K_MEM_SLAB_DEFINE(hdlc_slab, sizeof(struct hdlc_block), CONFIG_WPANUSB_HDLC_NUM_BLOCKS, 4);
K_SEM_DEFINE(hdlc_sem, 0, 1);

struct wpan_driver_context {
	const struct device *uart_dev;

	struct ieee802154_radio_api *radio_api;
	const struct device *ieee802154_dev;

	uint8_t rx_buffer_len;
	uint8_t rx_buffer[HDLC_BUFFER_SIZE];
	struct hdlc_block *tx_hdlc_block;

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

	uint16_t crc;

	struct set_channel channel;

	uint8_t next_escaped;

	uint8_t hdlc_send_seq;
	uint8_t hdlc_rx_send_seq;
};

static struct wpan_driver_context wpan_context_data;

/* Decode wpanusb commands */

static int set_channel(struct wpan_driver_context *wpan)
{
	struct set_channel *req = (struct set_channel *)&wpan->rx_buffer[10];

	wpan->channel.page = req->page;
	wpan->channel.channel = req->channel;
	LOG_INF("page %u channel %u", wpan->channel.page, wpan->channel.channel);
	return wpan->radio_api->set_channel(wpan->ieee802154_dev, req->channel);
}

static int set_ieee_addr(struct wpan_driver_context *wpan)
{
	struct set_ieee_addr *req = (struct set_ieee_addr *)&wpan->rx_buffer[10];

	if (IEEE802154_HW_FILTER &
		wpan->radio_api->get_capabilities(wpan->ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.ieee_addr = (uint8_t *)&req->ieee_addr;

		LOG_INF("%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
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
	struct set_short_addr *req = (struct set_short_addr *)&wpan->rx_buffer[10];

	LOG_INF("short addr : %04X", req->short_addr);

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
	struct set_pan_id *req = (struct set_pan_id *)&wpan->rx_buffer[10];
	
	LOG_INF("pan id : %04X", req->pan_id);

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

static int get_supported_channels(struct wpan_driver_context *wpan)
{	struct hdlc_block *block_ptr;
	uint32_t valid_channels = CONFIG_WPANUSB_DEVICE_VALID_CHANNELS;

	if (k_mem_slab_alloc(&hdlc_slab, (void**)&block_ptr, K_NO_WAIT)) {
		LOG_ERR("RX No Mem");
		return -ENOMEM;
	}
	block_ptr->address = ADDRESS_WPAN;
	block_ptr->ctrl = 1;
	block_ptr->length = sizeof(uint32_t);
	memcpy(block_ptr->buffer, (char*)&valid_channels, sizeof(uint32_t));
	k_fifo_put(&wpan->tx_queue, block_ptr);
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

static void block_out(struct wpan_driver_context *wpan, struct hdlc_block *block_ptr)
{
	uint16_t crc = 0xffff;

	uart_poll_out(wpan->uart_dev, HDLC_FRAME);
	uart_poll_out_crc(wpan->uart_dev, block_ptr->address, &crc);
	
	if (block_ptr->ctrl == 0) {
		uart_poll_out_crc(wpan->uart_dev, wpan->hdlc_send_seq << 1, &crc);
	} else {
		uart_poll_out_crc(wpan->uart_dev, block_ptr->ctrl, &crc);
	}
	
	for(int i=0; i<block_ptr->length; i++) {
		uart_poll_out_crc(wpan->uart_dev, block_ptr->buffer[i], &crc);
	}

	uint16_t crc_calc = crc ^ 0xffff;
	uart_poll_out_crc(wpan->uart_dev, crc_calc, &crc);
	uart_poll_out_crc(wpan->uart_dev, crc_calc >> 8, &crc);
	uart_poll_out(wpan->uart_dev, HDLC_FRAME);
}

static void init_hdlc(struct wpan_driver_context *wpan)
{
	struct hdlc_block block;
	block.address = ADDRESS_HW;
	block.length = 0;
	block.ctrl = 0;

	while(1) {
		block_out(wpan, &block);
		if (k_sem_take(&hdlc_sem, K_MSEC(100))) {
			//pulse the BOOT line
			gpio_pin_set(dev_gpio15, PIN15, 0);
			k_msleep(100);
			gpio_pin_set(dev_gpio15, PIN15, 1);
			k_msleep(500);
			LOG_ERR("RETRY HDLC INIT");
		} else {
			LOG_INF("HDLC Ready");
			wpan->hdlc_rx_send_seq = 1;
			return;
		}
	}
}

static void tx_thread(void *p1)
{
	struct wpan_driver_context *wpan = p1;
	struct hdlc_block *block_ptr;
	struct hdlc_block debug_block;

	debug_block.address = ADDRESS_CDC;

	init_hdlc(wpan);

	while(1) {
		block_ptr = k_fifo_get(&wpan->tx_queue, K_FOREVER);
		if (!block_ptr) {
			LOG_ERR("FIFO Get Error");
			continue;
		}

		uint8_t block_pending = 0;
		int retries = 0;
		int wait_time = 15;

		do {
			block_out(wpan, block_ptr);

			if (block_ptr->ctrl == 0) {
				if (k_sem_take(&hdlc_sem, K_MSEC(wait_time))) {
					if (retries++ < 3) {
						LOG_DBG("I-Frame Retry");
						wait_time += 5;
						block_pending = 1;
					} else if (wpan->hdlc_rx_send_seq == ((wpan->hdlc_send_seq + 1) & 0x07)) {
						LOG_DBG("I-Frame Seq Retry");
						wait_time += 5;
						block_pending = 1;
					} else {
						LOG_ERR("No ACK, Drop Packet");
						block_pending = 0;
					}
				} else {
					wpan->hdlc_rx_send_seq = (wpan->hdlc_rx_send_seq + 1) & 0x07;
					block_pending = 0;
				}
			}
		} while (block_pending);

		k_mem_slab_free(&hdlc_slab, (void **) &block_ptr);
	}
}

static int tx(struct wpan_driver_context *wpan)
{
	uint16_t index = (wpan->rx_buffer[7] << 8) | wpan->rx_buffer[6];
	uint16_t length = (wpan->rx_buffer[9] << 8) | wpan->rx_buffer[8];
	uint8_t *buf = &wpan->rx_buffer[10];
	int retries = 3;
	int ret;
	struct net_buf frag = {
		.data = buf,
		.size = IEEE802154_MTU + 2,
		.frags = NULL,
		.len = length,
		.__buf = buf,
	};
	struct hdlc_block *block_ptr;
	uint8_t seq = (uint8_t)index;

	if (dev_led0) {
		gpio_pin_set(dev_led0, PIN0, 1);
	}

	do {
		ret = wpan->radio_api->tx(wpan->ieee802154_dev, IEEE802154_TX_MODE_CSMA_CA, NULL, &frag);
	} while (ret && retries--);

	if (dev_led0) {
		gpio_pin_set(dev_led0, PIN0, 0);
	}

	if (ret) {
		LOG_ERR("Error sending data, seq %u", seq);
		seq = 0U;
	}

	LOG_HEXDUMP_DBG(buf, length, "TX");

	if (k_mem_slab_alloc(&hdlc_slab, (void**)&block_ptr, K_NO_WAIT)) {
		LOG_ERR("TX No Mem");
		return -ENOMEM;
	}

	LOG_DBG("Alloc TX: %p, u: %d len: 1 (%d)", block_ptr, 
		k_mem_slab_num_used_get(&hdlc_slab), block_ptr->length);

	printk("TX: %d\n", length);
	
	block_ptr->buffer[0] = seq;
	block_ptr->length = 1;
	block_ptr->address = ADDRESS_WPAN;
	block_ptr->ctrl = 0;

	k_fifo_put(&wpan->tx_queue, block_ptr);

	return ret;
}

static void init_tx_queue(struct wpan_driver_context *wpan)
{
	/* Transmit queue init */
	k_fifo_init(&wpan->tx_queue);

	k_thread_create(&wpan->tx_thread_data, tx_stack,
			K_THREAD_STACK_SIZEOF(tx_stack),
			(k_thread_entry_t)tx_thread,
			wpan, NULL, NULL, K_PRIO_COOP(8), 0, K_FOREVER);
	k_thread_name_set(&wpan->tx_thread_data, "wpan_hdlc_tx");
	k_thread_start(&wpan->tx_thread_data);
}

/**
 * Interface to the network stack, will be called when the packet is
 * received
 */
int net_recv_data(struct net_if *iface, struct net_pkt *rx_pkt)
{
	struct hdlc_block *block_ptr;
	struct wpan_driver_context *wpan = &wpan_context_data;
	size_t len = net_pkt_get_len(rx_pkt);

	LOG_DBG("Got data, pkt %p, len %d", rx_pkt, len);

	/* len + packet + lqi */
	if (len + 2 > HDLC_BUFFER_SIZE) {
		LOG_ERR("RX TOO BIG");
		return -ENOMEM;
	}

	if (k_mem_slab_alloc(&hdlc_slab, (void**)&block_ptr, K_NO_WAIT)) {
		LOG_ERR("RX No Mem");
		return -ENOMEM;
	}

	if (dev_led1) {
		gpio_pin_set(dev_led1, PIN1, 1);
	}

	block_ptr->address = ADDRESS_WPAN;
	block_ptr->ctrl = 0;

	LOG_DBG("Alloc RX: %p, u: %d len: %d (%d)", block_ptr, 
		k_mem_slab_num_used_get(&hdlc_slab), len+2, block_ptr->length);

	printk("RX: %d\n", len-2); //exclude FCS because MAC layer drops it for wireshark

	//pre-pend packet length, post-pend LQI
	block_ptr->length = len + 2;
	block_ptr->buffer[0] = len;
	for(int i=1;i<=len;i++) {
		block_ptr->buffer[i] = net_buf_pull_u8(rx_pkt->buffer);
	}
	block_ptr->buffer[len+1] = net_pkt_ieee802154_lqi(rx_pkt);

	net_pkt_unref(rx_pkt);

	k_fifo_put(&wpan->tx_queue, block_ptr);

	if (dev_led1) {
		gpio_pin_set(dev_led1, PIN1, 0);
	}

	return 0;
}

static void wpan_process_ctrl_frame(struct wpan_driver_context *wpan)
{
	int ret = 0;
	// uint8_t *buf = wpan->rx_buffer;
	uint8_t cmd = wpan->rx_buffer[3];
	// //uint16_t value = (buf[4] << 8) | buf[5];
	// uint16_t index = (buf[7] << 8) | buf[6];
	// uint16_t length = (buf[9] << 8) | buf[8];

	switch (cmd) {
	case RESET:
		LOG_DBG("Reset device");
		break;
	case TX:
		ret = tx(wpan);
		break;
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
	case GET_SUPPORTED_CHANNELS:
		ret = get_supported_channels(wpan);
		break;
	default:
		LOG_ERR("%x: Not handled for now", cmd);
		break;
	}

	if (ret != 0) {
		LOG_ERR("Command 0x%02x failed: (%d)", cmd, ret);
	}
}

static void wpan_process_frame(struct wpan_driver_context *wpan)
{
	if(wpan->rx_buffer[0] == 0xEE) {
		LOG_HEXDUMP_ERR(wpan->rx_buffer, 8, "HDLC ERROR");
	}
	else if (wpan->rx_buffer_len > 3 && wpan->crc == 0xf0b8) {
		uint8_t address = wpan->rx_buffer[0];
		uint8_t ctrl = wpan->rx_buffer[1];

		if ((ctrl & 1) == 0) {
			wpan->hdlc_rx_send_seq = (ctrl >> 5) & 0x07;
			k_sem_give(&hdlc_sem);
		} else if (address == ADDRESS_CTRL && wpan->rx_buffer_len > 9) {
			wpan_process_ctrl_frame(wpan);
		} else if (address == ADDRESS_CDC) {
			LOG_WRN("Ignore CDC Frame");
		} else {
			LOG_ERR("Dropped HDLC addr:%x ctrl:%x", address, ctrl);
			LOG_HEXDUMP_DBG(wpan->rx_buffer, wpan->rx_buffer_len, "rx_buffer");
		}
	} else {
		LOG_ERR("Dropped HDLC crc:%04x len:%d", wpan->crc, wpan->rx_buffer_len);
	}

	wpan->crc = 0xffff;
	wpan->rx_buffer_len = 0;
}

static int wpan_save_byte(struct wpan_driver_context *wpan, uint8_t byte)
{
	if (wpan->rx_buffer_len >= HDLC_BUFFER_SIZE) {
		LOG_ERR("HDLC RX Buffer Overflow");
		wpan->crc = 0xffff;
		wpan->rx_buffer_len = 0;
	}

	wpan->rx_buffer[wpan->rx_buffer_len++] = byte;

	return 0;
}

static void wpan_input_byte(struct wpan_driver_context *wpan, uint8_t byte)
{
	if(byte == HDLC_FRAME) {
		if (wpan->rx_buffer_len) {
			wpan_process_frame(wpan);
		}
	} else {
		if(byte == HDLC_ESC) {
			wpan->next_escaped = true;
		} else {
			if(wpan->next_escaped) {
				byte ^= 0x20;
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

	tmp = len;

	do {
		wpan_input_byte(wpan, *data++);
	} while (--tmp);

	ret = ring_buf_get_finish(&wpan->rx_ringbuf, len);
	if (ret < 0) {
		LOG_ERR("Cannot flush ring buffer (%d)", ret);
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
	dev_led0 = device_get_binding(LED0);
	dev_led1 = device_get_binding(LED1);
	dev_gpio15 = device_get_binding(GPIO15);

	gpio_pin_configure(dev_led0, PIN0, GPIO_OUTPUT_INACTIVE | FLAGS0);
	gpio_pin_configure(dev_led1, PIN1, GPIO_OUTPUT_INACTIVE | FLAGS1);
	/* Increase drive-strength to overcome strong pull-up */
	gpio_pin_configure(dev_gpio15, PIN15, GPIO_OUTPUT_ACTIVE | GPIO_DS_ALT_LOW | GPIO_DS_ALT_HIGH | FLAGS15);

	LOG_INF("Starting wpanusb");

	net_pkt_init();

	ring_buf_init(&wpan->rx_ringbuf, sizeof(wpan->rx_buf), wpan->rx_buf);
	k_work_init(&wpan->cb_work, wpan_isr_cb_work);

	k_work_q_start(&wpan->cb_workq, wpan_workq,
				K_KERNEL_STACK_SIZEOF(wpan_workq),
				K_PRIO_COOP(CONFIG_WPANUSB_RX_PRIORITY));
	k_thread_name_set(&wpan->cb_workq.thread, "wpan_workq");

	wpan->uart_dev = device_get_binding(CONFIG_WPANUSB_UART_NAME);
	if (!wpan->uart_dev) {
		LOG_ERR("Cannot get UART");
		return;
	}

	init_tx_queue(wpan);

	uart_irq_callback_user_data_set(wpan->uart_dev, wpan_isr_uart, wpan);
	uart_irq_rx_enable(wpan->uart_dev);

	wpan->ieee802154_dev = device_get_binding(CONFIG_NET_CONFIG_IEEE802154_DEV_NAME);
	if (!wpan->ieee802154_dev) {
		LOG_ERR("Cannot get IEEE802.15.4 device");
		return;
	}

	wpan->radio_api = (struct ieee802154_radio_api *)wpan->ieee802154_dev->api;

	LOG_DBG("radio_api %p initialized", wpan->radio_api);
}

#if defined(CONFIG_PRINTK) || defined(CONFIG_STDOUT_CONSOLE)

extern void __printk_hook_install(int (*fn)(int));
extern void __stdout_hook_install(int (*fn)(int));

static int wpanusb_console_out(int c)
{
	struct wpan_driver_context *wpan = &wpan_context_data;

	if (!wpan->tx_hdlc_block) {
		if (k_mem_slab_alloc(&hdlc_slab, (void *) &wpan->tx_hdlc_block, K_NO_WAIT)) {
			wpan->tx_hdlc_block = NULL;
			return -ENOMEM;
		} else {
			wpan->tx_hdlc_block->length = 0;
			wpan->tx_hdlc_block->address = ADDRESS_CDC;
			wpan->tx_hdlc_block->ctrl = 0x03;
		}
	}

	int send = false;
	int cr_lf = false;
	int len = wpan->tx_hdlc_block->length;

	if ('\n' == c)
	{
		send = true;
		if (len >= HDLC_BUFFER_SIZE-3) {
			cr_lf = true;
		} else {
			wpan->tx_hdlc_block->buffer[wpan->tx_hdlc_block->length++] = '\r';
			wpan->tx_hdlc_block->buffer[wpan->tx_hdlc_block->length++] = '\n';
		}
	} else {
		wpan->tx_hdlc_block->buffer[wpan->tx_hdlc_block->length++] = c;
		if (len >= HDLC_BUFFER_SIZE-2) {
			send = true;
		}
	}

	if (send) {
		k_fifo_put(&wpan->tx_queue, wpan->tx_hdlc_block);
		wpan->tx_hdlc_block = NULL;

		if (cr_lf) {
			if (k_mem_slab_alloc(&hdlc_slab, (void *) &wpan->tx_hdlc_block, K_NO_WAIT)) {
				wpan->tx_hdlc_block = NULL;
				return -ENOMEM;
			} else {
				wpan->tx_hdlc_block->address = ADDRESS_CDC;
			}

			wpan->tx_hdlc_block->buffer[wpan->tx_hdlc_block->length++] = '\r';
			wpan->tx_hdlc_block->buffer[wpan->tx_hdlc_block->length++] = '\n';

			k_fifo_put(&wpan->tx_queue, wpan->tx_hdlc_block);
			wpan->tx_hdlc_block = NULL;
		}
	}

	return c;
}
#endif

/**
 * @brief Initialize the console/debug port
 * @return 0 if successful, otherwise failed.
 */
static int wpanusb_console_init(const struct device *arg)
{
	ARG_UNUSED(arg);
	__stdout_hook_install(wpanusb_console_out);
	__printk_hook_install(wpanusb_console_out);

	return 0;
}

/* UART console initializes after the UART device itself */
SYS_INIT(wpanusb_console_init,
	 APPLICATION,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
