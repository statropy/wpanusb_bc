/*
 * Copyright (c) 2016-2019 Intel Corporation
 * Copyright (c) 2020 Statropy Software LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_USB_DEVICE_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(wpanusb_bc, LOG_LEVEL_WRN);

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

#define LED2_NODE DT_ALIAS(led2)
#define LED2	DT_GPIO_LABEL(LED2_NODE, gpios)
#define PIN2	DT_GPIO_PIN(LED2_NODE, gpios)
#define FLAGS2	DT_GPIO_FLAGS(LED2_NODE, gpios)

static const struct device *dev_led0;
static const struct device *dev_led1;
static const struct device *dev_led2;

#include "wpanusb.h"

#define ADDRESS_CTRL    0x01
#define ADDRESS_WPAN    0x03
#define ADDRESS_CDC     0x05

#define HDLC_FRAME      0x7E
#define HDLC_ESC        0x7D
#define HDLC_ESC_FRAME  0x5E
#define HDLC_ESC_ESC    0x5D

#define HDLC_BUFFER_SIZE 140

struct hdlc_block {
	void *fifo_reserved;
	uint8_t address;
	uint8_t length;
	uint8_t buffer[HDLC_BUFFER_SIZE];
};

K_KERNEL_STACK_DEFINE(wpan_workq, CONFIG_WPANUSB_RX_STACK_SIZE);
K_THREAD_STACK_DEFINE(tx_stack, CONFIG_WPAN_TX_STACK_SIZE);
K_MEM_SLAB_DEFINE(hdlc_slab, sizeof(struct hdlc_block), CONFIG_WPANUSB_HDLC_NUM_BLOCKS, 4);


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
};

static struct wpan_driver_context wpan_context_data;

/* Decode wpanusb commands */

static int set_channel(struct wpan_driver_context *wpan)
{
	struct set_channel *req = (struct set_channel *)&wpan->rx_buffer[10];

	wpan->channel.page = req->page;
	wpan->channel.channel = req->channel;
	LOG_DBG("page %u channel %u", wpan->channel.page, wpan->channel.channel);
	//printk("page %u channel %u\n", wpan->channel.page, wpan->channel.channel);

	return wpan->radio_api->set_channel(wpan->ieee802154_dev, req->channel);
}

static int set_ieee_addr(struct wpan_driver_context *wpan)
{
	struct set_ieee_addr *req = (struct set_ieee_addr *)&wpan->rx_buffer[10];

	if (IEEE802154_HW_FILTER &
		wpan->radio_api->get_capabilities(wpan->ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.ieee_addr = (uint8_t *)&req->ieee_addr;

		LOG_DBG("%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
			filter.ieee_addr[0], filter.ieee_addr[1], filter.ieee_addr[2], filter.ieee_addr[3], 
			filter.ieee_addr[4], filter.ieee_addr[5], filter.ieee_addr[6], filter.ieee_addr[7]);
		// printk("%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n", 
		// 	filter.ieee_addr[0], filter.ieee_addr[1], filter.ieee_addr[2], filter.ieee_addr[3], 
		// 	filter.ieee_addr[4], filter.ieee_addr[5], filter.ieee_addr[6], filter.ieee_addr[7]);

		return wpan->radio_api->filter(wpan->ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_IEEE_ADDR,
					 &filter);
	}

	return 0;
}

static int set_short_addr(struct wpan_driver_context *wpan)
{
	struct set_short_addr *req = (struct set_short_addr *)&wpan->rx_buffer[10];

	LOG_DBG("%04X", req->short_addr);
	//printk("Short %04X\n", req->short_addr);

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
	
	LOG_DBG("%04X", req->pan_id);
	//printk("PAN %04X\n", req->pan_id);

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
	//printk("Start IEEE 802.15.4 device\n");
	int ret = wpan->radio_api->set_channel(wpan->ieee802154_dev, wpan->channel.channel);
	if(ret == 0) {
		return wpan->radio_api->start(wpan->ieee802154_dev);
	}
	return ret;
}

static int stop(struct wpan_driver_context *wpan)
{
	LOG_INF("Stop IEEE 802.15.4 device");
	//printk("Stop IEEE 802.15.4 device\n");

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
	uart_poll_out_crc(wpan->uart_dev, 0x03, &crc);
	for(int i=0; i<block_ptr->length; i++) {
		uart_poll_out_crc(wpan->uart_dev, block_ptr->buffer[i], &crc);
	}

	uint16_t crc_calc = crc ^ 0xffff;
	uart_poll_out_crc(wpan->uart_dev, crc_calc, &crc);
	uart_poll_out_crc(wpan->uart_dev, crc_calc >> 8, &crc);
	uart_poll_out(wpan->uart_dev, HDLC_FRAME);
}

static void tx_thread(void *p1)
{
	struct wpan_driver_context *wpan = p1;
	struct hdlc_block *block_ptr;
	struct hdlc_block debug_block;

	debug_block.address = ADDRESS_CDC;

	while(1) {
		block_ptr = k_fifo_get(&wpan->tx_queue, K_FOREVER);
		if (!block_ptr) {
			LOG_ERR("FIFO Get Error");
			continue;
		}

		uint32_t num_used = k_mem_slab_num_used_get(&hdlc_slab);

		// debug_block.length = sprintf(debug_block.buffer, "< %d %d|%d txq: %p, p: %p, len: %d\r\n", 
		// 	block_ptr->address,
		// 	num_used, k_mem_slab_num_free_get(&hdlc_slab),
		// 	block_ptr->fifo_reserved,
		// 	block_ptr, block_ptr->length);
		// block_out(wpan, &debug_block);
		
		block_out(wpan, block_ptr);

		//TODO: ACK Semaphore?
		k_msleep(5);

		if (num_used > CONFIG_WPANUSB_HDLC_NUM_BLOCKS) {
			LOG_ERR("SUSPENDED!");
			// debug_block.length = sprintf(debug_block.buffer, "SUSPENDED!!!\r\n");
			// block_out(wpan, &debug_block);
			k_thread_suspend(k_current_get());
		}

		if (block_ptr) {
			LOG_DBG("Free  %s: %p, u: %d len: %d", block_ptr->length == 1 ? "TX" : "RX",
				block_ptr, k_mem_slab_num_used_get(&hdlc_slab), block_ptr->length);

			// if (block_ptr->address != ADDRESS_CDC) {
			// 	printk("Free  %s: %p, u: %d len: %d\n", block_ptr->length == 1 ? "TX" : "RX",
			// 		block_ptr, k_mem_slab_num_used_get(&hdlc_slab), block_ptr->length);
			// }
			block_ptr->length = 0; //0-out before free
			k_mem_slab_free(&hdlc_slab, (void **) &block_ptr);
		} else {
			// debug_block.length = sprintf(debug_block.buffer, "TX can't free block!\r\n");
			// block_out(wpan, &debug_block);
			LOG_ERR("Can't free block!!!");
		}
		
		//k_msleep(10);
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

	do {
		ret = wpan->radio_api->tx(wpan->ieee802154_dev, IEEE802154_TX_MODE_CSMA_CA, NULL, &frag);
	} while (ret && retries--);

	if (ret) {
		LOG_ERR("Error sending data, seq %u", seq);
		//printk("Error sending data, seq %u\n", seq);
		/* Send seq = 0 for unsuccessful send */
		seq = 0U;
	}
	LOG_HEXDUMP_DBG(buf, length, "TX");

	if (k_mem_slab_alloc(&hdlc_slab, (void**)&block_ptr, K_NO_WAIT)) {
		if (dev_led1) {
			gpio_pin_set(dev_led1, PIN1, 1);
		}
		LOG_ERR("TX No Mem");
		//printk("TX No Mem\n");
		return -ENOMEM;
	}

	LOG_DBG("Alloc TX: %p, u: %d len: 1 (%d)", block_ptr, 
		k_mem_slab_num_used_get(&hdlc_slab), block_ptr->length);

	printk("TX: %02X %d\n", seq, length);

	// printk("Alloc TX: %p, u: %d len: 1 (%d)\n", block_ptr, 
	// 	k_mem_slab_num_used_get(&hdlc_slab), block_ptr->length);
	
	block_ptr->buffer[0] = seq;
	block_ptr->length = 1;
	block_ptr->address = ADDRESS_WPAN;

	// LOG_DBG("Sent Data: Seq:%d Len:%d", seq, length);
	// printk("Sent Data: Seq:%d Len:%d\n", seq, length);

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
	// k_thread_access_grant(&wpan->tx_thread_data, &wpan->tx_queue,
	// 		      &hdlc_slab, &wpan);
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
	static uint8_t seq = 1;

	gpio_pin_toggle(dev_led2, PIN2);

	LOG_DBG("Got data, pkt %p, len %d", rx_pkt, len);
	//printk("GOT data, pkt %p, len %d ", rx_pkt, len);

	/* len + packet + lqi */
	if (len + 2 > HDLC_BUFFER_SIZE) {
		LOG_ERR("RX TOO BIG");
		//printk("RX TOO BIG\n");
		return -ENOMEM;
	}

	if (k_mem_slab_alloc(&hdlc_slab, (void**)&block_ptr, K_NO_WAIT)) {// K_MSEC(5))) {
		if (dev_led1) {
			gpio_pin_set(dev_led1, PIN1, 1);
		}
		LOG_ERR("RX No Mem");
		//printk("RX No Mem\n");
		return -ENOMEM;
	}

	//block_ptr->length = 0;
	block_ptr->address = ADDRESS_WPAN;

	LOG_DBG("Alloc RX: %p, u: %d len: %d (%d)", block_ptr, 
		k_mem_slab_num_used_get(&hdlc_slab), len+2, block_ptr->length);

	printk("RX: %02X %d\n", seq, len-2); //exclude FCS because MAC layer drops it for wireshark
	seq++;
	if (!seq) {
		seq = 1;
	}

	// printk("Alloc RX: %p, u: %d len: %d (%d)\n", block_ptr, 
	// 	k_mem_slab_num_used_get(&hdlc_slab), len+2, block_ptr->length);
	//pre-pend packet length, post-pend LQI
	block_ptr->length = len + 2;
	block_ptr->buffer[0] = len;
	for(int i=1;i<=len;i++) {
		block_ptr->buffer[i] = net_buf_pull_u8(rx_pkt->buffer);
	}
	block_ptr->buffer[len+1] = net_pkt_ieee802154_lqi(rx_pkt);

	net_pkt_unref(rx_pkt);

	k_fifo_put(&wpan->tx_queue, block_ptr);
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
		//printk("Reset device\n");
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
	default:
		LOG_ERR("%x: Not handled for now", cmd);
		//printk("%x: Not handled for now\n", cmd);
		break;
	}

	if (ret != 0) {
		LOG_ERR("Command 0x%02x failed: (%d)", cmd, ret);
		//printk("Command 0x%02x failed: (%d)\n", cmd, ret);
	}
}

static void wpan_process_frame(struct wpan_driver_context *wpan)
{
	if(wpan->rx_buffer[0] == 0xEE) {
		LOG_HEXDUMP_ERR(wpan->rx_buffer, 8, "MSP430 ERROR");
		//printk("MSP430 ERROR\n");
	}
	else if (wpan->rx_buffer_len > 3 && wpan->crc == 0xf0b8) {
		uint8_t address = wpan->rx_buffer[0];
		uint8_t ctrl = wpan->rx_buffer[1];

		if (address == ADDRESS_CTRL && wpan->rx_buffer_len > 9) {
			wpan_process_ctrl_frame(wpan);
		} else if (address == ADDRESS_CDC) {
			//printk("Ignore CDC Frame\n");
			LOG_WRN("Ignore CDC Frame");
		} else {
			LOG_ERR("Dropped HDLC addr:%x ctrl:%x", address, ctrl);
			//printk("Dropped HDLC addr:%x ctrl:%x\n", address, ctrl);
			LOG_HEXDUMP_DBG(wpan->rx_buffer, wpan->rx_buffer_len, "rx_buffer");
		}
	} else {
		LOG_ERR("Dropped HDLC crc:%04x len:%d", wpan->crc, wpan->rx_buffer_len);
		//printk("Dropped HDLC crc:%04x len:%d\n", wpan->crc, wpan->rx_buffer_len);
	}

	wpan->crc = 0xffff;
	wpan->rx_buffer_len = 0;
}

static int wpan_save_byte(struct wpan_driver_context *wpan, uint8_t byte)
{
	// if (!wpan->rx_hdlc_block) {
	// 	if (k_mem_slab_alloc(&hdlc_slab, (void *) &wpan->rx_hdlc_block, K_NO_WAIT)) {
	// 		if (dev_led1) {
	// 			gpio_pin_set(dev_led1, PIN1, 1);
	// 		}
	// 		wpan->rx_hdlc_block = NULL;
	// 		return -ENOMEM;
	// 	}
	// 	wpan->rx_hdlc_block->length = 0;
	// 	//LOG_DBG("Alloc %p", wpan->rx_hdlc_block);
	// }

	if (wpan->rx_buffer_len >= HDLC_BUFFER_SIZE) {
		//printk("HDLC RX Buffer Overflow\n");
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
				//TODO assert byte != HDLC_FRAME
				//LOG_DBG("ESC: 0x%02X->", byte);
				byte ^= 0x20;
				//LOG_DBG("0x%02X", byte);
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

static uint32_t isr_count = 0;

static void wpan_isr_uart(const struct device *uart, void *user_data)
{
	struct wpan_driver_context *wpan = user_data;
	int rx = 0, ret;

	isr_count++;
	gpio_pin_toggle(dev_led0, PIN0);

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
	dev_led2 = device_get_binding(LED2);
	gpio_pin_configure(dev_led0, PIN0, GPIO_OUTPUT_ACTIVE | FLAGS0);
	gpio_pin_configure(dev_led1, PIN1, GPIO_OUTPUT_INACTIVE | FLAGS1);
	gpio_pin_configure(dev_led2, PIN2, GPIO_OUTPUT_ACTIVE | FLAGS2);

	LOG_INF("Starting wpanusb");
	// printk("Ready\n");
	/* Initialize net_pkt */
	net_pkt_init();

	/* Initialize transmit queue */
	init_tx_queue(wpan);

	ring_buf_init(&wpan->rx_ringbuf, sizeof(wpan->rx_buf), wpan->rx_buf);
	k_work_init(&wpan->cb_work, wpan_isr_cb_work);

	k_work_q_start(&wpan->cb_workq, wpan_workq,
				K_KERNEL_STACK_SIZEOF(wpan_workq),
				K_PRIO_COOP(CONFIG_WPANUSB_RX_PRIORITY));
	k_thread_name_set(&wpan->cb_workq.thread, "wpan_workq");

	wpan->uart_dev = device_get_binding(CONFIG_WPANUSB_UART_NAME); //UART_1
	if (!wpan->uart_dev) {
		while (1) {
			gpio_pin_toggle(dev_led0, PIN0);
			gpio_pin_toggle(dev_led1, PIN1);
			k_msleep(250);
		}
	}
	uart_irq_callback_user_data_set(wpan->uart_dev, wpan_isr_uart, wpan);
	uart_irq_rx_enable(wpan->uart_dev);

	wpan->ieee802154_dev = device_get_binding(CONFIG_NET_CONFIG_IEEE802154_DEV_NAME);
	if (!wpan->ieee802154_dev) {
		LOG_ERR("Cannot get IEEE802.15.4 device");
		// printk("Cannot get IEEE802.15.4 device\n");
		return;
	}

	wpan->radio_api = (struct ieee802154_radio_api *)wpan->ieee802154_dev->api;


	LOG_DBG("radio_api %p initialized", wpan->radio_api);
	printk("radio_api %p initialized\n", wpan->radio_api);

	// while (1) {
	// 	k_msleep(5000);
	// 	//wpanusb_console_out(1);
	// 	printk("C:%u B:%d\n", isr_count, k_mem_slab_num_used_get(&hdlc_slab));
	// 	//LOG_ERR("Test1");
	// }
}

#if defined(CONFIG_PRINTK) || defined(CONFIG_STDOUT_CONSOLE)

extern void __printk_hook_install(int (*fn)(int));
extern void __stdout_hook_install(int (*fn)(int));

static int wpanusb_console_out(int c)
{
	struct wpan_driver_context *wpan = &wpan_context_data;

	if (!wpan->tx_hdlc_block) {
		if (k_mem_slab_alloc(&hdlc_slab, (void *) &wpan->tx_hdlc_block, K_NO_WAIT)) {
			if (dev_led1) {
				gpio_pin_set(dev_led1, PIN1, 1);
			}
			wpan->tx_hdlc_block = NULL;
			return -ENOMEM;
		} else {
			wpan->tx_hdlc_block->length = 0;
			wpan->tx_hdlc_block->address = ADDRESS_CDC;
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
				if (dev_led1) {
					gpio_pin_set(dev_led1, PIN1, 1);
				}
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
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);//95);//CONFIG_WPANUSB_CONSOLE_INIT_PRIORITY);
/* TODO Add Kconfig for priority */
