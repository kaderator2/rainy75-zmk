/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * B91 radio for the open link layer, built on hal_telink rf.c (Apache-2.0).
 * Uses the BLE TX/RX state machine: stx2rx sends a packet and turns the
 * radio around into RX in hardware; stx sends a single packet at a tick.
 *
 * DMA buffer layout (TX): [0..3] DMA length word, [4] PDU header, [5] PDU
 * length, [6..] payload. RX adds a trailer after the payload: CRC (3),
 * timestamp (4), freq offset (2), RSSI (1), status (see ext_rf.h).
 */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include "types.h"            /* u8/u16 used by ext_rf.h */
#include "rf.h"
#include "stimer.h"
#include "ext_driver/ext_rf.h"
#include "ll_defs.h"
#include "ll_radio.h"

LOG_MODULE_DECLARE(openll, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);

#define RF_IRQ                 (IRQ_TO_L2(15) | 11)
#define DMA_BUF_SIZE           64   /* multiple of 16; 4 + 2 + 37 + 16 trailer = 59 */
/* Worst case the DMA could write if the baseband does not enforce
 * rf_set_rx_maxlen (noise packet with a full 255-byte length field; maxlen
 * enforcement is unverified on this hardware): 4 DMA length word + 4 header/
 * len + 2 + 255 payload + 16 trailer, rounded up to a multiple of 16. */
#define RX_BUF_SIZE            288  /* (4 + 4 + 2 + 255 + 16) rounded up to 16 */
#define ADV_RX_MAXLEN          37
/* Settle times from ext_rf.h (LL_TX_STL_ADV_1M, LL_SCANRSP_TX_SETTLE). */
#define TX_SETTLE_ADV_US       84
#define TX_SETTLE_RSP_US       78
/* Refuse a SCAN_RSP whose TX trigger tick is closer than this to now: a
 * trigger in the past would leave the FSM waiting and stall advertising. */
#define RSP_MIN_LEAD_TICKS     (10 * LL_TICKS_PER_US)
#define POWER_INDEX_0DBM       11   /* same index the blob shim uses, about 0 dBm */

/* The RX timestamp is taken to mark the end of the access address; the
 * packet then has header (2) + payload + CRC (3) bytes at 8 us/byte.
 * Measured (indirectly, by sniffer): consistent with the SCAN_RSP timing,
 * whose T_IFS of about 209 us is explained by a fixed TX path delay. */
#define RX_TS_TO_END_TICKS(plen) (((plen) + 2 + 3) * 8 * LL_TICKS_PER_US)

static uint8_t tx_buf[DMA_BUF_SIZE] __aligned(4);
static uint8_t rsp_buf[DMA_BUF_SIZE] __aligned(4);
static uint8_t rx_buf[RX_BUF_SIZE] __aligned(4);
static ll_radio_cb_t radio_cb;
static volatile bool rsp_in_flight;

static atomic_t cnt_tx2rx, cnt_rx_ok, cnt_rx_crc, cnt_rx_timeout, cnt_rsp_tx, cnt_rsp_late;

static void load(uint8_t *dma, const uint8_t *pdu, uint8_t len)
{
	uint32_t dlen = rf_tx_packet_dma_len(len);

	dma[0] = dlen & 0xFF;
	dma[1] = (dlen >> 8) & 0xFF;
	dma[2] = (dlen >> 16) & 0xFF;
	dma[3] = (dlen >> 24) & 0xFF;
	memcpy(&dma[4], pdu, len);
}

static void rf_isr(const void *arg)
{
	uint16_t st = reg_rf_irq_status;

	ARG_UNUSED(arg);
	rf_clr_irq_status(FLD_RF_IRQ_ALL);

	if (st & FLD_RF_IRQ_RX) {
		uint8_t *p = rx_buf;

		if (!RF_BLE_PACKET_VALIDITY_CHECK(p)) {
			atomic_inc(&cnt_rx_crc);
			radio_cb(LL_RADIO_RX_CRC_ERR, NULL, 0, 0);
			return;
		}
		uint32_t ts = ll_get_le32(&p[DMA_RFRX_OFFSET_TIME_STAMP(p)]);

		atomic_inc(&cnt_rx_ok);
		radio_cb(LL_RADIO_RX_OK, &p[DMA_RFRX_OFFSET_HEADER], p[DMA_RFRX_OFFSET_RFLEN] + 2,
			 ts + RX_TS_TO_END_TICKS(p[DMA_RFRX_OFFSET_RFLEN]));
	} else if (st & (FLD_RF_IRQ_RX_TIMEOUT | FLD_RF_IRQ_FIRST_TIMEOUT)) {
		atomic_inc(&cnt_rx_timeout);
		radio_cb(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	} else if ((st & FLD_RF_IRQ_TX) && rsp_in_flight) {
		rsp_in_flight = false;
		radio_cb(LL_RADIO_TX_DONE, NULL, 0, 0);
	}
	/* TX IRQ of a tx2rx command: the FSM continues into RX by itself. */
}

int ll_radio_init(ll_radio_cb_t cb)
{
	radio_cb = cb;
	rf_mode_init();
	rf_set_ble_1M_mode();
	rf_set_power_level_index((rf_power_level_index_e)POWER_INDEX_0DBM);
	rf_set_tx_dma(2, DMA_BUF_SIZE);
	rf_set_rx_dma(rx_buf, 0, RX_BUF_SIZE);   /* single RX FIFO entry; assumed, unverified on hardware */
	rf_set_rx_maxlen(ADV_RX_MAXLEN);
	rf_set_ble_access_code_adv();
	rf_set_ble_crc_adv();
	rf_clr_irq_status(FLD_RF_IRQ_ALL);
	rf_set_irq_mask(FLD_RF_IRQ_RX | FLD_RF_IRQ_TX | FLD_RF_IRQ_RX_TIMEOUT |
			FLD_RF_IRQ_FIRST_TIMEOUT);
	IRQ_CONNECT(RF_IRQ, 1, rf_isr, NULL, 0);
	irq_enable(RF_IRQ);
	return 0;
}

uint32_t ll_radio_now(void)
{
	return stimer_get_tick();
}

void ll_radio_set_adv_channel(uint8_t ch)
{
	rf_set_tx_rx_off_auto_mode();
	rf_set_ble_chn(ch);          /* also sets the whitening seed */
	rf_set_ble_access_code_adv();
	rf_set_ble_crc_adv();
}

void ll_radio_tx_then_rx(const uint8_t *pdu, uint8_t len, uint32_t start_tick,
			 uint32_t rx_window_us)
{
	load(tx_buf, pdu, len);
	rsp_in_flight = false;
	rf_tx_settle_us(TX_SETTLE_ADV_US);
	rf_ble_set_rx_timeout(rx_window_us);
	atomic_inc(&cnt_tx2rx);
	rf_start_stx2rx(tx_buf, start_tick);
}

void ll_radio_prepare_rsp(const uint8_t *pdu, uint8_t len)
{
	load(rsp_buf, pdu, len);
}

/* Measured: the SCAN_RSP reaches the air about 209 us after the SCAN_REQ
 * ends (59 us late), from a fixed delay in the TX path. */
bool ll_radio_tx_rsp_at(uint32_t tick)
{
	uint32_t trigger_tick = tick - TX_SETTLE_RSP_US * LL_TICKS_PER_US;

	if ((int32_t)(trigger_tick - ll_radio_now()) < RSP_MIN_LEAD_TICKS) {
		atomic_inc(&cnt_rsp_late);
		return false;
	}
	rsp_in_flight = true;
	rf_tx_settle_us(TX_SETTLE_RSP_US);
	atomic_inc(&cnt_rsp_tx);
	rf_start_stx(rsp_buf, trigger_tick);
	return true;
}

void ll_radio_get_stats(struct ll_radio_stats *s)
{
	s->tx2rx = (uint32_t)atomic_get(&cnt_tx2rx);
	s->rx_ok = (uint32_t)atomic_get(&cnt_rx_ok);
	s->rx_crc = (uint32_t)atomic_get(&cnt_rx_crc);
	s->rx_timeout = (uint32_t)atomic_get(&cnt_rx_timeout);
	s->rsp_tx = (uint32_t)atomic_get(&cnt_rsp_tx);
	s->rsp_late = (uint32_t)atomic_get(&cnt_rsp_late);
}

void ll_radio_stop(void)
{
	rsp_in_flight = false;
	rf_set_tx_rx_off_auto_mode();
}
