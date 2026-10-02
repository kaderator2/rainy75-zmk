/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Radio interface used by the link layer. Implemented by ll_radio.c on B91,
 * faked in host tests. Ticks are 16 MHz system timer ticks.
 */
#ifndef LL_RADIO_H_
#define LL_RADIO_H_

#include <stdint.h>

enum ll_radio_evt {
	LL_RADIO_TX_DONE,     /* a standalone TX (ll_radio_tx_rsp_at) finished */
	LL_RADIO_RX_OK,       /* packet with valid CRC; pdu/len/end_tick valid */
	LL_RADIO_RX_TIMEOUT,  /* nothing received in the RX window */
	LL_RADIO_RX_CRC_ERR,  /* packet received with bad CRC or length */
};

/* ISR context. pdu points at the 2-byte PDU header; end_tick is the system
 * timer tick at the end of the packet's last CRC bit. */
typedef void (*ll_radio_cb_t)(enum ll_radio_evt evt, const uint8_t *pdu,
			      uint8_t len, uint32_t end_tick);

int ll_radio_init(ll_radio_cb_t cb);
uint32_t ll_radio_now(void);
void ll_radio_set_adv_channel(uint8_t ch);   /* 37..39; adv AA + CRC init */
/* TX pdu at start_tick, then listen up to rx_window_us for a reply. */
void ll_radio_tx_then_rx(const uint8_t *pdu, uint8_t len, uint32_t start_tick,
			 uint32_t rx_window_us);
/* Pre-load the response buffer (SCAN_RSP), so the IFS path only triggers. */
void ll_radio_prepare_rsp(const uint8_t *pdu, uint8_t len);
/* Send the prepared response so that its first bit is on air at tick. */
void ll_radio_tx_rsp_at(uint32_t tick);
void ll_radio_stop(void);

#endif /* LL_RADIO_H_ */
