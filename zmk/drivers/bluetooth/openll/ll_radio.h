/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Radio interface used by the link layer. Implemented by ll_radio.c on B91,
 * faked in host tests. Ticks are 16 MHz system timer ticks.
 */
#ifndef LL_RADIO_H_
#define LL_RADIO_H_

#include <stdbool.h>
#include <stdint.h>

enum ll_radio_evt {
	LL_RADIO_TX_DONE,     /* a standalone TX (ll_radio_tx_rsp_at) finished */
	LL_RADIO_RX_OK,       /* packet with valid CRC; pdu/len/end_tick valid */
	LL_RADIO_RX_TIMEOUT,  /* nothing received in the RX window */
	LL_RADIO_RX_CRC_ERR,  /* packet received with bad CRC or length */
	/* Connection mode (ll_radio_conn_event), added in slice 2: */
	LL_RADIO_CONN_RX,     /* one CRC-valid data PDU from the central */
	LL_RADIO_CONN_DONE,   /* the connection event (BRX command) ended */
};

/* ISR context. pdu points at the 2-byte PDU header; end_tick is the system
 * timer tick at the end of the packet's last CRC bit.
 *
 * Connection mode uses different argument meanings:
 * - LL_RADIO_CONN_RX: pdu = 2-byte data PDU header + payload (still
 *   encrypted, MIC included), len = 2 + payload length, end_tick = stimer
 *   tick at the END OF THE ACCESS ADDRESS (RX DMA trailer timestamp), used
 *   for anchor re-sync. Reported once per CRC-valid packet, in order; pdu is
 *   only valid during the callback. CRC-bad packets are not reported (they
 *   are counted in ll_radio_stats.rx_crc).
 * - LL_RADIO_CONN_DONE: pdu = NULL, len = number of CRC-valid packets
 *   received in this event (0: first-RX timeout, nothing received),
 *   end_tick = stimer tick when the event ended. Exactly one per
 *   ll_radio_conn_event(), always after its LL_RADIO_CONN_RX callbacks. */
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
/* Send the prepared response so that its first bit is on air at tick.
 * Returns false (and starts nothing, so no LL_RADIO_TX_DONE follows) when
 * the TX trigger tick is already too close or in the past. */
bool ll_radio_tx_rsp_at(uint32_t tick);
void ll_radio_stop(void);

/* ---- Connection mode (slice 2; implemented in ll_radio.c, faked in host
 * tests). One connection, peripheral role, 1M PHY. ---- */
/* Per connection: access address, CRC init, TX ring base, reset_sn_nesn,
 * RX DMA ring. Call after leaving advertising, before the first event. */
void ll_radio_conn_setup(uint32_t aa, uint32_t crc_init);
/* Issue one BRX connection event on data channel ch (0..36): the RX window
 * opens at open_tick; if nothing is received within first_timeout_us the
 * event ends. The hardware then chains RX/TX exchanges while MD. Ends with
 * LL_RADIO_CONN_DONE. */
void ll_radio_conn_event(uint8_t ch, uint32_t open_tick, uint32_t first_timeout_us);
/* SN of our last transmitted packet, programmed before every BRX. */
void ll_radio_conn_set_sn_init(uint8_t sn);
/* TX FIFO (pipe 0): rptr is advanced by hardware on ack, wptr by software.
 * Both are free-running 8-bit counters; the ring has 4 entries (idx & 3). */
uint8_t ll_radio_fifo_rptr(void);
uint8_t ll_radio_fifo_wptr(void);
/* Write data PDU header byte 0 (LLID; NESN/SN/MD are set by hardware) and
 * payload (len <= LL_DATA_PDU_MAX + LL_MIC_LEN) into ring entry idx & 3. */
void ll_radio_fifo_write(uint8_t idx, uint8_t hdr0, const uint8_t *payload, uint8_t len);
void ll_radio_fifo_set_wptr(uint8_t wptr);
/* Return to advertising after a connection (baseband reset + re-init). */
void ll_radio_adv_restore(void);

/* Stall visibility: cumulative counts since boot, read from the controller
 * thread for periodic logging. Not used by the host-tested ll_adv.c. */
struct ll_radio_stats {
	uint32_t tx2rx;
	uint32_t rx_ok;
	uint32_t rx_crc;
	uint32_t rx_timeout;
	uint32_t rsp_tx;     /* SCAN_RSP TX triggered (not confirmed sent) */
	uint32_t rsp_late;   /* SCAN_RSP refused: trigger tick too late */
};
void ll_radio_get_stats(struct ll_radio_stats *s);

#endif /* LL_RADIO_H_ */
