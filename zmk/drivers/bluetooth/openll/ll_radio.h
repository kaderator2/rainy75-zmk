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
	LL_RADIO_CONN_RX_CRC_ERR, /* a packet with a bad CRC in a connection event */
	LL_RADIO_CONN_RX_NODATA,  /* a packet received without an RX entry (Task 10) */
};

/* ISR context. pdu points at the 2-byte PDU header; end_tick is the system
 * timer tick at the end of the packet's last CRC bit.
 *
 * Connection mode uses different argument meanings:
 * - LL_RADIO_CONN_RX: pdu = 2-byte data PDU header + payload (still
 *   encrypted, MIC included), len = 2 + payload length, end_tick = stimer
 *   tick at the END OF THE ACCESS ADDRESS (RX DMA trailer timestamp), used
 *   for anchor re-sync. Reported once per CRC-valid packet, in order; pdu is
 *   only valid during the callback.
 * - LL_RADIO_CONN_RX_CRC_ERR: a packet with a bad CRC (also counted in
 *   ll_radio_stats.rx_crc); pdu = NULL, len = 0, end_tick = its RX DMA
 *   timestamp. Reported in packet order with the CONN_RX callbacks, so the
 *   receiver knows which packet was the event's first (only that one marks
 *   the anchor).
 * - LL_RADIO_CONN_RX_NODATA: an RX IRQ with no new RX DMA entry, before
 *   any entry of the event: the hardware received a CRC-valid packet it
 *   does not deliver, in practice the central's retransmission of a packet
 *   we already have (acked via NESN, not written to the RX FIFO). pdu =
 *   NULL, len = 0, end_tick = stimer tick at the IRQ. Like CONN_RX_CRC_ERR
 *   it tells the receiver that the event's first packet (the anchor) has
 *   passed, so a later chained packet does not re-anchor.
 * - LL_RADIO_CONN_DONE: pdu = NULL, len = number of CRC-valid packets
 *   received in this event (0: first-RX timeout, nothing received),
 *   end_tick = stimer tick when the event ended. Exactly one per
 *   ll_radio_conn_event(), always after its LL_RADIO_CONN_RX callbacks. */
typedef void (*ll_radio_cb_t)(enum ll_radio_evt evt, const uint8_t *pdu,
			      uint8_t len, uint32_t end_tick);

int ll_radio_init(ll_radio_cb_t cb);
uint32_t ll_radio_now(void);
void ll_radio_set_adv_channel(uint8_t ch);   /* 37..39; adv AA + CRC init */
/* TX pdu at start_tick, then listen up to rx_window_us for a reply. A
 * guard alarm (ll_sched_guard_at) ends it as LL_RADIO_RX_TIMEOUT after a
 * baseband restore if no end IRQ comes (also for ll_radio_tx_rsp_at). */
void ll_radio_tx_then_rx(const uint8_t *pdu, uint8_t len, uint32_t start_tick,
			 uint32_t rx_window_us);
/* Pre-load the response buffer (SCAN_RSP), so the IFS path only triggers. */
void ll_radio_prepare_rsp(const uint8_t *pdu, uint8_t len);
/* Send the prepared response so that its first bit is on air at tick.
 * Returns false (and starts nothing, so no LL_RADIO_TX_DONE follows) when
 * the TX trigger tick is already too close or in the past. */
bool ll_radio_tx_rsp_at(uint32_t tick);
void ll_radio_stop(void);
/* Before SoC poweroff (device only): stop the FSM, clear all RF IRQ masks and
 * status, mask the RF PLIC IRQ. Irq-lock safe, no re-enable path (the SoC
 * cold-boots on wakeup). */
void ll_radio_quiesce(void);

/* ---- Connection mode (slice 2; implemented in ll_radio.c, faked in host
 * tests). Peripheral role, 1M PHY. One radio: the links' connection events
 * and advertising events take turns (slice 6a, ml-spike-report). ---- */
/* One-time connection-mode setup (TX ring base, empty base PDU,
 * reset_sn_nesn, FSM off, guard streak and RX ring position reset);
 * idempotent, call before the first connection event ever. Replaces the
 * one-time part of the former ll_radio_conn_setup(). Called at every
 * connection start (CONNECT_IND RX ISR) so that it also covers a baseband
 * reset by ll_radio_adv_restore(): the setup runs once after boot and after
 * each restore, every other call returns without touching the radio, so
 * starting link N never disturbs an open event or another link's state.
 * ISR or thread. */
void ll_radio_conn_init(void);
/* Per event, register writes only (about 8 us, S2/S3): the access address
 * and CRC init of the link that owns the next BRX, plus the connection
 * values of the registers an advertising event changes (ll_ctrl_1 with
 * first-RX timeout, keeping the SN/NESN init bits ll_txq programs; TX
 * timestamps; TX DMA source = ring base; RX maxlen; IRQ mask; mode). Call
 * before ll_txq_event_start() and ll_radio_conn_event() of the event. ISR:
 * stimer ISR context only; it must not be preempted by the RF ISR (no
 * event may be open, and the RF IRQ must not see half-written registers). */
void ll_radio_conn_select(uint32_t aa, uint32_t crc_init);
/* Advertising event after connection events (ml-spike-report S3): set
 * wptr = rptr (empty ring, else stx2rx wedges the FSM in 0x03), then the
 * adv register switch (about 60 us with the channel set; no baseband
 * reset). ll_radio_adv_restore() only as stall recovery and when the last
 * link has ended, not per adv event (142 us, hurts T_IFS). The next
 * connection event rebuilds its link's ring (ll_txq_event_start). No
 * connection event may be open. ISR. */
void ll_radio_adv_enter(void);
/* Issue one BRX connection event on data channel ch (0..36): the RX window
 * opens at open_tick; if nothing is received within first_timeout_us the
 * event ends. The hardware then chains RX/TX exchanges while MD. Ends with
 * LL_RADIO_CONN_DONE: from the radio's end IRQ, or from a guard alarm
 * (ll_sched_guard_at, open_tick + max_event_us) that stops the FSM if no end
 * IRQ arrives, so the event always ends. The guard also cuts a central MD
 * burst that would run into the next event; max_event_us comes from ll_conn
 * (interval based, LL_CONN_EVENT_SAFETY_US) and exceeds first_timeout_us. */
void ll_radio_conn_event(uint8_t ch, uint32_t open_tick, uint32_t first_timeout_us,
			 uint32_t max_event_us);
/* Consecutive guard-ended connection events that received no CRC-valid
 * packet: the radio-wedge indicator, counted on the shared radio over all
 * links (the glue ends every active link with 0x08 at 3). A guard
 * that cut a long MD burst (packets received) is a healthy event and resets
 * the streak to 0, as does any event that ended with a radio IRQ and a new
 * connection. ISR context. */
uint8_t ll_radio_conn_guard_streak(void);
/* SN of our last transmitted packet, programmed before every BRX. */
void ll_radio_conn_set_sn_init(uint8_t sn);
/* BRX NESN init (ll_ctrl_1): the expected SN of the central's packet at the
 * first RX of the next BRX command. Read per command like SN init (Task 9);
 * left at 0, every central packet with SN 1 is acked but dropped. */
void ll_radio_conn_set_nesn_init(uint8_t nesn);
/* The pipe 0 TX FIFO pointers are 5-bit registers (Task 9: read back
 * values wrap at 32, a written wptr keeps its low 5 bits). All pointer
 * arithmetic is modulo LL_RADIO_FIFO_PTR_MASK + 1; RING entries are
 * pointer & 3. ll_radio_fifo_rptr/wptr return masked values. */
#define LL_RADIO_FIFO_PTR_MASK 0x1f
/* TX FIFO (pipe 0): rptr is advanced by hardware on ack, wptr by software.
 * Both are 5-bit counters (LL_RADIO_FIFO_PTR_MASK); the ring has 4 entries
 * (idx & 3). */
uint8_t ll_radio_fifo_rptr(void);
uint8_t ll_radio_fifo_wptr(void);
/* Write data PDU header byte 0 (LLID; NESN/SN/MD are set by hardware) and
 * payload (len <= LL_DATA_PDU_MAX + LL_MIC_LEN) into ring entry idx & 3. */
void ll_radio_fifo_write(uint8_t idx, uint8_t hdr0, const uint8_t *payload, uint8_t len);
void ll_radio_fifo_set_wptr(uint8_t wptr);
/* Return to advertising after the (last) connection: baseband reset +
 * re-init. Also the recovery for a stalled radio. */
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
	/* Connection mode */
	uint32_t conn_events;   /* BRX commands issued */
	uint32_t conn_rx;       /* CRC-valid packets (CRC-bad ones count in rx_crc) */
	uint32_t conn_tx;       /* TX IRQs (our packets sent, incl. retransmissions) */
	uint32_t conn_fto;      /* events ended by the first-RX timeout */
	uint32_t conn_guard;    /* events ended by the guard alarm (no end IRQ) */
	uint32_t rx_ptr_odd;    /* RX IRQ without a new RX DMA ring entry, or ring overrun */
	uint32_t rx_ptr_skip;   /* hw rx wptr jumped by more than the ring: entries skipped */
	uint32_t fst_capped;    /* events whose RX window exceeded the 12-bit rx_timeout */
	uint8_t rx_wptr_max;    /* largest raw hw rx wptr seen (its counter width) */
	/* First-exchange T_IFS from the TX timestamp (us, rounded) */
	uint32_t tifs_le150;
	uint32_t tifs_151_152;
	uint32_t tifs_gt152;
	/* ll_radio_adv_restore(): calls, and the TX FIFO pointers around the
	 * last baseband reset (rptr << 8 | wptr) */
	uint32_t restores;
	/* advertising TX/RX ended by the adv guard (no end IRQ; recovered
	 * with ll_radio_adv_restore, also counted in restores) */
	uint32_t adv_guard;
	uint16_t restore_ptrs_before;
	uint16_t restore_ptrs_after;
};
void ll_radio_get_stats(struct ll_radio_stats *s);

#endif /* LL_RADIO_H_ */
