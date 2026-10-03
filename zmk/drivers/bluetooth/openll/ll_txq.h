/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * TX queue: software backlog in front of the 4-entry hardware TX FIFO
 * (accessed via ll_radio_fifo_*), placeholder rule, SN_INIT bookkeeping and
 * ack detection by rptr advance (md-spike-report "TX FIFO model").
 *
 * Build option LL_TXQ_SAFE_MODE (fallback of the connection spec): the ring
 * is refilled only when it is empty, with one data PDU per event (no MD for
 * our data). The placeholder rule still applies in this mode, because the
 * measured FIFO pops a newly queued head when the central acks the base
 * empty PDU (see ll_txq.c).
 */
#ifndef LL_TXQ_H_
#define LL_TXQ_H_

#include <stdint.h>

enum ll_txq_kind {
	LL_TXQ_EMPTY,   /* empty PDU (placeholder), no completion reported */
	LL_TXQ_ACL,     /* counts toward HCI Number Of Completed Packets */
	LL_TXQ_CTRL,    /* LL control PDU, completion goes to ll_llcp */
};

/* ISR context. ctrl_opcode is the plaintext opcode given to ll_txq_push
 * (only meaningful for LL_TXQ_CTRL). */
typedef void (*ll_txq_done_cb_t)(enum ll_txq_kind kind, uint8_t ctrl_opcode);

#define LL_TXQ_BACKLOG 8

/* Invariant: the backlog and the ring are strictly FIFO across all kinds
 * (no priority lane for control PDUs). The encryption start procedure
 * relies on it: ACL queued before LL_ENC_RSP leaves before it (plaintext,
 * counted before the procedure), and everything queued after our encrypted
 * LL_START_ENC_RSP leaves after it, so on-air order equals the order in
 * which ll_llcp assigned TX packet counters. */

/* Per connection, after ll_radio_conn_setup() (reset_sn_nesn). */
void ll_txq_reset(ll_txq_done_cb_t done);
/* Queue one data PDU into the backlog. payload is already encrypted if
 * needed (len includes the MIC then). ctrl_opcode is the plaintext opcode
 * of an LL_TXQ_CTRL PDU (the payload may be ciphertext), ignored for other
 * kinds. Returns 0, -ENOMEM (backlog full) or -EINVAL (len too long).
 * Thread context, caller holds ll_plat_lock(). */
int ll_txq_push(enum ll_txq_kind kind, uint8_t llid, const uint8_t *payload, uint8_t len,
		uint8_t ctrl_opcode);
/* ISR, before each BRX: refill the ring from the backlog (placeholder rule)
 * and program SN_INIT and NESN_INIT via ll_radio_conn_set_sn_init() and
 * ll_radio_conn_set_nesn_init(). */
void ll_txq_event_start(void);
/* ISR, for each LL_RADIO_CONN_RX of the event, in order: header byte 0 of
 * the central's packet. Its NESN is the SN of our response to it (hardware
 * SN/NESN), so the last one gives SN_INIT for the next event; its SN ^ 1 is
 * NESN_INIT (the hardware delivers only new packets). Acks of the
 * base empty PDU do not move rptr, so the SN cannot be tracked from rptr. */
void ll_txq_rx(uint8_t hdr0);
/* ISR, after LL_RADIO_CONN_DONE: read rptr, complete acked entries, track
 * our SN. */
void ll_txq_event_end(void);
/* PDUs queued and not yet acked (backlog + ring). */
unsigned int ll_txq_backlog(void);

#endif /* LL_TXQ_H_ */
