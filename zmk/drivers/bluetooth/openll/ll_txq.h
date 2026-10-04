/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * TX queue, per link: software backlog in front of the 4-entry hardware TX
 * FIFO (accessed via ll_radio_fifo_*), placeholder rule, SN_INIT bookkeeping
 * and ack detection by rptr advance (md-spike-report "TX FIFO model"), ring
 * rebuilt per event from software copies (ml-spike-report S1).
 *
 * Build option LL_TXQ_SAFE_MODE (fallback of the connection spec): the ring
 * is refilled only when it is empty, with one data PDU per event (no MD for
 * our data). The placeholder rule still applies in this mode, because the
 * measured FIFO pops a newly queued head when the central acks the base
 * empty PDU (see ll_txq.c).
 */
#ifndef LL_TXQ_H_
#define LL_TXQ_H_

#include <stdbool.h>
#include <stdint.h>

enum ll_txq_kind {
	LL_TXQ_EMPTY,   /* empty PDU (placeholder), no completion reported */
	LL_TXQ_ACL,     /* counts toward HCI Number Of Completed Packets */
	LL_TXQ_CTRL,    /* LL control PDU, completion goes to ll_llcp */
};

/* ISR context. link is the link whose entry was acked; ctrl_opcode is the
 * plaintext opcode given to ll_txq_push (only meaningful for LL_TXQ_CTRL);
 * last is the flag given to ll_txq_push (slice 6b: false for every
 * fragment but the last of a host ACL packet, whose Number Of Completed
 * Packets credit comes back only with the last one). */
typedef void (*ll_txq_done_cb_t)(uint8_t link, enum ll_txq_kind kind, uint8_t ctrl_opcode,
				 bool last);

/* Per link (slice 6b Task 4, long PDUs): at most LL_TXQ_ENTRIES PDUs queued
 * and not yet acked (backlog + ring), their bytes (len, each rounded up to
 * 4) in one FIFO area of LL_TXQ_POOL_BYTES (ll_fifo.h). The ring copies are
 * the oldest records of the same area, so a PDU is copied in once and
 * stays until it is acked. A host ACL packet of LL_ACL_MTU split into
 * 27-octet fragments (10 x (27 + MIC)) and 5 maximum PDUs (255) fit an
 * empty area. */
#define LL_TXQ_ENTRIES 16
#ifndef LL_TXQ_POOL_BYTES
#define LL_TXQ_POOL_BYTES 1280
#endif

/* Invariant: per link, the backlog and the ring are strictly FIFO across
 * all kinds (no priority lane for control PDUs). The encryption start
 * procedure relies on it: ACL queued before LL_ENC_RSP leaves before it
 * (plaintext, counted before the procedure), and everything queued after
 * our encrypted LL_START_ENC_RSP leaves after it, so on-air order equals
 * the order in which ll_llcp assigned TX packet counters. */

/* Multilink (slice 6a, ml-spike-report S1): the hardware has one TX FIFO
 * for all links and its rptr cannot be reset. Each link keeps a software
 * copy of every ring entry until it is acked; ll_txq_event_start(link)
 * empties the ring (wptr = rptr) and rewrites the link's unacked copies,
 * then its backlog, starting at the current rptr. The rptr advance during
 * the link's own event acks its oldest entries (ll_txq_event_end). Another
 * link's (or advertising's) use of the FIFO between two events of a link
 * therefore does not matter: the hardware reads the head entry from RAM at
 * TX time (S2b). All link ids are 0 <= link < LL_MAX_CONN; other ids are
 * ignored (push: -EINVAL, backlog: 0). */

/* Calling context: ll_txq_init() once, before any other call. ll_txq_reset()
 * runs in ISR context, from ll_conn_start() in the RX ISR of the CONNECT_IND
 * (before the first event of that link, so no event_start/event_end of it
 * can be running); push runs in thread context under ll_plat_lock();
 * event_start/rx/event_end run in the radio/stimer ISRs. ll_conn guarantees
 * that every ll_txq_event_end(link) follows an ll_txq_event_start(link) of
 * the same event with no other link's event in between (one radio), and a
 * skipped late event calls neither, so the per-event state is always
 * initialized. */

/* Once: all links empty, completion callback set. */
void ll_txq_init(ll_txq_done_cb_t done);
/* Per connection of link: drop its backlog and unacked entries (no
 * completions), SN/NESN 0 (the radio's per-command SN/NESN init bits carry
 * them, so no hardware access is needed here). ISR. */
void ll_txq_reset(uint8_t link);
/* Queue one data PDU into link's backlog. payload is already encrypted if
 * needed (len includes the MIC then, at most LL_DATA_PDU_MAX + LL_MIC_LEN).
 * ctrl_opcode is the plaintext opcode of an LL_TXQ_CTRL PDU (the payload
 * may be ciphertext), ignored for other kinds; last is handed to the
 * completion. Returns 0, -ENOMEM (LL_TXQ_ENTRIES or the area full) or
 * -EINVAL (bad link). Thread context, caller holds
 * ll_plat_lock(); one producer at a time (ll_plat_tx_lock()). */
int ll_txq_push(uint8_t link, enum ll_txq_kind kind, uint8_t llid, const uint8_t *payload,
		uint8_t len, uint8_t ctrl_opcode, bool last);
/* Whether n PDUs (n - 1 of len octets, then one of last_len; on-air
 * lengths, MIC included) would all be accepted by ll_txq_push now. The
 * consumer only frees room, so with the producer serialized
 * (ll_plat_tx_lock()) a true answer holds until the pushes: the glue queues
 * all fragments of a host packet or none. False for a bad link, n 0 or a
 * length above LL_DATA_PDU_MAX + LL_MIC_LEN. Thread. */
bool ll_txq_fits(uint8_t link, uint8_t n, uint8_t len, uint8_t last_len);
/* ISR, before each BRX of link: set wptr = rptr, rewrite the ring from the
 * link's unacked copies then its backlog (placeholder rule), program
 * SN_INIT and NESN_INIT from the link's state via
 * ll_radio_conn_set_sn_init() / ll_radio_conn_set_nesn_init(). */
void ll_txq_event_start(uint8_t link);
/* ISR, for each LL_RADIO_CONN_RX of link's event, in order: header byte 0
 * of the central's packet. Its NESN is the SN of our response to it
 * (hardware SN/NESN), so the last one gives SN_INIT for the next event; its
 * SN ^ 1 is NESN_INIT (the hardware delivers only new packets). Acks of the
 * base empty PDU do not move rptr, so the SN cannot be tracked from rptr. */
void ll_txq_rx(uint8_t link, uint8_t hdr0);
/* ISR, after LL_RADIO_CONN_DONE of link's event: read rptr, complete the
 * acked entries, track our SN. */
void ll_txq_event_end(uint8_t link);
/* link's PDUs queued and not yet acked (backlog + ring). */
unsigned int ll_txq_backlog(uint8_t link);

#endif /* LL_TXQ_H_ */
