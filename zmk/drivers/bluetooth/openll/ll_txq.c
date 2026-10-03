/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * TX queue on the B91 pipe 0 TX FIFO (md-spike-report "TX FIFO model"):
 * - Empty FIFO (rptr == wptr): the hardware sends the base buffer, an empty
 *   PDU. Otherwise it sends ring entry rptr & 3; MD = (wptr - rptr) > 1.
 * - The hardware pops (rptr++) when the central acks the head. At the first
 *   RX of a BRX command the ack is judged against SN_INIT, so SN_INIT must be
 *   the SN of our last transmitted packet.
 * - Our SN is set by hardware: every response carries SN = NESN of the
 *   central packet it answers. The NESN of the central's last received
 *   packet is therefore the SN of our last TX (ll_txq_rx). Acks of the base
 *   PDU do not move rptr, so rptr alone cannot track the SN.
 * - The expected SN of the central's next new packet (our NESN) is also
 *   taken from ll_ctrl_1 at the first RX of each BRX command (BRX NESN
 *   init, Task 9 device trace: with it left at 0, every central packet with
 *   SN 1 was acked but dropped as a retransmission). The hardware writes
 *   only new packets into the RX FIFO, so the last one received (ll_txq_rx)
 *   gives NESN_INIT = its SN ^ 1.
 * - The FIFO pointers are 5 bits wide (LL_RADIO_FIFO_PTR_MASK, Task 9): wp
 *   and rp are kept in that range and differences are taken modulo 32.
 * - Entries are written only between events (event_start) and never while
 *   queued; an entry is complete when rptr has passed it (event_end).
 * Runs in ISR context except ll_txq_push / ll_txq_backlog: no allocation,
 * no blocking.
 */
#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include "ll_defs.h"
#include "ll_radio.h"
#include "ll_txq.h"

#define RING_DEPTH   4      /* hardware entries (tx_chn_dep 2) */
#define HDR_NESN     0x04
#define HDR_SN       0x08
#define PDU_MAX      (LL_DATA_PDU_MAX + LL_MIC_LEN)
#define PTR(x)       ((uint8_t)((x) & LL_RADIO_FIFO_PTR_MASK))

_Static_assert((LL_TXQ_BACKLOG & (LL_TXQ_BACKLOG - 1)) == 0 && LL_TXQ_BACKLOG <= 128,
	       "LL_TXQ_BACKLOG must be a power of two that fits the 8-bit counters");

struct txq_pdu {
	uint8_t kind;
	uint8_t llid;
	uint8_t len;
	uint8_t opcode;
	uint8_t data[PDU_MAX];
};

struct txq_slot {           /* software view of one ring entry */
	uint8_t kind;
	uint8_t opcode;
	bool placeholder;
};

static struct {
	ll_txq_done_cb_t done;
	struct txq_pdu backlog[LL_TXQ_BACKLOG];
	uint8_t bl_head;         /* free-running, written by ll_txq_push */
	uint8_t bl_tail;         /* free-running, written by event_start */
	struct txq_slot ring[RING_DEPTH];
	uint8_t wp;              /* our copy of the hardware wptr */
	uint8_t rp;              /* rptr value up to which entries are completed */
	uint8_t in_ring;         /* data PDUs (non-placeholder) in the ring */
	uint8_t sn;              /* SN of our last transmitted packet */
	uint8_t rx_nesn;         /* NESN of the central's last packet this event */
	uint8_t nesn;            /* expected SN of the central's next new packet */
	bool rx_seen;            /* a central packet was received this event */
	bool base_last;          /* our last transmitted packet was the base PDU */
} q;

static uint8_t backlog_count(void)
{
	return (uint8_t)(q.bl_head - q.bl_tail);
}

static void ring_put(uint8_t kind, uint8_t llid, const uint8_t *data, uint8_t len,
		     uint8_t opcode, bool placeholder)
{
	struct txq_slot *s = &q.ring[q.wp % RING_DEPTH];

	ll_radio_fifo_write(q.wp, llid, data, len);
	s->kind = kind;
	s->opcode = opcode;
	s->placeholder = placeholder;
	if (!placeholder) {
		q.in_ring++;
	}
	q.wp = PTR(q.wp + 1);
}

/*
 * Placeholder rule (derived from the measured model, not yet verified on
 * air). If our last transmitted packet was the base empty PDU, the next
 * command's first ack refers to the base, but the hardware pops the ring head
 * on it. An empty placeholder entry in front of new data takes that pop: if
 * the central acked the base, the placeholder is popped unsent (correct, it
 * stands for the acked base); if not, the placeholder is resent with the
 * base's SN and the same (empty) content, a correct retransmission. Called
 * only with an empty ring.
 */
static bool placeholder_needed(void)
{
	return q.base_last;
}

/* How many data PDUs may be moved into the ring now (used = entries queued,
 * including a placeholder written in this call). */
static uint8_t ring_room(uint8_t used, bool was_empty)
{
#ifdef LL_TXQ_SAFE_MODE
	/* Fallback: one data PDU per event, only after the ring drained. */
	(void)used;
	return was_empty ? 1 : 0;
#else
	(void)was_empty;
	return (uint8_t)(RING_DEPTH - used);
#endif
}

void ll_txq_reset(ll_txq_done_cb_t done)
{
	uint8_t r = ll_radio_fifo_rptr();

	/* rptr cannot be reset: drop leftovers of the previous connection by
	 * moving wptr back to rptr (empty FIFO, base sent). */
	ll_radio_fifo_set_wptr(r);
	memset(&q, 0, sizeof(q));
	q.done = done;
	q.wp = r;
	q.rp = r;
	/* After reset_sn_nesn the central's first NESN is 0 = SN_INIT, so the
	 * first ack is not judged and no placeholder is needed. */
	q.sn = 0;
	q.nesn = 0;
	q.base_last = false;
}

int ll_txq_push(enum ll_txq_kind kind, uint8_t llid, const uint8_t *payload, uint8_t len,
		uint8_t ctrl_opcode)
{
	struct txq_pdu *p;

	if (len > PDU_MAX) {
		return -EINVAL;
	}
	if (backlog_count() >= LL_TXQ_BACKLOG) {
		return -ENOMEM;
	}
	p = &q.backlog[q.bl_head % LL_TXQ_BACKLOG];
	p->kind = (uint8_t)kind;
	p->llid = llid;
	p->len = len;
	p->opcode = ctrl_opcode;
	if (len) {
		memcpy(p->data, payload, len);
	}
	q.bl_head++;
	return 0;
}

void ll_txq_event_start(void)
{
	uint8_t used = PTR(q.wp - ll_radio_fifo_rptr());
	bool was_empty = used == 0;
	uint8_t room;

	ll_radio_conn_set_sn_init(q.sn);
	ll_radio_conn_set_nesn_init(q.nesn);
	q.rx_seen = false;
	if (backlog_count() == 0) {
		return;
	}
	if (was_empty && placeholder_needed()) {
		ring_put(LL_TXQ_EMPTY, LL_LLID_CONT, NULL, 0, 0, true);
		used++;
	}
	room = ring_room(used, was_empty);
	while (room-- && backlog_count()) {
		const struct txq_pdu *p = &q.backlog[q.bl_tail % LL_TXQ_BACKLOG];

		ring_put(p->kind, p->llid, p->data, p->len, p->opcode, false);
		q.bl_tail++;
	}
	ll_radio_fifo_set_wptr(q.wp);
}

void ll_txq_rx(uint8_t hdr0)
{
	q.rx_nesn = (hdr0 & HDR_NESN) ? 1 : 0;
	q.nesn = (hdr0 & HDR_SN) ? 0 : 1;
	q.rx_seen = true;
}

void ll_txq_event_end(void)
{
	uint8_t r = ll_radio_fifo_rptr();

	while (q.rp != r && q.rp != q.wp) {
		const struct txq_slot *s = &q.ring[q.rp % RING_DEPTH];

		q.rp = PTR(q.rp + 1);
		if (s->placeholder) {
			continue;
		}
		q.in_ring--;
		if (s->kind != LL_TXQ_EMPTY && q.done) {
			q.done((enum ll_txq_kind)s->kind, s->opcode);
		}
	}
	if (q.rx_seen) {
		/* We answered every received central packet; the last answer
		 * was the base iff the FIFO was empty when it was sent, and no
		 * pop follows our last TX within the event. */
		q.sn = q.rx_nesn;
		q.base_last = q.rp == q.wp;
		q.rx_seen = false;
	}
}

unsigned int ll_txq_backlog(void)
{
	return (unsigned int)backlog_count() + q.in_ring;
}
