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
 * - Multilink (ml-spike-report S1, verified on the device with forced NACKs
 *   and with foreign TX between events): one FIFO serves all links and its
 *   rptr cannot be reset, so each link keeps a software copy of every entry
 *   until it is acked. event_start(link) empties the ring (wptr = rptr),
 *   rewrites the link's unacked copies in order from rptr, then appends its
 *   backlog. rp is the rptr at the link's event start, so the rptr advance
 *   up to event_end is exactly the acks of this link's entries. The
 *   hardware sends the head entry from RAM at TX time (S2b); the rewrite
 *   lands on whatever slots rptr points at. With one link the rewrite puts
 *   the same content into the same slots (S1: rptr never moves between our
 *   events), so slice 5 behaviour is unchanged.
 * Runs in ISR context except ll_txq_push / ll_txq_backlog: no allocation,
 * no blocking.
 *
 * Each link's backlog is a single-producer (ll_txq_push, thread) / single-consumer
 * (ll_txq_event_start, ISR) ring: bl_head is written only by the producer,
 * bl_tail only by the consumer. The callers hold ll_plat_lock() around
 * ll_txq_push (which already implies a compiler barrier), but the ring does
 * not rely on that: RING_BARRIER() orders the slot write before the bl_head
 * advance (publish), the bl_head read before the slot read (consume) and
 * the slot read before the bl_tail advance (release).
 * A single hart needs no hardware fence.
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
/* compiler-only barrier (no fence instruction); see the file comment */
#define RING_BARRIER() __atomic_signal_fence(__ATOMIC_SEQ_CST)

_Static_assert((LL_TXQ_BACKLOG & (LL_TXQ_BACKLOG - 1)) == 0 && LL_TXQ_BACKLOG <= 128,
	       "LL_TXQ_BACKLOG must be a power of two that fits the 8-bit counters");
_Static_assert(LL_MAX_CONN >= 1 && LL_MAX_CONN <= 5, "LL_MAX_CONN must be 1..5");

struct txq_pdu {
	uint8_t kind;
	uint8_t llid;
	uint8_t len;
	uint8_t opcode;
	uint8_t data[PDU_MAX];
};

struct txq_slot {           /* software copy of one ring entry, until acked */
	struct txq_pdu pdu;
	bool placeholder;
};

struct txq_link {
	struct txq_pdu backlog[LL_TXQ_BACKLOG];
	uint8_t bl_head;         /* free-running, written by ll_txq_push */
	uint8_t bl_tail;         /* free-running, written by event_start */
	struct txq_slot ring[RING_DEPTH];   /* unacked entries, oldest at head */
	uint8_t head;            /* ring index of the oldest unacked entry */
	uint8_t n;               /* entries in the ring (incl. a placeholder) */
	uint8_t rp;              /* hardware rptr at the link's last event start */
	uint8_t in_ring;         /* data PDUs (non-placeholder) in the ring */
	uint8_t sn;              /* SN of our last transmitted packet */
	uint8_t rx_nesn;         /* NESN of the central's last packet this event */
	uint8_t nesn;            /* expected SN of the central's next new packet */
	bool rx_seen;            /* a central packet was received this event */
	bool base_last;          /* our last transmitted packet was the base PDU */
};

static struct {
	ll_txq_done_cb_t done;
	struct txq_link l[LL_MAX_CONN];
} q;

static struct txq_link *get(uint8_t link)
{
	return link < LL_MAX_CONN ? &q.l[link] : NULL;
}

static uint8_t backlog_count(const struct txq_link *l)
{
	return (uint8_t)(l->bl_head - l->bl_tail);
}

/* Append one entry: software copy at the tail, hardware entry rp + n. */
static void ring_put(struct txq_link *l, const struct txq_pdu *p, bool placeholder)
{
	struct txq_slot *s = &l->ring[(l->head + l->n) % RING_DEPTH];

	ll_radio_fifo_write(PTR(l->rp + l->n), p->llid, p->data, p->len);
	s->pdu.kind = p->kind;
	s->pdu.llid = p->llid;
	s->pdu.len = p->len;
	s->pdu.opcode = p->opcode;
	if (p->len) {
		memcpy(s->pdu.data, p->data, p->len);
	}
	s->placeholder = placeholder;
	if (!placeholder) {
		l->in_ring++;
	}
	l->n++;
}

/*
 * Placeholder rule (derived from the measured model; verified on the device
 * in Task 10 under forced NACKs: 2686/2686 encrypted echoes, about 12000
 * encrypted data PDUs at an 18 % NACK rate, no duplicate and no loss). If
 * our last transmitted packet was the base empty PDU, the next command's
 * first ack refers to the base, but the hardware pops the ring head on it.
 * An empty placeholder entry in front of new data takes that pop: if the
 * central acked the base, the placeholder is popped unsent (correct, it
 * stands for the acked base); if not, the placeholder is resent with the
 * base's SN and the same (empty) content, a correct retransmission. Called
 * only with an empty ring. The rule is per link: base_last is the link's
 * own last TX, and SN_INIT is programmed per command from the link's state.
 */
static bool placeholder_needed(const struct txq_link *l)
{
	return l->base_last;
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

void ll_txq_init(ll_txq_done_cb_t done)
{
	memset(&q, 0, sizeof(q));
	q.done = done;
}

void ll_txq_reset(uint8_t link)
{
	struct txq_link *l = get(link);

	if (!l) {
		return;
	}
	/* The ring itself is emptied at the link's next event start. After
	 * the connection starts the central's first NESN is 0 = SN_INIT, so
	 * the first ack is not judged and no placeholder is needed. */
	memset(l, 0, sizeof(*l));
}

int ll_txq_push(uint8_t link, enum ll_txq_kind kind, uint8_t llid, const uint8_t *payload,
		uint8_t len, uint8_t ctrl_opcode)
{
	struct txq_link *l = get(link);
	struct txq_pdu *p;

	if (!l || len > PDU_MAX) {
		return -EINVAL;
	}
	if (backlog_count(l) >= LL_TXQ_BACKLOG) {
		return -ENOMEM;
	}
	p = &l->backlog[l->bl_head % LL_TXQ_BACKLOG];
	p->kind = (uint8_t)kind;
	p->llid = llid;
	p->len = len;
	p->opcode = ctrl_opcode;
	if (len) {
		memcpy(p->data, payload, len);
	}
	RING_BARRIER();   /* publish: slot complete before bl_head moves */
	l->bl_head++;
	return 0;
}

void ll_txq_event_start(uint8_t link)
{
	static const struct txq_pdu empty = { .kind = LL_TXQ_EMPTY, .llid = LL_LLID_CONT };
	struct txq_link *l = get(link);
	bool was_empty;
	uint8_t room;

	if (!l) {
		return;
	}
	/* Rebuild: rptr may have been moved (another link's acks) and the
	 * entries overwritten since this link's last event. Empty the ring
	 * first, so no entry is written while it is queued. */
	l->rp = ll_radio_fifo_rptr();
	ll_radio_fifo_set_wptr(l->rp);
	for (uint8_t i = 0; i < l->n; i++) {
		const struct txq_pdu *p = &l->ring[(l->head + i) % RING_DEPTH].pdu;

		ll_radio_fifo_write(PTR(l->rp + i), p->llid, p->data, p->len);
	}
	was_empty = l->n == 0;
	ll_radio_conn_set_sn_init(l->sn);
	ll_radio_conn_set_nesn_init(l->nesn);
	l->rx_seen = false;
	if (backlog_count(l) != 0) {
		if (was_empty && placeholder_needed(l)) {
			ring_put(l, &empty, true);
		}
		room = ring_room(l->n, was_empty);
		RING_BARRIER();   /* consume: bl_head observed before the slots are read */
		while (room-- && backlog_count(l)) {
			ring_put(l, &l->backlog[l->bl_tail % LL_TXQ_BACKLOG], false);
			RING_BARRIER();   /* release: slot read before bl_tail frees it */
			l->bl_tail++;
		}
	}
	ll_radio_fifo_set_wptr(PTR(l->rp + l->n));
}

void ll_txq_rx(uint8_t link, uint8_t hdr0)
{
	struct txq_link *l = get(link);

	if (!l) {
		return;
	}
	l->rx_nesn = (hdr0 & HDR_NESN) ? 1 : 0;
	l->nesn = (hdr0 & HDR_SN) ? 0 : 1;
	l->rx_seen = true;
}

void ll_txq_event_end(uint8_t link)
{
	struct txq_link *l = get(link);
	uint8_t r;

	if (!l) {
		return;
	}
	r = ll_radio_fifo_rptr();
	/* every rptr step since the event start acked this link's head */
	while (l->rp != r && l->n != 0) {
		const struct txq_slot *s = &l->ring[l->head];

		l->rp = PTR(l->rp + 1);
		l->head = (uint8_t)((l->head + 1) % RING_DEPTH);
		l->n--;
		if (s->placeholder) {
			continue;
		}
		l->in_ring--;
		if (s->pdu.kind != LL_TXQ_EMPTY && q.done) {
			q.done(link, (enum ll_txq_kind)s->pdu.kind, s->pdu.opcode);
		}
	}
	if (l->rx_seen) {
		/* We answered every received central packet; the last answer
		 * was the base iff the FIFO was empty when it was sent, and no
		 * pop follows our last TX within the event. */
		l->sn = l->rx_nesn;
		l->base_last = l->n == 0;
		l->rx_seen = false;
	}
}

unsigned int ll_txq_backlog(uint8_t link)
{
	const struct txq_link *l = get(link);

	return l ? (unsigned int)backlog_count(l) + l->in_ring : 0u;
}
