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
 * Runs in ISR context except ll_txq_push / ll_txq_fits / ll_txq_backlog:
 * no allocation, no blocking.
 *
 * Storage (slice 6b Task 4, PDUs up to 251 + MIC): per link one FIFO of
 * LL_TXQ_ENTRIES entries whose bytes are records in one byte area
 * (ll_fifo.h). Entries tail..mid-1 are the ring (sent or queued in the
 * hardware FIFO, unacked), mid..head-1 the backlog. A PDU is copied into
 * the area once, by ll_txq_push, and stays there until its ack: the ring
 * is rebuilt from it at every event start. A placeholder (no record) may
 * precede the ring entries in the hardware FIFO (ph).
 *
 * Each link's FIFO is single-producer (ll_txq_push, thread) / single-
 * consumer (event_start / event_end, ISR): head is written only by the
 * producer, mid and tail only by the consumer. The area is freed in FIFO
 * order: the producer places a record only in the gap behind the newest
 * record and in front of the oldest unacked one (ent[tail]), or at 0 when
 * nothing is queued. The callers hold ll_plat_lock() around ll_txq_push
 * (which already implies a compiler barrier), but the FIFO does not rely
 * on that: RING_BARRIER() orders the record write before the head advance
 * (publish), the head read before the record read (consume) and the
 * record read before the tail advance (release).
 * A single hart needs no hardware fence.
 */
#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include "ll_defs.h"
#include "ll_fifo.h"
#include "ll_radio.h"
#include "ll_txq.h"

#define RING_DEPTH   4      /* hardware entries (tx_chn_dep 2) */
#define HDR_NESN     0x04
#define HDR_SN       0x08
#define PTR(x)       ((uint8_t)((x) & LL_RADIO_FIFO_PTR_MASK))
_Static_assert(LL_DATA_PDU_MAX + LL_MIC_LEN == 255, "a PDU length is a uint8_t");
#define ENT(i)       ((uint8_t)((uint8_t)(i) % LL_TXQ_ENTRIES))
/* compiler-only barrier (no fence instruction); see the file comment */
#define RING_BARRIER() __atomic_signal_fence(__ATOMIC_SEQ_CST)

_Static_assert((LL_TXQ_ENTRIES & (LL_TXQ_ENTRIES - 1)) == 0 && LL_TXQ_ENTRIES <= 128,
	       "LL_TXQ_ENTRIES must be a power of two that fits the 8-bit counters");
_Static_assert(LL_TXQ_POOL_BYTES % LL_FIFO_ALIGN == 0 && LL_TXQ_POOL_BYTES <= 0xFFFF,
	       "LL_TXQ_POOL_BYTES must be a multiple of 4 below 64 KiB");
/* a host ACL packet of LL_ACL_MTU in the smallest fragments (27 + MIC), and
 * the 4 ring entries of maximum PDUs, fit an empty area */
_Static_assert(LL_TXQ_POOL_BYTES >= 4 * 256 &&
	       LL_TXQ_POOL_BYTES >= ((LL_ACL_MTU + 26) / 27) * 32,
	       "LL_TXQ_POOL_BYTES too small");
_Static_assert(LL_TXQ_ENTRIES >= (LL_ACL_MTU + 26) / 27 + RING_DEPTH,
	       "LL_TXQ_ENTRIES must hold a fragmented host packet behind a full ring");

struct txq_ent {
	uint16_t off;            /* record in the link's area */
	uint8_t kind;
	uint8_t llid;
	uint8_t len;
	uint8_t opcode;
	bool last;
};

struct txq_link {
	/* the records start 2 bytes past a word boundary, like the payload
	 * (offset 6) of a hardware FIFO entry: ll_fifo_copy then moves words */
	uint8_t area[LL_TXQ_POOL_BYTES + 2] __attribute__((aligned(4)));
	struct txq_ent ent[LL_TXQ_ENTRIES];
	uint8_t head;            /* free-running, written by ll_txq_push */
	uint8_t mid;             /* free-running, first backlog entry (consumer) */
	uint8_t tail;            /* free-running, oldest unacked entry (consumer) */
	bool ph;                 /* a placeholder precedes the ring entries */
	uint8_t rp;              /* hardware rptr at the link's last event start */
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

static uint8_t *rec(struct txq_link *l, const struct txq_ent *e)
{
	return &l->area[2 + e->off];
}

static uint8_t backlog_count(const struct txq_link *l)
{
	return (uint8_t)(l->head - l->mid);
}

/* hardware entries of the ring: the placeholder plus the unacked entries */
static uint8_t ring_n(const struct txq_link *l)
{
	return (uint8_t)((l->ph ? 1 : 0) + (uint8_t)(l->mid - l->tail));
}

/* Producer side: the live records as the producer sees them (tail read
 * once; the consumer only advances it, which only frees room). */
struct span {
	uint8_t used;            /* live records */
	uint16_t oldest;         /* offset of the oldest */
	uint16_t last_off;       /* offset of the newest */
	uint16_t last_end;       /* end of the newest (rounded size) */
};

static void span_now(const struct txq_link *l, struct span *sp)
{
	uint8_t tail = l->tail;

	RING_BARRIER();   /* tail read before the entries it frees are reused */
	sp->used = (uint8_t)(l->head - tail);
	if (sp->used) {
		const struct txq_ent *o = &l->ent[ENT(tail)];
		const struct txq_ent *w = &l->ent[ENT(l->head - 1)];

		sp->oldest = o->off;
		sp->last_off = w->off;
		sp->last_end = (uint16_t)(w->off + ll_fifo_size(w->len));
	} else {
		sp->oldest = 0;
		sp->last_off = 0;
		sp->last_end = 0;
	}
}

/* Place one record of len bytes behind sp (and account for it): its offset,
 * or -1 when the entries or the area are full. */
static int32_t span_add(struct span *sp, uint8_t len)
{
	int32_t at;

	if (sp->used >= LL_TXQ_ENTRIES) {
		return -1;
	}
	at = ll_fifo_place(LL_TXQ_POOL_BYTES, sp->used, sp->oldest, sp->last_off, sp->last_end,
			   len);
	if (at < 0) {
		return -1;
	}
	sp->used++;   /* with none live the record is at 0 = oldest */
	sp->last_off = (uint16_t)at;
	sp->last_end = (uint16_t)(at + ll_fifo_size(len));
	return at;
}

/* Write the hardware entry for FIFO slot idx from entry e (NULL: empty). */
static void hw_put(struct txq_link *l, uint8_t idx, const struct txq_ent *e)
{
	if (e) {
		ll_radio_fifo_write(idx, e->llid, rec(l, e), e->len);
	} else {
		ll_radio_fifo_write(idx, LL_LLID_CONT, NULL, 0);
	}
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
	 * the first ack is not judged and no placeholder is needed. The area
	 * keeps its bytes (no records are live). */
	l->head = 0;
	l->mid = 0;
	l->tail = 0;
	l->ph = false;
	l->rp = 0;
	l->sn = 0;
	l->rx_nesn = 0;
	l->nesn = 0;
	l->rx_seen = false;
	l->base_last = false;
}

bool ll_txq_fits(uint8_t link, uint8_t n, uint8_t len, uint8_t last_len)
{
	const struct txq_link *l = get(link);
	struct span sp;

	if (!l || n == 0) {
		return false;
	}
	span_now(l, &sp);
	for (uint8_t i = 0; i < n; i++) {
		if (span_add(&sp, i + 1 == n ? last_len : len) < 0) {
			return false;
		}
	}
	return true;
}

int ll_txq_push(uint8_t link, enum ll_txq_kind kind, uint8_t llid, const uint8_t *payload,
		uint8_t len, uint8_t ctrl_opcode, bool last)
{
	struct txq_link *l = get(link);
	struct txq_ent *e;
	struct span sp;
	int32_t at;

	if (!l) {
		return -EINVAL;   /* len: any uint8_t is at most PDU_MAX (255) */
	}
	span_now(l, &sp);
	at = span_add(&sp, len);
	if (at < 0) {
		return -ENOMEM;
	}
	e = &l->ent[ENT(l->head)];
	e->off = (uint16_t)at;
	e->kind = (uint8_t)kind;
	e->llid = llid;
	e->len = len;
	e->opcode = ctrl_opcode;
	e->last = last;
	if (len) {
		ll_fifo_copy(rec(l, e), payload, len);
	}
	RING_BARRIER();   /* publish: record complete before head moves */
	l->head++;
	return 0;
}

void ll_txq_event_start(uint8_t link)
{
	struct txq_link *l = get(link);
	bool was_empty;
	uint8_t room, n, k;

	if (!l) {
		return;
	}
	/* Rebuild: rptr may have been moved (another link's acks) and the
	 * entries overwritten since this link's last event. Empty the ring
	 * first, so no entry is written while it is queued. */
	l->rp = ll_radio_fifo_rptr();
	ll_radio_fifo_set_wptr(l->rp);
	k = 0;
	if (l->ph) {
		hw_put(l, PTR(l->rp + k++), NULL);
	}
	for (uint8_t i = l->tail; i != l->mid; i++) {
		hw_put(l, PTR(l->rp + k++), &l->ent[ENT(i)]);
	}
	was_empty = k == 0;
	ll_radio_conn_set_sn_init(l->sn);
	ll_radio_conn_set_nesn_init(l->nesn);
	l->rx_seen = false;
	if (backlog_count(l) != 0) {
		if (was_empty && placeholder_needed(l)) {
			l->ph = true;
			hw_put(l, PTR(l->rp + k++), NULL);
		}
		room = ring_room(k, was_empty);
		RING_BARRIER();   /* consume: head observed before the records are read */
		n = backlog_count(l);
		while (room-- && n--) {
			hw_put(l, PTR(l->rp + k++), &l->ent[ENT(l->mid)]);
			l->mid++;
		}
	}
	ll_radio_fifo_set_wptr(PTR(l->rp + k));
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
	while (l->rp != r && ring_n(l) != 0) {
		l->rp = PTR(l->rp + 1);
		if (l->ph) {
			l->ph = false;
			continue;
		}
		{
			const struct txq_ent *e = &l->ent[ENT(l->tail)];
			enum ll_txq_kind kind = (enum ll_txq_kind)e->kind;
			uint8_t op = e->opcode;
			bool last = e->last;

			RING_BARRIER();   /* release: entry read before tail frees it */
			l->tail++;
			if (kind != LL_TXQ_EMPTY && q.done) {
				q.done(link, kind, op, last);
			}
		}
	}
	if (l->rx_seen) {
		/* We answered every received central packet; the last answer
		 * was the base iff the FIFO was empty when it was sent, and no
		 * pop follows our last TX within the event. */
		l->sn = l->rx_nesn;
		l->base_last = ring_n(l) == 0;
		l->rx_seen = false;
	}
}

unsigned int ll_txq_backlog(uint8_t link)
{
	const struct txq_link *l = get(link);

	return l ? (unsigned int)(uint8_t)(l->head - l->tail) : 0u;
}
