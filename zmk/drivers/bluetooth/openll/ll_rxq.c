/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * RX queue: per link, a lock-free FIFO of LL_RXQ_ENTRIES entries between the
 * RX ISR (single producer, ll_rxq_isr_put, copies each CRC-valid connection
 * PDU out of the RX DMA ring before it is overwritten) and the controller
 * thread (single consumer, ll_rxq_get). Slice 6b Task 4: the payloads are
 * variable-length records in one per-link byte area of LL_RXQ_POOL_BYTES
 * (ll_fifo.h), freed in FIFO order, so a PDU of up to 255 bytes needs no
 * 255-byte slot per entry. head/tail are free-running 8-bit counters, each
 * written by exactly one side; the record is written by the producer
 * strictly before head is advanced, and only read by the consumer after it
 * observes that advance (the producer places records only in the gap in
 * front of the oldest unconsumed one, ent[tail]), so on a single core with
 * ISR/thread separation
 * (ll_rxq_isr_put cannot run while ll_rxq_get is executing) no
 * ll_plat_lock() is needed. head/tail are volatile, but volatile does not
 * order the plain slot accesses around them, so explicit compiler barriers
 * (RING_BARRIER) sit at the publish point (slot written, then head) and at
 * the consume points (head read, then slot; slot read, then tail). A single
 * hart needs no hardware fence: the ISR sees the thread's stores in program
 * order.
 *
 * Overflow is an unrecoverable loss. The baseband acks every new central
 * packet in hardware before the ISR copies it here, so a data PDU that
 * finds the ring full (or has a malformed length) will never be resent by
 * the central. ll_rxq_isr_put returns false and ll_conn ends the link
 * deterministically (0x08 at the end of the event, ll_conn.c on_rx)
 * rather than continuing with a hole in the stream.
 *
 * No duplicate detection here: the baseband filters retransmissions itself.
 * With NESN init programmed per BRX (ll_txq), a central packet whose SN is
 * not the expected one is acked but never written into the RX FIFO, so
 * every PDU reaching ll_rxq_isr_put is new (Task 10, device + sniffer:
 * hundreds of central retransmissions on air under forced NACKs, seen as RX
 * IRQs without an RX entry, and none in software). A software SN check on
 * top was redundant and harmful: an RX ring overflow drops entries, and the
 * next new PDU with the dropped one's SN was then discarded as a duplicate.
 *
 * Empty PDUs are not queued (nothing to deliver; their SN no longer
 * matters), so the ring holds data PDUs only and does not overflow while
 * the controller thread is blocked (the host's LE Connection Complete
 * processing holds it for about 300 ms, Task 10).
 *
 * Slice 6a: one ring, crypt pointer, overflow count, MIC failure and wake
 * flag per link (LL_MAX_CONN). The RX DMA ring stays shared; the radio ISR
 * puts each packet into the ring of the link that owns the running event.
 * The rules above hold per link.
 *
 * Decryption happens in the consumer, in RX order. On LL_RXQ_MIC_FAIL the
 * caller is expected to terminate the connection (Core Spec Vol 6 Part B
 * 5.1.3.1), so there is no in-connection retry. The failure is sticky:
 * every later ll_rxq_get() returns LL_RXQ_MIC_FAIL until ll_rxq_reset(), so
 * no PDU after the hole in the stream reaches the host, whatever the caller
 * does while the link goes down.
 */
#include <string.h>

#include "ll_fifo.h"
#include "ll_rxq.h"

#define ENT(i)     ((uint8_t)((uint8_t)(i) % LL_RXQ_ENTRIES))
/* compiler-only barrier (no fence instruction); see the file comment */
#define RING_BARRIER() __atomic_signal_fence(__ATOMIC_SEQ_CST)

_Static_assert((LL_RXQ_ENTRIES & (LL_RXQ_ENTRIES - 1)) == 0 && LL_RXQ_ENTRIES <= 128,
	       "LL_RXQ_ENTRIES must be a power of two that fits the 8-bit counters");
_Static_assert(LL_RXQ_POOL_BYTES % LL_FIFO_ALIGN == 0 && LL_RXQ_POOL_BYTES >= 2 * 256 &&
	       LL_RXQ_POOL_BYTES <= 0xFFFF, "LL_RXQ_POOL_BYTES: multiple of 4, >= 2 maximum PDUs");

struct rxq_ent {
	uint16_t off;  /* record in the link's area */
	uint8_t hdr0;
	uint8_t len;   /* payload length as received (ciphertext + MIC if encrypted) */
};

static struct rxq_link {
	/* records start 2 bytes past a word boundary, like the payload
	 * (offset 6) of an RX DMA entry: ll_fifo_copy then moves words */
	uint8_t area[LL_RXQ_POOL_BYTES + 2] __attribute__((aligned(4)));
	struct rxq_ent ent[LL_RXQ_ENTRIES];
	volatile uint8_t head;   /* next free entry, advanced by the producer (ISR) */
	volatile uint8_t tail;   /* next entry to consume, advanced by the consumer (thread) */
	uint32_t overflow;
	struct ll_crypt *crypt;
	bool mic_failed;         /* sticky until ll_rxq_reset() (consumer only) */
	bool queued;             /* data PDU queued since the last take (producer only) */
} links[LL_MAX_CONN];

void ll_rxq_reset(uint8_t link)
{
	if (link < LL_MAX_CONN) {
		struct rxq_link *q = &links[link];

		/* the area keeps its bytes: no record is live after this */
		q->head = 0;
		q->tail = 0;
		q->overflow = 0;
		q->crypt = NULL;
		q->mic_failed = false;
		q->queued = false;
	}
}

bool ll_rxq_isr_take_queued(void)
{
	bool r = false;

	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		r |= links[i].queued;
		links[i].queued = false;
	}
	return r;
}

void ll_rxq_set_crypt(uint8_t link, struct ll_crypt *c)
{
	if (link < LL_MAX_CONN) {
		links[link].crypt = c;
	}
}

bool ll_rxq_isr_put(uint8_t link, const uint8_t *pdu, uint16_t len)
{
	struct rxq_link *q;
	uint8_t head, tail, used;
	uint16_t paylen;
	int32_t at;
	struct rxq_ent *e;

	if (link >= LL_MAX_CONN) {
		return false;
	}
	q = &links[link];
	head = q->head;
	if (len < 2 || len > 2 + LL_DATA_PDU_MAX + LL_MIC_LEN) {
		q->overflow++;
		return false;
	}
	paylen = (uint16_t)(len - 2);
	if (paylen == 0) {
		return true;   /* empty PDU: nothing to deliver */
	}
	tail = q->tail;
	RING_BARRIER();   /* tail read before its record's room is reused */
	used = (uint8_t)(head - tail);
	if (used >= LL_RXQ_ENTRIES) {
		q->overflow++;
		return false;
	}
	if (used) {
		const struct rxq_ent *w = &q->ent[ENT(head - 1)];

		at = ll_fifo_place(LL_RXQ_POOL_BYTES, used, q->ent[ENT(tail)].off, w->off,
				   (uint16_t)(w->off + ll_fifo_size(w->len)), paylen);
	} else {
		at = 0;
	}
	if (at < 0) {
		q->overflow++;
		return false;
	}
	e = &q->ent[ENT(head)];
	e->off = (uint16_t)at;
	e->hdr0 = pdu[0];
	e->len = (uint8_t)paylen;
	ll_fifo_copy(&q->area[2 + at], &pdu[2], paylen);
	RING_BARRIER();   /* publish: record complete before head moves */
	q->head = (uint8_t)(head + 1);
	q->queued = true;
	return true;
}

enum ll_rxq_result ll_rxq_get(uint8_t link, struct ll_rx_pdu *out)
{
	struct rxq_link *q;
	uint8_t tail;

	if (link >= LL_MAX_CONN) {
		return LL_RXQ_EMPTY;
	}
	q = &links[link];
	tail = q->tail;

	if (q->mic_failed) {
		return LL_RXQ_MIC_FAIL;   /* sticky: nothing more of this link */
	}
	if (tail != q->head) {
		struct rxq_ent *e = &q->ent[ENT(tail)];
		uint8_t *data;
		enum ll_rxq_result res = LL_RXQ_OK;

		RING_BARRIER();   /* consume: head observed before the record is read */
		data = &q->area[2 + e->off];
		if (q->crypt && q->crypt->enc_rx) {
			int r = ll_crypt_decrypt(q->crypt, e->hdr0, data, e->len);

			if (r < 0) {
				res = LL_RXQ_MIC_FAIL;
				q->mic_failed = true;
			} else {
				out->len = (uint8_t)r;
			}
		} else {
			out->len = e->len;
		}
		if (res == LL_RXQ_OK) {
			out->hdr0 = e->hdr0;
			ll_fifo_copy(out->data, data, out->len);
		}
		RING_BARRIER();   /* release: record fully read before tail frees it */
		q->tail = (uint8_t)(tail + 1);
		return res;
	}
	return LL_RXQ_EMPTY;
}

uint32_t ll_rxq_overflow_count(uint8_t link)
{
	return link < LL_MAX_CONN ? links[link].overflow : 0;
}
