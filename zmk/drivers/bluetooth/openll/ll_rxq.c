/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * RX queue: a lock-free 16-entry software ring between the RX ISR (single
 * producer, ll_rxq_isr_put, copies each CRC-valid connection PDU out of the
 * RX DMA ring before it is overwritten) and the controller thread (single
 * consumer, ll_rxq_get). head/tail are free-running 8-bit counters, each
 * written by exactly one side; the slot content is written by the producer
 * strictly before head is advanced, and only read by the consumer after it
 * observes that advance, so on a single core with ISR/thread separation
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
 * Decryption happens in the consumer, in RX order. On LL_RXQ_MIC_FAIL the
 * caller is expected to terminate the connection (Core Spec Vol 6 Part B
 * 5.1.3.1), so there is no in-connection retry.
 */
#include <string.h>

#include "ll_rxq.h"

#define RING_DEPTH 16
#define RING_MASK  (RING_DEPTH - 1)
/* compiler-only barrier (no fence instruction); see the file comment */
#define RING_BARRIER() __atomic_signal_fence(__ATOMIC_SEQ_CST)

_Static_assert((RING_DEPTH & RING_MASK) == 0, "RING_DEPTH must be a power of two");

struct rxq_entry {
	uint8_t hdr0;
	uint8_t len;   /* payload length as received (ciphertext + MIC if encrypted) */
	uint8_t data[LL_DATA_PDU_MAX + LL_MIC_LEN];
};

static struct {
	struct rxq_entry ring[RING_DEPTH];
	volatile uint8_t head;   /* next free slot, advanced by the producer (ISR) */
	volatile uint8_t tail;   /* next slot to consume, advanced by the consumer (thread) */
	uint32_t overflow;
	struct ll_crypt *crypt;
} q;

void ll_rxq_reset(void)
{
	memset(&q, 0, sizeof(q));
}

void ll_rxq_set_crypt(struct ll_crypt *c)
{
	q.crypt = c;
}

bool ll_rxq_isr_put(const uint8_t *pdu, uint8_t len)
{
	uint8_t head = q.head;
	uint8_t paylen;
	struct rxq_entry *e;

	if (len < 2) {
		q.overflow++;
		return false;
	}
	paylen = (uint8_t)(len - 2);
	if (paylen == 0) {
		return true;   /* empty PDU: nothing to deliver */
	}
	if ((uint8_t)(head - q.tail) >= RING_DEPTH) {
		q.overflow++;
		return false;
	}
	e = &q.ring[head & RING_MASK];
	if (paylen > sizeof(e->data)) {
		q.overflow++;
		return false;
	}
	e->hdr0 = pdu[0];
	e->len = paylen;
	memcpy(e->data, &pdu[2], paylen);
	RING_BARRIER();   /* publish: slot complete before head moves */
	q.head = (uint8_t)(head + 1);
	return true;
}

enum ll_rxq_result ll_rxq_get(struct ll_rx_pdu *out)
{
	uint8_t tail = q.tail;

	if (tail != q.head) {
		struct rxq_entry *e = &q.ring[tail & RING_MASK];
		enum ll_rxq_result res = LL_RXQ_OK;

		RING_BARRIER();   /* consume: head observed before the slot is read */
		if (q.crypt && q.crypt->enc_rx) {
			int r = ll_crypt_decrypt(q.crypt, e->hdr0, e->data, e->len);

			if (r < 0) {
				res = LL_RXQ_MIC_FAIL;
			} else {
				out->len = (uint8_t)r;
			}
		} else {
			out->len = e->len;
		}
		if (res == LL_RXQ_OK) {
			out->hdr0 = e->hdr0;
			memcpy(out->data, e->data, out->len);
		}
		RING_BARRIER();   /* release: slot fully read before tail frees it */
		q.tail = (uint8_t)(tail + 1);
		return res;
	}
	return LL_RXQ_EMPTY;
}

uint32_t ll_rxq_overflow_count(void)
{
	return q.overflow;
}
