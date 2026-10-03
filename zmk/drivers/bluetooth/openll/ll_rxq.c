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
 * (ll_rxq_isr_put cannot run while ll_rxq_get is executing) plain volatile
 * loads/stores are sufficient, no ll_plat_lock() needed.
 *
 * Duplicate detection (central retransmission because our ack was lost) and
 * decryption happen in the consumer, in SN order, so a retransmission never
 * reaches ll_crypt and never changes its counters. The first PDU of a
 * connection is accepted whatever its SN (ll_rxq_reset() clears the "have a
 * last SN" state).
 *
 * SN tracking is a Link Layer concept and is updated for every accepted
 * (non-duplicate) PDU before decryption is attempted, independent of
 * whether that decryption succeeds: a MIC failure is a property of the
 * payload, not of the on-air sequencing. On LL_RXQ_MIC_FAIL the caller is
 * expected to terminate the connection (Core Spec Vol 6 Part B 5.1.3.1), so
 * there is no legitimate in-connection retry to track.
 */
#include <string.h>

#include "ll_rxq.h"

#define RING_DEPTH 16
#define RING_MASK  (RING_DEPTH - 1)
#define HDR_SN     0x08   /* data PDU header byte 0, bit 3 */

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
	bool have_last_sn;
	uint8_t last_sn;
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

	if (len < 2 || (uint8_t)(head - q.tail) >= RING_DEPTH) {
		q.overflow++;
		return false;
	}
	paylen = (uint8_t)(len - 2);
	e = &q.ring[head & RING_MASK];
	if (paylen > sizeof(e->data)) {
		q.overflow++;
		return false;
	}
	e->hdr0 = pdu[0];
	e->len = paylen;
	if (paylen) {
		memcpy(e->data, &pdu[2], paylen);
	}
	q.head = (uint8_t)(head + 1);
	return true;
}

enum ll_rxq_result ll_rxq_get(struct ll_rx_pdu *out)
{
	while (q.tail != q.head) {
		struct rxq_entry *e = &q.ring[q.tail & RING_MASK];
		uint8_t sn = (e->hdr0 & HDR_SN) ? 1 : 0;
		bool dup = q.have_last_sn && sn == q.last_sn;

		q.tail = (uint8_t)(q.tail + 1);
		if (dup) {
			continue;
		}
		q.have_last_sn = true;
		q.last_sn = sn;

		if (e->len == 0) {
			/* Empty PDU: consumes the SN (handled above), nothing
			 * to deliver, never encrypted. */
			continue;
		}
		if (q.crypt && q.crypt->enc_rx) {
			int r = ll_crypt_decrypt(q.crypt, e->hdr0, e->data, e->len);

			if (r < 0) {
				return LL_RXQ_MIC_FAIL;
			}
			out->len = (uint8_t)r;
		} else {
			out->len = e->len;
		}
		out->hdr0 = e->hdr0;
		memcpy(out->data, e->data, out->len);
		return LL_RXQ_OK;
	}
	return LL_RXQ_EMPTY;
}

uint32_t ll_rxq_overflow_count(void)
{
	return q.overflow;
}
