/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * RX queue: ISR-filled software ring of received data PDUs per link,
 * consumed by the controller thread (decrypt). Empty PDUs are dropped at
 * the put; there is no SN duplicate check, the baseband delivers only new
 * packets (ll_rxq.c). Every entry point takes the link id (0 <= link <
 * LL_MAX_CONN); an out-of-range id is ignored (put: false, get: EMPTY).
 */
#ifndef LL_RXQ_H_
#define LL_RXQ_H_

#include <stdbool.h>
#include <stdint.h>

#include "ll_crypt.h"
#include "ll_defs.h"

struct ll_rx_pdu {
	uint8_t hdr0;   /* data PDU header byte 0 (LLID, NESN, SN, MD) */
	uint8_t len;    /* payload length (after decryption: without MIC) */
	/* connEventCounter of the connection event the PDU was received in
	 * (instants are judged against it, not against the event counter at
	 * the time the controller thread handles the PDU) */
	uint16_t event;
	uint8_t data[LL_DATA_PDU_MAX + LL_MIC_LEN];
};

/* Per link (slice 6b Task 4, long PDUs): at most LL_RXQ_ENTRIES PDUs
 * queued, their payload bytes (each rounded up to 4) in one FIFO area of
 * LL_RXQ_POOL_BYTES (ll_fifo.h): 16 short PDUs, or 8 of the maximum 255
 * bytes (251 + MIC). */
#define LL_RXQ_ENTRIES 16
#ifndef LL_RXQ_POOL_BYTES
#define LL_RXQ_POOL_BYTES 2048
#endif

enum ll_rxq_result {
	LL_RXQ_EMPTY,     /* ring drained, nothing to deliver */
	LL_RXQ_OK,        /* out filled with a new, non-empty, decrypted PDU */
	LL_RXQ_MIC_FAIL,  /* decrypt of the next PDU failed its MIC;
			   * caller terminates the connection (LL_ST_MIC_FAILURE).
			   * Sticky: every later ll_rxq_get() returns MIC_FAIL
			   * (nothing is delivered, out untouched) until
			   * ll_rxq_reset() of that link. */
};

/* Empty the link's ring, clear its crypt pointer, overflow count, MIC
 * failure and wake flag. Thread (the link's consumer), while the link
 * produces nothing (before it starts or after it ended). */
void ll_rxq_reset(uint8_t link);
/* Set (or clear, with NULL) the encryption context used to decrypt the
 * link's incoming PDUs. Thread context; ll_llcp sets the link's own
 * context when the encryption procedure starts (enc_rx turns on later,
 * under ll_plat_lock()). */
void ll_rxq_set_crypt(uint8_t link, struct ll_crypt *c);
/* ISR: copy one CRC-valid PDU of the link that owns the running event
 * (2-byte header + payload, as delivered by LL_RADIO_CONN_RX: pdu[0] =
 * header byte 0, pdu[1] = on-air length, pdu[2..] = payload; len = 2 +
 * payload length, up to 257: uint16_t, slice 6b), received in connection
 * event `event` (handed out with it, ll_rx_pdu.event). Single producer (the
 * radio ISR). An empty PDU is accepted and not queued. Returns false on
 * overflow (entries or bytes) or a malformed length (dropped and counted).
 * A dropped data PDU is lost for good (the hardware has acked it), so the
 * caller ends the link (ll_conn: 0x08). */
bool ll_rxq_isr_put(uint8_t link, const uint8_t *pdu, uint16_t len, uint16_t event);
/* ISR (the producer side, e.g. at LL_RADIO_CONN_DONE): true if a data PDU
 * was queued on any link since the last call (or that link's
 * ll_rxq_reset()), and clears that flag for all links. Empty and dropped
 * PDUs do not count, so the consumer is woken only when there is something
 * to deliver. */
bool ll_rxq_isr_take_queued(void);
/* ISR (producer side): true when the link's queue is at least half full,
 * in entries or in bytes, so the consumer should drain it before the
 * event ends (slice 6b Task 4: a long central burst). */
bool ll_rxq_isr_half_full(uint8_t link);
/* ISR (producer side, slice 7 Task 2c): true when n more PDUs of len
 * payload bytes each (as received, MIC included) fit into the link's
 * queue now, in entries and in bytes (the same placement as
 * ll_rxq_isr_put). The radio acks every new central packet in hardware, so
 * the producer stops receiving (ll_conn) while fewer than the PDUs that can
 * still arrive would fit, instead of losing one. n = 0: true; an
 * out-of-range link: false. */
bool ll_rxq_isr_room(uint8_t link, uint8_t n, uint16_t len);
/* Thread: the link's next queued PDU, decrypted when the link's encryption
 * is on; call again after LL_RXQ_OK to continue draining. Single consumer. */
enum ll_rxq_result ll_rxq_get(uint8_t link, struct ll_rx_pdu *out);
/* Count of the link's ll_rxq_isr_put() calls dropped for lack of ring room
 * (or a malformed length), since its last ll_rxq_reset(). */
uint32_t ll_rxq_overflow_count(uint8_t link);

#endif /* LL_RXQ_H_ */
