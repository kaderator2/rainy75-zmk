/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * RX queue: ISR-filled software ring of received data PDUs, consumed by the
 * controller thread (decrypt). Empty PDUs are dropped at the put; there is
 * no SN duplicate check, the baseband delivers only new packets (ll_rxq.c).
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
	uint8_t data[LL_DATA_PDU_MAX + LL_MIC_LEN];
};

enum ll_rxq_result {
	LL_RXQ_EMPTY,     /* ring drained, nothing to deliver */
	LL_RXQ_OK,        /* out filled with a new, non-empty, decrypted PDU */
	LL_RXQ_MIC_FAIL,  /* decrypt of the next PDU failed its MIC;
			   * caller terminates the connection (LL_ST_MIC_FAILURE).
			   * Sticky: every later ll_rxq_get() returns MIC_FAIL
			   * (nothing is delivered, out untouched) until
			   * ll_rxq_reset(). */
};

void ll_rxq_reset(void);
/* Set (or clear, with NULL) the encryption context used to decrypt incoming
 * PDUs. Thread context; call once the LLCP encryption procedure has enabled
 * enc_rx on *c (ll_crypt itself is owned by ll_llcp/ll_conn). */
void ll_rxq_set_crypt(struct ll_crypt *c);
/* ISR: copy one CRC-valid PDU (2-byte header + payload, as delivered by
 * LL_RADIO_CONN_RX: pdu[0] = header byte 0, pdu[1] = on-air length,
 * pdu[2..] = payload; len = 2 + payload length). Single producer. An empty
 * PDU is accepted and not queued. Returns false on overflow or a malformed
 * length (dropped and counted). A dropped data PDU is lost for good (the
 * hardware has acked it), so the caller ends the link (ll_conn: 0x08). */
bool ll_rxq_isr_put(const uint8_t *pdu, uint8_t len);
/* ISR (the producer side, e.g. at LL_RADIO_CONN_DONE): true if a data PDU
 * was queued since the last call or ll_rxq_reset(), and clears that flag.
 * Empty and dropped PDUs do not count, so the consumer is woken only when
 * there is something to deliver. */
bool ll_rxq_isr_take_queued(void);
/* Thread: next queued PDU, decrypted when encryption is on; call again
 * after LL_RXQ_OK to continue draining. Single consumer. */
enum ll_rxq_result ll_rxq_get(struct ll_rx_pdu *out);
/* Count of ll_rxq_isr_put() calls dropped for lack of ring room (or a
 * malformed length), since the last ll_rxq_reset(). */
uint32_t ll_rxq_overflow_count(void);

#endif /* LL_RXQ_H_ */
