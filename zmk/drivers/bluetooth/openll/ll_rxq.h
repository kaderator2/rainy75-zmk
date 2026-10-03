/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * RX queue: ISR-filled software ring of received data PDUs, consumed by the
 * controller thread (duplicate detection by SN, decrypt, empty PDUs dropped).
 */
#ifndef LL_RXQ_H_
#define LL_RXQ_H_

#include <stdbool.h>
#include <stdint.h>

#include "ll_defs.h"

struct ll_rx_pdu {
	uint8_t hdr0;   /* data PDU header byte 0 (LLID, NESN, SN, MD) */
	uint8_t len;    /* payload length (after decryption: without MIC) */
	uint8_t data[LL_DATA_PDU_MAX + LL_MIC_LEN];
};

void ll_rxq_reset(void);
/* ISR: copy one CRC-valid PDU (2-byte header + payload, as delivered by
 * LL_RADIO_CONN_RX). Returns false on overflow (dropped and counted). */
bool ll_rxq_isr_put(const uint8_t *pdu, uint8_t len);
/* Thread: next new (non-duplicate), non-empty PDU, decrypted when
 * encryption is on. Returns false when nothing is pending. MIC failure
 * reporting is defined in Task 4. */
bool ll_rxq_get(struct ll_rx_pdu *out);

#endif /* LL_RXQ_H_ */
