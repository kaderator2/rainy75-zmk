/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Advertising channel PDU encode/decode. All addresses are in air order
 * (little-endian, LSB first), matching HCI BD_ADDR order.
 */
#ifndef LL_PDU_H_
#define LL_PDU_H_

#include <stdbool.h>
#include <stdint.h>

struct ll_connect_ind {
	uint8_t init_a[6];
	uint8_t init_addr_random; /* TxAdd */
	uint8_t chsel;            /* ChSel: central supports CSA#2 */
	uint32_t aa;
	uint32_t crc_init;
	uint8_t win_size;         /* 1.25 ms units */
	uint16_t win_offset;      /* 1.25 ms units */
	uint16_t interval;        /* 1.25 ms units */
	uint16_t latency;
	uint16_t timeout;         /* 10 ms units */
	uint8_t chm[5];
	uint8_t hop;
	uint8_t sca;
	/* Slice 6c: the local address the CONNECT_IND was sent to (our AdvA
	 * at that advertising event) and its type (RxAdd); kept with the
	 * link as the record of the address it was made with. */
	uint8_t adv_a[6];
	uint8_t adv_addr_random;
};

/* Advertising PDU (ADV_IND / ADV_NONCONN_IND / ADV_SCAN_IND / SCAN_RSP)
 * with AdvA = adva; tx_add != 0 sets TxAdd (AdvA is a random address,
 * Vol 6 Part B 2.3.1). */
uint8_t ll_pdu_build_adv(uint8_t *out, uint8_t pdu_type, const uint8_t adva[6], uint8_t tx_add,
			 const uint8_t *data, uint8_t len);
/* SCAN_REQ / CONNECT_IND for us: AdvA == adva and RxAdd == (tx_add != 0),
 * the address type we advertise with. */
bool ll_pdu_is_scan_req_for(const uint8_t *pdu, uint8_t len, const uint8_t adva[6],
			    uint8_t tx_add);
int ll_pdu_parse_connect_ind(const uint8_t *pdu, uint8_t len, const uint8_t adva[6],
			     uint8_t tx_add, struct ll_connect_ind *ci);

#endif /* LL_PDU_H_ */
