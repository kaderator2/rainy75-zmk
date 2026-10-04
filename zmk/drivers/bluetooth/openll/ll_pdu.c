/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "ll_pdu.h"
#include "ll_defs.h"

#define HDR_TYPE(h)   ((h) & 0x0F)
#define HDR_CHSEL(h)  (((h) >> 5) & 1)
#define HDR_CHSEL_BIT 0x20
#define HDR_TXADD(h)  (((h) >> 6) & 1)
#define HDR_RXADD(h)  (((h) >> 7) & 1)
#define HDR_TXADD_BIT 0x40

uint8_t ll_pdu_build_adv(uint8_t *out, uint8_t pdu_type, const uint8_t adva[6], uint8_t tx_add,
			 const uint8_t *data, uint8_t len)
{
	/* TxAdd: 0 public AdvA, 1 random (Own_Address_Type 1). ChSel 1 (CSA#2 supported, Vol 6 Part B
	 * 2.3.1) in the connectable PDUs; RFU (0) in the others. A link uses
	 * CSA#2 only when the CONNECT_IND has ChSel 1 too (4.5.8.1). */
	out[0] = pdu_type & 0x0F;
	if (out[0] == LL_PDU_ADV_IND || out[0] == LL_PDU_ADV_DIRECT_IND) {
		out[0] |= HDR_CHSEL_BIT;
	}
	if (tx_add) {
		out[0] |= HDR_TXADD_BIT;
	}
	out[1] = 6 + len;
	memcpy(&out[2], adva, 6);
	if (len) {
		memcpy(&out[8], data, len);
	}
	return 8 + len;
}

bool ll_pdu_is_scan_req_for(const uint8_t *pdu, uint8_t len, const uint8_t adva[6],
			    uint8_t tx_add)
{
	return len >= 14 && HDR_TYPE(pdu[0]) == LL_PDU_SCAN_REQ && pdu[1] == 12 &&
	       HDR_RXADD(pdu[0]) == (tx_add ? 1 : 0) && memcmp(&pdu[8], adva, 6) == 0;
}

int ll_pdu_parse_connect_ind(const uint8_t *pdu, uint8_t len, const uint8_t adva[6],
			     uint8_t tx_add, struct ll_connect_ind *ci)
{
	const uint8_t *p = &pdu[2];

	if (len < 36 || HDR_TYPE(pdu[0]) != LL_PDU_CONNECT_IND || pdu[1] != 34 ||
	    HDR_RXADD(pdu[0]) != (tx_add ? 1 : 0) || memcmp(&p[6], adva, 6) != 0) {
		return -1;
	}
	memcpy(ci->adv_a, &p[6], 6);
	ci->adv_addr_random = HDR_RXADD(pdu[0]);
	memcpy(ci->init_a, &p[0], 6);
	ci->init_addr_random = HDR_TXADD(pdu[0]);
	ci->chsel = HDR_CHSEL(pdu[0]);
	ci->aa = ll_get_le32(&p[12]);
	ci->crc_init = ll_get_le24(&p[16]);
	ci->win_size = p[19];
	ci->win_offset = ll_get_le16(&p[20]);
	ci->interval = ll_get_le16(&p[22]);
	ci->latency = ll_get_le16(&p[24]);
	ci->timeout = ll_get_le16(&p[26]);
	memcpy(ci->chm, &p[28], 5);
	ci->hop = p[33] & 0x1F;
	ci->sca = p[33] >> 5;
	return 0;
}
