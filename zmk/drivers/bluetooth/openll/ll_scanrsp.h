/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * SCAN_REQ -> SCAN_RSP decision and timing (pure, host-tested; slice 7
 * Task 2 item 1). The radio answers in its RX ISR, before the link layer
 * callback: a SCAN_REQ addressed to the AdvA (and TxAdd) of the prepared
 * SCAN_RSP gets a scheduled single TX (STX) whose first bit is on air
 * T_IFS (150 us) after the request ends. Nothing else is ever answered.
 *
 * Why not the hardware RX -> TX turnaround (BRX / RX2TX): the baseband
 * then transmits after whatever it receives (CONNECT_IND, a SCAN_REQ for
 * another AdvA); only a CPU veto in the RX ISR could stop it, and that ISR
 * can be delayed by interrupt-locked sections (flash erase 13..26 ms), so
 * the controller would sometimes answer a packet it must not answer. The
 * STX path is the reverse: no answer unless the CPU decided in time.
 *
 * Timing (device + sniffer, s7-task-2-report.md): the first bit is on air
 * at trigger + TX settle + LL_SCANRSP_TX_PATH_US. With settle 50 the trigger
 * lies 41 us after the request's end; the RX ISR has decided by then in
 * almost every case (ISR entry 10..20 us after the end, decision 10..30 us).
 * A later decision is not answered (the scanner retries; its backoff).
 * The previous trigger (settle 78 after a 72 us lead) put the SCAN_RSP on
 * air 209 us after the request, outside the scanner's window.
 *
 * Both functions are static inline, without calls: the RX ISR runs them
 * from RAM (.ram_code). Calling flash-resident code there (a first version
 * called ll_pdu_is_scan_req_for() and memcmp() through XIP) moved the
 * decision from 10..20 us to 50..60 us after the request and made most
 * responses late.
 */
#ifndef LL_SCANRSP_H_
#define LL_SCANRSP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "ll_defs.h"

/* TX settle of the SCAN_RSP STX (us). */
#define LL_SCANRSP_SETTLE_US    50
/* Fixed delay from the end of the TX settle to the first bit on air (us),
 * measured on the B91 STX path (TX timestamp + sniffer). */
#define LL_SCANRSP_TX_PATH_US   59
/* A trigger closer than this to now is refused: the register writes after
 * the check must still land before the trigger tick (a trigger in the past
 * would leave the FSM waiting). */
#define LL_SCANRSP_MIN_LEAD_US  3

/* true if req (PDU header + payload, req_len bytes) is a SCAN_REQ for the
 * AdvA and TxAdd of rsp (the prepared SCAN_RSP PDU, rsp_len bytes). false
 * for anything else, also when rsp is not a SCAN_RSP. */
static inline bool ll_scanrsp_for_us(const uint8_t *rsp, uint8_t rsp_len, const uint8_t *req,
				     uint8_t req_len)
{
	/* SCAN_RSP: header (2) + AdvA (6) at least; SCAN_REQ: header (2) +
	 * ScanA (6) + AdvA (6), length field 12, RxAdd = our TxAdd */
	if (rsp == NULL || req == NULL || rsp_len < 8 || (rsp[0] & 0x0F) != LL_PDU_SCAN_RSP ||
	    req_len < 14 || (req[0] & 0x0F) != LL_PDU_SCAN_REQ || req[1] != 12 ||
	    ((req[0] >> 7) & 1) != ((rsp[0] >> 6) & 1)) {
		return false;
	}
	for (int i = 0; i < 6; i++) {
		if (req[8 + i] != rsp[2 + i]) {
			return false;
		}
	}
	return true;
}

/* STX trigger tick for a SCAN_RSP whose first bit is on air LL_T_IFS_US
 * after req_end (end of the request's last CRC bit, stimer ticks). Returns
 * false (and leaves *trigger alone) when the trigger is less than
 * LL_SCANRSP_MIN_LEAD_US after now or already past, or more than
 * LL_T_IFS_US ahead (only a bogus timestamp gives that); 32-bit tick wrap
 * handled. */
static inline bool ll_scanrsp_trigger(uint32_t req_end, uint32_t now, uint32_t *trigger)
{
	uint32_t t = req_end + (uint32_t)(LL_T_IFS_US - LL_SCANRSP_SETTLE_US -
					  LL_SCANRSP_TX_PATH_US) * LL_TICKS_PER_US;
	int32_t lead = (int32_t)(t - now);

	if (lead < (int32_t)(LL_SCANRSP_MIN_LEAD_US * LL_TICKS_PER_US) ||
	    lead > (int32_t)(LL_T_IFS_US * LL_TICKS_PER_US)) {
		return false;
	}
	*trigger = t;
	return true;
}

#endif /* LL_SCANRSP_H_ */
