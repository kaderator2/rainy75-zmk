/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Responder LL control procedures, LL_TERMINATE_IND and the encryption
 * procedure (spec LLCP table). Thread context. The hook ops struct (HCI
 * events, txq push) is defined in Task 6.
 */
#ifndef LL_LLCP_H_
#define LL_LLCP_H_

#include <stdint.h>

void ll_llcp_reset(void);
/* One received LLID 3 PDU (decrypted payload: opcode + CtrData). */
void ll_llcp_rx(const uint8_t *payload, uint8_t len);
/* From HCI LE Long Term Key Request Reply / Negative Reply. */
void ll_llcp_ltk_reply(const uint8_t ltk[16]);
void ll_llcp_ltk_neg_reply(void);
/* From HCI Disconnect. */
void ll_llcp_terminate(uint8_t reason);
/* Procedure response timeouts (40 s, then terminate with 0x22). */
void ll_llcp_tick(uint32_t now_tick);

#endif /* LL_LLCP_H_ */
