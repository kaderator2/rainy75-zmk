/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Responder LL control procedures, LL_TERMINATE_IND and the encryption
 * start procedure (spec "LLCP (responder)" table; Core Spec Vol 6 Part B
 * 2.4.2 PDU formats, 5.1.3.1 encryption start, 5.2 response timeout).
 * ll_llcp owns the link encryption context (struct ll_crypt) and is the
 * single place where outgoing data PDUs are encrypted and pushed to ll_txq,
 * so the TX packet counter follows the queue order.
 *
 * Slice 6a: all state is per link (procedure state, encryption context,
 * feature/version flags, the 40 s response timer). Every entry point takes
 * the link id (0 <= link < LL_MAX_CONN) except ll_llcp_init(),
 * ll_llcp_tick() and ll_llcp_timeout_ticks(), which cover all links. An
 * out-of-range id is refused (see each function). ll_plat_tx_lock() stays
 * one mutex for all links.
 *
 * Context: thread only (except ll_llcp_busy(), ISR-safe), but entry points
 * may come from different threads (controller thread: ll_llcp_rx,
 * ll_llcp_tick, ll_llcp_tx; HCI thread: ll_llcp_ltk_reply/_neg_reply/
 * _terminate). State changes and the encrypt+push step run under
 * ll_plat_tx_lock() (a thread mutex); the IRQ lock ll_plat_lock() is taken
 * only around ll_txq_push(), the switch of the RX decryption context and
 * inside ll_conn_kick(), so AES never runs with interrupts locked except
 * inside one ll_plat_aes_ecb() block. The ops callbacks and
 * ll_conn_* calls are made without ll_plat_lock() held.
 */
#ifndef LL_LLCP_H_
#define LL_LLCP_H_

#include <stdbool.h>
#include <stdint.h>

#include "ll_txq.h"

/* Events toward the host (HCI). Called in the context of the entry point
 * that caused them, without ll_plat_lock() or ll_plat_tx_lock() held. Disconnection Complete
 * and LE Connection Update Complete come from ll_conn, not from here. */
struct ll_llcp_ops {
	/* HCI LE Long Term Key Request: rand as on air / HCI (LSB first),
	 * EDIV. Answered with ll_llcp_ltk_reply() or ll_llcp_ltk_neg_reply(). */
	void (*ltk_req)(uint8_t link, const uint8_t rand[8], uint16_t ediv);
	/* HCI Encryption Change (status, Encryption_Enabled). Reported with
	 * (LL_ST_SUCCESS, true) once our encrypted LL_START_ENC_RSP is
	 * queued. Not reported after a negative reply (the host chose it) or
	 * when the link ends during the procedure (Disconnection Complete). */
	void (*enc_change)(uint8_t link, uint8_t status, bool enabled);
};

/* Once at startup. ops is copied (members may be NULL). Resets all links. */
void ll_llcp_init(const struct ll_llcp_ops *ops);
/* Per connection (call when ll_conn reports CONNECTED or DISCONNECTED for
 * the link): forgets the link's procedure state, session key and
 * encryption flags; the other links are untouched. ll_rxq_reset(link)
 * clears ll_rxq's crypt pointer itself. Thread context. */
void ll_llcp_reset(uint8_t link);
/* One received LLID 3 PDU of the link (decrypted payload: opcode +
 * CtrData). Thread (controller thread, in the link's RX order). Ignored
 * for an out-of-range link. */
void ll_llcp_rx(uint8_t link, const uint8_t *payload, uint8_t len);
/* From HCI LE Long Term Key Request Reply / Negative Reply for the link
 * (ltk in HCI order, LSB first). Return an HCI status: LL_ST_SUCCESS, or
 * LL_ST_DISALLOWED when no LTK request is pending on the link (or the link
 * is out of range). */
uint8_t ll_llcp_ltk_reply(uint8_t link, const uint8_t ltk[16]);
uint8_t ll_llcp_ltk_neg_reply(uint8_t link);
/* From HCI Disconnect (reason already validated by HCI): LL_TERMINATE_IND
 * via ll_conn_terminate(). Returns LL_ST_SUCCESS, or
 * LL_ST_UNKNOWN_CONN_ID without a connection on the link. */
uint8_t ll_llcp_terminate(uint8_t link, uint8_t reason);
/* Procedure response timeout (Vol 6 Part B 5.2), per link: 40 s from the
 * last LL control PDU the link queued in a procedure that waits on the
 * central (or on the host's LTK), then ll_conn_end(LL_ST_LMP_TIMEOUT) of
 * that link only. Checks all links. Call from the controller thread with
 * the stimer tick when ll_llcp_timeout_ticks() says it is due (calling it
 * earlier or more often is harmless). */
void ll_llcp_tick(uint32_t now_tick);
/* Ticks until the earliest running per-link procedure response timer
 * expires at now_tick (0 when one is due), or -1 while none is running.
 * The controller thread arms its wakeup for ll_llcp_tick() from this, so
 * nothing polls while no procedure is pending. Thread. */
int32_t ll_llcp_timeout_ticks(uint32_t now_tick);
/* ll_conn_ops.busy hook (peripheral latency): true while an LL control
 * procedure of the link waits on the host or the central (encryption
 * start: LL_ENC_RSP queued until our LL_START_ENC_RSP is queued). Reads
 * the procedure state without ll_plat_tx_lock(): ISR-safe, never blocks; a
 * stale answer costs at most one skip window. False for an out-of-range
 * link. */
bool ll_llcp_busy(uint8_t link);

/* Encrypt (when the link is encrypted) and queue one data PDU on the link:
 * ll_txq_push(link, ...) with ctrl_opcode = payload[0] for LL_TXQ_CTRL.
 * len 1..LL_DATA_PDU_MAX (plaintext). Returns 0, -EINVAL (also for an
 * out-of-range link), -ENOMEM (backlog full, nothing queued,
 * counter unchanged) or, for LL_TXQ_ACL, -EAGAIN while the encryption
 * procedure pauses data PDUs (from LL_ENC_REQ until our LL_START_ENC_RSP
 * is queued, Vol 6 Part B 5.1.3.1; keep the PDU and retry later). After
 * a successful push it calls ll_conn_kick() (without ll_plat_lock() held),
 * so the PDU leaves at the next regular connection event even while
 * peripheral latency skips events. Thread. */
int ll_llcp_tx(uint8_t link, enum ll_txq_kind kind, uint8_t llid, const uint8_t *payload,
	       uint8_t len);
/* ll_conn_ops.ctrl_tx hook: ll_llcp_tx(link, LL_TXQ_CTRL, LL_LLID_CTRL, ...). */
int ll_llcp_ctrl_tx(uint8_t link, const uint8_t *payload, uint8_t len);

#endif /* LL_LLCP_H_ */
