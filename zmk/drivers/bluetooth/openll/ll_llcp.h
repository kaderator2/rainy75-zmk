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

/* Effective data length of a link (connEffectiveMaxTxOctets / TxTime /
 * RxOctets / RxTime, Vol 6 Part B 4.5.10, 1M). Octets exclude the MIC. */
struct ll_llcp_dle {
	uint16_t max_tx_octets, max_tx_time, max_rx_octets, max_rx_time;
};

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
	/* HCI LE Data Length Change: the link's effective values
	 * (connEffectiveMax*, Vol 6 Part B 4.5.10) changed. Only on a change. */
	void (*data_len_change)(uint8_t link, const struct ll_llcp_dle *eff);
};

/* Owed control PDUs (slice 7). A control PDU we must send (a response, our
 * own request, LL_TERMINATE_IND through ll_llcp_ctrl_tx) whose push finds
 * the link's TX backlog full is owed: the procedure state moves on as if
 * it was queued, the PDU waits in the link's owed queue (at most
 * LL_LLCP_OWE_N, in order; later control PDUs of the link queue behind it
 * and host ACL of the link gets -EAGAIN, so control PDUs take the next free
 * slot) and ll_llcp_retry() pushes it. While a link owes a PDU,
 * ll_llcp_timeout_ticks() asks for a wakeup within LL_LLCP_RETRY_MS, and
 * ll_llcp_busy() is true. Owing never ends a link: a dead link ends by
 * supervision, a stuck procedure by its 40 s response timer. */
#define LL_LLCP_OWE_N           4
#define LL_LLCP_RETRY_MS        10

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
/* Procedure response timeout (Vol 6 Part B 5.2), per link and procedure
 * (encryption start, our LENGTH request, the PHY update after our
 * LL_PHY_RSP): 40 s from the last LL control PDU the procedure queued while
 * it waits on the central (or on the host's LTK), then
 * ll_conn_end(LL_ST_LMP_TIMEOUT) of that link only (its owed PDUs are
 * dropped with it). Checks all links. Call from the controller thread with
 * the stimer tick when ll_llcp_timeout_ticks() says it is due (calling it
 * earlier or more often is harmless). */
void ll_llcp_tick(uint32_t now_tick);
/* Push the link's owed control PDUs, in order, until the backlog is full
 * again (runs the steps that wait for them, e.g. Encryption Change after
 * our LL_START_ENC_RSP). Controller thread: call it for every link before
 * its host ACL, at every wakeup. No-op for an out-of-range link or when
 * nothing is owed. */
void ll_llcp_retry(uint8_t link);
/* Ticks until the earliest running per-link procedure response timer
 * expires at now_tick (0 when one is due), at most LL_LLCP_RETRY_MS while
 * a link owes control PDUs (retry wakeup), or -1 while none is running.
 * The controller thread arms its wakeup for ll_llcp_tick() from this, so
 * nothing polls while no procedure is pending. Thread. */
int32_t ll_llcp_timeout_ticks(uint32_t now_tick);
/* ll_conn_ops.busy hook (peripheral latency): true while an LL control
 * procedure of the link waits on the host or the central (encryption
 * start: LL_ENC_RSP queued until our LL_START_ENC_RSP is queued; our
 * LL_LENGTH_REQ until LL_LENGTH_RSP; our LL_PHY_RSP until
 * LL_PHY_UPDATE_IND), and while the link owes a control PDU. Reads
 * the procedure state without ll_plat_tx_lock(): ISR-safe, never blocks; a
 * stale answer costs at most one skip window. False for an out-of-range
 * link. */
bool ll_llcp_busy(uint8_t link);

/* Data Length Update procedure (Vol 6 Part B 5.1.9), per link.
 * Responder: an LL_LENGTH_REQ is answered with LL_LENGTH_RSP carrying our
 * connMax values (Rx: LL_DLE_SUPP_*, Tx: 27 / 328 or what the host set);
 * the central's values (clamped to 27..251 / 328..17040) become the remote
 * ones. Initiator (host LE Set Data Length): connMaxTx = the request
 * clamped to 27..LL_DLE_SUPP_OCTETS / 328..LL_DLE_SUPP_TIME; when the
 * central does not know these values yet, LL_LENGTH_REQ (deferred while
 * the encryption start runs, sent right after it), 40 s response timer;
 * LL_LENGTH_RSP completes it, LL_UNKNOWN_RSP / LL_REJECT_EXT_IND for
 * opcode 0x14 ends it without a change. A crossing LL_LENGTH_REQ is
 * answered normally and ours stays pending. ops.data_len_change reports
 * every change of the effective values.
 * Returns LL_ST_SUCCESS, LL_ST_UNKNOWN_CONN_ID (link out of range or not
 * connected), LL_ST_DISALLOWED (our procedure still runs),
 * LL_ST_UNSUPP_REMOTE (the central rejected LL_LENGTH_REQ before, or its
 * features lack DLE) or LL_ST_MEM_CAPACITY (backlog and owed queue full,
 * nothing changed; a merely full backlog owes the request, slice 7).
 * Thread. */
uint8_t ll_llcp_set_data_len(uint8_t link, uint16_t tx_octets, uint16_t tx_time);
/* The link's effective values; 27 / 328 for a new connection and for an
 * out-of-range link. Thread. */
void ll_llcp_get_dle(uint8_t link, struct ll_llcp_dle *out);

/* PHY Update procedure (5.1.10), handled by ll_llcp_rx() (there is no
 * separate entry point): as a 1M-only responder it answers LL_PHY_REQ with
 * LL_PHY_RSP(TX 1M, RX 1M) and runs the 40 s timer until the central's
 * LL_PHY_UPDATE_IND. Every LL_PHY_UPDATE_IND keeps 1M: 0 (no change), 1M,
 * and a PHY we lack / an RFU bit / several bits ("shall not change the PHY
 * in that direction"); nothing goes to the host (no change, not host
 * initiated). HCI LE Read PHY / LE Set PHY are answered by the glue
 * (always 1M). */

/* Encrypt (when the link is encrypted) and queue one data PDU on the link:
 * ll_txq_push(link, ...) with ctrl_opcode = payload[0] for LL_TXQ_CTRL.
 * len 1..LL_DATA_PDU_MAX (plaintext). Returns 0, -EINVAL (also for an
 * out-of-range link), -ENOMEM (backlog full, nothing queued,
 * counter unchanged) or, for LL_TXQ_ACL, -EAGAIN while the encryption
 * procedure pauses data PDUs (from LL_ENC_REQ until our LL_START_ENC_RSP
 * is queued, Vol 6 Part B 5.1.3.1) or the link owes control PDUs (slice
 * 7); keep the PDU and retry later. After
 * a successful push it calls ll_conn_kick() (without ll_plat_lock() held),
 * so the PDU leaves at the next regular connection event even while
 * peripheral latency skips events. Thread. */
int ll_llcp_tx(uint8_t link, enum ll_txq_kind kind, uint8_t llid, const uint8_t *payload,
	       uint8_t len);
/* ll_conn_ops.ctrl_tx hook: one of our control PDUs (opcode first),
 * queued like ll_llcp_tx(link, LL_TXQ_CTRL, LL_LLID_CTRL, ...), or owed when
 * the backlog is full or the link owes PDUs already (retried by
 * ll_llcp_retry()). Returns 0 (queued or owed), -EINVAL, or -ENOMEM when
 * the owed queue is full too (nothing kept). Thread. */
int ll_llcp_ctrl_tx(uint8_t link, const uint8_t *payload, uint8_t len);

#endif /* LL_LLCP_H_ */
