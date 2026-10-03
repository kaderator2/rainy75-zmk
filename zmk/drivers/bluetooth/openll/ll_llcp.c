/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Responder LL control procedures (spec "LLCP (responder)" table), PDU
 * formats per Core Spec Vol 6 Part B 2.4.2.
 *
 * Encryption start (Vol 6 Part B 5.1.3.1, peripheral side):
 *   RX LL_ENC_REQ (Rand, EDIV, SKDm, IVm)  -> data PDUs paused,
 *      TX LL_ENC_RSP (SKDs, IVs) plaintext, LTK request to the host
 *   LTK reply  -> SK = e(LTK, SKDs || SKDm), rx decryption on,
 *      TX LL_START_ENC_REQ plaintext
 *   RX LL_START_ENC_RSP (arrives encrypted, ll_rxq decrypted it)
 *      -> tx encryption on, TX LL_START_ENC_RSP encrypted, data PDUs
 *      resume, Encryption Change to the host
 *   LTK negative reply -> LL_REJECT_EXT_IND(LL_ENC_REQ, 0x06) when both
 *      sides support Extended Reject Indication (feature exchange done),
 *      else LL_REJECT_IND(0x06); data PDUs resume unencrypted.
 * The 40 s procedure response timer (5.2) restarts whenever we queue a
 * control PDU of the procedure (LL_ENC_RSP, LL_START_ENC_REQ) and stops
 * when it completes; it expiring ends the link with 0x22.
 *
 * All outgoing data PDUs go through ll_llcp_tx(): encrypt (when enc_tx)
 * and push under ll_plat_tx_lock(), so the TX packet counter order equals
 * the queue order whichever thread queues. The procedure state is
 * thread-only and also guarded by ll_plat_tx_lock(); ll_plat_lock()
 * (interrupts off) is held only for ll_txq_push() and for the switch of
 * the RX decryption context, never across AES (Task 10: encrypt + push
 * under the IRQ lock held it for 360-398 us).
 *
 * Slice 6a: the state above exists once per link (links[LL_MAX_CONN]); a
 * procedure, timer or MIC failure of one link never touches another. The
 * single ll_plat_tx_lock() mutex serializes all links (cheap, and keeps
 * each link's TX counter order).
 */
#include <errno.h>
#include <string.h>

#include "ll_conn.h"
#include "ll_crypt.h"
#include "ll_defs.h"
#include "ll_llcp.h"
#include "ll_plat.h"
#include "ll_radio.h"
#include "ll_rxq.h"
#include "ll_txq.h"

/* LL control PDU opcodes (Vol 6 Part B 2.4.2) */
#define OP_CONN_UPDATE_IND   0x00
#define OP_CHANNEL_MAP_IND   0x01
#define OP_TERMINATE_IND     0x02
#define OP_ENC_REQ           0x03
#define OP_ENC_RSP           0x04
#define OP_START_ENC_REQ     0x05
#define OP_START_ENC_RSP     0x06
#define OP_UNKNOWN_RSP       0x07
#define OP_FEATURE_REQ       0x08
#define OP_FEATURE_RSP       0x09
#define OP_PAUSE_ENC_REQ     0x0A
#define OP_PAUSE_ENC_RSP     0x0B
#define OP_VERSION_IND       0x0C
#define OP_REJECT_IND        0x0D
#define OP_CONN_PARAM_RSP    0x10
#define OP_REJECT_EXT_IND    0x11
#define OP_PING_RSP          0x13
#define OP_LENGTH_RSP        0x15
#define OP_PHY_RSP           0x17

/* payload length incl. the opcode */
#define LEN_CONN_UPDATE_IND  12
#define LEN_CHANNEL_MAP_IND  8
#define LEN_TERMINATE_IND    2
#define LEN_ENC_REQ          23
#define LEN_FEATURE_REQ      9
#define LEN_VERSION_IND      6
#define LEN_START_ENC_RSP    1
#define LEN_PAUSE_ENC_REQ    1

#define RSP_TIMEOUT_TICKS    (40000000u * LL_TICKS_PER_US)

enum enc_state {
	ENC_IDLE,
	ENC_WAIT_LTK,        /* LL_ENC_RSP queued, LTK request at the host */
	ENC_WAIT_START_RSP,  /* LL_START_ENC_REQ queued */
};


static struct ll_llcp_ops ops;

static struct llcp_link {
	struct ll_crypt crypt;
	enum enc_state enc;
	bool paused;          /* data PDUs held back (encryption procedure) */
	bool version_sent;
	bool peer_feat_valid;
	uint8_t peer_feat0;   /* byte 0 of the central's LL_FEATURE_REQ */
	uint8_t skdm[8];
	uint8_t skds[8];
	bool tmr_on;
	uint32_t tmr_start;
} links[LL_MAX_CONN];

/* Caller holds ll_plat_tx_lock(); link < LL_MAX_CONN. */
static int tx_locked(uint8_t link, enum ll_txq_kind kind, uint8_t llid, const uint8_t *payload,
		     uint8_t len)
{
	struct llcp_link *s = &links[link];
	uint8_t buf[LL_DATA_PDU_MAX + LL_MIC_LEN];
	uint8_t n = len;
	unsigned int key;
	int ret;

	if (len == 0 || len > LL_DATA_PDU_MAX) {
		return -EINVAL;
	}
	if (kind == LL_TXQ_ACL && s->paused) {
		return -EAGAIN;
	}
	memcpy(buf, payload, len);
	if (s->crypt.enc_tx) {
		/* the AAD only uses the LLID of hdr0 */
		n = (uint8_t)ll_crypt_encrypt(&s->crypt, llid, buf, len);
	}
	key = ll_plat_lock();
	ret = ll_txq_push(link, kind, llid, buf, n, kind == LL_TXQ_CTRL ? payload[0] : 0);
	ll_plat_unlock(key);
	if (ret != 0) {
		if (s->crypt.enc_tx) {
			s->crypt.tx_ctr--;   /* not queued: the counter value is reused */
		}
		return ret;
	}
	/* Peripheral latency: listen at the next regular event instead of
	 * the planned (skipped-ahead) one. Outside the IRQ lock (it takes
	 * ll_plat_lock() itself), after the push, so a plan racing with it
	 * either sees the backlog or is re-planned here. */
	ll_conn_kick(link);
	return 0;
}

int ll_llcp_tx(uint8_t link, enum ll_txq_kind kind, uint8_t llid, const uint8_t *payload,
	       uint8_t len)
{
	int ret;

	if (link >= LL_MAX_CONN) {
		return -EINVAL;
	}
	ll_plat_tx_lock();
	ret = tx_locked(link, kind, llid, payload, len);
	ll_plat_tx_unlock();
	return ret;
}

int ll_llcp_ctrl_tx(uint8_t link, const uint8_t *payload, uint8_t len)
{
	return ll_llcp_tx(link, LL_TXQ_CTRL, LL_LLID_CTRL, payload, len);
}

/* Responses are not retried when the backlog is full: the central's own
 * response timeout then ends the link. */
static void ctrl(uint8_t link, const uint8_t *pdu, uint8_t len)
{
	(void)ll_llcp_ctrl_tx(link, pdu, len);
}

static void unknown_rsp(uint8_t link, uint8_t op)
{
	const uint8_t pdu[2] = {OP_UNKNOWN_RSP, op};

	ctrl(link, pdu, sizeof(pdu));
}

/* LL_REJECT_EXT_IND if both sides support Extended Reject Indication
 * (known after the feature exchange), else LL_REJECT_IND (Vol 6 Part B
 * 2.4.2.18 / 5.1.3.1). */
static void reject(uint8_t link, uint8_t op, uint8_t err)
{
	const struct llcp_link *s = &links[link];

	if (s->peer_feat_valid && (s->peer_feat0 & LL_FEATURES_LOW & LL_FEAT_EXT_REJ_IND)) {
		const uint8_t pdu[3] = {OP_REJECT_EXT_IND, op, err};

		ctrl(link, pdu, sizeof(pdu));
	} else {
		const uint8_t pdu[2] = {OP_REJECT_IND, err};

		ctrl(link, pdu, sizeof(pdu));
	}
}

/* caller holds ll_plat_tx_lock() */
static void timer_start(struct llcp_link *s)
{
	s->tmr_on = true;
	s->tmr_start = ll_radio_now();
}

static void put_le32(uint8_t *p, uint32_t v)
{
	ll_put_le16(p, (uint16_t)v);
	ll_put_le16(p + 2, (uint16_t)(v >> 16));
}

static void rx_enc_req(uint8_t link, const uint8_t *p)
{
	struct llcp_link *s = &links[link];
	uint8_t rsp[13];
	bool ok, sent = false;

	ll_plat_tx_lock();
	/* a running procedure, or an encrypted link without the pause
	 * procedure (unsupported): not allowed */
	ok = s->enc == ENC_IDLE && !s->crypt.enc_tx;
	if (ok) {
		memset(&s->crypt, 0, sizeof(s->crypt));
		memcpy(s->skdm, &p[11], 8);
		memcpy(&s->crypt.iv[0], &p[19], 4);   /* IVm */
		put_le32(&s->skds[0], ll_plat_rand32());
		put_le32(&s->skds[4], ll_plat_rand32());
		put_le32(&s->crypt.iv[4], ll_plat_rand32());   /* IVs */
		rsp[0] = OP_ENC_RSP;
		memcpy(&rsp[1], s->skds, 8);
		memcpy(&rsp[9], &s->crypt.iv[4], 4);
		if (tx_locked(link, LL_TXQ_CTRL, LL_LLID_CTRL, rsp, sizeof(rsp)) == 0) {
			s->enc = ENC_WAIT_LTK;
			s->paused = true;
			timer_start(s);
			sent = true;
		} else {
			/* backlog full: the central waits for LL_ENC_RSP and
			 * pauses its data meanwhile, so the procedure cannot
			 * complete. End the link now rather than leave the
			 * host an LTK request for a procedure the central
			 * never sees answered (the 40 s timer would end it
			 * anyway). */
			memset(&s->crypt, 0, sizeof(s->crypt));
		}
	}
	ll_plat_tx_unlock();
	if (!ok) {
		reject(link, OP_ENC_REQ, LL_ST_LMP_PDU_NOT_ALLOWED);
		return;
	}
	if (!sent) {
		ll_conn_end(link, LL_ST_UNSPECIFIED);
		return;
	}
	/* same thread as the ll_rxq consumer; enc_rx is still off */
	ll_rxq_set_crypt(link, &s->crypt);
	if (ops.ltk_req) {
		ops.ltk_req(link, &p[1], ll_get_le16(&p[9]));
	}
}

static void rx_start_enc_rsp(uint8_t link)
{
	static const uint8_t rsp[1] = {OP_START_ENC_RSP};
	struct llcp_link *s = &links[link];
	bool done = false;

	ll_plat_tx_lock();
	if (s->enc == ENC_WAIT_START_RSP) {
		s->crypt.enc_tx = true;
		if (tx_locked(link, LL_TXQ_CTRL, LL_LLID_CTRL, rsp, sizeof(rsp)) == 0) {
			s->enc = ENC_IDLE;
			s->paused = false;
			s->tmr_on = false;
			done = true;
		} else {
			/* backlog full: stay in the procedure, the
			 * response timer ends the link */
			s->crypt.enc_tx = false;
		}
	}
	ll_plat_tx_unlock();
	if (done && ops.enc_change) {
		ops.enc_change(link, LL_ST_SUCCESS, true);
	}
}

static void rx_feature_req(uint8_t link, const uint8_t *p)
{
	struct llcp_link *s = &links[link];
	uint8_t rsp[9] = {OP_FEATURE_RSP};

	ll_plat_tx_lock();
	s->peer_feat_valid = true;
	s->peer_feat0 = p[1];
	ll_plat_tx_unlock();
	/* byte 0: the features used on this link (ours AND the central's),
	 * the other bytes are ours (none) */
	rsp[1] = LL_FEATURES_LOW & p[1];
	ctrl(link, rsp, sizeof(rsp));
}

static void rx_version_ind(uint8_t link)
{
	static const uint8_t rsp[6] = {OP_VERSION_IND, LL_HCI_VERSION,
				       LL_COMPANY_ID & 0xFF, LL_COMPANY_ID >> 8,
				       LL_SUBVERSION & 0xFF, LL_SUBVERSION >> 8};
	struct llcp_link *s = &links[link];

	ll_plat_tx_lock();
	/* answered once per connection (Vol 6 Part B 5.1.5) */
	if (!s->version_sent && tx_locked(link, LL_TXQ_CTRL, LL_LLID_CTRL, rsp, sizeof(rsp)) == 0) {
		s->version_sent = true;
	}
	ll_plat_tx_unlock();
}

/* Result of ll_conn_update_at / ll_conn_chmap_at. LL_ST_INSTANT_PASSED:
 * ll_conn already ends the link (0x28). LL_ST_INVALID_LL_PARAM: we end it.
 * LL_ST_DISALLOWED (no connection any more): nothing to do. */
static void instant_result(uint8_t link, int r)
{
	if (r == LL_ST_INVALID_LL_PARAM) {
		ll_conn_end(link, LL_ST_INVALID_LL_PARAM);
	}
}

static void rx_conn_update(uint8_t link, const uint8_t *p)
{
	struct ll_conn_params cp = {
		.interval = ll_get_le16(&p[4]),
		.latency = ll_get_le16(&p[6]),
		.timeout = ll_get_le16(&p[8]),
	};
	instant_result(link, ll_conn_update_at(link, ll_get_le16(&p[10]), p[1],
						 ll_get_le16(&p[2]), &cp));
}

/* Channel map with at least 2 used channels (bits 0..36, Vol 6 Part B
 * 2.4.2.2), else the link ends with 0x1E. */
static void rx_channel_map(uint8_t link, const uint8_t *p)
{
	unsigned int used = 0;

	for (unsigned int ch = 0; ch < 37; ch++) {
		used += (p[1 + ch / 8] >> (ch % 8)) & 1u;
	}
	if (used < 2) {
		ll_conn_end(link, LL_ST_INVALID_LL_PARAM);
		return;
	}
	instant_result(link, ll_conn_chmap_at(link, ll_get_le16(&p[6]), &p[1]));
}

/* length the request must have, 0 = not a request we answer by content */
static uint8_t expected_len(uint8_t op)
{
	switch (op) {
	case OP_CONN_UPDATE_IND: return LEN_CONN_UPDATE_IND;
	case OP_CHANNEL_MAP_IND: return LEN_CHANNEL_MAP_IND;
	case OP_TERMINATE_IND:   return LEN_TERMINATE_IND;
	case OP_ENC_REQ:         return LEN_ENC_REQ;
	case OP_FEATURE_REQ:     return LEN_FEATURE_REQ;
	case OP_VERSION_IND:     return LEN_VERSION_IND;
	case OP_PAUSE_ENC_REQ:   return LEN_PAUSE_ENC_REQ;
	default:                 return 0;
	}
}

/* responses to procedures we never start: dropped silently (never answer
 * an LL_UNKNOWN_RSP / reject with another one) */
static bool ignored(uint8_t op)
{
	switch (op) {
	case OP_START_ENC_RSP:   /* only expected inside the procedure */
	case OP_UNKNOWN_RSP:
	case OP_FEATURE_RSP:
	case OP_PAUSE_ENC_RSP:
	case OP_REJECT_IND:
	case OP_CONN_PARAM_RSP:
	case OP_REJECT_EXT_IND:
	case OP_PING_RSP:
	case OP_LENGTH_RSP:
	case OP_PHY_RSP:
		return true;
	default:
		return false;
	}
}

void ll_llcp_rx(uint8_t link, const uint8_t *payload, uint8_t len)
{
	uint8_t op, want;

	if (len == 0 || link >= LL_MAX_CONN) {
		return;
	}
	op = payload[0];
	if (op == OP_START_ENC_RSP && len == LEN_START_ENC_RSP) {
		rx_start_enc_rsp(link);
		return;
	}
	if (ignored(op)) {
		return;
	}
	want = expected_len(op);
	if (want == 0 || len != want) {
		/* Unsupported (LENGTH, PHY, PING, PERIPHERAL_FEATURE,
		 * CONN_PARAM, ...), or a known request whose length is not
		 * exactly the specified one: LL_UNKNOWN_RSP, as Zephyr ll_sw
		 * does for PDUs failing its exact-length validation. */
		unknown_rsp(link, op);
		return;
	}
	switch (op) {
	case OP_CONN_UPDATE_IND:
		rx_conn_update(link, payload);
		break;
	case OP_CHANNEL_MAP_IND:
		rx_channel_map(link, payload);
		break;
	case OP_TERMINATE_IND:
		ll_conn_end(link, payload[1]);
		break;
	case OP_ENC_REQ:
		rx_enc_req(link, payload);
		break;
	case OP_FEATURE_REQ:
		rx_feature_req(link, payload);
		break;
	case OP_VERSION_IND:
		rx_version_ind(link);
		break;
	case OP_PAUSE_ENC_REQ:
		/* encryption pause / key refresh not supported */
		reject(link, OP_PAUSE_ENC_REQ, LL_ST_UNSUPP_REMOTE);
		break;
	default:
		break;
	}
}

uint8_t ll_llcp_ltk_reply(uint8_t link, const uint8_t ltk[16])
{
	static const uint8_t req[1] = {OP_START_ENC_REQ};
	uint8_t st = LL_ST_DISALLOWED;
	struct llcp_link *s;
	uint8_t sk[16];

	if (link >= LL_MAX_CONN) {
		return LL_ST_DISALLOWED;
	}
	s = &links[link];
	ll_plat_tx_lock();
	if (s->enc == ENC_WAIT_LTK) {
		/* AES outside the IRQ lock */
		ll_crypt_session_key(ltk, s->skdm, s->skds, sk);

		/* The controller thread decrypts (ll_rxq) without the TX
		 * lock: the key, counters and enc_rx change together under the
		 * IRQ lock. The central answers LL_START_ENC_REQ encrypted, so
		 * decryption must be on before that PDU can be on air;
		 * enabling it now (before the push) guarantees that. Turning
		 * it on early is safe: since its LL_ENC_REQ the central sends
		 * no data PDUs and no other control PDU of a procedure
		 * (5.1.3.1), only empty PDUs, which ll_rxq never decrypts.
		 * Anything non-empty arriving in between would fail its MIC
		 * and end the link (0x3D), which is the right outcome for such
		 * a protocol violation.
		 * One legal exception: the central may send LL_TERMINATE_IND
		 * at any time (e.g. its host disconnects during pairing). Sent
		 * before it has our LL_START_ENC_REQ, it is plaintext, fails
		 * the MIC here, and the link ends with 0x3D: our host then
		 * sees Disconnection Complete with 0x3D (MIC failure) instead
		 * of the central's reason. The link ends either way; accepted
		 * rather than special-casing a plaintext TERMINATE_IND. */
		unsigned int key = ll_plat_lock();

		memcpy(s->crypt.sk, sk, sizeof(sk));
		s->crypt.tx_ctr = 0;
		s->crypt.rx_ctr = 0;
		s->crypt.enc_tx = false;
		s->crypt.enc_rx = true;
		ll_plat_unlock(key);
		ll_crypt_wipe(sk, sizeof(sk));
		s->enc = ENC_WAIT_START_RSP;
		(void)tx_locked(link, LL_TXQ_CTRL, LL_LLID_CTRL, req, sizeof(req));
		timer_start(s);
		st = LL_ST_SUCCESS;
	}
	ll_plat_tx_unlock();
	return st;
}

uint8_t ll_llcp_ltk_neg_reply(uint8_t link)
{
	struct llcp_link *s;
	bool ok;

	if (link >= LL_MAX_CONN) {
		return LL_ST_DISALLOWED;
	}
	s = &links[link];
	ll_plat_tx_lock();
	ok = s->enc == ENC_WAIT_LTK;
	if (ok) {
		s->enc = ENC_IDLE;
		s->paused = false;
		s->tmr_on = false;
	}
	ll_plat_tx_unlock();
	if (!ok) {
		return LL_ST_DISALLOWED;
	}
	reject(link, OP_ENC_REQ, LL_ST_PIN_KEY_MISSING);
	return LL_ST_SUCCESS;
}

uint8_t ll_llcp_terminate(uint8_t link, uint8_t reason)
{
	if (link >= LL_MAX_CONN || !ll_conn_active(link)) {
		return LL_ST_UNKNOWN_CONN_ID;
	}
	ll_conn_terminate(link, reason);
	return LL_ST_SUCCESS;
}

void ll_llcp_tick(uint32_t now_tick)
{
	bool expired[LL_MAX_CONN];

	ll_plat_tx_lock();
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		struct llcp_link *s = &links[i];

		expired[i] = s->tmr_on &&
			     (int32_t)(now_tick - s->tmr_start) >= (int32_t)RSP_TIMEOUT_TICKS;
		if (expired[i]) {
			s->tmr_on = false;
			s->enc = ENC_IDLE;
			s->paused = false;
		}
	}
	ll_plat_tx_unlock();
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		if (expired[i]) {
			ll_conn_end(i, LL_ST_LMP_TIMEOUT);
		}
	}
}

int32_t ll_llcp_timeout_ticks(uint32_t now_tick)
{
	int32_t min = -1;

	ll_plat_tx_lock();
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		const struct llcp_link *s = &links[i];
		int32_t left;

		if (!s->tmr_on) {
			continue;
		}
		left = (int32_t)RSP_TIMEOUT_TICKS - (int32_t)(now_tick - s->tmr_start);
		if (left < 0) {
			left = 0;
		}
		if (min < 0 || left < min) {
			min = left;
		}
	}
	ll_plat_tx_unlock();
	return min;
}

/* ISR context, no lock: each field is one aligned word or byte (a single
 * load), written by threads under ll_plat_tx_lock(). volatile so the
 * compiler reads the current values. */
bool ll_llcp_busy(uint8_t link)
{
	struct llcp_link *s;

	if (link >= LL_MAX_CONN) {
		return false;
	}
	s = &links[link];
	return *(volatile enum enc_state *)&s->enc != ENC_IDLE ||
	       *(volatile bool *)&s->paused || *(volatile bool *)&s->tmr_on;
}

void ll_llcp_init(const struct ll_llcp_ops *o)
{
	memset(&ops, 0, sizeof(ops));
	if (o) {
		ops = *o;
	}
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		ll_llcp_reset(i);
	}
}

void ll_llcp_reset(uint8_t link)
{
	if (link >= LL_MAX_CONN) {
		return;
	}
	ll_plat_tx_lock();
	memset(&links[link], 0, sizeof(links[link]));   /* also wipes the session key */
	ll_plat_tx_unlock();
}
