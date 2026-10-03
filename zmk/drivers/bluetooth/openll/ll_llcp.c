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
 *
 * Slice 6b: Data Length Update (5.1.9, values per 4.5.10) as responder and
 * initiator, and the PHY Update procedure (5.1.10) as a 1M-only responder.
 * Each procedure kind has its own 40 s response timer (TMR_ENC, TMR_DLE,
 * TMR_PHY): they can overlap (LENGTH has no instant, so it is compatible
 * with the others, 5.3), and one completing must not stop another's timer.
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
#define OP_LENGTH_REQ        0x14
#define OP_LENGTH_RSP        0x15
#define OP_PHY_REQ           0x16
#define OP_PHY_RSP           0x17
#define OP_PHY_UPDATE_IND    0x18

/* payload length incl. the opcode */
#define LEN_CONN_UPDATE_IND  12
#define LEN_CHANNEL_MAP_IND  8
#define LEN_TERMINATE_IND    2
#define LEN_ENC_REQ          23
#define LEN_FEATURE_REQ      9
#define LEN_VERSION_IND      6
#define LEN_START_ENC_RSP    1
#define LEN_PAUSE_ENC_REQ    1
#define LEN_UNKNOWN_RSP      2
#define LEN_REJECT_EXT_IND   3
#define LEN_LENGTH           9    /* LL_LENGTH_REQ and LL_LENGTH_RSP */
#define LEN_PHY_REQ          3
#define LEN_PHY_UPDATE_IND   5

#define RSP_TIMEOUT_TICKS    (40000000u * LL_TICKS_PER_US)

/* per-link procedure response timers (Vol 6 Part B 5.2) */
enum {
	TMR_ENC,   /* encryption start: waits on the host's LTK / the central */
	TMR_DLE,   /* our LL_LENGTH_REQ: waits on LL_LENGTH_RSP */
	TMR_PHY,   /* our LL_PHY_RSP: waits on LL_PHY_UPDATE_IND */
	TMR_N,
};

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
	bool tmr_on[TMR_N];
	uint32_t tmr_start[TMR_N];
	/* Data length (4.5.10), each as {tx octets, tx time, rx octets, rx time}:
	 * own = connMax*, remote = connRemoteMax* (the central's Tx / Rx),
	 * told = our values as the central knows them (27 / 328 until our
	 * LL_LENGTH_REQ or LL_LENGTH_RSP carried others), eff = effective. */
	struct ll_llcp_dle own, remote, told, eff;
	bool dle_pending;     /* our LL_LENGTH_REQ queued, LL_LENGTH_RSP awaited */
	bool dle_want;        /* own != told: send LL_LENGTH_REQ after the encryption start */
	bool dle_unsupp;      /* the central answered LL_UNKNOWN_RSP to LL_LENGTH_REQ */
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
static void timer_start(struct llcp_link *s, unsigned int t)
{
	s->tmr_on[t] = true;
	s->tmr_start[t] = ll_radio_now();
}

/* ---- data length (Vol 6 Part B 4.5.10, 5.1.9) ---- */

static const struct ll_llcp_dle dle_default = {
	.max_tx_octets = LL_DLE_MIN_OCTETS, .max_tx_time = LL_DLE_MIN_TIME,
	.max_rx_octets = LL_DLE_MIN_OCTETS, .max_rx_time = LL_DLE_MIN_TIME,
};

static uint16_t min_u16(uint16_t a, uint16_t b)
{
	return a < b ? a : b;
}

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

static bool dle_same(const struct ll_llcp_dle *a, const struct ll_llcp_dle *b)
{
	return a->max_tx_octets == b->max_tx_octets && a->max_tx_time == b->max_tx_time &&
	       a->max_rx_octets == b->max_rx_octets && a->max_rx_time == b->max_rx_time;
}

/* New connection: connMaxTx = 27 / 328 until the host (or the glue, from
 * the suggested default) sets more; connMaxRx = what we can receive; the
 * remote values are 27 / 328 (4.5.10 "For a new connection"). */
static void dle_init(struct llcp_link *s)
{
	s->own = dle_default;
	s->own.max_rx_octets = LL_DLE_SUPP_OCTETS;
	s->own.max_rx_time = LL_DLE_SUPP_TIME;
	s->remote = dle_default;
	s->told = dle_default;
	s->eff = dle_default;
}

/* Recompute the effective values; true when one changed. Caller holds
 * ll_plat_tx_lock(). */
static bool dle_eff_update(struct llcp_link *s)
{
	struct ll_llcp_dle e = {
		.max_tx_octets = min_u16(s->own.max_tx_octets, s->remote.max_rx_octets),
		.max_tx_time = min_u16(s->own.max_tx_time, s->remote.max_rx_time),
		.max_rx_octets = min_u16(s->own.max_rx_octets, s->remote.max_tx_octets),
		.max_rx_time = min_u16(s->own.max_rx_time, s->remote.max_tx_time),
	};
	bool changed = !dle_same(&e, &s->eff);

	s->eff = e;
	return changed;
}

/* CtrData of LL_LENGTH_REQ / LL_LENGTH_RSP: MaxRxOctets, MaxRxTime,
 * MaxTxOctets, MaxTxTime (2.4.2.21). */
static void dle_build(uint8_t pdu[LEN_LENGTH], uint8_t op, const struct ll_llcp_dle *v)
{
	pdu[0] = op;
	ll_put_le16(&pdu[1], v->max_rx_octets);
	ll_put_le16(&pdu[3], v->max_rx_time);
	ll_put_le16(&pdu[5], v->max_tx_octets);
	ll_put_le16(&pdu[7], v->max_tx_time);
}

/* The central's values; below the minimum (it "shall" not send that) taken
 * as the minimum, above the Table 4.6 range capped. */
static void dle_parse_remote(struct llcp_link *s, const uint8_t *p)
{
	s->remote.max_rx_octets = clamp_u16(ll_get_le16(&p[1]), LL_DLE_MIN_OCTETS, LL_DLE_MAX_OCTETS);
	s->remote.max_rx_time = clamp_u16(ll_get_le16(&p[3]), LL_DLE_MIN_TIME, LL_DLE_MAX_TIME_ANY);
	s->remote.max_tx_octets = clamp_u16(ll_get_le16(&p[5]), LL_DLE_MIN_OCTETS, LL_DLE_MAX_OCTETS);
	s->remote.max_tx_time = clamp_u16(ll_get_le16(&p[7]), LL_DLE_MIN_TIME, LL_DLE_MAX_TIME_ANY);
}

/* Our LL_LENGTH_REQ (initiator). Caller holds ll_plat_tx_lock(). */
static int dle_send_req_locked(uint8_t link)
{
	struct llcp_link *s = &links[link];
	uint8_t pdu[LEN_LENGTH];
	int ret;

	dle_build(pdu, OP_LENGTH_REQ, &s->own);
	ret = tx_locked(link, LL_TXQ_CTRL, LL_LLID_CTRL, pdu, sizeof(pdu));
	if (ret == 0) {
		s->told = s->own;
		s->dle_pending = true;
		s->dle_want = false;
		timer_start(s, TMR_DLE);
	}
	return ret;
}

/* A request deferred by the encryption start leaves once the link is out
 * of it. A failed push (backlog full) is not retried: the central keeps
 * our previous values, which stay valid. Caller holds ll_plat_tx_lock(). */
static void dle_flush_locked(uint8_t link)
{
	struct llcp_link *s = &links[link];

	if (!s->dle_want || s->enc != ENC_IDLE || s->dle_pending) {
		return;
	}
	s->dle_want = false;
	if (!dle_same(&s->own, &s->told)) {
		(void)dle_send_req_locked(link);
	}
}

static void dle_notify(uint8_t link, const struct ll_llcp_dle *eff)
{
	if (ops.data_len_change) {
		ops.data_len_change(link, eff);
	}
}

/* Responder: answer with our connMax values (also while our own request
 * runs: a crossing is harmless, 5.1.9 Note). */
static void rx_length_req(uint8_t link, const uint8_t *p)
{
	struct llcp_link *s = &links[link];
	struct ll_llcp_dle eff;
	uint8_t rsp[LEN_LENGTH];
	bool changed;

	ll_plat_tx_lock();
	dle_parse_remote(s, p);
	changed = dle_eff_update(s);
	eff = s->eff;
	dle_build(rsp, OP_LENGTH_RSP, &s->own);
	if (tx_locked(link, LL_TXQ_CTRL, LL_LLID_CTRL, rsp, sizeof(rsp)) == 0) {
		/* "use the response to communicate the changes" */
		s->told = s->own;
	}
	ll_plat_tx_unlock();
	if (changed) {
		dle_notify(link, &eff);
	}
}

/* Initiator: the response to our LL_LENGTH_REQ completes the procedure;
 * any other LL_LENGTH_RSP is ignored (5.1.9: only "a response to an
 * LL_LENGTH_REQ" updates the remote values). */
static void rx_length_rsp(uint8_t link, const uint8_t *p)
{
	struct llcp_link *s = &links[link];
	struct ll_llcp_dle eff;
	bool changed = false;

	ll_plat_tx_lock();
	if (s->dle_pending) {
		s->dle_pending = false;
		s->tmr_on[TMR_DLE] = false;
		dle_parse_remote(s, p);
		changed = dle_eff_update(s);
	}
	eff = s->eff;
	ll_plat_tx_unlock();
	if (changed) {
		dle_notify(link, &eff);
	}
}

/* LL_UNKNOWN_RSP / LL_REJECT_EXT_IND naming one of our requests: the
 * procedure ends without a change. */
static void rx_proc_refused(uint8_t link, uint8_t op, bool unknown)
{
	struct llcp_link *s = &links[link];

	if (op != OP_LENGTH_REQ) {
		return;
	}
	ll_plat_tx_lock();
	if (s->dle_pending) {
		s->dle_pending = false;
		s->tmr_on[TMR_DLE] = false;
		if (unknown) {
			s->dle_unsupp = true;
		}
	}
	ll_plat_tx_unlock();
}

uint8_t ll_llcp_set_data_len(uint8_t link, uint16_t tx_octets, uint16_t tx_time)
{
	struct llcp_link *s;
	struct ll_llcp_dle prev, eff;
	bool changed = false;
	uint8_t st;

	if (link >= LL_MAX_CONN || !ll_conn_active(link)) {
		return LL_ST_UNKNOWN_CONN_ID;
	}
	s = &links[link];
	tx_octets = clamp_u16(tx_octets, LL_DLE_MIN_OCTETS, LL_DLE_SUPP_OCTETS);
	tx_time = clamp_u16(tx_time, LL_DLE_MIN_TIME, LL_DLE_SUPP_TIME);

	ll_plat_tx_lock();
	prev = s->own;
	s->own.max_tx_octets = tx_octets;
	s->own.max_tx_time = tx_time;
	if (s->dle_pending) {
		s->own = prev;
		st = LL_ST_DISALLOWED;
	} else if (dle_same(&s->own, &s->told)) {
		/* the central knows these values: no procedure needed */
		s->dle_want = false;
		changed = dle_eff_update(s);
		st = LL_ST_SUCCESS;
	} else if (s->dle_unsupp ||
		   (s->peer_feat_valid && !(s->peer_feat0 & LL_FEAT_DLE))) {
		s->own = prev;
		st = LL_ST_UNSUPP_REMOTE;
	} else if (s->enc != ENC_IDLE) {
		/* no other procedure's PDU during the encryption start
		 * (5.1.3.1): sent by dle_flush_locked() once it ended */
		s->dle_want = true;
		changed = dle_eff_update(s);
		st = LL_ST_SUCCESS;
	} else if (dle_send_req_locked(link) == 0) {
		/* a lower connMaxTx applies at once, a higher one up to what
		 * the central said it receives */
		changed = dle_eff_update(s);
		st = LL_ST_SUCCESS;
	} else {
		s->own = prev;
		st = LL_ST_MEM_CAPACITY;
	}
	eff = s->eff;
	ll_plat_tx_unlock();
	if (changed) {
		dle_notify(link, &eff);
	}
	return st;
}

void ll_llcp_get_dle(uint8_t link, struct ll_llcp_dle *out)
{
	if (link >= LL_MAX_CONN) {
		*out = dle_default;
		return;
	}
	ll_plat_tx_lock();
	*out = links[link].eff;
	ll_plat_tx_unlock();
}

/* ---- PHY (5.1.10), 1M only ---- */

static void rx_phy_req(uint8_t link)
{
	static const uint8_t rsp[3] = {OP_PHY_RSP, LL_PHY_1M, LL_PHY_1M};
	struct llcp_link *s = &links[link];

	ll_plat_tx_lock();
	if (tx_locked(link, LL_TXQ_CTRL, LL_LLID_CTRL, rsp, sizeof(rsp)) == 0) {
		timer_start(s, TMR_PHY);   /* until LL_PHY_UPDATE_IND */
	}
	ll_plat_tx_unlock();
}

/* Every LL_PHY_UPDATE_IND keeps 1M in both directions: 0 = unchanged (no
 * instant), 0x01 = the PHY in use, anything else (2M, Coded, an RFU bit,
 * several bits) is a PHY we did not offer and "the Peripheral shall not
 * change the PHY in that direction". No change and not host initiated:
 * the host is not told. The procedure is complete. */
static void rx_phy_update_ind(uint8_t link)
{
	ll_plat_tx_lock();
	links[link].tmr_on[TMR_PHY] = false;
	ll_plat_tx_unlock();
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
			timer_start(s, TMR_ENC);
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
			s->tmr_on[TMR_ENC] = false;
			done = true;
			/* a LENGTH request the host made meanwhile */
			dle_flush_locked(link);
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
	 * the other bytes are ours (byte 1: CSA#2, valid from controller to
	 * controller) */
	rsp[1] = LL_FEATURES_LOW & p[1];
	rsp[2] = LL_FEATURES_BYTE1;
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
	case OP_LENGTH_REQ:      return LEN_LENGTH;
	case OP_PHY_REQ:         return LEN_PHY_REQ;
	case OP_PHY_UPDATE_IND:  return LEN_PHY_UPDATE_IND;
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
	/* responses to our own requests (else dropped by ignored()) */
	if (op == OP_LENGTH_RSP && len == LEN_LENGTH) {
		rx_length_rsp(link, payload);
		return;
	}
	if (op == OP_UNKNOWN_RSP && len == LEN_UNKNOWN_RSP) {
		rx_proc_refused(link, payload[1], true);
		return;
	}
	if (op == OP_REJECT_EXT_IND && len == LEN_REJECT_EXT_IND) {
		rx_proc_refused(link, payload[1], false);
		return;
	}
	if (ignored(op)) {
		return;
	}
	want = expected_len(op);
	if (want == 0 || len != want) {
		/* Unsupported (PING, PERIPHERAL_FEATURE, MIN_USED_CHANNELS,
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
	case OP_LENGTH_REQ:
		rx_length_req(link, payload);
		break;
	case OP_PHY_REQ:
		rx_phy_req(link);
		break;
	case OP_PHY_UPDATE_IND:
		rx_phy_update_ind(link);
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
		timer_start(s, TMR_ENC);
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
		s->tmr_on[TMR_ENC] = false;
	}
	ll_plat_tx_unlock();
	if (!ok) {
		return LL_ST_DISALLOWED;
	}
	reject(link, OP_ENC_REQ, LL_ST_PIN_KEY_MISSING);
	/* a LENGTH request the host made meanwhile, after the reject */
	ll_plat_tx_lock();
	dle_flush_locked(link);
	ll_plat_tx_unlock();
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

		expired[i] = false;
		for (unsigned int t = 0; t < TMR_N; t++) {
			expired[i] |= s->tmr_on[t] &&
				      (int32_t)(now_tick - s->tmr_start[t]) >=
					      (int32_t)RSP_TIMEOUT_TICKS;
		}
		if (expired[i]) {
			/* the link is lost: every procedure on it ends */
			memset(s->tmr_on, 0, sizeof(s->tmr_on));
			s->enc = ENC_IDLE;
			s->paused = false;
			s->dle_pending = false;
			s->dle_want = false;
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

		for (unsigned int t = 0; t < TMR_N; t++) {
			int32_t left;

			if (!s->tmr_on[t]) {
				continue;
			}
			left = (int32_t)RSP_TIMEOUT_TICKS - (int32_t)(now_tick - s->tmr_start[t]);
			if (left < 0) {
				left = 0;
			}
			if (min < 0 || left < min) {
				min = left;
			}
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
	if (*(volatile enum enc_state *)&s->enc != ENC_IDLE || *(volatile bool *)&s->paused) {
		return true;
	}
	for (unsigned int t = 0; t < TMR_N; t++) {
		if (*(volatile bool *)&s->tmr_on[t]) {
			return true;
		}
	}
	return false;
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
	dle_init(&links[link]);
	ll_plat_tx_unlock();
}
