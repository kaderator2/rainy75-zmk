/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_llcp host tests. ll_conn, ll_txq_push, ll_rxq_set_crypt and the radio
 * clock are faked; ll_crypt is real (with the test-only software AES).
 * Built with LL_LLCP_HOST_CONN: ll_llcp calls the link-aware llcp_conn_*
 * fakes below instead of the (still single-link) ll_conn API.
 *
 * Slice 6a: the single-link suite runs on link 0 and on link LL_MAX_CONN - 1
 * (every fake records per link, the suite reads the records of its link L
 * through the cn / hci / rxq_crypt macros); the multi-link tests follow.
 *
 * PDU layouts: Core Spec Vol 6 Part B 2.4.2. Encryption procedure: Vol 6
 * Part B 5.1.3.1. Keys and the encrypted LL_START_ENC_RSP: Vol 6 Part C
 * "Encryption sample data" (same values as tests/test_crypt.c, see there
 * for the source). The spec's central is the remote device; its SKD_C/IV_C
 * arrive in LL_ENC_REQ, its SKD_P/IV_P are the values we must send, so the
 * fake random source returns them.
 */
#include <errno.h>
#include <string.h>
#include "test.h"
#include "../ll_conn.h"
#include "../ll_crypt.h"
#include "../ll_defs.h"
#include "../ll_llcp.h"
#include "../ll_plat.h"
#include "../ll_radio.h"
#include "../ll_rxq.h"
#include "../ll_txq.h"

#define T(us)       ((uint32_t)(us) * LL_TICKS_PER_US)
#define TIMEOUT_US  40000000u

void aes_ref_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

/* ---------------- sample data (Vol 6 Part C) ---------------- */

static const uint8_t ltk[16] = {0xBF, 0x01, 0xFB, 0x9D, 0x4E, 0xF3, 0xBC, 0x36,
				0xD8, 0x74, 0xF5, 0x39, 0x41, 0x38, 0x68, 0x4C};
static const uint8_t skdm[8] = {0x13, 0x02, 0xF1, 0xE0, 0xDF, 0xCE, 0xBD, 0xAC};
static const uint8_t skds[8] = {0x79, 0x68, 0x57, 0x46, 0x35, 0x24, 0x13, 0x02};
static const uint8_t ivm[4] = {0x24, 0xAB, 0xDC, 0xBA};
static const uint8_t ivs[4] = {0xBE, 0xBA, 0xAF, 0xDE};
static const uint8_t sk_msb[16] = {0x99, 0xAD, 0x1B, 0x52, 0x26, 0xA3, 0x7E, 0x3E,
				   0x05, 0x8E, 0x3B, 0x8E, 0x27, 0xC2, 0xC6, 0x66};
/* Rand and EDIV of the sample LL_ENC_REQ */
static const uint8_t rnd[8] = {0x90, 0x78, 0x56, 0x34, 0x12, 0xEF, 0xCD, 0xAB};
static const uint16_t ediv = 0x2474;
/* LL_START_ENC_RSP1 (central -> us), LL_START_ENC_RSP2 (us -> central) */
static const uint8_t rsp1_air[5] = {0x9F, 0xCD, 0xA7, 0xF4, 0x48};
static const uint8_t rsp2_air[5] = {0xA3, 0x4C, 0x13, 0xA4, 0x15};
/* LL_DATA2 (us -> central, packet counter 1), LLID 2 */
static const uint8_t data2_clear[27] = {
	0x17, 0x00, 0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x30, 0x41, 0x42, 0x43, 0x44,
	0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51};
static const uint8_t data2_air[31] = {
	0xF3, 0x88, 0x81, 0xE7, 0xBD, 0x94, 0xC9, 0xC3, 0x69, 0xB9, 0xA6, 0x68, 0x46, 0xDD,
	0x47, 0x86, 0xAA, 0x8C, 0x39, 0xCE, 0x54, 0x0D, 0x0D, 0xAE, 0x3A, 0xDC, 0xDF,
	0x89, 0xB9, 0x60, 0x88};

/* ---------------- fakes ---------------- */

static uint32_t now;
static int locks;
static uint32_t rand_seq[8];
static int rand_n, rand_i;

uint32_t ll_radio_now(void) { return now; }
uint32_t ll_plat_rand32(void) { return rand_i < rand_n ? rand_seq[rand_i++] : 0x5A5A5A5A; }
unsigned int ll_plat_lock(void) { locks++; return 0; }
void ll_plat_unlock(unsigned int k) { (void)k; locks--; }

/* TX producer serialization (thread mutex on the device) */
static int tx_locks, tx_lock_calls;
void ll_plat_tx_lock(void) { tx_locks++; tx_lock_calls++; }
void ll_plat_tx_unlock(void) { tx_locks--; }

/* AES never runs with interrupts locked by the caller (Task 10: the IRQ
 * lock of encrypt + push was 360-398 us; the B91 glue locks per block). */
void ll_plat_aes_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
	CHECK(locks == 0);
	aes_ref_encrypt(key, in, out);
}

/* link the single-link suite runs on */
static uint8_t L;

#define MAX_PUSH 16
static struct {
	int n;
	struct {
		uint8_t link;
		enum ll_txq_kind kind;
		uint8_t llid, len, op;
		uint8_t d[40];
	} p[MAX_PUSH];
	int fail;   /* next pushes return -ENOMEM */
} tx;

int ll_txq_push(uint8_t link, enum ll_txq_kind kind, uint8_t llid, const uint8_t *payload,
		uint8_t len, uint8_t ctrl_opcode)
{
	CHECK(link < LL_MAX_CONN);
	/* pushed under both: the IRQ lock for the queue, the TX lock for the
	 * counter order */
	CHECK(locks > 0);
	CHECK(tx_locks > 0);
	if (tx.fail) {
		tx.fail--;
		return -ENOMEM;
	}
	if (len > 31) {
		return -EINVAL;
	}
	if (tx.n < MAX_PUSH) {
		tx.p[tx.n].link = link;
		tx.p[tx.n].kind = kind;
		tx.p[tx.n].llid = llid;
		tx.p[tx.n].len = len;
		tx.p[tx.n].op = ctrl_opcode;
		memcpy(tx.p[tx.n].d, payload, len);
	}
	tx.n++;
	return 0;
}

static struct ll_crypt *rxq_crypt_l[LL_MAX_CONN];
static int rxq_set_calls;
#define rxq_crypt (rxq_crypt_l[L])
void ll_rxq_set_crypt(uint8_t link, struct ll_crypt *c)
{
	CHECK(link < LL_MAX_CONN);
	rxq_crypt_l[link] = c;
	rxq_set_calls++;
}

static struct {
	bool active;
	int upd_calls, chm_calls, term_calls, end_calls;
	int upd_ret, chm_ret;
	uint16_t instant;
	uint8_t win_size;
	uint16_t win_offset;
	struct ll_conn_params p;
	uint8_t chm[5];
	uint8_t term_reason, end_reason;
} cnl[LL_MAX_CONN];
#define cn (cnl[L])

/* link-aware ll_conn fakes (LL_LLCP_HOST_CONN) */
int llcp_conn_update_at(uint8_t link, uint16_t instant, uint8_t win_size, uint16_t win_offset,
			const struct ll_conn_params *p);
int llcp_conn_chmap_at(uint8_t link, uint16_t instant, const uint8_t chm[5]);
void llcp_conn_terminate(uint8_t link, uint8_t reason);
void llcp_conn_end(uint8_t link, uint8_t reason);
bool llcp_conn_active(uint8_t link);
void llcp_conn_kick(uint8_t link);

int llcp_conn_update_at(uint8_t link, uint16_t instant, uint8_t win_size, uint16_t win_offset,
			const struct ll_conn_params *p)
{
	CHECK(link < LL_MAX_CONN);
	cnl[link].upd_calls++;
	cnl[link].instant = instant;
	cnl[link].win_size = win_size;
	cnl[link].win_offset = win_offset;
	cnl[link].p = *p;
	return cnl[link].upd_ret;
}
int llcp_conn_chmap_at(uint8_t link, uint16_t instant, const uint8_t chm[5])
{
	CHECK(link < LL_MAX_CONN);
	cnl[link].chm_calls++;
	cnl[link].instant = instant;
	memcpy(cnl[link].chm, chm, 5);
	return cnl[link].chm_ret;
}
void llcp_conn_terminate(uint8_t link, uint8_t reason)
{
	CHECK(locks == 0);
	CHECK(link < LL_MAX_CONN);
	cnl[link].term_calls++;
	cnl[link].term_reason = reason;
}
void llcp_conn_end(uint8_t link, uint8_t reason)
{
	CHECK(locks == 0);
	CHECK(link < LL_MAX_CONN);
	cnl[link].end_calls++;
	cnl[link].end_reason = reason;
}
bool llcp_conn_active(uint8_t link)
{
	CHECK(link < LL_MAX_CONN);
	return cnl[link].active;
}

/* Kick after every successful push (slice 5): outside the IRQ lock, after
 * the PDU is in the queue, on the link of that push. n_at_kick: pushes seen
 * at the last kick. */
static struct {
	int calls;
	int n_at_kick;
	int bad;   /* kicks with the IRQ lock held, before a new push or on another link */
} kk;
void llcp_conn_kick(uint8_t link)
{
	if (locks != 0 || tx.n <= kk.n_at_kick ||
	    (tx.n <= MAX_PUSH && tx.p[tx.n - 1].link != link)) {
		kk.bad++;
	}
	kk.calls++;
	kk.n_at_kick = tx.n;
}

/* ---------------- HCI ops ---------------- */

static struct {
	int ltk_req;
	uint8_t rand[8];
	uint16_t ediv;
	int enc_change;
	uint8_t status;
	bool enabled;
} hcil[LL_MAX_CONN];
#define hci (hcil[L])

static void on_ltk_req(uint8_t link, const uint8_t r[8], uint16_t e)
{
	CHECK(locks == 0);
	CHECK(link < LL_MAX_CONN);
	hcil[link].ltk_req++;
	memcpy(hcil[link].rand, r, 8);
	hcil[link].ediv = e;
}

static void on_enc_change(uint8_t link, uint8_t status, bool enabled)
{
	CHECK(locks == 0);
	CHECK(link < LL_MAX_CONN);
	hcil[link].enc_change++;
	hcil[link].status = status;
	hcil[link].enabled = enabled;
}

static const struct ll_llcp_ops ops = {.ltk_req = on_ltk_req, .enc_change = on_enc_change};

/* ---------------- helpers ---------------- */

static void fresh(void)
{
	memset(&tx, 0, sizeof(tx));
	memset(cnl, 0, sizeof(cnl));
	memset(hcil, 0, sizeof(hcil));
	memset(&kk, 0, sizeof(kk));
	memset(rxq_crypt_l, 0, sizeof(rxq_crypt_l));
	rxq_set_calls = 0;
	rand_n = rand_i = 0;
	now = 1000;
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		cnl[i].active = true;
	}
	ll_llcp_init(&ops);
	ll_llcp_reset(L);
}

static void rx_l(uint8_t link, const uint8_t *pdu, uint8_t len)
{
	ll_llcp_rx(link, pdu, len);
	CHECK(locks == 0);
	CHECK(tx_locks == 0);
}

static void rx(const uint8_t *pdu, uint8_t len)
{
	ll_llcp_rx(L, pdu, len);
	CHECK(locks == 0);
	CHECK(tx_locks == 0);
}

/* last push is exactly {kind CTRL, LLID 3, opcode, bytes} */
static int last_is(const uint8_t *exp, uint8_t len)
{
	if (tx.n == 0) {
		return 0;
	}
	return tx.p[tx.n - 1].link == L &&
	       tx.p[tx.n - 1].kind == LL_TXQ_CTRL && tx.p[tx.n - 1].llid == LL_LLID_CTRL &&
	       tx.p[tx.n - 1].len == len && tx.p[tx.n - 1].op == exp[0] &&
	       memcmp(tx.p[tx.n - 1].d, exp, len) == 0;
}

static void build_enc_req(uint8_t pdu[23])
{
	pdu[0] = 0x03;
	memcpy(&pdu[1], rnd, 8);
	ll_put_le16(&pdu[9], ediv);
	memcpy(&pdu[11], skdm, 8);
	memcpy(&pdu[19], ivm, 4);
}

static void sample_rand(void)
{
	/* SKDs then IVs, each rand32 little-endian on air */
	rand_seq[0] = 0x46576879;
	rand_seq[1] = 0x02132435;
	rand_seq[2] = 0xDEAFBABE;
	rand_n = 3;
	rand_i = 0;
}

/* Feature exchange with the central's byte 0 = f0. */
static void features(uint8_t f0)
{
	uint8_t req[9] = {0x08, f0, 0xFF, 0, 0, 0, 0, 0, 0};

	rx(req, sizeof(req));
}

/* Full encryption start up to (and including) the encrypted START_ENC_RSP. */
static void start_encryption(void)
{
	uint8_t req[23], buf[8];
	int n0;

	sample_rand();
	build_enc_req(req);
	rx(req, sizeof(req));
	CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
	CHECK(rxq_crypt != NULL);
	memcpy(buf, rsp1_air, 5);
	CHECK(ll_crypt_decrypt(rxq_crypt, 0x0F, buf, 5) == 1);  /* as ll_rxq does */
	n0 = tx.n;
	rx(buf, 1);
	CHECK(tx.n == n0 + 1);
}

static void single_link_suite(void)
{
	/* ---- LL_FEATURE_REQ -> LL_FEATURE_RSP, byte 0 = ours AND central's ---- */
	fresh();
	features(0xFF);
	{
		/* ours: LE Encryption (bit 0) + Extended Reject Indication (bit 2) */
		static const uint8_t exp[9] = {0x09, 0x05, 0, 0, 0, 0, 0, 0, 0};

		CHECK(LL_FEATURES_LOW == 0x05);
		CHECK(tx.n == 1);
		CHECK(last_is(exp, 9));
	}
	features(0x01);
	{
		static const uint8_t exp[9] = {0x09, 0x01, 0, 0, 0, 0, 0, 0, 0};

		CHECK(last_is(exp, 9));
	}
	features(0x00);
	{
		static const uint8_t exp[9] = {0x09, 0, 0, 0, 0, 0, 0, 0, 0};

		CHECK(tx.n == 3);
		CHECK(last_is(exp, 9));
	}

	/* ---- LL_VERSION_IND: answered once per connection ---- */
	fresh();
	{
		static const uint8_t vi[6] = {0x0C, 0x0A, 0x02, 0x00, 0x34, 0x12};
		static const uint8_t exp[6] = {0x0C, 0x09, 0xFF, 0xFF, 0x01, 0x00};

		rx(vi, 6);
		CHECK(tx.n == 1);
		CHECK(last_is(exp, 6));
		rx(vi, 6);
		CHECK(tx.n == 1);
		ll_llcp_reset(L);   /* new connection */
		rx(vi, 6);
		CHECK(tx.n == 2);
		CHECK(last_is(exp, 6));
	}

	/* ---- unknown / unsupported -> LL_UNKNOWN_RSP(opcode) ---- */
	{
		static const uint8_t ops_unk[] = {0x14, 0x16, 0x12, 0x0E, 0x0F, 0x04, 0x05,
						  0x18, 0x20, 0xFF};
		static const uint8_t lens[] = {9, 3, 1, 9, 24, 13, 1, 5, 1, 1};

		for (unsigned int i = 0; i < sizeof(ops_unk); i++) {
			uint8_t pdu[27] = {0};
			uint8_t exp[2] = {0x07, ops_unk[i]};

			fresh();
			pdu[0] = ops_unk[i];
			rx(pdu, lens[i]);
			CHECK(tx.n == 1);
			CHECK(last_is(exp, 2));
		}
	}
	/* responses we never asked for, empty payload: no answer at all */
	{
		static const uint8_t ops_ign[] = {0x06, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x11, 0x13,
						  0x15, 0x17};
		static const uint8_t lens[] = {1, 2, 9, 1, 2, 24, 3, 1, 9, 3};

		fresh();
		for (unsigned int i = 0; i < sizeof(ops_ign); i++) {
			uint8_t pdu[27] = {0};

			pdu[0] = ops_ign[i];
			rx(pdu, lens[i]);
		}
		rx(ops_ign, 0);
		CHECK(tx.n == 0);
		CHECK(cn.end_calls == 0);
	}
	/* a known request with a wrong length -> LL_UNKNOWN_RSP */
	{
		static const uint8_t bad_feat[5] = {0x08, 1, 2, 3, 4};
		static const uint8_t bad_upd[6] = {0x00, 1, 2, 3, 4, 5};
		static const uint8_t exp_f[2] = {0x07, 0x08};
		static const uint8_t exp_u[2] = {0x07, 0x00};

		fresh();
		rx(bad_feat, sizeof(bad_feat));
		CHECK(last_is(exp_f, 2));
		rx(bad_upd, sizeof(bad_upd));
		CHECK(last_is(exp_u, 2));
		CHECK(cn.upd_calls == 0);
	}

	/* ---- LL_CONNECTION_UPDATE_IND ---- */
	fresh();
	{
		/* WinSize 2, WinOffset 3, Interval 6, Latency 30, Timeout 400, Instant 0x1234 */
		static const uint8_t upd[12] = {0x00, 0x02, 0x03, 0x00, 0x06, 0x00, 0x1E, 0x00,
						0x90, 0x01, 0x34, 0x12};

		rx(upd, 12);
		CHECK(cn.upd_calls == 1);
		CHECK(cn.instant == 0x1234);
		CHECK(cn.win_size == 2 && cn.win_offset == 3);
		CHECK(cn.p.interval == 6 && cn.p.latency == 30 && cn.p.timeout == 400);
		CHECK(tx.n == 0 && cn.end_calls == 0);
		/* instant passed: ll_conn has ended the link itself */
		cn.upd_ret = LL_ST_INSTANT_PASSED;
		rx(upd, 12);
		CHECK(cn.end_calls == 0 && tx.n == 0);
		/* invalid parameters: ll_llcp ends the link with 0x1E */
		cn.upd_ret = LL_ST_INVALID_LL_PARAM;
		rx(upd, 12);
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_INVALID_LL_PARAM);
		CHECK(tx.n == 0);
	}

	/* ---- LL_CHANNEL_MAP_IND ---- */
	fresh();
	{
		static const uint8_t chm[8] = {0x01, 0xFF, 0x00, 0xF0, 0x0F, 0x1F, 0x05, 0x00};

		rx(chm, 8);
		CHECK(cn.chm_calls == 1);
		CHECK(cn.instant == 5);
		CHECK(memcmp(cn.chm, &chm[1], 5) == 0);
		CHECK(tx.n == 0 && cn.end_calls == 0);
		cn.chm_ret = LL_ST_INSTANT_PASSED;
		rx(chm, 8);
		CHECK(cn.end_calls == 0);   /* ll_conn ends the link itself */
		cn.chm_ret = LL_ST_INVALID_LL_PARAM;
		rx(chm, 8);
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_INVALID_LL_PARAM);
		CHECK(tx.n == 0);
	}
	/* fewer than 2 used channels (bits 0..36; 37..39 do not count): 0x1E,
	 * ll_conn not asked */
	{
		static const uint8_t one[8] = {0x01, 0x00, 0x00, 0x00, 0x10, 0xE0, 0x05, 0x00};
		static const uint8_t none[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x05, 0x00};
		static const uint8_t two[8] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x10, 0x05, 0x00};

		fresh();
		rx(one, 8);
		CHECK(cn.chm_calls == 0);
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_INVALID_LL_PARAM);
		fresh();
		rx(none, 8);
		CHECK(cn.chm_calls == 0);
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_INVALID_LL_PARAM);
		fresh();
		rx(two, 8);   /* channels 0 and 36 */
		CHECK(cn.chm_calls == 1 && cn.end_calls == 0);
		CHECK(tx.n == 0);
	}

	/* ---- LL_TERMINATE_IND -> ll_conn_end(reason from PDU) ---- */
	fresh();
	{
		static const uint8_t term[2] = {0x02, 0x13};

		rx(term, 2);
		CHECK(cn.end_calls == 1 && cn.end_reason == 0x13);
		CHECK(tx.n == 0);
	}

	/* ---- HCI Disconnect -> ll_conn_terminate ---- */
	fresh();
	CHECK(ll_llcp_terminate(L, 0x13) == LL_ST_SUCCESS);
	CHECK(cn.term_calls == 1 && cn.term_reason == 0x13);
	cn.active = false;
	CHECK(ll_llcp_terminate(L, 0x13) == LL_ST_UNKNOWN_CONN_ID);
	CHECK(cn.term_calls == 1);

	/* ---- LL_PAUSE_ENC_REQ -> reject 0x1A ---- */
	fresh();
	{
		static const uint8_t pause[1] = {0x0A};
		static const uint8_t exp_ext[3] = {0x11, 0x0A, 0x1A};
		static const uint8_t exp_rej[2] = {0x0D, 0x1A};

		/* central's features unknown: LL_REJECT_IND */
		rx(pause, 1);
		CHECK(last_is(exp_rej, 2));
		/* central supports Extended Reject Indication: LL_REJECT_EXT_IND */
		features(LL_FEAT_LE_ENC | LL_FEAT_EXT_REJ_IND);
		rx(pause, 1);
		CHECK(last_is(exp_ext, 3));
		/* central without it */
		features(LL_FEAT_LE_ENC);
		rx(pause, 1);
		CHECK(last_is(exp_rej, 2));
	}

	/* ---- encryption start, Vol 6 Part C sample data ---- */
	fresh();
	features(0xFF);
	tx.n = 0;
	{
		uint8_t req[23], buf[40];
		static const uint8_t exp_start_req[1] = {0x05};
		uint8_t exp_rsp[13];

		/* nothing to answer yet: no LTK reply without a request */
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_DISALLOWED);
		CHECK(ll_llcp_ltk_neg_reply(L) == LL_ST_DISALLOWED);
		CHECK(tx.n == 0);

		/* ACL flows before the procedure */
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
		CHECK(tx.n == 1 && tx.p[0].kind == LL_TXQ_ACL && tx.p[0].llid == LL_LLID_START);
		CHECK(tx.p[0].len == 27 && memcmp(tx.p[0].d, data2_clear, 27) == 0);
		tx.n = 0;

		/* LL_ENC_REQ: LL_ENC_RSP (SKDs, IVs) plaintext, LTK request */
		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		exp_rsp[0] = 0x04;
		memcpy(&exp_rsp[1], skds, 8);
		memcpy(&exp_rsp[9], ivs, 4);
		CHECK(tx.n == 1);
		CHECK(last_is(exp_rsp, 13));
		CHECK(hci.ltk_req == 1);
		CHECK(memcmp(hci.rand, rnd, 8) == 0 && hci.ediv == ediv);
		CHECK(rand_i == 3);
		CHECK(hci.enc_change == 0);
		/* rx decryption still off while waiting for the LTK */
		CHECK(rxq_crypt == NULL || !rxq_crypt->enc_rx);

		/* data PDUs paused from LL_ENC_REQ on; control PDUs not */
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -EAGAIN);
		CHECK(tx.n == 1);

		/* LTK reply: SK, LL_START_ENC_REQ plaintext, rx decryption on */
		now += T(1000);
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		CHECK(locks == 0);
		CHECK(tx.n == 2);
		CHECK(last_is(exp_start_req, 1));
		CHECK(rxq_crypt != NULL);
		CHECK(rxq_crypt->enc_rx && !rxq_crypt->enc_tx);
		CHECK(memcmp(rxq_crypt->sk, sk_msb, 16) == 0);
		CHECK(memcmp(rxq_crypt->iv, ivm, 4) == 0 && memcmp(&rxq_crypt->iv[4], ivs, 4) == 0);
		CHECK(rxq_crypt->tx_ctr == 0 && rxq_crypt->rx_ctr == 0);
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_DISALLOWED);   /* answered already */
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -EAGAIN);
		CHECK(hci.enc_change == 0);

		/* central's encrypted LL_START_ENC_RSP, decrypted by ll_rxq */
		memcpy(buf, rsp1_air, 5);
		CHECK(ll_crypt_decrypt(rxq_crypt, 0x0F, buf, 5) == 1);
		CHECK(buf[0] == 0x06);
		rx(buf, 1);
		/* our LL_START_ENC_RSP, encrypted: the sample's packet 0 */
		CHECK(tx.n == 3);
		CHECK(tx.p[2].kind == LL_TXQ_CTRL && tx.p[2].llid == LL_LLID_CTRL);
		CHECK(tx.p[2].op == 0x06);
		CHECK(tx.p[2].len == 5 && memcmp(tx.p[2].d, rsp2_air, 5) == 0);
		CHECK(rxq_crypt->enc_tx && rxq_crypt->enc_rx);
		CHECK(hci.enc_change == 1 && hci.status == LL_ST_SUCCESS && hci.enabled);

		/* data flows again, encrypted: the sample's packet 1 */
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
		CHECK(tx.n == 4);
		CHECK(tx.p[3].kind == LL_TXQ_ACL && tx.p[3].llid == LL_LLID_START);
		CHECK(tx.p[3].len == 31 && memcmp(tx.p[3].d, data2_air, 31) == 0);
		CHECK(rxq_crypt->tx_ctr == 2);

		/* a second START_ENC_RSP is ignored; no timeout fires later */
		rx(buf, 1);
		CHECK(tx.n == 4 && hci.enc_change == 1);
		now += T(TIMEOUT_US) * 2;
		ll_llcp_tick(now);
		CHECK(cn.end_calls == 0);

		/* responses on an encrypted link are encrypted too */
		{
			uint8_t vi[6] = {0x0C, 0x0A, 0x02, 0x00, 0x34, 0x12};

			rx(vi, 6);
			CHECK(tx.n == 5 && tx.p[4].len == 10 && tx.p[4].op == 0x0C);
			memcpy(buf, tx.p[4].d, 10);
			CHECK(rxq_crypt->tx_ctr == 3);
		}
		/* ll_conn's ctrl_tx hook (LL_TERMINATE_IND) goes through the same path */
		{
			static const uint8_t ti[2] = {0x02, 0x13};

			CHECK(ll_llcp_ctrl_tx(L, ti, 2) == 0);
			CHECK(tx.n == 6 && tx.p[5].len == 6 && tx.p[5].op == 0x02);
			CHECK(tx.p[5].llid == LL_LLID_CTRL && tx.p[5].kind == LL_TXQ_CTRL);
			CHECK(rxq_crypt->tx_ctr == 4);
		}
		/* backlog full: nothing queued, counter unchanged */
		tx.fail = 1;
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -ENOMEM);
		CHECK(rxq_crypt->tx_ctr == 4 && tx.n == 6);
		/* length limits */
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 0) == -EINVAL);
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, buf, 28) == -EINVAL);
		CHECK(rxq_crypt->tx_ctr == 4 && tx.n == 6);

		/* LL_ENC_REQ on an encrypted link (no pause): rejected */
		rx(req, sizeof(req));
		CHECK(hci.ltk_req == 1);
		CHECK(tx.n == 7 && tx.p[6].op == 0x11 && tx.p[6].len == 3 + LL_MIC_LEN);
		/* new connection: encryption off, plaintext again */
		ll_llcp_reset(L);
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
		CHECK(tx.p[7].len == 27 && memcmp(tx.p[7].d, data2_clear, 27) == 0);
	}

	/* ---- negative reply -> LL_REJECT_EXT_IND(ENC_REQ, 0x06) ---- */
	fresh();
	features(LL_FEAT_LE_ENC | LL_FEAT_EXT_REJ_IND);
	tx.n = 0;
	{
		uint8_t req[23];
		static const uint8_t exp[3] = {0x11, 0x03, 0x06};

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(tx.n == 1 && hci.ltk_req == 1);
		CHECK(ll_llcp_ltk_neg_reply(L) == LL_ST_SUCCESS);
		CHECK(tx.n == 2);
		CHECK(last_is(exp, 3));
		CHECK(hci.enc_change == 0);
		CHECK(rxq_crypt == NULL || !rxq_crypt->enc_rx);
		CHECK(ll_llcp_ltk_neg_reply(L) == LL_ST_DISALLOWED);
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_DISALLOWED);
		/* data resumes, plaintext; timer stopped */
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
		CHECK(tx.p[2].len == 27);
		now += T(TIMEOUT_US) + 1;
		ll_llcp_tick(now);
		CHECK(cn.end_calls == 0);
		/* a new LL_ENC_REQ starts a new procedure */
		sample_rand();
		rx(req, sizeof(req));
		CHECK(hci.ltk_req == 2);
	}
	/* negative reply, central's features unknown: LL_REJECT_IND(0x06) */
	fresh();
	{
		uint8_t req[23];
		static const uint8_t exp[2] = {0x0D, 0x06};

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(ll_llcp_ltk_neg_reply(L) == LL_ST_SUCCESS);
		CHECK(last_is(exp, 2));
	}

	/* ---- 40 s response timeout ---- */
	/* waiting for the host's LTK: from the queued LL_ENC_RSP */
	fresh();
	{
		uint8_t req[23];

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		ll_llcp_tick(now + T(TIMEOUT_US) - 1);
		CHECK(cn.end_calls == 0);
		ll_llcp_tick(now + T(TIMEOUT_US));
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_LMP_TIMEOUT);
		ll_llcp_tick(now + T(TIMEOUT_US) * 2);
		CHECK(cn.end_calls == 1);
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_DISALLOWED);
	}
	/* waiting for LL_START_ENC_RSP: restarted at the queued LL_START_ENC_REQ */
	fresh();
	{
		uint8_t req[23];
		uint32_t t0;

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		now += T(30000000);
		t0 = now;
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		ll_llcp_tick(t0 + T(TIMEOUT_US) - 1);
		CHECK(cn.end_calls == 0);
		ll_llcp_tick(t0 + T(TIMEOUT_US));
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_LMP_TIMEOUT);
	}
	/* across the 32-bit tick wrap */
	fresh();
	{
		uint8_t req[23];

		now = 0xFFFFFF00u;
		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		ll_llcp_tick(now + T(1000000));
		CHECK(cn.end_calls == 0);
		ll_llcp_tick(now + T(TIMEOUT_US));
		CHECK(cn.end_calls == 1);
	}
	/* a reset (disconnect) clears the timer */
	fresh();
	{
		uint8_t req[23];

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		ll_llcp_reset(L);
		ll_llcp_tick(now + T(TIMEOUT_US));
		CHECK(cn.end_calls == 0);
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_DISALLOWED);
	}

	/* ---- encryption start through the helper, then TERMINATE_IND still works ---- */
	fresh();
	start_encryption();
	CHECK(hci.enc_change == 1);
	{
		uint8_t term[2] = {0x02, 0x13};

		rx(term, 2);
		CHECK(cn.end_calls == 1 && cn.end_reason == 0x13);
	}

	/* START_ENC_RSP push fails (backlog full): encryption not reported,
	 * tx stays plaintext, the 40 s timer still ends the link */
	fresh();
	{
		uint8_t req[23], buf[8];

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		memcpy(buf, rsp1_air, 5);
		CHECK(ll_crypt_decrypt(rxq_crypt, 0x0F, buf, 5) == 1);
		tx.fail = 1;
		rx(buf, 1);
		CHECK(hci.enc_change == 0);
		CHECK(!rxq_crypt->enc_tx && rxq_crypt->tx_ctr == 0);
		ll_llcp_tick(now + T(TIMEOUT_US));
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_LMP_TIMEOUT);
	}

	/* LL_ENC_RSP push fails (backlog full): the procedure cannot go on
	 * (the central waits for LL_ENC_RSP), so no LTK request reaches the
	 * host, nothing stays paused, and the link ends now (0x1F) instead of
	 * after the 40 s response timer */
	fresh();
	{
		uint8_t req[23];

		sample_rand();
		build_enc_req(req);
		tx.fail = 1;
		rx(req, sizeof(req));
		CHECK(tx.n == 0);
		CHECK(hci.ltk_req == 0);
		CHECK(rxq_set_calls == 0);
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_UNSPECIFIED);
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_DISALLOWED);
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
		ll_llcp_tick(now + T(TIMEOUT_US));
		CHECK(cn.end_calls == 1);
	}

	/* ---- ll_conn_kick after every successful push (slice 5) ---- */
	fresh();
	features(0xFF);                       /* LLCP response */
	CHECK(kk.calls == 1 && kk.bad == 0);
	CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
	CHECK(kk.calls == 2 && kk.bad == 0);
	CHECK(ll_llcp_ctrl_tx(L, (const uint8_t[]){0x02, 0x13}, 2) == 0);   /* ops.ctrl_tx path */
	CHECK(kk.calls == 3 && kk.bad == 0);
	/* nothing queued: no kick */
	tx.fail = 1;
	CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -ENOMEM);
	CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 0) == -EINVAL);
	CHECK(kk.calls == 3);
	/* encryption start: LL_ENC_RSP, LL_START_ENC_REQ (HCI thread),
	 * LL_START_ENC_RSP each kick; paused ACL does not */
	fresh();
	{
		uint8_t req[23];

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(kk.calls == 1);
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -EAGAIN);
		CHECK(kk.calls == 1);
	}
	fresh();
	start_encryption();
	CHECK(kk.calls == 3 && kk.bad == 0);
	/* encrypted ACL kicks too */
	CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
	CHECK(kk.calls == 4 && kk.bad == 0);
	/* negative LTK reply: the reject kicks */
	fresh();
	{
		uint8_t req[23];

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(ll_llcp_ltk_neg_reply(L) == LL_ST_SUCCESS);
		CHECK(kk.calls == 2 && kk.bad == 0);
	}

	/* ---- ll_llcp_busy: a procedure waits (slice 5 latency hook) ---- */
	fresh();
	CHECK(!ll_llcp_busy(L));
	features(0xFF);
	CHECK(!ll_llcp_busy(L));               /* answered at once: not a waiting procedure */
	{
		uint8_t req[23], buf[8];
		int calls;

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		/* ISR-called: never takes the TX mutex */
		calls = tx_lock_calls;
		CHECK(ll_llcp_busy(L));            /* waiting for the host's LTK */
		CHECK(tx_lock_calls == calls && locks == 0);
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		CHECK(ll_llcp_busy(L));            /* waiting for LL_START_ENC_RSP */
		memcpy(buf, rsp1_air, 5);
		CHECK(ll_crypt_decrypt(rxq_crypt, 0x0F, buf, 5) == 1);
		rx(buf, 1);
		CHECK(hci.enc_change == 1);
		CHECK(!ll_llcp_busy(L));           /* done */
	}
	/* negative reply, timeout and reset end it */
	fresh();
	{
		uint8_t req[23];

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(ll_llcp_busy(L));
		CHECK(ll_llcp_ltk_neg_reply(L) == LL_ST_SUCCESS);
		CHECK(!ll_llcp_busy(L));
		sample_rand();
		rx(req, sizeof(req));
		CHECK(ll_llcp_busy(L));
		ll_llcp_tick(now + T(TIMEOUT_US));
		CHECK(!ll_llcp_busy(L));
		sample_rand();
		rx(req, sizeof(req));
		CHECK(ll_llcp_busy(L));
		ll_llcp_reset(L);
		CHECK(!ll_llcp_busy(L));
	}

	/* ---- ll_llcp_timeout_ticks: arm the 40 s timer only while running ---- */
	fresh();
	CHECK(ll_llcp_timeout_ticks(now) == -1);
	features(0xFF);
	CHECK(ll_llcp_timeout_ticks(now) == -1);
	{
		uint8_t req[23];
		uint32_t t0 = now;

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(ll_llcp_timeout_ticks(t0) == (int32_t)T(TIMEOUT_US));
		CHECK(ll_llcp_timeout_ticks(t0 + 5) == (int32_t)T(TIMEOUT_US) - 5);
		CHECK(ll_llcp_timeout_ticks(t0 + T(TIMEOUT_US)) == 0);
		CHECK(ll_llcp_timeout_ticks(t0 + T(TIMEOUT_US) + 100) == 0);
		/* restarted by LL_START_ENC_REQ */
		now = t0 + T(10000000);
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		CHECK(ll_llcp_timeout_ticks(t0 + T(TIMEOUT_US)) == (int32_t)T(10000000));
		ll_llcp_reset(L);
		CHECK(ll_llcp_timeout_ticks(now) == -1);
	}
	/* across the 32-bit tick wrap */
	fresh();
	{
		uint8_t req[23];

		now = 0xFFFFFF00u;
		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(ll_llcp_timeout_ticks(now + 0x200) == (int32_t)T(TIMEOUT_US) - 0x200);
		CHECK(ll_llcp_ltk_neg_reply(L) == LL_ST_SUCCESS);
		CHECK(ll_llcp_timeout_ticks(now) == -1);
	}
	CHECK(tx_locks == 0);
	CHECK(locks == 0);
}

/* ---------------- multi-link (slice 6a) ---------------- */

static uint8_t pushes_on(uint8_t link)
{
	uint8_t n = 0;

	for (int i = 0; i < tx.n && i < MAX_PUSH; i++) {
		n += tx.p[i].link == link;
	}
	return n;
}

static void enc_req_l(uint8_t link)
{
	uint8_t req[23];

	sample_rand();
	build_enc_req(req);
	rx_l(link, req, sizeof(req));
}

/* Out-of-range link ids: refused, nothing queued, no callback, no link touched. */
static void test_link_bounds(void)
{
	static const uint8_t vi[6] = {0x0C, 0x0A, 0x02, 0x00, 0x34, 0x12};
	uint8_t req[23];

	fresh();
	sample_rand();
	build_enc_req(req);
	ll_llcp_rx(LL_MAX_CONN, req, sizeof(req));
	ll_llcp_rx(LL_MAX_CONN, vi, sizeof(vi));
	CHECK(tx.n == 0 && rxq_set_calls == 0);
	CHECK(ll_llcp_ltk_reply(LL_MAX_CONN, ltk) == LL_ST_DISALLOWED);
	CHECK(ll_llcp_ltk_neg_reply(LL_MAX_CONN) == LL_ST_DISALLOWED);
	CHECK(ll_llcp_terminate(LL_MAX_CONN, 0x13) == LL_ST_UNKNOWN_CONN_ID);
	CHECK(ll_llcp_tx(LL_MAX_CONN, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -EINVAL);
	CHECK(ll_llcp_ctrl_tx(0xFF, vi, 6) == -EINVAL);
	CHECK(!ll_llcp_busy(LL_MAX_CONN));
	ll_llcp_reset(LL_MAX_CONN);
	CHECK(tx.n == 0 && kk.calls == 0);
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		CHECK(hcil[i].ltk_req == 0 && cnl[i].term_calls == 0 && cnl[i].end_calls == 0);
	}
	CHECK(ll_llcp_timeout_ticks(now) == -1);
}

/* Encryption start on link 1 while link 0 sends ACL: link 0 stays
 * plaintext and is never paused; link 1 gets the sample's exact
 * ciphertexts (its counter starts at 0 whatever link 0 queued). Then link 0
 * is encrypted too: again the sample's packets 0 and 1, so the counters
 * are independent, and the two links hold separate contexts. */
static void test_enc_one_link_while_other_sends(void)
{
	const uint8_t a = 0, b = 1;
	uint8_t buf[8];
	struct ll_crypt *cb;

	if (LL_MAX_CONN < 2) {
		return;
	}
	fresh();
	CHECK(ll_llcp_tx(a, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
	enc_req_l(b);
	CHECK(hcil[b].ltk_req == 1 && hcil[a].ltk_req == 0);
	CHECK(ll_llcp_busy(b) && !ll_llcp_busy(a));
	CHECK(rxq_crypt_l[b] != NULL && rxq_crypt_l[a] == NULL);
	/* link 0 is not paused; link 1 is */
	CHECK(ll_llcp_tx(a, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
	CHECK(ll_llcp_tx(b, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -EAGAIN);
	/* the LTK reply of link 0 has nothing to answer */
	CHECK(ll_llcp_ltk_reply(a, ltk) == LL_ST_DISALLOWED);
	CHECK(ll_llcp_ltk_neg_reply(a) == LL_ST_DISALLOWED);
	CHECK(ll_llcp_busy(b));
	CHECK(ll_llcp_ltk_reply(b, ltk) == LL_ST_SUCCESS);
	cb = rxq_crypt_l[b];
	CHECK(cb->enc_rx && !cb->enc_tx);
	CHECK(memcmp(cb->sk, sk_msb, 16) == 0);
	CHECK(ll_llcp_tx(a, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
	memcpy(buf, rsp1_air, 5);
	CHECK(ll_crypt_decrypt(cb, 0x0F, buf, 5) == 1);
	rx_l(b, buf, 1);
	CHECK(hcil[b].enc_change == 1 && hcil[b].enabled && hcil[a].enc_change == 0);
	CHECK(tx.n == 6);
	CHECK(tx.p[5].link == b && tx.p[5].len == 5 && memcmp(tx.p[5].d, rsp2_air, 5) == 0);
	/* link 0's ACL all plaintext */
	for (int i = 0; i < tx.n; i++) {
		if (tx.p[i].link == a) {
			CHECK(tx.p[i].kind == LL_TXQ_ACL && tx.p[i].len == 27);
			CHECK(memcmp(tx.p[i].d, data2_clear, 27) == 0);
		}
	}
	CHECK(pushes_on(a) == 3 && pushes_on(b) == 3);   /* ENC_RSP, START_ENC_REQ, START_ENC_RSP */
	/* link 1 data: sample packet 1 */
	CHECK(ll_llcp_tx(b, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
	CHECK(tx.p[6].link == b && tx.p[6].len == 31 && memcmp(tx.p[6].d, data2_air, 31) == 0);
	CHECK(cb->tx_ctr == 2 && cb->rx_ctr == 1);
	CHECK(kk.bad == 0);

	/* now link 0: its own context and counters from 0 */
	enc_req_l(a);
	CHECK(hcil[a].ltk_req == 1 && hcil[b].ltk_req == 1);
	CHECK(rxq_crypt_l[a] != NULL && rxq_crypt_l[a] != cb);
	CHECK(ll_llcp_busy(a) && !ll_llcp_busy(b));
	/* link 1 keeps sending encrypted while link 0 is paused */
	CHECK(ll_llcp_tx(b, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
	CHECK(tx.p[tx.n - 1].link == b && tx.p[tx.n - 1].len == 31);
	CHECK(cb->tx_ctr == 3);
	CHECK(ll_llcp_tx(a, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -EAGAIN);
	/* the second LTK reply on link 1 has nothing to answer */
	CHECK(ll_llcp_ltk_reply(b, ltk) == LL_ST_DISALLOWED);
	CHECK(ll_llcp_ltk_reply(a, ltk) == LL_ST_SUCCESS);
	memcpy(buf, rsp1_air, 5);
	CHECK(ll_crypt_decrypt(rxq_crypt_l[a], 0x0F, buf, 5) == 1);
	rx_l(a, buf, 1);
	CHECK(hcil[a].enc_change == 1);
	CHECK(tx.p[tx.n - 1].link == a && tx.p[tx.n - 1].len == 5);
	CHECK(memcmp(tx.p[tx.n - 1].d, rsp2_air, 5) == 0);   /* counter 0 on link 0 */
	CHECK(ll_llcp_tx(a, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
	CHECK(tx.p[tx.n - 1].link == a && memcmp(tx.p[tx.n - 1].d, data2_air, 31) == 0);
	CHECK(rxq_crypt_l[a]->tx_ctr == 2 && cb->tx_ctr == 3);
	CHECK(rxq_crypt_l[a]->rx_ctr == 1 && cb->rx_ctr == 1);

	/* link 1 ends (reset): link 0 stays encrypted and keeps its counter */
	ll_llcp_reset(b);
	CHECK(!cb->enc_tx && !cb->enc_rx);
	CHECK(ll_llcp_tx(b, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
	CHECK(tx.p[tx.n - 1].len == 27);   /* plaintext on the reset link */
	CHECK(ll_llcp_tx(a, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
	CHECK(tx.p[tx.n - 1].len == 31 && rxq_crypt_l[a]->tx_ctr == 3);
	CHECK(kk.bad == 0);
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		CHECK(cnl[i].end_calls == 0);
	}
}

/* A MIC failure is detected in ll_rxq (test_rxq) and ends the link via
 * ll_conn; here: a link that went through an encryption start and ended
 * leaves another link's encrypted traffic and procedure untouched, and
 * the ended link starts over cleanly on reuse. */
static void test_link_end_isolated(void)
{
	const uint8_t a = 0, b = (uint8_t)(LL_MAX_CONN - 1);
	uint8_t buf[8];

	if (LL_MAX_CONN < 2) {
		return;
	}
	fresh();
	enc_req_l(a);
	enc_req_l(b);
	CHECK(ll_llcp_ltk_reply(b, ltk) == LL_ST_SUCCESS);
	/* link b fails (e.g. MIC failure: the glue ends it and resets it) */
	ll_llcp_reset(b);
	CHECK(!ll_llcp_busy(b) && ll_llcp_busy(a));
	CHECK(ll_llcp_ltk_reply(a, ltk) == LL_ST_SUCCESS);
	memcpy(buf, rsp1_air, 5);
	CHECK(ll_crypt_decrypt(rxq_crypt_l[a], 0x0F, buf, 5) == 1);
	rx_l(a, buf, 1);
	CHECK(hcil[a].enc_change == 1 && hcil[b].enc_change == 0);
	CHECK(memcmp(tx.p[tx.n - 1].d, rsp2_air, 5) == 0);
	/* link b reused: a fresh procedure with the same sample data */
	enc_req_l(b);
	CHECK(hcil[b].ltk_req == 2);
	CHECK(ll_llcp_ltk_reply(b, ltk) == LL_ST_SUCCESS);
	memcpy(buf, rsp1_air, 5);
	CHECK(ll_crypt_decrypt(rxq_crypt_l[b], 0x0F, buf, 5) == 1);
	rx_l(b, buf, 1);
	CHECK(hcil[b].enc_change == 1);
	CHECK(tx.p[tx.n - 1].link == b && memcmp(tx.p[tx.n - 1].d, rsp2_air, 5) == 0);
}

/* Per-link 40 s timers: ll_llcp_timeout_ticks is the minimum over the
 * running ones; each expiry ends only its link. The links start in reverse
 * order (the last one first), 1 s apart. */
static void test_timeouts_per_link(void)
{
	uint32_t t0;

	fresh();
	t0 = now;
	for (int i = LL_MAX_CONN - 1; i >= 0; i--) {
		enc_req_l((uint8_t)i);
		now += T(1000000);
	}
	/* link N-1 started at t0, link i at t0 + (N-1-i) s */
	CHECK(ll_llcp_timeout_ticks(t0) == (int32_t)T(TIMEOUT_US));
	CHECK(ll_llcp_timeout_ticks(t0 + T(500000)) == (int32_t)T(TIMEOUT_US) - (int32_t)T(500000));
	/* a negative reply on the earliest link drops it from the minimum */
	if (LL_MAX_CONN > 1) {
		CHECK(ll_llcp_ltk_neg_reply(LL_MAX_CONN - 1) == LL_ST_SUCCESS);
		CHECK(ll_llcp_timeout_ticks(t0) == (int32_t)T(TIMEOUT_US) + (int32_t)T(1000000));
		CHECK(!ll_llcp_busy(LL_MAX_CONN - 1));
		/* restart it as the latest one */
		enc_req_l(LL_MAX_CONN - 1);   /* at t0 + N s */
	}
	for (int k = 0; k < LL_MAX_CONN; k++) {
		/* expiry order: links N-2 .. 0, then N-1 (restarted last) */
		uint8_t link = LL_MAX_CONN == 1 ? 0 :
			       (k < LL_MAX_CONN - 1 ? (uint8_t)(LL_MAX_CONN - 2 - k) :
						      (uint8_t)(LL_MAX_CONN - 1));
		uint32_t start = LL_MAX_CONN == 1 ? t0 :
				 (k < LL_MAX_CONN - 1 ? t0 + T(1000000) * (uint32_t)(k + 1) :
							t0 + T(1000000) * LL_MAX_CONN);
		uint32_t due = start + T(TIMEOUT_US);

		CHECK(ll_llcp_timeout_ticks(due - 7) == 7);
		ll_llcp_tick(due - 1);
		for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
			CHECK(cnl[i].end_calls == 0 || cnl[i].end_reason == LL_ST_LMP_TIMEOUT);
		}
		CHECK(cnl[link].end_calls == 0);
		ll_llcp_tick(due);
		CHECK(cnl[link].end_calls == 1 && cnl[link].end_reason == LL_ST_LMP_TIMEOUT);
		CHECK(!ll_llcp_busy(link));
		CHECK(ll_llcp_ltk_reply(link, ltk) == LL_ST_DISALLOWED);
		/* only this link ended now; the others still run (or ended before) */
		for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
			int ended = 0;

			for (int j = 0; j <= k; j++) {
				uint8_t lj = LL_MAX_CONN == 1 ? 0 :
					     (j < LL_MAX_CONN - 1 ? (uint8_t)(LL_MAX_CONN - 2 - j) :
								    (uint8_t)(LL_MAX_CONN - 1));
				ended |= lj == i;
			}
			CHECK(cnl[i].end_calls == ended);
			CHECK(ll_llcp_busy(i) == !ended);
		}
	}
	CHECK(ll_llcp_timeout_ticks(now) == -1);
	ll_llcp_tick(now + T(TIMEOUT_US) * 3);
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		CHECK(cnl[i].end_calls == 1);
	}
}

/* busy, terminate and the instant procedures act on their own link only. */
static void test_routing_per_link(void)
{
	static const uint8_t upd[12] = {0x00, 0x02, 0x03, 0x00, 0x06, 0x00, 0x1E, 0x00,
					0x90, 0x01, 0x34, 0x12};
	static const uint8_t chm[8] = {0x01, 0xFF, 0x00, 0xF0, 0x0F, 0x1F, 0x05, 0x00};
	static const uint8_t term[2] = {0x02, 0x13};

	for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
		fresh();
		enc_req_l(k);
		for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
			CHECK(ll_llcp_busy(i) == (i == k));
		}
		CHECK(ll_llcp_terminate(k, 0x13) == LL_ST_SUCCESS);
		cnl[k].upd_ret = LL_ST_INVALID_LL_PARAM;
		rx_l(k, upd, 12);
		rx_l(k, chm, 8);
		rx_l(k, term, 2);
		for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
			int me = i == k;

			CHECK(cnl[i].term_calls == me && cnl[i].upd_calls == me);
			CHECK(cnl[i].chm_calls == me);
			CHECK(cnl[i].end_calls == 2 * me);   /* invalid update + TERMINATE_IND */
		}
		CHECK(cnl[k].end_reason == 0x13);
		/* a link without a connection */
		cnl[k].active = false;
		CHECK(ll_llcp_terminate(k, 0x13) == LL_ST_UNKNOWN_CONN_ID);
		for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
			if (i != k) {
				CHECK(ll_llcp_terminate(i, 0x13) == LL_ST_SUCCESS);
			}
		}
		/* reset of another link leaves the procedure running */
		for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
			if (i != k) {
				ll_llcp_reset(i);
			}
		}
		CHECK(ll_llcp_busy(k));
		CHECK(pushes_on(k) == 1);
		CHECK(tx.n == 1 && kk.bad == 0);
	}
}

int main(void)
{
	L = 0;
	single_link_suite();
	L = (uint8_t)(LL_MAX_CONN - 1);
	single_link_suite();
	L = 0;
	test_link_bounds();
	test_enc_one_link_while_other_sends();
	test_link_end_isolated();
	test_timeouts_per_link();
	test_routing_per_link();
	CHECK(locks == 0 && tx_locks == 0);
	DONE();
}
