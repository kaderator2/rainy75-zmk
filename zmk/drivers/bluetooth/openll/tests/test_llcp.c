/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_llcp host tests. ll_conn, ll_txq_push, ll_rxq_set_crypt and the radio
 * clock are faked; ll_crypt is real (with the test-only software AES).
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
static int tx_locks;
void ll_plat_tx_lock(void) { tx_locks++; }
void ll_plat_tx_unlock(void) { tx_locks--; }

/* AES never runs with interrupts locked by the caller (Task 10: the IRQ
 * lock of encrypt + push was 360-398 us; the B91 glue locks per block). */
void ll_plat_aes_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
	CHECK(locks == 0);
	aes_ref_encrypt(key, in, out);
}

#define MAX_PUSH 16
static struct {
	int n;
	struct {
		enum ll_txq_kind kind;
		uint8_t llid, len, op;
		uint8_t d[40];
	} p[MAX_PUSH];
	int fail;   /* next pushes return -ENOMEM */
} tx;

int ll_txq_push(enum ll_txq_kind kind, uint8_t llid, const uint8_t *payload, uint8_t len,
		uint8_t ctrl_opcode)
{
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
		tx.p[tx.n].kind = kind;
		tx.p[tx.n].llid = llid;
		tx.p[tx.n].len = len;
		tx.p[tx.n].op = ctrl_opcode;
		memcpy(tx.p[tx.n].d, payload, len);
	}
	tx.n++;
	return 0;
}

static struct ll_crypt *rxq_crypt;
static int rxq_set_calls;
void ll_rxq_set_crypt(struct ll_crypt *c)
{
	rxq_crypt = c;
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
} cn;

int ll_conn_update_at(uint16_t instant, uint8_t win_size, uint16_t win_offset,
		      const struct ll_conn_params *p)
{
	cn.upd_calls++;
	cn.instant = instant;
	cn.win_size = win_size;
	cn.win_offset = win_offset;
	cn.p = *p;
	return cn.upd_ret;
}
int ll_conn_chmap_at(uint16_t instant, const uint8_t chm[5])
{
	cn.chm_calls++;
	cn.instant = instant;
	memcpy(cn.chm, chm, 5);
	return cn.chm_ret;
}
void ll_conn_terminate(uint8_t reason)
{
	CHECK(locks == 0);
	cn.term_calls++;
	cn.term_reason = reason;
}
void ll_conn_end(uint8_t reason)
{
	CHECK(locks == 0);
	cn.end_calls++;
	cn.end_reason = reason;
}
bool ll_conn_active(void) { return cn.active; }

/* ---------------- HCI ops ---------------- */

static struct {
	int ltk_req;
	uint8_t rand[8];
	uint16_t ediv;
	int enc_change;
	uint8_t status;
	bool enabled;
} hci;

static void on_ltk_req(const uint8_t r[8], uint16_t e)
{
	CHECK(locks == 0);
	hci.ltk_req++;
	memcpy(hci.rand, r, 8);
	hci.ediv = e;
}

static void on_enc_change(uint8_t status, bool enabled)
{
	CHECK(locks == 0);
	hci.enc_change++;
	hci.status = status;
	hci.enabled = enabled;
}

static const struct ll_llcp_ops ops = {.ltk_req = on_ltk_req, .enc_change = on_enc_change};

/* ---------------- helpers ---------------- */

static void fresh(void)
{
	memset(&tx, 0, sizeof(tx));
	memset(&cn, 0, sizeof(cn));
	memset(&hci, 0, sizeof(hci));
	rxq_crypt = NULL;
	rxq_set_calls = 0;
	rand_n = rand_i = 0;
	now = 1000;
	cn.active = true;
	ll_llcp_init(&ops);
	ll_llcp_reset();
}

static void rx(const uint8_t *pdu, uint8_t len)
{
	ll_llcp_rx(pdu, len);
	CHECK(locks == 0);
	CHECK(tx_locks == 0);
}

/* last push is exactly {kind CTRL, LLID 3, opcode, bytes} */
static int last_is(const uint8_t *exp, uint8_t len)
{
	if (tx.n == 0) {
		return 0;
	}
	return tx.p[tx.n - 1].kind == LL_TXQ_CTRL && tx.p[tx.n - 1].llid == LL_LLID_CTRL &&
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
	CHECK(ll_llcp_ltk_reply(ltk) == LL_ST_SUCCESS);
	CHECK(rxq_crypt != NULL);
	memcpy(buf, rsp1_air, 5);
	CHECK(ll_crypt_decrypt(rxq_crypt, 0x0F, buf, 5) == 1);  /* as ll_rxq does */
	n0 = tx.n;
	rx(buf, 1);
	CHECK(tx.n == n0 + 1);
}

int main(void)
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
		ll_llcp_reset();   /* new connection */
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
	CHECK(ll_llcp_terminate(0x13) == LL_ST_SUCCESS);
	CHECK(cn.term_calls == 1 && cn.term_reason == 0x13);
	cn.active = false;
	CHECK(ll_llcp_terminate(0x13) == LL_ST_UNKNOWN_CONN_ID);
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
		CHECK(ll_llcp_ltk_reply(ltk) == LL_ST_DISALLOWED);
		CHECK(ll_llcp_ltk_neg_reply() == LL_ST_DISALLOWED);
		CHECK(tx.n == 0);

		/* ACL flows before the procedure */
		CHECK(ll_llcp_tx(LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
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
		CHECK(ll_llcp_tx(LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -EAGAIN);
		CHECK(tx.n == 1);

		/* LTK reply: SK, LL_START_ENC_REQ plaintext, rx decryption on */
		now += T(1000);
		CHECK(ll_llcp_ltk_reply(ltk) == LL_ST_SUCCESS);
		CHECK(locks == 0);
		CHECK(tx.n == 2);
		CHECK(last_is(exp_start_req, 1));
		CHECK(rxq_crypt != NULL);
		CHECK(rxq_crypt->enc_rx && !rxq_crypt->enc_tx);
		CHECK(memcmp(rxq_crypt->sk, sk_msb, 16) == 0);
		CHECK(memcmp(rxq_crypt->iv, ivm, 4) == 0 && memcmp(&rxq_crypt->iv[4], ivs, 4) == 0);
		CHECK(rxq_crypt->tx_ctr == 0 && rxq_crypt->rx_ctr == 0);
		CHECK(ll_llcp_ltk_reply(ltk) == LL_ST_DISALLOWED);   /* answered already */
		CHECK(ll_llcp_tx(LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -EAGAIN);
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
		CHECK(ll_llcp_tx(LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
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

			CHECK(ll_llcp_ctrl_tx(ti, 2) == 0);
			CHECK(tx.n == 6 && tx.p[5].len == 6 && tx.p[5].op == 0x02);
			CHECK(tx.p[5].llid == LL_LLID_CTRL && tx.p[5].kind == LL_TXQ_CTRL);
			CHECK(rxq_crypt->tx_ctr == 4);
		}
		/* backlog full: nothing queued, counter unchanged */
		tx.fail = 1;
		CHECK(ll_llcp_tx(LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -ENOMEM);
		CHECK(rxq_crypt->tx_ctr == 4 && tx.n == 6);
		/* length limits */
		CHECK(ll_llcp_tx(LL_TXQ_ACL, LL_LLID_START, data2_clear, 0) == -EINVAL);
		CHECK(ll_llcp_tx(LL_TXQ_ACL, LL_LLID_START, buf, 28) == -EINVAL);
		CHECK(rxq_crypt->tx_ctr == 4 && tx.n == 6);

		/* LL_ENC_REQ on an encrypted link (no pause): rejected */
		rx(req, sizeof(req));
		CHECK(hci.ltk_req == 1);
		CHECK(tx.n == 7 && tx.p[6].op == 0x11 && tx.p[6].len == 3 + LL_MIC_LEN);
		/* new connection: encryption off, plaintext again */
		ll_llcp_reset();
		CHECK(ll_llcp_tx(LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
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
		CHECK(ll_llcp_ltk_neg_reply() == LL_ST_SUCCESS);
		CHECK(tx.n == 2);
		CHECK(last_is(exp, 3));
		CHECK(hci.enc_change == 0);
		CHECK(rxq_crypt == NULL || !rxq_crypt->enc_rx);
		CHECK(ll_llcp_ltk_neg_reply() == LL_ST_DISALLOWED);
		CHECK(ll_llcp_ltk_reply(ltk) == LL_ST_DISALLOWED);
		/* data resumes, plaintext; timer stopped */
		CHECK(ll_llcp_tx(LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
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
		CHECK(ll_llcp_ltk_neg_reply() == LL_ST_SUCCESS);
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
		CHECK(ll_llcp_ltk_reply(ltk) == LL_ST_DISALLOWED);
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
		CHECK(ll_llcp_ltk_reply(ltk) == LL_ST_SUCCESS);
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
		ll_llcp_reset();
		ll_llcp_tick(now + T(TIMEOUT_US));
		CHECK(cn.end_calls == 0);
		CHECK(ll_llcp_ltk_reply(ltk) == LL_ST_DISALLOWED);
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
		CHECK(ll_llcp_ltk_reply(ltk) == LL_ST_SUCCESS);
		memcpy(buf, rsp1_air, 5);
		CHECK(ll_crypt_decrypt(rxq_crypt, 0x0F, buf, 5) == 1);
		tx.fail = 1;
		rx(buf, 1);
		CHECK(hci.enc_change == 0);
		CHECK(!rxq_crypt->enc_tx && rxq_crypt->tx_ctr == 0);
		ll_llcp_tick(now + T(TIMEOUT_US));
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_LMP_TIMEOUT);
	}

	CHECK(locks == 0);
	DONE();
}
