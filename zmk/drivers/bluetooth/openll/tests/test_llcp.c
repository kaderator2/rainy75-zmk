/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_llcp host tests. ll_conn, ll_txq_push, ll_rxq_set_crypt and the radio
 * clock are faked; ll_crypt is real (with the test-only software AES).
 * The link-aware ll_conn fakes below replace ll_conn.c (slice 6a Task 4:
 * ll_llcp calls the per-link ll_conn API directly).
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
/* slice 6d: authenticatedPayloadTO in ticks (v <= 26843 fits 32 bits) */
#define APTO_T(v)   ((uint32_t)(v) * 10000u * LL_TICKS_PER_US)

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
		bool last;
		uint8_t d[255];
	} p[MAX_PUSH];
	int fail;   /* next pushes return -ENOMEM */
	int ok_first;   /* ... after this many more successful pushes */
	int room;       /* ll_txq_fits: PDUs that fit (0: no limit) */
	int fits_calls;
	uint8_t fits_n, fits_len, fits_last;
} tx;

bool ll_txq_fits(uint8_t link, uint8_t n, uint8_t len, uint8_t last_len)
{
	CHECK(link < LL_MAX_CONN);
	CHECK(tx_locks > 0);   /* the answer holds only with the producer serialized */
	tx.fits_calls++;
	tx.fits_n = n;
	tx.fits_len = len;
	tx.fits_last = last_len;
	return tx.room == 0 || n <= tx.room;
}

/* effective times handed to ll_conn (arbiter span, guard floor) */
static struct {
	int calls;
	uint16_t rx_time, tx_time;
} dtl[LL_MAX_CONN];

void ll_conn_set_dle_times(uint8_t link, uint16_t max_rx_time, uint16_t max_tx_time)
{
	CHECK(link < LL_MAX_CONN);
	dtl[link].calls++;
	dtl[link].rx_time = max_rx_time;
	dtl[link].tx_time = max_tx_time;
}

int ll_txq_push(uint8_t link, enum ll_txq_kind kind, uint8_t llid, const uint8_t *payload,
		uint8_t len, uint8_t ctrl_opcode, bool last)
{
	CHECK(link < LL_MAX_CONN);
	/* pushed under both: the IRQ lock for the queue, the TX lock for the
	 * counter order */
	CHECK(locks > 0);
	CHECK(tx_locks > 0);
	if (tx.ok_first) {
		tx.ok_first--;
	} else if (tx.fail) {
		tx.fail--;
		return -ENOMEM;
	}
	if (tx.n < MAX_PUSH) {
		tx.p[tx.n].link = link;
		tx.p[tx.n].kind = kind;
		tx.p[tx.n].llid = llid;
		tx.p[tx.n].len = len;
		tx.p[tx.n].op = ctrl_opcode;
		tx.p[tx.n].last = last;
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

/* link-aware ll_conn fakes */

int ll_conn_update_at(uint8_t link, uint16_t instant, uint8_t win_size, uint16_t win_offset,
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
int ll_conn_chmap_at(uint8_t link, uint16_t instant, const uint8_t chm[5])
{
	CHECK(link < LL_MAX_CONN);
	cnl[link].chm_calls++;
	cnl[link].instant = instant;
	memcpy(cnl[link].chm, chm, 5);
	return cnl[link].chm_ret;
}
void ll_conn_terminate(uint8_t link, uint8_t reason)
{
	CHECK(locks == 0);
	CHECK(link < LL_MAX_CONN);
	cnl[link].term_calls++;
	cnl[link].term_reason = reason;
}
void ll_conn_end(uint8_t link, uint8_t reason)
{
	CHECK(locks == 0);
	CHECK(link < LL_MAX_CONN);
	cnl[link].end_calls++;
	cnl[link].end_reason = reason;
}
/* slice 6d Task 2: instants not yet reached (LL_CONN_PENDING_*) */
static uint8_t pend_inst[LL_MAX_CONN];
uint8_t ll_conn_pending_instants(uint8_t link)
{
	CHECK(link < LL_MAX_CONN);
	return pend_inst[link];
}
bool ll_conn_active(uint8_t link)
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
void ll_conn_kick(uint8_t link)
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

/* LE Data Length Change (slice 6b Task 3): per link, the last effective values */
static struct {
	int calls;
	struct ll_llcp_dle eff;
} dlcl[LL_MAX_CONN];
#define dlc (dlcl[L])

static void on_data_len_change(uint8_t link, const struct ll_llcp_dle *eff)
{
	CHECK(locks == 0);
	CHECK(tx_locks == 0);
	CHECK(link < LL_MAX_CONN);
	dlcl[link].calls++;
	dlcl[link].eff = *eff;
}

/* Authenticated Payload Timeout Expired (slice 6d), per link */
static int aptol[LL_MAX_CONN];
#define apto_ev (aptol[L])

static void on_apto_expired(uint8_t link)
{
	CHECK(locks == 0);
	CHECK(tx_locks == 0);
	CHECK(link < LL_MAX_CONN);
	aptol[link]++;
}

/* LE Remote Connection Parameter Request (slice 6d Task 2), per link: the
 * last request; `masked` makes the op report the event as not sent */
static struct {
	int calls;
	struct ll_llcp_cpr req;
	bool masked;
} cprl[LL_MAX_CONN];
#define cpr_ev (cprl[L])

static bool on_conn_param_req(uint8_t link, const struct ll_llcp_cpr *req)
{
	CHECK(locks == 0);
	CHECK(tx_locks == 0);
	CHECK(link < LL_MAX_CONN);
	cprl[link].calls++;
	cprl[link].req = *req;
	return !cprl[link].masked;
}

static const struct ll_llcp_ops ops = {.ltk_req = on_ltk_req, .enc_change = on_enc_change,
				       .data_len_change = on_data_len_change,
				       .apto_expired = on_apto_expired,
				       .conn_param_req = on_conn_param_req};

/* ---------------- helpers ---------------- */

static void fresh(void)
{
	memset(&tx, 0, sizeof(tx));
	memset(cnl, 0, sizeof(cnl));
	memset(hcil, 0, sizeof(hcil));
	memset(dlcl, 0, sizeof(dlcl));
	memset(aptol, 0, sizeof(aptol));
	memset(cprl, 0, sizeof(cprl));
	memset(pend_inst, 0, sizeof(pend_inst));
	memset(dtl, 0, sizeof(dtl));
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
		/* ours: LE Encryption (bit 0) + Connection Parameters Request
		 * procedure (bit 1, slice 6d Task 2) + Extended Reject
		 * Indication (bit 2) + LE Ping (bit 4, slice 6d) + LE Data
		 * Packet Length Extension (bit 5); byte 1 is ours: CSA#2 (bit 14) */
		static const uint8_t exp[9] = {0x09, 0x37, 0x40, 0, 0, 0, 0, 0, 0};

		CHECK(LL_FEATURES_LOW == 0x37);
		CHECK(tx.n == 1);
		CHECK(last_is(exp, 9));
	}
	features(0x01);
	{
		static const uint8_t exp[9] = {0x09, 0x01, 0x40, 0, 0, 0, 0, 0, 0};

		CHECK(last_is(exp, 9));
	}
	features(0x00);
	{
		static const uint8_t exp[9] = {0x09, 0, 0x40, 0, 0, 0, 0, 0, 0};

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
		/* LL_LENGTH_REQ (0x14), LL_PHY_REQ (0x16) and LL_PHY_UPDATE_IND
		 * (0x18) are answered since slice 6b Task 3 (dle_phy_suite) */
		/* LL_PING_REQ (0x12) is answered since slice 6d (ping_suite),
		 * LL_CONNECTION_PARAM_REQ (0x0F) since slice 6d Task 2
		 * (cpr_suite) */
		static const uint8_t ops_unk[] = {0x19, 0x1A, 0x0E, 0x04, 0x05,
						  0x1F, 0x20, 0xFF};
		static const uint8_t lens[] = {3, 1, 9, 13, 1, 5, 1, 1};

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

		/* a second START_ENC_RSP is ignored; no timeout fires later
		 * (the central's packets keep the slice 6d payload timer from
		 * expiring) */
		rx(buf, 1);
		CHECK(tx.n == 4 && hci.enc_change == 1);
		for (int k = 0; k < 4; k++) {
			now += T(TIMEOUT_US) / 2;
			ll_llcp_rx_auth(L);
			ll_llcp_tick(now);
		}
		CHECK(cn.end_calls == 0 && tx.n == 4);

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

	/* START_ENC_RSP push fails with the owed queue full too (a backlog
	 * merely full owes it, push_retry_suite): encryption not reported, tx
	 * stays plaintext, the 40 s timer still ends the link */
	fresh();
	{
		uint8_t req[23], buf[8];
		static const uint8_t dummy[2] = {0x07, 0x00};

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		memcpy(buf, rsp1_air, 5);
		CHECK(ll_crypt_decrypt(rxq_crypt, 0x0F, buf, 5) == 1);
		tx.fail = 1000;
		for (int i = 0; i < LL_LLCP_OWE_N; i++) {
			CHECK(ll_llcp_ctrl_tx(L, dummy, 2) == 0);
		}
		CHECK(ll_llcp_ctrl_tx(L, dummy, 2) == -ENOMEM);
		rx(buf, 1);
		CHECK(hci.enc_change == 0);
		CHECK(!rxq_crypt->enc_tx && rxq_crypt->tx_ctr == 0);
		/* the encryption start's 40 s timer ends it (0x22) */
		ll_llcp_tick(now + T(TIMEOUT_US));
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_LMP_TIMEOUT);
		tx.fail = 0;
	}

	/* LL_ENC_RSP push fails with the owed queue full: the procedure cannot
	 * go on (the central waits for LL_ENC_RSP), so no LTK request reaches
	 * the host, nothing stays paused, and the link ends now (0x1F) instead
	 * of after the 40 s response timer (a merely full backlog owes it:
	 * push_retry_suite) */
	fresh();
	{
		uint8_t req[23];
		static const uint8_t dummy[2] = {0x07, 0x00};

		sample_rand();
		build_enc_req(req);
		tx.fail = 1000;
		for (int i = 0; i < LL_LLCP_OWE_N; i++) {
			CHECK(ll_llcp_ctrl_tx(L, dummy, 2) == 0);
		}
		rx(req, sizeof(req));
		CHECK(tx.n == 0);
		CHECK(hci.ltk_req == 0);
		CHECK(rxq_set_calls == 0);
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_UNSPECIFIED);
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_DISALLOWED);
		tx.fail = 0;
		ll_llcp_retry(L);
		CHECK(tx.n == LL_LLCP_OWE_N);
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

/* ---------------- LENGTH and PHY procedures (slice 6b Task 3) ----------------
 *
 * Data Length Update (Vol 6 Part B 2.4.2.21, 4.5.10, 5.1.9): Octets are the
 * Payload length without the MIC; times are 1M packet times incl. the MIC
 * (LL_DLE_TIME_1M). Our connMaxRx* is the supported maximum (SUP / SUPT),
 * connMaxTx* starts at 27 / 328 and follows ll_llcp_set_data_len(). The
 * suite runs for the device value (SUP 27 in this task, nothing can grow)
 * and with -DLL_DLE_SUPP_OCTETS=251 (test_llcp_dle_n<N>), where every rule
 * is visible.
 * PHY (5.1.10): 1M only; LL_PHY_REQ -> LL_PHY_RSP(1M, 1M), any
 * LL_PHY_UPDATE_IND keeps 1M (an unsupported / reserved / multi-bit PHY:
 * "shall not change the PHY in that direction"). */

#define SUP   ((uint16_t)LL_DLE_SUPP_OCTETS)
#define SUPT  ((uint16_t)LL_DLE_TIME_1M(LL_DLE_SUPP_OCTETS))

static void length_pdu(uint8_t p[9], uint8_t op, uint16_t rxo, uint16_t rxt, uint16_t txo,
		       uint16_t txt)
{
	p[0] = op;
	ll_put_le16(&p[1], rxo);
	ll_put_le16(&p[3], rxt);
	ll_put_le16(&p[5], txo);
	ll_put_le16(&p[7], txt);
}

static int dle_is(const struct ll_llcp_dle *d, uint16_t txo, uint16_t txt, uint16_t rxo,
		  uint16_t rxt)
{
	return d->max_tx_octets == txo && d->max_tx_time == txt && d->max_rx_octets == rxo &&
	       d->max_rx_time == rxt;
}

static int eff_is(uint8_t link, uint16_t txo, uint16_t txt, uint16_t rxo, uint16_t rxt)
{
	struct ll_llcp_dle d;

	memset(&d, 0xEE, sizeof(d));
	ll_llcp_get_dle(link, &d);
	return dle_is(&d, txo, txt, rxo, rxt);
}

/* last push is our LL_LENGTH_RSP / LL_LENGTH_REQ with these values */
static int last_length(uint8_t op, uint16_t rxo, uint16_t rxt, uint16_t txo, uint16_t txt)
{
	uint8_t exp[9];

	length_pdu(exp, op, rxo, rxt, txo, txt);
	return last_is(exp, 9);
}

static uint16_t min16(uint16_t a, uint16_t b) { return a < b ? a : b; }

static void dle_responder(void)
{
	uint8_t req[9];
	int n0;

	CHECK(LL_DLE_TIME_1M(27) == 328 && LL_DLE_TIME_1M(251) == 2120);
	CHECK(LL_DLE_MAX_OCTETS == 251 && LL_DLE_MAX_TIME_1M == 2120);
	CHECK(SUP >= 27 && SUP <= 251);

	/* new connection: 27 / 328 everywhere */
	fresh();
	CHECK(eff_is(L, 27, 328, 27, 328));

	/* central offers 251 / 2120 both ways: we answer our connMax values
	 * (Tx 27 / 328 until the host asks, Rx our supported maximum) */
	length_pdu(req, 0x14, 251, 2120, 251, 2120);
	rx(req, 9);
	CHECK(tx.n == 1 && last_length(0x15, SUP, SUPT, 27, 328));
	CHECK(tx.p[0].kind == LL_TXQ_CTRL);
	/* effective Rx = min(our Rx, central Tx), Tx unchanged */
	CHECK(eff_is(L, 27, 328, SUP, SUPT));
	CHECK(dlc.calls == (SUP > 27 ? 1 : 0));
	if (SUP > 27) {
		CHECK(dle_is(&dlc.eff, 27, 328, SUP, SUPT));
	}
	/* the responder procedure ends with our response: nothing pending */
	CHECK(!ll_llcp_busy(L));
	CHECK(ll_llcp_timeout_ticks(now) == -1);
	CHECK(kk.bad == 0 && kk.calls == 1);

	/* the same request again: answered, no change, no event */
	n0 = dlc.calls;
	rx(req, 9);
	CHECK(tx.n == 2 && last_length(0x15, SUP, SUPT, 27, 328));
	CHECK(dlc.calls == n0);

	/* the central lowers its Tx to 100 / 912: eff Rx follows */
	length_pdu(req, 0x14, 251, 2120, 100, 912);
	rx(req, 9);
	CHECK(eff_is(L, 27, 328, min16(SUP, 100), min16(SUPT, 912)));
	CHECK(dlc.calls == n0 + (SUP > 27 ? 1 : 0));

	/* values below the minimum are taken as the minimum (27 / 328) */
	n0 = dlc.calls;
	length_pdu(req, 0x14, 10, 100, 5, 50);
	rx(req, 9);
	CHECK(last_length(0x15, SUP, SUPT, 27, 328));
	CHECK(eff_is(L, 27, 328, 27, 328));
	CHECK(dlc.calls == n0 + (SUP > 27 ? 1 : 0));
	if (SUP > 27) {
		CHECK(dle_is(&dlc.eff, 27, 328, 27, 328));
	}
	/* values above the range are capped (Octets 251, Time 17040): the
	 * effective values are ours */
	length_pdu(req, 0x14, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF);
	rx(req, 9);
	CHECK(eff_is(L, 27, 328, SUP, SUPT));

	/* a LENGTH_RSP we did not ask for is ignored */
	n0 = tx.n;
	length_pdu(req, 0x15, 27, 328, 27, 328);
	rx(req, 9);
	CHECK(tx.n == n0 && eff_is(L, 27, 328, SUP, SUPT));
	/* wrong length -> LL_UNKNOWN_RSP(0x14) */
	{
		static const uint8_t exp[2] = {0x07, 0x14};

		length_pdu(req, 0x14, 251, 2120, 251, 2120);
		rx(req, 8);
		CHECK(last_is(exp, 2));
	}
	CHECK(cn.end_calls == 0);

	/* a new connection on the link starts over */
	ll_llcp_reset(L);
	CHECK(eff_is(L, 27, 328, 27, 328));
}

static void dle_initiator(void)
{
	uint8_t pdu[9];
	uint32_t t0;

	/* host asks for 251 / 2120: connMaxTx = min(asked, supported). With
	 * SUP 27 nothing changes: no PDU, success. */
	fresh();
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	CHECK(locks == 0 && tx_locks == 0);
	if (SUP == 27) {
		CHECK(tx.n == 0 && dlc.calls == 0 && !ll_llcp_busy(L));
		CHECK(eff_is(L, 27, 328, 27, 328));
		/* below the minimum is clamped up, too */
		CHECK(ll_llcp_set_data_len(L, 0, 0) == LL_ST_SUCCESS);
		CHECK(tx.n == 0);
		return;
	}
	/* LL_LENGTH_REQ with our new connMax values */
	CHECK(tx.n == 1 && last_length(0x14, SUP, SUPT, SUP, SUPT));
	CHECK(kk.calls == 1 && kk.bad == 0);
	/* the central still only receives 27: no change yet */
	CHECK(eff_is(L, 27, 328, 27, 328) && dlc.calls == 0);
	CHECK(ll_llcp_busy(L));
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)T(TIMEOUT_US));
	/* the central answers 251 / 2120: all effective values grow, one event */
	length_pdu(pdu, 0x15, 251, 2120, 251, 2120);
	rx(pdu, 9);
	CHECK(tx.n == 1);
	CHECK(eff_is(L, SUP, SUPT, SUP, SUPT));
	CHECK(dlc.calls == 1 && dle_is(&dlc.eff, SUP, SUPT, SUP, SUPT));
	CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
	/* slice 6b Task 4: ll_conn follows the effective times (guard floor,
	 * arbiter span) */
	CHECK(dtl[L].calls == 1 && dtl[L].rx_time == SUPT && dtl[L].tx_time == SUPT);
	/* a second RSP: not a response to anything, ignored */
	length_pdu(pdu, 0x15, 27, 328, 27, 328);
	rx(pdu, 9);
	CHECK(eff_is(L, SUP, SUPT, SUP, SUPT) && dlc.calls == 1);

	/* the host lowers Tx to 100 / 912: eff Tx drops at once (we may send
	 * shorter PDUs any time), then the procedure tells the central */
	CHECK(ll_llcp_set_data_len(L, 100, 912) == LL_ST_SUCCESS);
	CHECK(last_length(0x14, SUP, SUPT, 100, 912));
	CHECK(eff_is(L, 100, 912, SUP, SUPT) && dlc.calls == 2);
	length_pdu(pdu, 0x15, 251, 2120, 251, 2120);
	rx(pdu, 9);
	CHECK(dlc.calls == 2);   /* nothing changed by the response */
	/* the same values again: the central knows them, no procedure */
	CHECK(ll_llcp_set_data_len(L, 100, 912) == LL_ST_SUCCESS);
	CHECK(tx.n == 2 && !ll_llcp_busy(L));
	/* TxTime only: Octets and Time need not match (4.5.10) */
	CHECK(ll_llcp_set_data_len(L, 100, 2120) == LL_ST_SUCCESS);
	CHECK(last_length(0x14, SUP, SUPT, 100, SUPT));
	CHECK(eff_is(L, 100, SUPT, SUP, SUPT) && dlc.calls == 3);

	/* crossing: ours queued, the central's LL_LENGTH_REQ arrives first.
	 * We answer it as usual, ours stays pending until its LL_LENGTH_RSP. */
	fresh();
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	CHECK(tx.n == 1 && last_length(0x14, SUP, SUPT, SUP, SUPT));
	length_pdu(pdu, 0x14, 200, 1712, 200, 1712);
	rx(pdu, 9);
	CHECK(tx.n == 2 && last_length(0x15, SUP, SUPT, SUP, SUPT));
	CHECK(eff_is(L, min16(SUP, 200), min16(SUPT, 1712), min16(SUP, 200), min16(SUPT, 1712)));
	CHECK(dlc.calls == 1);
	CHECK(ll_llcp_busy(L));
	length_pdu(pdu, 0x15, 200, 1712, 200, 1712);
	rx(pdu, 9);
	CHECK(!ll_llcp_busy(L) && dlc.calls == 1 && tx.n == 2);

	/* the central does not know LL_LENGTH_REQ: LL_UNKNOWN_RSP ends the
	 * procedure without a change; later requests are refused with
	 * Unsupported Remote Feature */
	fresh();
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	{
		static const uint8_t unk[2] = {0x07, 0x14};

		rx(unk, 2);
	}
	CHECK(!ll_llcp_busy(L) && dlc.calls == 0 && eff_is(L, 27, 328, 27, 328));
	CHECK(ll_llcp_set_data_len(L, 100, 912) == LL_ST_UNSUPP_REMOTE);
	CHECK(tx.n == 1);
	/* an LL_UNKNOWN_RSP for another opcode leaves ours running */
	fresh();
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	{
		static const uint8_t unk[2] = {0x07, 0x16};

		rx(unk, 2);
	}
	CHECK(ll_llcp_busy(L));
	/* LL_REJECT_EXT_IND(LL_LENGTH_REQ) ends it, too */
	{
		static const uint8_t rej[3] = {0x11, 0x14, 0x1A};

		rx(rej, 3);
	}
	CHECK(!ll_llcp_busy(L) && dlc.calls == 0 && tx.n == 1);

	/* the feature exchange said: no DLE on the central */
	fresh();
	features(0x01);
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_UNSUPP_REMOTE);
	CHECK(tx.n == 1);   /* only the LL_FEATURE_RSP */
	fresh();
	features(0x21);
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	CHECK(tx.n == 2 && last_length(0x14, SUP, SUPT, SUP, SUPT));

	/* 40 s procedure response timeout: the link ends with 0x22 */
	fresh();
	t0 = now;
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	CHECK(ll_llcp_timeout_ticks(t0 + 5) == (int32_t)T(TIMEOUT_US) - 5);
	ll_llcp_tick(t0 + T(TIMEOUT_US) - 1);
	CHECK(cn.end_calls == 0);
	ll_llcp_tick(t0 + T(TIMEOUT_US));
	CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_LMP_TIMEOUT);
	CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);

	/* backlog and owed queue full: Memory Capacity Exceeded, nothing
	 * changed (a merely full backlog owes the request:
	 * push_retry_dle_phy) */
	fresh();
	{
		static const uint8_t dummy[2] = {0x07, 0x00};

		tx.fail = 1000;
		for (int i = 0; i < LL_LLCP_OWE_N; i++) {
			CHECK(ll_llcp_ctrl_tx(L, dummy, 2) == 0);
		}
		CHECK(ll_llcp_set_data_len(L, 251, 2120) == 0x07);
		tx.fail = 0;
		ll_llcp_retry(L);
		CHECK(tx.n == LL_LLCP_OWE_N && !ll_llcp_busy(L));
		tx.n = 0;
		kk.calls = 0;
		kk.n_at_kick = 0;
	}
	CHECK(tx.n == 0 && !ll_llcp_busy(L) && kk.calls == 0);
	CHECK(eff_is(L, 27, 328, 27, 328));
	/* our connMaxTx is still 27 / 328: the next LL_LENGTH_RSP says so */
	length_pdu(pdu, 0x14, 251, 2120, 251, 2120);
	rx(pdu, 9);
	CHECK(tx.n == 1 && last_length(0x15, SUP, SUPT, 27, 328));
	CHECK(eff_is(L, 27, 328, SUP, SUPT));
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	CHECK(tx.n == 2 && last_length(0x14, SUP, SUPT, SUP, SUPT));

	/* connMaxRx above 27 alone is news for the central (4.5.10: "should
	 * initiate" when a value is not 27 / 328): the host's 27 / 328 still
	 * sends LL_LENGTH_REQ ... */
	fresh();
	CHECK(ll_llcp_set_data_len(L, 27, 328) == LL_ST_SUCCESS);
	CHECK(tx.n == 1 && last_length(0x14, SUP, SUPT, 27, 328));
	/* ... unless our LL_LENGTH_RSP already carried it */
	fresh();
	length_pdu(pdu, 0x14, 27, 328, 27, 328);
	rx(pdu, 9);
	CHECK(tx.n == 1);
	CHECK(ll_llcp_set_data_len(L, 27, 328) == LL_ST_SUCCESS);
	CHECK(tx.n == 1 && !ll_llcp_busy(L));

	/* during the encryption start (data paused, Vol 6 Part B 5.1.3.1)
	 * the request waits and leaves, encrypted, right after our
	 * LL_START_ENC_RSP */
	fresh();
	{
		uint8_t req[23], buf[8];

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
		CHECK(tx.n == 1 && tx.p[0].op == 0x04);   /* only LL_ENC_RSP */
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		CHECK(tx.n == 2 && tx.p[1].op == 0x05);
		memcpy(buf, rsp1_air, 5);
		CHECK(ll_crypt_decrypt(rxq_crypt, 0x0F, buf, 5) == 1);
		rx(buf, 1);
		CHECK(tx.n == 4);
		CHECK(tx.p[2].op == 0x06 && memcmp(tx.p[2].d, rsp2_air, 5) == 0);
		CHECK(tx.p[3].op == 0x14 && tx.p[3].len == 9 + LL_MIC_LEN);
		CHECK(ll_llcp_busy(L));
		CHECK(hci.enc_change == 1);
		/* and after a negative reply */
		fresh();
		sample_rand();
		rx(req, sizeof(req));
		CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
		CHECK(tx.n == 1);
		CHECK(ll_llcp_ltk_neg_reply(L) == LL_ST_SUCCESS);
		CHECK(tx.n == 3 && tx.p[1].op == 0x0D);
		CHECK(last_length(0x14, SUP, SUPT, SUP, SUPT));
	}
	CHECK(kk.bad == 0);
}

static void phy_procedure(void)
{
	static const uint8_t phy_rsp[3] = {0x17, 0x01, 0x01};
	uint8_t ind[5] = {0x18, 0, 0, 0x34, 0x12};
	uint32_t t0;

	/* central prefers 1M|2M both ways: we answer 1M only */
	fresh();
	{
		static const uint8_t req[3] = {0x16, 0x03, 0x03};

		rx(req, 3);
	}
	CHECK(tx.n == 1 && last_is(phy_rsp, 3));
	CHECK(kk.bad == 0);
	/* the procedure waits on the central's LL_PHY_UPDATE_IND */
	CHECK(ll_llcp_busy(L));
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)T(TIMEOUT_US));
	/* no change (both 0, no instant): done, nothing else happens */
	rx(ind, 5);
	CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
	CHECK(tx.n == 1 && cn.end_calls == 0 && cn.upd_calls == 0);

	/* LL_PHY_UPDATE_IND with 1M (no real change), an unsupported PHY,
	 * a reserved bit or several bits: we keep 1M ("shall not change the
	 * PHY in that direction"), the link stays */
	{
		static const uint8_t fields[][2] = {{0x01, 0x01}, {0x01, 0x00}, {0x02, 0x02},
						    {0x04, 0x00}, {0x03, 0x05}, {0x80, 0x00},
						    {0x00, 0x08}};

		for (unsigned int i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
			static const uint8_t req[3] = {0x16, 0x01, 0x01};

			fresh();
			rx(req, 3);
			CHECK(ll_llcp_busy(L));
			ind[1] = fields[i][0];
			ind[2] = fields[i][1];
			rx(ind, 5);
			CHECK(!ll_llcp_busy(L));
			CHECK(tx.n == 1 && cn.end_calls == 0);
		}
	}
	/* an unsolicited LL_PHY_UPDATE_IND: no answer, no end */
	fresh();
	ind[1] = 0x02;
	rx(ind, 5);
	CHECK(tx.n == 0 && cn.end_calls == 0 && !ll_llcp_busy(L));

	/* PHY_REQ with an empty field still gets our 1M answer */
	{
		static const uint8_t req[3] = {0x16, 0x00, 0x00};

		rx(req, 3);
		CHECK(tx.n == 1 && last_is(phy_rsp, 3));
	}

	/* wrong lengths -> LL_UNKNOWN_RSP */
	fresh();
	{
		static const uint8_t req[4] = {0x16, 0x01, 0x01, 0x00};
		static const uint8_t exp_r[2] = {0x07, 0x16};
		static const uint8_t exp_i[2] = {0x07, 0x18};

		rx(req, 4);
		CHECK(last_is(exp_r, 2));
		rx(ind, 4);
		CHECK(last_is(exp_i, 2));
		CHECK(!ll_llcp_busy(L));
	}

	/* no LL_PHY_UPDATE_IND within 40 s: link lost (0x22) */
	fresh();
	t0 = now;
	{
		static const uint8_t req[3] = {0x16, 0x01, 0x01};

		rx(req, 3);
	}
	ll_llcp_tick(t0 + T(TIMEOUT_US) - 1);
	CHECK(cn.end_calls == 0);
	ll_llcp_tick(t0 + T(TIMEOUT_US));
	CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_LMP_TIMEOUT);
	CHECK(!ll_llcp_busy(L));

	/* the PHY and LENGTH timers are independent: the PHY procedure
	 * completing leaves our LENGTH procedure's timer running */
	fresh();
	t0 = now;
	if (SUP > 27) {
		static const uint8_t req[3] = {0x16, 0x01, 0x01};

		CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
		now += T(1000000);
		rx(req, 3);
		ind[1] = 0;
		ind[2] = 0;
		rx(ind, 5);
		CHECK(ll_llcp_busy(L));
		CHECK(ll_llcp_timeout_ticks(t0) == (int32_t)T(TIMEOUT_US));
	}
}

/* Out-of-range and inactive links */
static void dle_bounds(void)
{
	struct ll_llcp_dle d;

	fresh();
	CHECK(ll_llcp_set_data_len(LL_MAX_CONN, 251, 2120) == LL_ST_UNKNOWN_CONN_ID);
	memset(&d, 0xEE, sizeof(d));
	ll_llcp_get_dle(LL_MAX_CONN, &d);
	CHECK(dle_is(&d, 27, 328, 27, 328));
	cnl[0].active = false;
	CHECK(ll_llcp_set_data_len(0, 251, 2120) == LL_ST_UNKNOWN_CONN_ID);
	CHECK(tx.n == 0);
}

/* LENGTH state is per link: a procedure on one link never changes
 * another's values, timer or events */
static void test_dle_per_link(void)
{
	const uint8_t a = 0, b = (uint8_t)(LL_MAX_CONN - 1);
	uint8_t pdu[9];

	if (LL_MAX_CONN < 2) {
		return;
	}
	fresh();
	length_pdu(pdu, 0x14, 251, 2120, 251, 2120);
	rx_l(b, pdu, 9);
	CHECK(tx.n == 1 && tx.p[0].link == b);
	CHECK(eff_is(a, 27, 328, 27, 328));
	CHECK(dlcl[a].calls == 0 && dlcl[b].calls == (SUP > 27 ? 1 : 0));
	if (SUP > 27) {
		CHECK(ll_llcp_set_data_len(a, 251, 2120) == LL_ST_SUCCESS);
		CHECK(tx.p[1].link == a && ll_llcp_busy(a) && !ll_llcp_busy(b));
		/* the RSP on link b is no answer to anything there */
		length_pdu(pdu, 0x15, 251, 2120, 251, 2120);
		rx_l(b, pdu, 9);
		CHECK(ll_llcp_busy(a) && eff_is(a, 27, 328, 27, 328));
		rx_l(a, pdu, 9);
		CHECK(!ll_llcp_busy(a) && eff_is(a, SUP, SUPT, SUP, SUPT));
		CHECK(eff_is(b, 27, 328, SUP, SUPT));
		CHECK(dlcl[a].calls == 1 && dlcl[b].calls == 1);
	}
	ll_llcp_reset(b);
	CHECK(eff_is(b, 27, 328, 27, 328));
	if (SUP > 27) {
		CHECK(eff_is(a, SUP, SUPT, SUP, SUPT));
	}
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

/* ---------------- owed control PDUs (slice 7) ----------------
 * A control PDU we must send whose push finds the TX backlog full is owed:
 * the procedure state moves on as if it was queued, the PDU waits in the
 * link's owed queue (in order; later control PDUs of the link queue behind
 * it, host ACL waits with -EAGAIN) and ll_llcp_retry() pushes it from the
 * controller thread. While something is owed ll_llcp_timeout_ticks() asks
 * for a wakeup within LL_LLCP_RETRY_MS; owing never ends the link (a dead
 * link ends by supervision, a stuck procedure by its 40 s timer). */

static const uint8_t vi_c[6] = {0x0C, 0x0A, 0x02, 0x00, 0x34, 0x12};
static const uint8_t vi_ours[6] = {0x0C, 0x09, 0xFF, 0xFF, 0x01, 0x00};
static const uint8_t feat_ours[9] = {0x09, 0x37, 0x40, 0, 0, 0, 0, 0, 0};

/* push k is exactly the control PDU exp */
static int push_is(int k, const uint8_t *exp, uint8_t len)
{
	return k < tx.n && tx.p[k].link == L && tx.p[k].kind == LL_TXQ_CTRL &&
	       tx.p[k].llid == LL_LLID_CTRL && tx.p[k].len == len && tx.p[k].op == exp[0] &&
	       memcmp(tx.p[k].d, exp, len) == 0;
}

static void push_retry_suite(void)
{
	const int32_t retry = (int32_t)T(LL_LLCP_RETRY_MS * 1000u);

	/* a response (LL_FEATURE_RSP) is owed, pushed at the next retry */
	fresh();
	tx.fail = 1;
	features(0xFF);
	CHECK(tx.n == 0 && kk.calls == 0 && cn.end_calls == 0);
	CHECK(ll_llcp_busy(L));
	CHECK(ll_llcp_timeout_ticks(now) == retry);
	ll_llcp_retry(L);
	CHECK(tx.n == 1 && push_is(0, feat_ours, 9));
	CHECK(kk.calls == 1 && kk.bad == 0);
	CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
	ll_llcp_retry(L);   /* nothing owed: nothing happens */
	CHECK(tx.n == 1);

	/* order: what comes later queues behind the owed PDU, host ACL waits */
	fresh();
	tx.fail = 1;
	{
		static const uint8_t unk[2] = {0x07, 0x19};
		uint8_t pdu[3] = {0x19, 0, 0};

		rx(pdu, 3);                            /* LL_UNKNOWN_RSP owed */
		rx(vi_c, 6);                           /* LL_VERSION_IND behind it */
		CHECK(tx.n == 0);
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -EAGAIN);
		/* still full at the first retry: nothing lost, still in order */
		tx.fail = 1;
		ll_llcp_retry(L);
		CHECK(tx.n == 0);
		ll_llcp_retry(L);
		CHECK(tx.n == 2 && push_is(0, unk, 2) && push_is(1, vi_ours, 6));
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
		rx(vi_c, 6);                           /* answered once per connection */
		CHECK(tx.n == 3);
	}

	/* only the full queue's link waits: another link's ACL goes on */
	if (LL_MAX_CONN > 1) {
		uint8_t other = L == 0 ? 1 : 0;

		fresh();
		tx.fail = 1;
		features(0xFF);
		CHECK(ll_llcp_tx(other, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
		CHECK(!ll_llcp_busy(other));
		ll_llcp_retry(other);
		CHECK(tx.n == 1 && tx.p[0].link == other);
		ll_llcp_retry(L);
		CHECK(tx.n == 2 && push_is(1, feat_ours, 9));
	}

	/* LL_ENC_RSP owed: the procedure goes on (LTK request at once), the
	 * host's LL_START_ENC_REQ queues behind it, both leave in order */
	fresh();
	{
		uint8_t req[23], buf[8];
		static const uint8_t ser[1] = {0x05};

		sample_rand();
		build_enc_req(req);
		tx.fail = 1;
		rx(req, sizeof(req));
		CHECK(tx.n == 0 && cn.end_calls == 0);
		CHECK(hci.ltk_req == 1 && rxq_crypt != NULL);
		if (rxq_crypt == NULL) {
			return;   /* the link ended: nothing more to check */
		}
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		CHECK(tx.n == 0);
		ll_llcp_retry(L);
		CHECK(tx.n == 2 && tx.p[0].op == 0x04 && tx.p[0].len == 13);
		CHECK(memcmp(&tx.p[0].d[1], skds, 8) == 0 && memcmp(&tx.p[0].d[9], ivs, 4) == 0);
		CHECK(push_is(1, ser, 1));             /* plaintext */
		memcpy(buf, rsp1_air, 5);
		CHECK(ll_crypt_decrypt(rxq_crypt, 0x0F, buf, 5) == 1);
		rx(buf, 1);
		CHECK(tx.n == 3 && tx.p[2].len == 5 && memcmp(tx.p[2].d, rsp2_air, 5) == 0);
		CHECK(hci.enc_change == 1 && hci.enabled);
		CHECK(!ll_llcp_busy(L));
	}

	/* LL_START_ENC_REQ owed alone (HCI thread): pushed at the retry */
	fresh();
	{
		uint8_t req[23];
		static const uint8_t ser[1] = {0x05};

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		tx.fail = 1;
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		CHECK(tx.n == 1);
		ll_llcp_retry(L);
		CHECK(tx.n == 2 && push_is(1, ser, 1));
	}

	/* our encrypted LL_START_ENC_RSP owed: data stays paused, Encryption
	 * Change only once it is queued (encrypted, counter 0) */
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
		CHECK(tx.n == 2 && hci.enc_change == 0 && ll_llcp_busy(L));
		CHECK(rxq_crypt->tx_ctr == 0);
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == -EAGAIN);
		ll_llcp_retry(L);
		CHECK(tx.n == 3 && tx.p[2].len == 5 && memcmp(tx.p[2].d, rsp2_air, 5) == 0);
		CHECK(hci.enc_change == 1 && hci.status == LL_ST_SUCCESS && hci.enabled);
		/* no procedure left; only the slice 6d payload timer asks for a wakeup */
		CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == (int32_t)APTO_T(3000));
		/* data resumes encrypted with counter 1 (Vol 6 Part C LL_DATA2) */
		CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, data2_clear, 27) == 0);
		CHECK(tx.n == 4 && tx.p[3].len == 31 && memcmp(tx.p[3].d, data2_air, 31) == 0);
	}

	/* LL_TERMINATE_IND through ll_conn's ctrl_tx hook */
	fresh();
	{
		static const uint8_t ti[2] = {0x02, 0x13};

		tx.fail = 1;
		CHECK(ll_llcp_ctrl_tx(L, ti, 2) == 0);
		CHECK(tx.n == 0);
		ll_llcp_retry(L);
		CHECK(tx.n == 1 && push_is(0, ti, 2));
	}

	/* owing never ends a link (review fix: supervision and the 40 s
	 * procedure timers cover dead links): a response owed for a minute
	 * keeps the 10 ms retry wakeup and goes out once there is room */
	fresh();
	{
		tx.fail = 100000;
		features(0xFF);
		for (int i = 0; i < 6000; i++) {
			now += T(LL_LLCP_RETRY_MS * 1000u);
			ll_llcp_retry(L);
			ll_llcp_tick(now);
			CHECK(ll_llcp_timeout_ticks(now) == retry);
		}
		CHECK(cn.end_calls == 0 && ll_llcp_busy(L) && tx.n == 0);
		tx.fail = 0;
		ll_llcp_retry(L);
		CHECK(tx.n == 1 && push_is(0, feat_ours, 9));
		CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
	}
	/* a procedure whose PDU stays owed: its own 40 s timer ends the link
	 * (0x22), and the owed PDU is dropped with it */
	fresh();
	{
		static const uint8_t req[3] = {0x16, 0x03, 0x03};
		uint32_t t0 = now;

		tx.fail = 1000;
		rx(req, 3);                            /* LL_PHY_RSP owed, timer on */
		CHECK(ll_llcp_timeout_ticks(t0) == retry);
		ll_llcp_tick(t0 + T(TIMEOUT_US) - 1);
		CHECK(cn.end_calls == 0);
		CHECK(ll_llcp_timeout_ticks(t0 + T(TIMEOUT_US) - 1) == 1);
		ll_llcp_tick(t0 + T(TIMEOUT_US));
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_LMP_TIMEOUT);
		CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
		tx.fail = 0;
		ll_llcp_retry(L);
		CHECK(tx.n == 0);
	}

	/* the owed queue holds LL_LLCP_OWE_N PDUs; one more response is
	 * dropped (the central's response timeout), an LL_ENC_REQ that cannot
	 * even be owed ends the link (0x1F) as before */
	fresh();
	{
		uint8_t pdu[3] = {0x19, 0, 0}, req[23];

		tx.fail = 1000;
		for (int i = 0; i < LL_LLCP_OWE_N + 1; i++) {
			pdu[0] = (uint8_t)(0x19 + i);
			rx(pdu, 3);
		}
		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(hci.ltk_req == 0);
		CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_UNSPECIFIED);
		tx.fail = 0;
		ll_llcp_retry(L);
		CHECK(tx.n == LL_LLCP_OWE_N);
		for (int i = 0; i < LL_LLCP_OWE_N; i++) {
			CHECK(tx.p[i].op == 0x07 && tx.p[i].d[1] == 0x19 + i);
		}
	}

	/* a new connection forgets what was owed */
	fresh();
	tx.fail = 1;
	features(0xFF);
	ll_llcp_reset(L);
	ll_llcp_retry(L);
	CHECK(tx.n == 0 && !ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
}

/* Data length and PHY PDUs owed (slice 7 / slice 6b Task 3 carry-over):
 * a link never stays at the initial values because one push failed. */
static void push_retry_dle_phy(void)
{
	uint8_t pdu[9];

	/* LL_LENGTH_RSP owed: the central learns our values from it */
	fresh();
	length_pdu(pdu, 0x14, 251, 2120, 251, 2120);
	tx.fail = 1;
	rx(pdu, 9);
	CHECK(tx.n == 0);
	ll_llcp_retry(L);
	CHECK(tx.n == 1 && last_length(0x15, SUP, SUPT, 27, 328));
	CHECK(eff_is(L, 27, 328, SUP, SUPT));
	/* the central knows our 27 / 328 now: nothing to send for them */
	CHECK(ll_llcp_set_data_len(L, 27, 328) == LL_ST_SUCCESS);
	CHECK(tx.n == 1);

	/* LL_PHY_RSP owed: the 40 s timer runs from the request */
	fresh();
	{
		static const uint8_t req[3] = {0x16, 0x03, 0x03};
		static const uint8_t phy_rsp[3] = {0x17, 0x01, 0x01};
		uint8_t ind[5] = {0x18, 0, 0, 0x34, 0x12};

		tx.fail = 1;
		rx(req, 3);
		CHECK(tx.n == 0 && ll_llcp_busy(L));
		ll_llcp_retry(L);
		CHECK(tx.n == 1 && last_is(phy_rsp, 3));
		CHECK(ll_llcp_timeout_ticks(now) == (int32_t)T(TIMEOUT_US));
		rx(ind, 5);
		CHECK(!ll_llcp_busy(L));
	}

	/* our own LL_LENGTH_REQ needs a supported maximum above 27 */
	if (SUP == 27) {
		return;
	}
	/* our LL_LENGTH_REQ (host or on-connect default) owed: accepted, the
	 * procedure runs, the request leaves at the retry */
	fresh();
	tx.fail = 1;
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	CHECK(tx.n == 0 && ll_llcp_busy(L));
	ll_llcp_retry(L);
	CHECK(tx.n == 1 && last_length(0x14, SUP, SUPT, SUP, SUPT));
	length_pdu(pdu, 0x15, 251, 2120, 251, 2120);
	rx(pdu, 9);
	CHECK(eff_is(L, SUP, SUPT, SUP, SUPT));
	CHECK(!ll_llcp_busy(L));

	/* deferred by the encryption start, then owed when it is sent right
	 * after our LL_START_ENC_RSP */
	fresh();
	{
		uint8_t req[23], buf[8];

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		memcpy(buf, rsp1_air, 5);
		CHECK(ll_crypt_decrypt(rxq_crypt, 0x0F, buf, 5) == 1);
		tx.ok_first = 1;   /* LL_START_ENC_RSP goes, LL_LENGTH_REQ does not */
		tx.fail = 1;
		rx(buf, 1);
		CHECK(tx.n == 3 && hci.enc_change == 1);
		CHECK(ll_llcp_busy(L));
		ll_llcp_retry(L);
		CHECK(tx.n == 4 && tx.p[3].op == 0x14 && tx.p[3].len == 9 + LL_MIC_LEN);
	}
}

/* Slice 6b Task 4 (Task 3 review carry-over b): a host LE Set Data Length
 * while our LL_LENGTH_REQ runs is stored and sent right after its
 * LL_LENGTH_RSP (no Command Disallowed). */
static void dle_initiator_pending(void)
{
	uint8_t pdu[9];

	if (SUP == 27) {
		return;
	}
	fresh();
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	CHECK(tx.n == 1 && last_length(0x14, SUP, SUPT, SUP, SUPT));
	/* while ours runs: success, nothing sent yet, still busy */
	CHECK(ll_llcp_set_data_len(L, 100, 912) == LL_ST_SUCCESS);
	CHECK(tx.n == 1 && ll_llcp_busy(L));
	CHECK(eff_is(L, 27, 328, 27, 328) && dlc.calls == 0);
	/* and again: the newest values win */
	CHECK(ll_llcp_set_data_len(L, 120, 1072) == LL_ST_SUCCESS);
	CHECK(tx.n == 1);
	/* the response: the stored values apply (Tx 120 / 1072 against the
	 * central's Rx 251), one event, and our next LL_LENGTH_REQ tells the
	 * central at once */
	length_pdu(pdu, 0x15, 251, 2120, 251, 2120);
	rx(pdu, 9);
	CHECK(eff_is(L, 120, 1072, SUP, SUPT));
	CHECK(dlc.calls == 1 && dle_is(&dlc.eff, 120, 1072, SUP, SUPT));
	CHECK(dtl[L].calls == 1 && dtl[L].rx_time == SUPT && dtl[L].tx_time == 1072);
	CHECK(tx.n == 2 && last_length(0x14, SUP, SUPT, 120, 1072));
	CHECK(ll_llcp_busy(L));
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)T(TIMEOUT_US));
	length_pdu(pdu, 0x15, 251, 2120, 251, 2120);
	rx(pdu, 9);
	CHECK(!ll_llcp_busy(L) && tx.n == 2 && dlc.calls == 1);

	/* stored values equal to what the central already knows: nothing more
	 * is sent after the response */
	fresh();
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	CHECK(ll_llcp_set_data_len(L, 100, 912) == LL_ST_SUCCESS);
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	length_pdu(pdu, 0x15, 251, 2120, 251, 2120);
	rx(pdu, 9);
	CHECK(tx.n == 1 && !ll_llcp_busy(L));
	CHECK(eff_is(L, SUP, SUPT, SUP, SUPT));

	/* the central refuses ours (LL_UNKNOWN_RSP): the stored values are
	 * not sent (it does not know the procedure) */
	fresh();
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	CHECK(ll_llcp_set_data_len(L, 100, 912) == LL_ST_SUCCESS);
	{
		static const uint8_t unk[2] = {0x07, 0x14};

		rx(unk, 2);
	}
	CHECK(tx.n == 1 && !ll_llcp_busy(L));
	CHECK(ll_llcp_set_data_len(L, 120, 1072) == LL_ST_UNSUPP_REMOTE);

	/* a crossing request of the central in between answers with the
	 * stored values (it tells the central), so nothing is left to send */
	fresh();
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	CHECK(ll_llcp_set_data_len(L, 100, 912) == LL_ST_SUCCESS);
	length_pdu(pdu, 0x14, 251, 2120, 251, 2120);
	rx(pdu, 9);
	CHECK(tx.n == 2 && last_length(0x15, SUP, SUPT, 100, 912));
	length_pdu(pdu, 0x15, 251, 2120, 251, 2120);
	rx(pdu, 9);
	CHECK(tx.n == 2 && !ll_llcp_busy(L));
	CHECK(eff_is(L, 100, 912, SUP, SUPT));
}

/* Slice 6b Task 4: host ACL in fragments, all or nothing, the credit flag
 * on the last fragment only, the per-link TX limit by Octets and Time. */
static void acl_fragments(void)
{
	uint8_t big[251], pdu[9];
	struct ll_acl_frag f[10];

	for (int i = 0; i < 251; i++) {
		big[i] = (uint8_t)(i ^ 0x5A);
	}
	/* a new link: 27 octets */
	fresh();
	CHECK(ll_llcp_tx_limit(L) == 27);
	CHECK(ll_llcp_tx_limit(LL_MAX_CONN) == 27);
	for (uint8_t i = 0; i < 10; i++) {
		f[i].off = (uint8_t)(27 * i);
		f[i].len = i < 9 ? 27 : 8;
		f[i].llid = i == 0 ? LL_LLID_START : LL_LLID_CONT;
	}
	CHECK(ll_llcp_tx_acl(L, big, f, 10) == 0);
	CHECK(tx.fits_calls == 1 && tx.fits_n == 10 && tx.fits_len == 27 && tx.fits_last == 8);
	CHECK(tx.n == 10);
	for (int i = 0; i < 10 && i < tx.n; i++) {
		CHECK(tx.p[i].kind == LL_TXQ_ACL && tx.p[i].link == L);
		CHECK(tx.p[i].llid == f[i].llid && tx.p[i].len == f[i].len);
		CHECK(memcmp(tx.p[i].d, &big[f[i].off], f[i].len) == 0);
		CHECK(tx.p[i].last == (i == 9));
	}
	CHECK(kk.bad == 0 && kk.calls >= 1);
	CHECK(locks == 0 && tx_locks == 0);
	/* review minor: the fit check covers every fragment but the last with
	 * the longest of them, not with the first one's length */
	f[0].len = 10;
	f[1].off = 10;
	f[2].off = 37;
	f[3].off = 64;
	f[3].len = 5;
	tx.n = 0;
	CHECK(ll_llcp_tx_acl(L, big, f, 4) == 0);
	CHECK(tx.fits_n == 4 && tx.fits_len == 27 && tx.fits_last == 5);
	for (uint8_t i = 0; i < 10; i++) {
		f[i].off = (uint8_t)(27 * i);
		f[i].len = i < 9 ? 27 : 8;
	}
	/* not all fit: nothing is queued */
	tx.n = 0;
	tx.room = 9;
	CHECK(ll_llcp_tx_acl(L, big, f, 10) == -ENOMEM);
	CHECK(tx.n == 0);
	tx.room = 0;
	/* a fragment above the link's limit: refused, nothing queued */
	f[0].len = 28;
	CHECK(ll_llcp_tx_acl(L, big, f, 1) == -EINVAL);
	CHECK(ll_llcp_tx(L, LL_TXQ_ACL, LL_LLID_START, big, 28) == -EINVAL);
	CHECK(tx.n == 0);
	CHECK(ll_llcp_tx_acl(L, big, f, 0) == -EINVAL);
	CHECK(ll_llcp_tx_acl(LL_MAX_CONN, big, f, 1) == -EINVAL);
	if (SUP == 27) {
		return;
	}

	/* 251 / 2120 both ways: one PDU of 251 */
	CHECK(ll_llcp_set_data_len(L, 251, 2120) == LL_ST_SUCCESS);
	length_pdu(pdu, 0x15, 251, 2120, 251, 2120);
	rx(pdu, 9);
	CHECK(ll_llcp_tx_limit(L) == 251);
	f[0].off = 0;
	f[0].len = 251;
	f[0].llid = LL_LLID_START;
	tx.n = 0;
	CHECK(ll_llcp_tx_acl(L, big, f, 1) == 0);
	CHECK(tx.n == 1 && tx.p[0].len == 251 && tx.p[0].last);
	CHECK(memcmp(tx.p[0].d, big, 251) == 0);

	/* the central receives 251 octets but only 1000 us: time-limited,
	 * 115 plain, 111 encrypted (the MIC counts in the time) */
	length_pdu(pdu, 0x14, 251, 1000, 251, 2120);
	rx(pdu, 9);
	CHECK(ll_llcp_tx_limit(L) == 115);
	f[0].len = 116;
	CHECK(ll_llcp_tx_acl(L, big, f, 1) == -EINVAL);
	f[0].len = 115;
	tx.n = 0;
	CHECK(ll_llcp_tx_acl(L, big, f, 1) == 0 && tx.n == 1);
	features(0xFF);
	start_encryption();
	CHECK(ll_llcp_tx_limit(L) == 111);
	{
		uint64_t c0 = rxq_crypt->tx_ctr;

		tx.n = 0;
		CHECK(ll_llcp_tx_acl(L, big, f, 1) == -EINVAL);   /* 115 + MIC too long now */
		f[0].len = 111;
		f[1].off = 111;
		f[1].len = 111;
		f[1].llid = LL_LLID_CONT;
		f[2].off = 222;
		f[2].len = 29;
		f[2].llid = LL_LLID_CONT;
		CHECK(ll_llcp_tx_acl(L, big, f, 3) == 0);
		CHECK(tx.fits_n == 3 && tx.fits_len == 111 + LL_MIC_LEN &&
		      tx.fits_last == 29 + LL_MIC_LEN);
		CHECK(tx.n == 3 && tx.p[0].len == 115 && tx.p[2].len == 33);
		CHECK(!tx.p[0].last && !tx.p[1].last && tx.p[2].last);
		CHECK(rxq_crypt->tx_ctr == c0 + 3);
		/* refused as a whole: the counter is not used */
		tx.room = 2;
		CHECK(ll_llcp_tx_acl(L, big, f, 3) == -ENOMEM);
		CHECK(rxq_crypt->tx_ctr == c0 + 3 && tx.n == 3);
		tx.room = 0;
	}
	/* data paused by the encryption start: -EAGAIN before anything */
	fresh();
	features(0xFF);
	{
		uint8_t req[23];
		int n0, c0;

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		n0 = tx.n;
		c0 = tx.fits_calls;
		f[0].off = 0;
		f[0].len = 27;
		f[0].llid = LL_LLID_START;
		CHECK(ll_llcp_tx_acl(L, big, f, 1) == -EAGAIN);
		CHECK(tx.n == n0 && tx.fits_calls == c0);
	}
	CHECK(locks == 0 && tx_locks == 0);
}

/* ---------------- LE Ping, authenticated payload timeout (slice 6d) ---------------- */

static const uint8_t ping_req[1] = {0x12};
static const uint8_t ping_rsp[1] = {0x13};

/* the last push is our encrypted LL_PING_REQ (opcode + MIC) on link */
static int last_is_ping_req(uint8_t link)
{
	return tx.n > 0 && tx.n <= MAX_PUSH && tx.p[tx.n - 1].link == link &&
	       tx.p[tx.n - 1].kind == LL_TXQ_CTRL && tx.p[tx.n - 1].llid == LL_LLID_CTRL &&
	       tx.p[tx.n - 1].op == 0x12 && tx.p[tx.n - 1].len == 1 + LL_MIC_LEN;
}

static void start_enc_l(uint8_t link)
{
	uint8_t req[23], buf[8];
	int e0 = hcil[link].enc_change;

	sample_rand();
	build_enc_req(req);
	rx_l(link, req, sizeof(req));
	CHECK(ll_llcp_ltk_reply(link, ltk) == LL_ST_SUCCESS);
	memcpy(buf, rsp1_air, 5);
	CHECK(ll_crypt_decrypt(rxq_crypt_l[link], 0x0F, buf, 5) == 1);
	rx_l(link, buf, 1);
	CHECK(hcil[link].enc_change == e0 + 1);
}

static void ping_suite(void)
{
	uint16_t v;
	uint32_t t0, t1;
	int n0;

	/* ---- responder, plaintext link (5.1.8: any time in the connection) ---- */
	fresh();
	rx(ping_req, 1);
	CHECK(tx.n == 1 && last_is(ping_rsp, 1));
	CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
	{
		const uint8_t bad[2] = {0x12, 0x00};
		const uint8_t exp[2] = {0x07, 0x12};

		rx(bad, 2);
		CHECK(tx.n == 2 && last_is(exp, 2));
	}
	/* an LL_PING_RSP we did not ask for: dropped */
	rx(ping_rsp, 1);
	CHECK(tx.n == 2 && cn.end_calls == 0 && !ll_llcp_busy(L));
	/* inside the encryption start: no answer (5.1.3.1) */
	fresh();
	{
		uint8_t req[23];

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		n0 = tx.n;
		rx(ping_req, 1);
		CHECK(tx.n == n0);
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		n0 = tx.n;
		rx(ping_req, 1);
		CHECK(tx.n == n0);
	}
	/* encrypted link: the answer is encrypted */
	fresh();
	start_encryption();
	rx(ping_req, 1);
	CHECK(tx.p[tx.n - 1].op == 0x13 && tx.p[tx.n - 1].len == 1 + LL_MIC_LEN);
	CHECK(tx.p[tx.n - 1].kind == LL_TXQ_CTRL && tx.p[tx.n - 1].link == L);

	/* ---- the timer does not run while the link is unencrypted ---- */
	fresh();
	ll_llcp_conn_params(L, 24, 4, 3200);
	features(0xFF);
	CHECK(ll_llcp_read_apto(L, &v) == LL_ST_SUCCESS && v == 3000);
	CHECK(ll_llcp_timeout_ticks(now) == -1);
	n0 = tx.n;
	ll_llcp_rx_auth(L);
	ll_llcp_tick(now + APTO_T(3000));
	ll_llcp_tick(now + APTO_T(3000) * 2);
	CHECK(apto_ev == 0 && tx.n == n0 && !ll_llcp_busy(L));
	/* nor during the encryption start: only its procedure timer */
	{
		uint8_t req[23];

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		CHECK(ll_llcp_timeout_ticks(now) == (int32_t)T(TIMEOUT_US));
	}

	/* ---- encrypted: 30 s from the end of the encryption start ---- */
	fresh();
	ll_llcp_conn_params(L, 24, 4, 3200);
	start_encryption();
	t0 = now;
	CHECK(ll_llcp_timeout_ticks(t0) == (int32_t)APTO_T(3000));
	CHECK(ll_llcp_timeout_ticks(t0 + 5) == (int32_t)APTO_T(3000) - 5);
	CHECK(!ll_llcp_busy(L));              /* the timer is no procedure */
	n0 = tx.n;
	ll_llcp_tick(t0 + APTO_T(3000) - 1);
	CHECK(apto_ev == 0 && tx.n == n0);
	now = t0 + APTO_T(3000);
	ll_llcp_tick(now);
	CHECK(apto_ev == 1);
	CHECK(tx.n == n0 + 1 && last_is_ping_req(L));
	CHECK(kk.bad == 0 && kk.n_at_kick == tx.n);   /* kicked: leaves at the next event */
	CHECK(ll_llcp_busy(L));               /* waits for LL_PING_RSP */
	/* the timer restarted at the expiry (5.4); the 40 s timer runs too */
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)APTO_T(3000));
	ll_llcp_tick(now);
	CHECK(apto_ev == 1 && tx.n == n0 + 1);
	/* the answer completes it; the glue reports its valid MIC */
	now += T(100000);
	rx(ping_rsp, 1);
	ll_llcp_rx_auth(L);
	CHECK(!ll_llcp_busy(L) && cn.end_calls == 0);
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)APTO_T(3000));
	/* each packet with a valid MIC restarts it (also across the tick wrap) */
	for (int k = 0; k < 12; k++) {
		now += APTO_T(3000) - 1;
		ll_llcp_tick(now);
		ll_llcp_rx_auth(L);
	}
	CHECK(apto_ev == 1 && tx.n == n0 + 1 && !ll_llcp_busy(L));
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)APTO_T(3000));
	/* a central without LE Ping: LL_UNKNOWN_RSP(0x12) completes ours */
	now += APTO_T(3000);
	ll_llcp_tick(now);
	CHECK(apto_ev == 2 && last_is_ping_req(L) && ll_llcp_busy(L));
	{
		const uint8_t unk[2] = {0x07, 0x12};

		rx(unk, 2);
	}
	CHECK(!ll_llcp_busy(L) && cn.end_calls == 0);
	/* no answer at all: the event repeats each timeout, one LL_PING_REQ
	 * waits, and its 40 s response timer ends the link */
	n0 = tx.n;
	t0 = now;                             /* last restart: the expiry above */
	now = t0 + APTO_T(3000);
	ll_llcp_tick(now);
	CHECK(apto_ev == 3 && tx.n == n0 + 1 && last_is_ping_req(L));
	t1 = now;
	now = t1 + APTO_T(3000);
	ll_llcp_tick(now);
	CHECK(apto_ev == 4 && tx.n == n0 + 1);   /* no second request */
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)(T(TIMEOUT_US) - APTO_T(3000)));
	ll_llcp_tick(t1 + T(TIMEOUT_US) - 1);
	CHECK(cn.end_calls == 0);
	ll_llcp_tick(t1 + T(TIMEOUT_US));
	CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_LMP_TIMEOUT);
	/* the lost link reports nothing more */
	CHECK(ll_llcp_timeout_ticks(now) == -1 && !ll_llcp_busy(L));
	ll_llcp_tick(t1 + T(TIMEOUT_US) + APTO_T(3000) * 2);
	CHECK(apto_ev == 4 && tx.n == n0 + 1);

	/* ---- a full backlog owes LL_PING_REQ (slice 7) ---- */
	fresh();
	ll_llcp_conn_params(L, 24, 4, 3200);
	start_encryption();
	t0 = now;
	n0 = tx.n;
	tx.fail = 1;
	ll_llcp_tick(t0 + APTO_T(3000));
	CHECK(apto_ev == 1 && tx.n == n0 && ll_llcp_busy(L));
	ll_llcp_retry(L);
	CHECK(tx.n == n0 + 1 && last_is_ping_req(L) && ll_llcp_busy(L));

	/* ---- HCI Read / Write ---- */
	fresh();
	ll_llcp_conn_params(L, 24, 4, 3200);       /* 30 ms x 5 = 150 ms = 15 x 10 ms */
	CHECK(ll_llcp_write_apto(L, 15) == LL_ST_SUCCESS);
	CHECK(ll_llcp_read_apto(L, &v) == LL_ST_SUCCESS && v == 15);
	CHECK(ll_llcp_write_apto(L, 14) == LL_ST_INVALID_PARAM);
	CHECK(ll_llcp_write_apto(L, 0) == LL_ST_INVALID_PARAM);
	CHECK(ll_llcp_read_apto(L, &v) == LL_ST_SUCCESS && v == 15);
	CHECK(ll_llcp_write_apto(L, 0xFFFF) == LL_ST_SUCCESS);
	CHECK(ll_llcp_read_apto(L, &v) == LL_ST_SUCCESS && v == 0xFFFF);
	/* 7.5 ms x 1 = 0.75 x 10 ms: 1 is enough */
	ll_llcp_conn_params(L, 6, 0, 3200);
	CHECK(ll_llcp_write_apto(L, 1) == LL_ST_SUCCESS);
	/* an update raises a timeout below the new minimum: 15 ms x 11 =
	 * 165 ms -> 17 (rounded up); a later shorter interval keeps it */
	ll_llcp_conn_params(L, 12, 10, 3200);
	CHECK(ll_llcp_read_apto(L, &v) == LL_ST_SUCCESS && v == 17);
	ll_llcp_conn_params(L, 6, 0, 3200);
	CHECK(ll_llcp_read_apto(L, &v) == LL_ST_SUCCESS && v == 17);
	/* saturates at the largest value */
	ll_llcp_conn_params(L, 3200, 499, 3200);
	CHECK(ll_llcp_read_apto(L, &v) == LL_ST_SUCCESS && v == 0xFFFF);
	/* out of range or not connected */
	CHECK(ll_llcp_read_apto(LL_MAX_CONN, &v) == LL_ST_UNKNOWN_CONN_ID);
	CHECK(ll_llcp_write_apto(LL_MAX_CONN, 3000) == LL_ST_UNKNOWN_CONN_ID);
	cn.active = false;
	CHECK(ll_llcp_read_apto(L, &v) == LL_ST_UNKNOWN_CONN_ID);
	CHECK(ll_llcp_write_apto(L, 3000) == LL_ST_UNKNOWN_CONN_ID);
	cn.active = true;
	/* a new connection: the default again */
	ll_llcp_reset(L);
	CHECK(ll_llcp_read_apto(L, &v) == LL_ST_SUCCESS && v == 3000);

	/* ---- a write restarts the running timer with the new value ---- */
	fresh();
	ll_llcp_conn_params(L, 24, 4, 3200);
	start_encryption();
	t0 = now;
	now = t0 + APTO_T(2000);
	CHECK(ll_llcp_write_apto(L, 500) == LL_ST_SUCCESS);
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)APTO_T(500));
	ll_llcp_tick(now + APTO_T(500) - 1);
	CHECK(apto_ev == 0);
	ll_llcp_tick(now + APTO_T(500));
	CHECK(apto_ev == 1 && last_is_ping_req(L));
	/* written before the encryption: used from its end */
	fresh();
	ll_llcp_conn_params(L, 24, 4, 3200);
	CHECK(ll_llcp_write_apto(L, 100) == LL_ST_SUCCESS);
	CHECK(ll_llcp_timeout_ticks(now) == -1);
	start_encryption();
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)APTO_T(100));

	/* ---- the longest timeout (655.35 s) spans several 32-bit tick wraps:
	 * the wakeup is capped so the controller never misses one ---- */
	fresh();
	ll_llcp_conn_params(L, 24, 4, 3200);
	CHECK(ll_llcp_write_apto(L, 0xFFFF) == LL_ST_SUCCESS);
	start_encryption();
	{
		const uint64_t due = (uint64_t)0xFFFF * 10000u * LL_TICKS_PER_US;
		uint64_t at = 0;
		uint32_t base = now;
		int wakes = 0;

		while (apto_ev == 0 && wakes < 100) {
			int32_t left = ll_llcp_timeout_ticks(base + (uint32_t)at);

			CHECK(left > 0 && left <= (int32_t)(1u << 30));
			if (left <= 0) {
				break;
			}
			if (at + (uint64_t)left == due) {
				ll_llcp_tick(base + (uint32_t)(due - 1));
				CHECK(apto_ev == 0);
			}
			at += (uint64_t)left;
			ll_llcp_tick(base + (uint32_t)at);
			wakes++;
		}
		CHECK(apto_ev == 1 && at == due && wakes >= 10 && wakes <= 12);
	}
	CHECK(locks == 0 && tx_locks == 0);
}

/* Per link: each link's own timer, value and ping */
static void test_ping_per_link(void)
{
	const uint8_t a = 0, b = (uint8_t)(LL_MAX_CONN - 1);
	uint32_t ta, tb;
	uint16_t v;

	if (LL_MAX_CONN < 2) {
		return;
	}
	fresh();
	ll_llcp_conn_params(a, 24, 4, 3200);
	ll_llcp_conn_params(b, 24, 4, 3200);
	start_enc_l(a);
	ta = now;
	now += T(1000000);
	start_enc_l(b);
	tb = now;
	/* the earliest one */
	CHECK(ll_llcp_timeout_ticks(tb) == (int32_t)(APTO_T(3000) - T(1000000)));
	/* a's packet restarts a only: b is now the earliest */
	now = ta + T(2000000);
	ll_llcp_rx_auth(a);
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)(APTO_T(3000) - T(1000000)));
	ll_llcp_tick(tb + APTO_T(3000));
	CHECK(aptol[b] == 1 && aptol[a] == 0 && last_is_ping_req(b));
	CHECK(ll_llcp_busy(b) && !ll_llcp_busy(a));
	/* values and parameters per link */
	CHECK(ll_llcp_write_apto(b, 100) == LL_ST_SUCCESS);
	CHECK(ll_llcp_read_apto(a, &v) == LL_ST_SUCCESS && v == 3000);
	ll_llcp_conn_params(a, 400, 30, 3200);     /* 500 ms x 31 = 15.5 s: a keeps 30 s */
	CHECK(ll_llcp_read_apto(a, &v) == LL_ST_SUCCESS && v == 3000);
	CHECK(ll_llcp_read_apto(b, &v) == LL_ST_SUCCESS && v == 100);
	ll_llcp_conn_params(b, 400, 30, 3200);
	CHECK(ll_llcp_read_apto(b, &v) == LL_ST_SUCCESS && v == 1550);
	CHECK(ll_llcp_read_apto(a, &v) == LL_ST_SUCCESS && v == 3000);
	/* b ends; a goes on */
	ll_llcp_reset(b);
	CHECK(!ll_llcp_busy(b));
	CHECK(ll_llcp_read_apto(b, &v) == LL_ST_SUCCESS && v == 3000);
	now = ta + T(2000000) + APTO_T(3000);
	ll_llcp_tick(now);
	CHECK(aptol[a] == 1 && last_is_ping_req(a) && aptol[b] == 1);
	/* b, unencrypted now, never expires */
	ll_llcp_tick(now + APTO_T(3000) * 3);
	CHECK(aptol[b] == 1);
	CHECK(locks == 0 && tx_locks == 0);
}

/* ---------------- Connection Parameters Request (slice 6d Task 2) ----------------
 * Vol 6 Part B 2.4.2.16/17 (CtrData: Interval_Min, Interval_Max, Latency,
 * Timeout (2 octets each), PreferredPeriodicity (1), ReferenceConnEventCount
 * (2), Offset0..5 (2 each); 24 octets with the opcode), 5.1.7, 5.3. */

#define CPR_LEN 24

static void build_cpr(uint8_t pdu[CPR_LEN], uint16_t imin, uint16_t imax, uint16_t lat,
		      uint16_t to, uint8_t pp, uint16_t ref, uint16_t off0)
{
	pdu[0] = 0x0F;
	ll_put_le16(&pdu[1], imin);
	ll_put_le16(&pdu[3], imax);
	ll_put_le16(&pdu[5], lat);
	ll_put_le16(&pdu[7], to);
	pdu[9] = pp;
	ll_put_le16(&pdu[10], ref);
	ll_put_le16(&pdu[12], off0);
	for (int i = 1; i < 6; i++) {
		ll_put_le16(&pdu[12 + 2 * i], 0xFFFF);
	}
}

static void rx_cpr(uint16_t imin, uint16_t imax, uint16_t lat, uint16_t to, uint8_t pp,
		   uint16_t ref)
{
	uint8_t pdu[CPR_LEN];

	build_cpr(pdu, imin, imax, lat, to, pp, ref, 0xFFFF);
	rx(pdu, sizeof(pdu));
}

/* last push is our LL_CONNECTION_PARAM_RSP with these values, no offset
 * preference (all 0xFFFF) */
static int last_is_cpr_rsp(uint16_t imin, uint16_t imax, uint16_t lat, uint16_t to, uint8_t pp,
			   uint16_t ref)
{
	uint8_t exp[CPR_LEN];

	build_cpr(exp, imin, imax, lat, to, pp, ref, 0xFFFF);
	exp[0] = 0x10;
	return last_is(exp, sizeof(exp));
}

static int last_is_rej_ext(uint8_t op, uint8_t err)
{
	const uint8_t exp[3] = {0x11, op, err};

	return last_is(exp, 3);
}

static uint8_t cpr_reply(uint16_t imin, uint16_t imax, uint16_t lat, uint16_t to)
{
	const struct ll_llcp_cpr p = {imin, imax, lat, to};
	uint8_t st = ll_llcp_conn_param_reply(L, &p);

	CHECK(locks == 0 && tx_locks == 0);
	return st;
}

static void cpr_suite(void)
{
	static const uint8_t upd[12] = {0x00, 0x02, 0x03, 0x00, 0x09, 0x00, 0x1E, 0x00,
					0x90, 0x01, 0x34, 0x12};
	uint32_t t0;

	/* ---- accepted by the host: event, Reply, LL_CONNECTION_PARAM_RSP
	 * with the host's values, then the central's LL_CONNECTION_UPDATE_IND
	 * goes to ll_conn ---- */
	fresh();
	ll_llcp_conn_params(L, 24, 0, 72);
	t0 = now;
	rx_cpr(6, 12, 30, 400, 4, 0x1234);
	CHECK(cpr_ev.calls == 1);
	CHECK(cpr_ev.req.interval_min == 6 && cpr_ev.req.interval_max == 12);
	CHECK(cpr_ev.req.latency == 30 && cpr_ev.req.timeout == 400);
	CHECK(tx.n == 0);   /* nothing on air until the host answers */
	CHECK(ll_llcp_busy(L));
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)T(TIMEOUT_US));
	now = t0 + T(1000000);
	CHECK(cpr_reply(6, 9, 30, 400) == LL_ST_SUCCESS);
	CHECK(tx.n == 1 && last_is_cpr_rsp(6, 9, 30, 400, 4, 0x1234));
	CHECK(kk.calls == 1 && kk.bad == 0);
	/* the timer restarts with the queued RSP; still waiting on the central */
	CHECK(ll_llcp_busy(L));
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)T(TIMEOUT_US));
	/* a second answer: nothing waits on the host any more */
	CHECK(cpr_reply(6, 9, 30, 400) == LL_ST_DISALLOWED);
	CHECK(ll_llcp_conn_param_neg_reply(L, 0x3B) == LL_ST_DISALLOWED);
	CHECK(tx.n == 1);
	rx(upd, sizeof(upd));
	CHECK(cn.upd_calls == 1 && cn.instant == 0x1234 && cn.p.interval == 9);
	CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
	CHECK(tx.n == 1 && cn.end_calls == 0);

	/* ---- PreferredPeriodicity of our RSP: the central's when 1..
	 * Interval_Max, else 1 ("shall be ... other than zero" when the
	 * interval is a range) or 0 for a fixed interval ---- */
	{
		static const struct {
			uint8_t c_pp;
			uint16_t h_min, h_max;
			uint8_t exp_pp;
		} v[] = {{4, 6, 9, 4}, {9, 6, 9, 9}, {10, 6, 9, 1}, {0, 6, 9, 1},
			 {0, 9, 9, 0}, {12, 9, 9, 0}, {3, 9, 9, 3}};

		for (unsigned int i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
			fresh();
			rx_cpr(6, 12, 30, 400, v[i].c_pp, 0x00AB);
			CHECK(cpr_reply(v[i].h_min, v[i].h_max, 30, 400) == LL_ST_SUCCESS);
			CHECK(last_is_cpr_rsp(v[i].h_min, v[i].h_max, 30, 400, v[i].exp_pp, 0x00AB));
		}
	}

	/* ---- Negative Reply: LL_REJECT_EXT_IND with the host's reason ---- */
	fresh();
	rx_cpr(6, 12, 30, 400, 0, 0);
	CHECK(ll_llcp_conn_param_neg_reply(L, LL_ST_UNACCEPT_CONN_PARAM) == LL_ST_SUCCESS);
	CHECK(tx.n == 1 && last_is_rej_ext(0x0F, 0x3B));
	CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
	CHECK(ll_llcp_conn_param_neg_reply(L, 0x3B) == LL_ST_DISALLOWED);
	CHECK(cpr_reply(6, 12, 30, 400) == LL_ST_DISALLOWED);
	CHECK(tx.n == 1);
	/* Zephyr rejects with Invalid LL Parameters: passed on as given */
	rx_cpr(6, 12, 30, 400, 0, 0);
	CHECK(ll_llcp_conn_param_neg_reply(L, LL_ST_INVALID_LL_PARAM) == LL_ST_SUCCESS);
	CHECK(tx.n == 2 && last_is_rej_ext(0x0F, 0x1E));
	CHECK(cpr_ev.calls == 2);

	/* ---- invalid parameters: LL_REJECT_EXT_IND(0x1E), the host is not
	 * asked, nothing waits ---- */
	{
		static const uint16_t bad[][4] = {
			{5, 12, 0, 400},       /* Interval_Min < 6 */
			{6, 3201, 0, 3200},    /* Interval_Max > 3200 */
			{13, 12, 0, 400},      /* Interval_Min > Interval_Max */
			{6, 12, 500, 3200},    /* Latency > 499 */
			{6, 12, 0, 9},         /* Timeout < 10 */
			{6, 12, 0, 3201},      /* Timeout > 3200 */
			{6, 12, 30, 93},       /* 930 ms <= 2 x 15 ms x 31 = 930 ms */
			{0, 0, 0, 0},
		};

		for (unsigned int i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
			fresh();
			rx_cpr(bad[i][0], bad[i][1], bad[i][2], bad[i][3], 0, 0);
			CHECK(tx.n == 1 && last_is_rej_ext(0x0F, 0x1E));
			CHECK(cpr_ev.calls == 0);
			CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
			CHECK(cpr_reply(6, 12, 0, 400) == LL_ST_DISALLOWED);
		}
	}
	/* the limits themselves are valid */
	{
		static const uint16_t good[][4] = {
			{6, 6, 0, 10},         /* 100 ms > 15 ms */
			{3200, 3200, 0, 3200}, /* 32 s > 8 s */
			{6, 12, 30, 94},       /* 940 ms > 930 ms */
			{6, 6, 499, 3200},     /* 32 s > 7.5 s */
		};

		for (unsigned int i = 0; i < sizeof(good) / sizeof(good[0]); i++) {
			fresh();
			rx_cpr(good[i][0], good[i][1], good[i][2], good[i][3], 0, 0);
			CHECK(tx.n == 0 && cpr_ev.calls == 1 && ll_llcp_busy(L));
		}
	}
	/* PreferredPeriodicity / offsets we do not use are not checked */
	fresh();
	{
		uint8_t pdu[CPR_LEN];

		build_cpr(pdu, 6, 12, 0, 400, 200, 0x0010, 50);
		rx(pdu, sizeof(pdu));
		CHECK(tx.n == 0 && cpr_ev.calls == 1);
	}
	/* a wrong length: LL_UNKNOWN_RSP, like any malformed request */
	fresh();
	{
		uint8_t pdu[CPR_LEN];
		static const uint8_t exp[2] = {0x07, 0x0F};

		build_cpr(pdu, 6, 12, 0, 400, 0, 0, 0xFFFF);
		rx(pdu, CPR_LEN - 1);
		CHECK(tx.n == 1 && last_is(exp, 2) && cpr_ev.calls == 0 && !ll_llcp_busy(L));
	}

	/* ---- the event is masked (or the host does not know the link): the
	 * Link Layer accepts with the central's values (5.1.7.2: not
	 * indicated, "proceed as if the Host has accepted") ---- */
	fresh();
	cpr_ev.masked = true;
	rx_cpr(8, 16, 4, 300, 8, 0x0042);
	CHECK(cpr_ev.calls == 1);
	CHECK(tx.n == 1 && last_is_cpr_rsp(8, 16, 4, 300, 8, 0x0042));
	CHECK(ll_llcp_busy(L));   /* until LL_CONNECTION_UPDATE_IND */
	CHECK(cpr_reply(8, 16, 4, 300) == LL_ST_DISALLOWED && tx.n == 1);
	rx(upd, sizeof(upd));
	CHECK(!ll_llcp_busy(L) && cn.upd_calls == 1);

	/* ---- only the anchor points move (same interval, latency and
	 * timeout): never indicated to the host (5.1.7.2), RSP at once ---- */
	fresh();
	ll_llcp_conn_params(L, 12, 30, 400);
	{
		uint8_t pdu[CPR_LEN];

		build_cpr(pdu, 12, 12, 30, 400, 0, 0x0100, 3);
		rx(pdu, sizeof(pdu));
	}
	CHECK(cpr_ev.calls == 0);
	CHECK(tx.n == 1 && last_is_cpr_rsp(12, 12, 30, 400, 0, 0x0100));
	CHECK(ll_llcp_busy(L));
	/* a changed timeout (or interval range, latency) is a real request */
	fresh();
	ll_llcp_conn_params(L, 12, 30, 400);
	rx_cpr(12, 12, 30, 500, 0, 0);
	CHECK(cpr_ev.calls == 1 && tx.n == 0);
	fresh();
	ll_llcp_conn_params(L, 12, 30, 400);
	rx_cpr(6, 12, 30, 400, 0, 0);
	CHECK(cpr_ev.calls == 1 && tx.n == 0);
	fresh();
	ll_llcp_conn_params(L, 12, 30, 400);
	rx_cpr(12, 12, 0, 400, 0, 0);
	CHECK(cpr_ev.calls == 1 && tx.n == 0);

	/* ---- collisions (5.3): the central starts an incompatible procedure
	 * (one with an instant) while one is in progress ---- */
	/* a second LL_CONNECTION_PARAM_REQ while the first waits on the host:
	 * same procedure, 0x23; the first goes on */
	fresh();
	rx_cpr(6, 12, 30, 400, 0, 0x0001);
	rx_cpr(6, 24, 0, 400, 0, 0x0002);
	CHECK(cpr_ev.calls == 1 && cpr_ev.req.interval_max == 12);
	CHECK(tx.n == 1 && last_is_rej_ext(0x0F, 0x23));
	CHECK(cpr_reply(6, 12, 30, 400) == LL_ST_SUCCESS);
	CHECK(tx.n == 2 && last_is_cpr_rsp(6, 12, 30, 400, 1, 0x0001));
	/* ... and while our RSP waits on the central's LL_CONNECTION_UPDATE_IND */
	rx_cpr(6, 24, 0, 400, 0, 0x0003);
	CHECK(cpr_ev.calls == 1 && tx.n == 3 && last_is_rej_ext(0x0F, 0x23));
	CHECK(ll_llcp_busy(L));
	rx(upd, sizeof(upd));
	CHECK(!ll_llcp_busy(L));
	/* a connection update instant still ahead: Connection Update vs.
	 * Connection Parameters Request, 0x23 */
	fresh();
	pend_inst[L] = LL_CONN_PENDING_UPDATE;
	rx_cpr(6, 12, 30, 400, 0, 0);
	CHECK(cpr_ev.calls == 0 && tx.n == 1 && last_is_rej_ext(0x0F, 0x23));
	CHECK(!ll_llcp_busy(L));
	/* a channel map instant ahead: a different procedure, 0x2A */
	fresh();
	pend_inst[L] = LL_CONN_PENDING_CHMAP;
	rx_cpr(6, 12, 30, 400, 0, 0);
	CHECK(cpr_ev.calls == 0 && tx.n == 1 && last_is_rej_ext(0x0F, 0x2A));
	pend_inst[L] = LL_CONN_PENDING_CHMAP | LL_CONN_PENDING_UPDATE;
	rx_cpr(6, 12, 30, 400, 0, 0);
	CHECK(cpr_ev.calls == 0 && tx.n == 2 && last_is_rej_ext(0x0F, 0x23));
	pend_inst[L] = 0;
	rx_cpr(6, 12, 30, 400, 0, 0);
	CHECK(cpr_ev.calls == 1 && tx.n == 2);
	/* the PHY Update procedure in progress (our LL_PHY_RSP waits on
	 * LL_PHY_UPDATE_IND): 0x2A; after the IND a request is fine */
	fresh();
	{
		static const uint8_t req[3] = {0x16, 0x01, 0x01};
		static const uint8_t ind[5] = {0x18, 0, 0, 0x34, 0x12};

		rx(req, 3);
		rx_cpr(6, 12, 30, 400, 0, 0);
		CHECK(cpr_ev.calls == 0 && tx.n == 2 && last_is_rej_ext(0x0F, 0x2A));
		rx(ind, 5);
		rx_cpr(6, 12, 30, 400, 0, 0);
		CHECK(cpr_ev.calls == 1 && tx.n == 2);
	}
	/* while the request waits on the host, the central's
	 * LL_CONNECTION_UPDATE_IND crosses it: applied, the procedure ends, a
	 * late host answer is refused and sends nothing */
	fresh();
	rx_cpr(6, 12, 30, 400, 0, 0);
	rx(upd, sizeof(upd));
	CHECK(cn.upd_calls == 1 && !ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
	CHECK(cpr_reply(6, 12, 30, 400) == LL_ST_DISALLOWED);
	CHECK(ll_llcp_conn_param_neg_reply(L, 0x3B) == LL_ST_DISALLOWED);
	CHECK(tx.n == 0);
	/* procedures without an instant are compatible: our LL_PING_REQ or
	 * LL_LENGTH_REQ waiting does not refuse the request */
	fresh();
	CHECK(ll_llcp_set_data_len(L, 100, LL_DLE_TIME_1M(100)) == LL_ST_SUCCESS);
	t0 = (uint32_t)tx.n;
	rx_cpr(6, 12, 30, 400, 0, 0);
	CHECK(cpr_ev.calls == 1 && (uint32_t)tx.n == t0);
	/* inside the encryption start the central "shall not" start another
	 * procedure (5.1.3.1): dropped like LL_PING_REQ, the host not asked */
	fresh();
	{
		uint8_t req[23];

		sample_rand();
		build_enc_req(req);
		rx(req, sizeof(req));
		CHECK(tx.n == 1);
		rx_cpr(6, 12, 30, 400, 0, 0);
		CHECK(tx.n == 1 && cpr_ev.calls == 0);
		CHECK(ll_llcp_ltk_reply(L, ltk) == LL_ST_SUCCESS);
		rx_cpr(6, 12, 30, 400, 0, 0);
		CHECK(tx.n == 2 && cpr_ev.calls == 0);
	}

	/* ---- the central refuses our RSP: LL_REJECT_EXT_IND naming 0x10 or
	 * 0x0F, or LL_UNKNOWN_RSP naming 0x10, ends the procedure ---- */
	{
		static const uint8_t ends[][3] = {{0x11, 0x10, 0x3B}, {0x11, 0x0F, 0x3B},
						  {0x07, 0x10, 0}};

		for (unsigned int i = 0; i < 3; i++) {
			fresh();
			rx_cpr(6, 12, 30, 400, 0, 0);
			CHECK(cpr_reply(6, 12, 30, 400) == LL_ST_SUCCESS);
			CHECK(ll_llcp_busy(L));
			rx(ends[i], ends[i][0] == 0x11 ? 3 : 2);
			CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
			CHECK(tx.n == 1 && cn.end_calls == 0);
		}
	}
	/* ... but not a reject naming another opcode */
	fresh();
	rx_cpr(6, 12, 30, 400, 0, 0);
	CHECK(cpr_reply(6, 12, 30, 400) == LL_ST_SUCCESS);
	{
		static const uint8_t rej[3] = {0x11, 0x14, 0x1A};
		static const uint8_t rej2[3] = {0x11, 0x16, 0x1A};
		static const uint8_t unk[2] = {0x07, 0x0C};

		rx(rej, 3);
		rx(rej2, 3);
		rx(unk, 2);
		CHECK(ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == (int32_t)T(TIMEOUT_US));
	}

	/* ---- procedure response timeout (5.2): no host answer within 40 s,
	 * or no LL_CONNECTION_UPDATE_IND within 40 s of our RSP: 0x22 ---- */
	fresh();
	t0 = now;
	rx_cpr(6, 12, 30, 400, 0, 0);
	ll_llcp_tick(t0 + T(TIMEOUT_US) - 1);
	CHECK(cn.end_calls == 0);
	ll_llcp_tick(t0 + T(TIMEOUT_US));
	CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_LMP_TIMEOUT);
	CHECK(!ll_llcp_busy(L));
	CHECK(cpr_reply(6, 12, 30, 400) == LL_ST_DISALLOWED);
	fresh();
	t0 = now;
	rx_cpr(6, 12, 30, 400, 0, 0);
	now = t0 + T(30000000);
	CHECK(cpr_reply(6, 12, 30, 400) == LL_ST_SUCCESS);
	ll_llcp_tick(t0 + T(TIMEOUT_US));
	CHECK(cn.end_calls == 0);
	ll_llcp_tick(now + T(TIMEOUT_US) - 1);
	CHECK(cn.end_calls == 0);
	ll_llcp_tick(now + T(TIMEOUT_US));
	CHECK(cn.end_calls == 1 && cn.end_reason == LL_ST_LMP_TIMEOUT);

	/* ---- a full backlog owes the RSP (slice 7), the state moves on ---- */
	fresh();
	rx_cpr(6, 12, 30, 400, 0, 0x0007);
	tx.fail = 1;
	CHECK(cpr_reply(6, 12, 30, 400) == LL_ST_SUCCESS);
	CHECK(tx.n == 0 && ll_llcp_busy(L));
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)T(LL_LLCP_RETRY_MS * 1000u));
	ll_llcp_retry(L);
	CHECK(tx.n == 1 && last_is_cpr_rsp(6, 12, 30, 400, 1, 0x0007));
	CHECK(ll_llcp_timeout_ticks(now) == (int32_t)T(TIMEOUT_US));
	/* the masked path owes it the same way */
	fresh();
	cpr_ev.masked = true;
	tx.fail = 1;
	rx_cpr(6, 12, 30, 400, 0, 0x0007);
	CHECK(tx.n == 0 && ll_llcp_busy(L));
	ll_llcp_retry(L);
	CHECK(tx.n == 1 && last_is_cpr_rsp(6, 12, 30, 400, 1, 0x0007));
	/* a reject is owed too */
	fresh();
	tx.fail = 1;
	rx_cpr(5, 12, 30, 400, 0, 0);
	CHECK(tx.n == 0);
	ll_llcp_retry(L);
	CHECK(tx.n == 1 && last_is_rej_ext(0x0F, 0x1E));

	/* ---- link checks and reset ---- */
	fresh();
	{
		const struct ll_llcp_cpr p = {6, 12, 30, 400};

		CHECK(ll_llcp_conn_param_reply(LL_MAX_CONN, &p) == LL_ST_UNKNOWN_CONN_ID);
		CHECK(ll_llcp_conn_param_neg_reply(LL_MAX_CONN, 0x3B) == LL_ST_UNKNOWN_CONN_ID);
		rx_cpr(6, 12, 30, 400, 0, 0);
		cn.active = false;
		CHECK(ll_llcp_conn_param_reply(L, &p) == LL_ST_UNKNOWN_CONN_ID);
		CHECK(ll_llcp_conn_param_neg_reply(L, 0x3B) == LL_ST_UNKNOWN_CONN_ID);
		cn.active = true;
		ll_llcp_reset(L);
		CHECK(!ll_llcp_busy(L) && ll_llcp_timeout_ticks(now) == -1);
		CHECK(ll_llcp_conn_param_reply(L, &p) == LL_ST_DISALLOWED);
		CHECK(tx.n == 0);
	}
	CHECK(locks == 0 && tx_locks == 0);
}

/* N >= 2: the procedure is per link */
static void test_cpr_per_link(void)
{
	const uint8_t a = 0, b = (uint8_t)(LL_MAX_CONN - 1);
	const struct ll_llcp_cpr p = {6, 12, 30, 400};
	uint8_t pdu[CPR_LEN];
	uint32_t t0;

	if (LL_MAX_CONN < 2) {
		return;
	}
	fresh();
	ll_llcp_reset(b);
	t0 = now;
	build_cpr(pdu, 6, 12, 30, 400, 0, 0, 0xFFFF);
	rx_l(a, pdu, sizeof(pdu));
	CHECK(cprl[a].calls == 1 && cprl[b].calls == 0);
	CHECK(ll_llcp_busy(a) && !ll_llcp_busy(b));
	/* b has no request: its answers are refused, a's stays */
	CHECK(ll_llcp_conn_param_reply(b, &p) == LL_ST_DISALLOWED);
	CHECK(ll_llcp_conn_param_neg_reply(b, 0x3B) == LL_ST_DISALLOWED);
	CHECK(tx.n == 0);
	/* b's own request, collisions judged per link */
	pend_inst[a] = LL_CONN_PENDING_UPDATE;
	rx_l(b, pdu, sizeof(pdu));
	CHECK(cprl[b].calls == 1 && tx.n == 0);
	CHECK(ll_llcp_conn_param_neg_reply(b, 0x3B) == LL_ST_SUCCESS);
	CHECK(tx.n == 1 && tx.p[0].link == b && tx.p[0].d[0] == 0x11);
	CHECK(ll_llcp_busy(a) && !ll_llcp_busy(b));
	/* a's timer ends only a */
	ll_llcp_tick(t0 + T(TIMEOUT_US));
	CHECK(cnl[a].end_calls == 1 && cnl[b].end_calls == 0);
	CHECK(locks == 0 && tx_locks == 0);
}

int main(void)
{
	for (int k = 0; k < 2; k++) {
		L = k == 0 ? 0 : (uint8_t)(LL_MAX_CONN - 1);
		single_link_suite();
		dle_responder();
		dle_initiator();
		dle_initiator_pending();
		acl_fragments();
		phy_procedure();
		push_retry_suite();
		push_retry_dle_phy();
		ping_suite();
		cpr_suite();
	}
	L = 0;
	dle_bounds();
	test_dle_per_link();
	test_link_bounds();
	test_enc_one_link_while_other_sends();
	test_link_end_isolated();
	test_timeouts_per_link();
	test_routing_per_link();
	test_ping_per_link();
	test_cpr_per_link();
	CHECK(locks == 0 && tx_locks == 0);
	DONE();
}
