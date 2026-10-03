#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include "test.h"
#include "../ll_hci.h"
#include "../ll_defs.h"

static uint8_t evt[128];
static uint16_t evt_len;
static int reset_calls, unknown_calls;
static uint16_t unknown_op;
static struct ll_adv_params got_params;
static uint8_t got_data[40], got_data_len;
static int data_calls, enable_arg = -1;
static uint8_t next_status;

static void sink(const uint8_t *h4, uint16_t len) { memcpy(evt, h4, len); evt_len = len; }
static void get_addr(uint8_t a[6]) { static const uint8_t m[6] = {1, 2, 3, 4, 5, 6}; memcpy(a, m, 6); }
static void rnd(uint8_t *o, uint8_t n) { for (uint8_t i = 0; i < n; i++) o[i] = 0xA0 + i; }
static void reset(void) { reset_calls++; }
static uint8_t set_params(const struct ll_adv_params *p) { got_params = *p; return next_status; }
static uint8_t set_data(const uint8_t *d, uint8_t l) { memcpy(got_data, d, l); got_data_len = l; data_calls++; return 0; }
static uint8_t enable_status;
static uint8_t enable(bool e) { enable_arg = e; return enable_status; }
static void unknown(uint16_t op) { unknown_calls++; unknown_op = op; }
static int disc_calls, ltk_calls, neg_calls;
static uint8_t disc_reason, got_ltk[16];
static uint16_t got_handle = 0xFFFF;
/* handles the fake controller reports as active links (bit per handle) */
static uint32_t valid_mask = 0x1;
static uint8_t disconnect(uint16_t h, uint8_t r)
{
	disc_calls++; disc_reason = r; got_handle = h; return next_status;
}
static uint8_t ltk_reply(uint16_t h, const uint8_t ltk[16])
{
	ltk_calls++; memcpy(got_ltk, ltk, 16); got_handle = h; return next_status;
}
static uint8_t ltk_neg(uint16_t h) { neg_calls++; got_handle = h; return next_status; }
static bool handle_valid(uint16_t h) { return h < 32 && ((valid_mask >> h) & 1u); }

static const struct ll_hci_ops ops = {
	.get_bd_addr = get_addr, .rand = rnd, .reset = reset,
	.adv_set_params = set_params, .adv_set_data = set_data,
	.adv_set_scan_rsp = set_data, .adv_enable = enable, .unknown = unknown,
	.disconnect = disconnect, .ltk_reply = ltk_reply, .ltk_neg_reply = ltk_neg,
	.handle_valid = handle_valid,
};

static void cmd(uint16_t op, const uint8_t *p, uint8_t plen)
{
	uint8_t b[3 + 255];
	b[0] = op & 0xFF; b[1] = op >> 8; b[2] = plen;
	memcpy(&b[3], p, plen);
	evt_len = 0;
	ll_hci_cmd(b, 3 + plen);
}

/* evt layout: [0]=0x04 [1]=0x0E [2]=plen [3]=ncmd [4..5]=opcode [6]=status [7..]=ret */
static int is_cc(uint16_t op, uint8_t status)
{
	return evt_len >= 7 && evt[0] == 0x04 && evt[1] == 0x0E && evt[2] == evt_len - 3 &&
	       evt[3] == 1 && ll_get_le16(&evt[4]) == op && evt[6] == status;
}

/* Command Status: [0]=0x04 [1]=0x0F [2]=4 [3]=status [4]=ncmd [5..6]=opcode */
static int is_cs(uint16_t op, uint8_t status)
{
	return evt_len == 7 && evt[0] == 0x04 && evt[1] == 0x0F && evt[2] == 4 &&
	       evt[3] == status && evt[4] == 1 && ll_get_le16(&evt[5]) == op;
}

static void test_conn_cmds(void)
{
	/* Disconnect: handle 0, valid reason -> Command Status with the op status */
	uint8_t d[3] = {0x00, 0x00, 0x13};
	next_status = LL_ST_SUCCESS;
	disc_calls = 0;
	cmd(0x0406, d, 3);
	CHECK(is_cs(0x0406, LL_ST_SUCCESS));
	CHECK(disc_calls == 1 && disc_reason == 0x13);
	next_status = LL_ST_UNKNOWN_CONN_ID;
	cmd(0x0406, d, 3);
	CHECK(is_cs(0x0406, LL_ST_UNKNOWN_CONN_ID));
	CHECK(disc_calls == 2);
	/* every allowed reason passes */
	static const uint8_t ok_reasons[] = {0x05, 0x13, 0x14, 0x15, 0x1A, 0x29, 0x3B};
	next_status = LL_ST_SUCCESS;
	for (size_t i = 0; i < sizeof(ok_reasons); i++) {
		d[2] = ok_reasons[i];
		cmd(0x0406, d, 3);
		CHECK(is_cs(0x0406, LL_ST_SUCCESS));
		CHECK(disc_reason == ok_reasons[i]);
	}
	/* other reasons: Invalid HCI Command Parameters, op not called */
	disc_calls = 0;
	d[2] = 0x16;
	cmd(0x0406, d, 3);
	CHECK(is_cs(0x0406, LL_ST_INVALID_PARAM));
	d[2] = 0x00;
	cmd(0x0406, d, 3);
	CHECK(is_cs(0x0406, LL_ST_INVALID_PARAM));
	CHECK(disc_calls == 0);
	/* unknown handle */
	d[0] = 0x01; d[2] = 0x13;
	cmd(0x0406, d, 3);
	CHECK(is_cs(0x0406, LL_ST_UNKNOWN_CONN_ID));
	CHECK(disc_calls == 0);
	d[0] = 0x00;
	/* wrong length */
	cmd(0x0406, d, 2);
	CHECK(is_cs(0x0406, LL_ST_INVALID_PARAM));
	CHECK(disc_calls == 0);

	/* LE LTK Request Reply: Command Complete with status + handle */
	uint8_t r[18] = {0x00, 0x00};
	for (int i = 0; i < 16; i++) r[2 + i] = (uint8_t)(0x40 + i);
	next_status = LL_ST_SUCCESS;
	cmd(0x201A, r, 18);
	CHECK(is_cc(0x201A, LL_ST_SUCCESS));
	CHECK(evt_len == 9 && ll_get_le16(&evt[7]) == 0x0000);
	CHECK(ltk_calls == 1 && got_ltk[0] == 0x40 && got_ltk[15] == 0x4F);
	next_status = LL_ST_DISALLOWED;
	cmd(0x201A, r, 18);
	CHECK(is_cc(0x201A, LL_ST_DISALLOWED));
	CHECK(evt_len == 9 && ltk_calls == 2);
	r[0] = 0x05;
	cmd(0x201A, r, 18);
	CHECK(is_cc(0x201A, LL_ST_UNKNOWN_CONN_ID));
	CHECK(evt_len == 9 && ll_get_le16(&evt[7]) == 0x0005);
	CHECK(ltk_calls == 2);
	r[0] = 0x00;
	cmd(0x201A, r, 17);
	CHECK(is_cc(0x201A, LL_ST_INVALID_PARAM));
	CHECK(ltk_calls == 2);

	/* LE LTK Request Negative Reply */
	uint8_t h[2] = {0x00, 0x00};
	next_status = LL_ST_SUCCESS;
	cmd(0x201B, h, 2);
	CHECK(is_cc(0x201B, LL_ST_SUCCESS));
	CHECK(evt_len == 9 && ll_get_le16(&evt[7]) == 0x0000);
	CHECK(neg_calls == 1);
	h[1] = 0x01;
	cmd(0x201B, h, 2);
	CHECK(is_cc(0x201B, LL_ST_UNKNOWN_CONN_ID));
	CHECK(ll_get_le16(&evt[7]) == 0x0100 && neg_calls == 1);
	cmd(0x201B, h, 1);
	CHECK(is_cc(0x201B, LL_ST_INVALID_PARAM));
	CHECK(neg_calls == 1);
}

static void all_events_on(void)
{
	uint8_t m[8];

	memset(m, 0xFF, 8);
	cmd(0x0C01, m, 8);
	cmd(0x2001, m, 8);
}

static void test_events(void)
{
	all_events_on();

	/* LE Connection Complete */
	struct ll_connect_ind ci;
	memset(&ci, 0, sizeof(ci));
	for (int i = 0; i < 6; i++) ci.init_a[i] = (uint8_t)(0x10 + i);
	ci.init_addr_random = 1;
	ci.interval = 0x000C; ci.latency = 0x001E; ci.timeout = 0x0190; ci.sca = 5;
	evt_len = 0;
	ll_hci_evt_conn_complete(0, &ci);
	static const uint8_t cc[] = {0x04, 0x3E, 19, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01,
				     0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
				     0x0C, 0x00, 0x1E, 0x00, 0x90, 0x01, 0x05};
	CHECK(evt_len == sizeof(cc) && memcmp(evt, cc, sizeof(cc)) == 0);

	/* Disconnection Complete */
	evt_len = 0;
	ll_hci_evt_disconn_complete(0, 0x13);
	static const uint8_t dc[] = {0x04, 0x05, 4, 0x00, 0x00, 0x00, 0x13};
	CHECK(evt_len == sizeof(dc) && memcmp(evt, dc, sizeof(dc)) == 0);

	/* Number Of Completed Packets */
	evt_len = 0;
	ll_hci_evt_num_completed(0, 3);
	static const uint8_t nc[] = {0x04, 0x13, 5, 0x01, 0x00, 0x00, 0x03, 0x00};
	CHECK(evt_len == sizeof(nc) && memcmp(evt, nc, sizeof(nc)) == 0);
	evt_len = 0;
	ll_hci_evt_num_completed(0, 0);
	CHECK(evt_len == 0);    /* nothing to report */

	/* LE Long Term Key Request */
	static const uint8_t rnd8[8] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7};
	evt_len = 0;
	ll_hci_evt_ltk_req(0, rnd8, 0x1234);
	static const uint8_t lr[] = {0x04, 0x3E, 13, 0x05, 0x00, 0x00,
				     0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0x34, 0x12};
	CHECK(evt_len == sizeof(lr) && memcmp(evt, lr, sizeof(lr)) == 0);

	/* Encryption Change */
	evt_len = 0;
	ll_hci_evt_enc_change(0, LL_ST_SUCCESS, true);
	static const uint8_t ec[] = {0x04, 0x08, 4, 0x00, 0x00, 0x00, 0x01};
	CHECK(evt_len == sizeof(ec) && memcmp(evt, ec, sizeof(ec)) == 0);
	evt_len = 0;
	ll_hci_evt_enc_change(0, LL_ST_MIC_FAILURE, false);
	CHECK(evt_len == 7 && evt[3] == LL_ST_MIC_FAILURE && evt[6] == 0x00);

	/* LE Connection Update Complete */
	struct ll_conn_params p = {.interval = 6, .latency = 30, .timeout = 300};
	evt_len = 0;
	ll_hci_evt_conn_update(0, &p);
	static const uint8_t cu[] = {0x04, 0x3E, 10, 0x03, 0x00, 0x00, 0x00,
				     0x06, 0x00, 0x1E, 0x00, 0x2C, 0x01};
	CHECK(evt_len == sizeof(cu) && memcmp(evt, cu, sizeof(cu)) == 0);
}

static void test_event_masks(void)
{
	struct ll_connect_ind ci;
	struct ll_conn_params p = {.interval = 6, .latency = 0, .timeout = 100};
	static const uint8_t rnd8[8] = {0};
	uint8_t m[8];

	memset(&ci, 0, sizeof(ci));

	/* Reset restores the spec defaults: event mask 0x00001FFFFFFFFFFF
	 * (LE Meta, bit 61, is off), LE event mask 0x1F */
	cmd(0x0C03, NULL, 0);
	evt_len = 0;
	ll_hci_evt_conn_complete(0, &ci);
	CHECK(evt_len == 0);
	evt_len = 0;
	ll_hci_evt_disconn_complete(0, 0x08);
	CHECK(evt_len == 7);
	evt_len = 0;
	ll_hci_evt_enc_change(0, 0, true);
	CHECK(evt_len == 7);

	/* Zephyr-like mask: LE Meta on, Disconnection Complete off */
	memset(m, 0, 8);
	m[7] = 0x20;                       /* bit 61 LE Meta */
	m[0] = 0x80;                       /* bit 7 Encryption Change */
	cmd(0x0C01, m, 8);
	CHECK(is_cc(0x0C01, LL_ST_SUCCESS));
	evt_len = 0;
	ll_hci_evt_disconn_complete(0, 0x08);
	CHECK(evt_len == 0);
	evt_len = 0;
	ll_hci_evt_enc_change(0, 0, true);
	CHECK(evt_len == 7);
	evt_len = 0;
	ll_hci_evt_conn_complete(0, &ci);     /* LE default mask has bit 0 */
	CHECK(evt_len == 22);
	evt_len = 0;
	ll_hci_evt_num_completed(0, 1);       /* not maskable */
	CHECK(evt_len == 8);

	/* LE mask: only LTK Request (bit 4) */
	memset(m, 0, 8);
	m[0] = 0x10;
	cmd(0x2001, m, 8);
	evt_len = 0;
	ll_hci_evt_conn_complete(0, &ci);
	CHECK(evt_len == 0);
	evt_len = 0;
	ll_hci_evt_conn_update(0, &p);
	CHECK(evt_len == 0);
	evt_len = 0;
	ll_hci_evt_ltk_req(0, rnd8, 0);
	CHECK(evt_len == 16);
	m[0] = 0x05;                       /* Conn Complete + Conn Update Complete */
	cmd(0x2001, m, 8);
	evt_len = 0;
	ll_hci_evt_conn_update(0, &p);
	CHECK(evt_len == 13);
	evt_len = 0;
	ll_hci_evt_ltk_req(0, rnd8, 0);
	CHECK(evt_len == 0);

	all_events_on();
}

static void test_acl(void)
{
	struct ll_hci_acl_pdu pdu;
	uint8_t a[4 + 40];

	/* host -> LL: PB 0x00 (first, non-flushable) -> LLID 2 */
	a[0] = 0x00; a[1] = 0x00; a[2] = 5; a[3] = 0;
	for (int i = 0; i < 27; i++) a[4 + i] = (uint8_t)(0x60 + i);
	memset(&pdu, 0xEE, sizeof(pdu));
	CHECK(ll_hci_acl_from_host(a, 4 + 5, &pdu) == 0);
	CHECK(pdu.llid == LL_LLID_START && pdu.len == 5);
	CHECK(pdu.data[0] == 0x60 && pdu.data[4] == 0x64);
	/* PB 0x02 (first, flushable) -> LLID 2 */
	a[1] = 0x20;
	CHECK(ll_hci_acl_from_host(a, 4 + 5, &pdu) == 0 && pdu.llid == LL_LLID_START);
	/* PB 0x01 (continuation) -> LLID 1 */
	a[1] = 0x10;
	CHECK(ll_hci_acl_from_host(a, 4 + 5, &pdu) == 0 && pdu.llid == LL_LLID_CONT);
	/* 27 bytes ok, 28 rejected */
	a[1] = 0x00; a[2] = 27;
	CHECK(ll_hci_acl_from_host(a, 4 + 27, &pdu) == 0 && pdu.len == 27 && pdu.data[26] == 0x60 + 26);
	a[2] = 28;
	a[4 + 27] = 0;
	CHECK(ll_hci_acl_from_host(a, 4 + 28, &pdu) == -EINVAL);
	/* PB 0x03 (complete, BR/EDR only), broadcast flags, zero length, length mismatch */
	a[2] = 5;
	a[1] = 0x30;
	CHECK(ll_hci_acl_from_host(a, 4 + 5, &pdu) == -EINVAL);
	a[1] = 0x40;
	CHECK(ll_hci_acl_from_host(a, 4 + 5, &pdu) == -EINVAL);
	a[1] = 0x00; a[2] = 0;
	CHECK(ll_hci_acl_from_host(a, 4, &pdu) == -EINVAL);
	a[2] = 5;
	CHECK(ll_hci_acl_from_host(a, 4 + 4, &pdu) == -EINVAL);
	CHECK(ll_hci_acl_from_host(a, 3, &pdu) == -EINVAL);
	a[3] = 1;                          /* 16-bit length 0x0105 */
	CHECK(ll_hci_acl_from_host(a, 4 + 5, &pdu) == -EINVAL);
	a[3] = 0;
	CHECK(pdu.handle == 0);
	/* other handle (not an active link) */
	a[0] = 0x01;
	CHECK(ll_hci_acl_from_host(a, 4 + 5, &pdu) == -ENOTCONN);
	a[0] = 0x00; a[1] = 0x01;          /* handle 0x100 */
	CHECK(ll_hci_acl_from_host(a, 4 + 5, &pdu) == -ENOTCONN);

	/* LL -> host: H4 0x02, handle 0, PB 0x02 for LLID 2, 0x01 for LLID 1 */
	uint8_t out[LL_HCI_ACL_MAX];
	static const uint8_t pl[3] = {0x07, 0x00, 0x04};
	memset(out, 0xEE, sizeof(out));
	CHECK(ll_hci_acl_to_host(out, 0, LL_LLID_START, pl, 3) == 8);
	static const uint8_t st[] = {0x02, 0x00, 0x20, 0x03, 0x00, 0x07, 0x00, 0x04};
	CHECK(memcmp(out, st, sizeof(st)) == 0);
	CHECK(ll_hci_acl_to_host(out, 0, LL_LLID_CONT, pl, 3) == 8);
	CHECK(out[2] == 0x10);
	uint8_t big[27] = {0};
	CHECK(ll_hci_acl_to_host(out, 0, LL_LLID_START, big, 27) == LL_HCI_ACL_MAX);
	CHECK(out[3] == 27 && out[4] == 0);
	/* not ACL: control, reserved, empty, too long */
	CHECK(ll_hci_acl_to_host(out, 0, LL_LLID_CTRL, pl, 3) == 0);
	CHECK(ll_hci_acl_to_host(out, 0, 0, pl, 3) == 0);
	CHECK(ll_hci_acl_to_host(out, 0, LL_LLID_CONT, pl, 0) == 0);
	CHECK(ll_hci_acl_to_host(out, 0, LL_LLID_START, big, 28) == 0);
}

/* Slice 6a: handle == link id; commands reach the given handle, events and
 * ACL carry it, handles that are not an active link are refused. */
static void test_handles(void)
{
	const uint16_t n = LL_MAX_CONN;
	const uint16_t last = n - 1;
	uint8_t d[3] = {0x00, 0x00, 0x13};
	uint8_t r[18] = {0};
	uint8_t h[2] = {0};
	struct ll_hci_acl_pdu pdu;
	uint8_t a[4 + 5] = {0x00, 0x00, 5, 0, 1, 2, 3, 4, 5};
	uint8_t out[LL_HCI_ACL_MAX];
	static const uint8_t pl[3] = {0x07, 0x00, 0x04};

	all_events_on();
	valid_mask = (1u << n) - 1u;
	next_status = LL_ST_SUCCESS;

	/* Disconnect on the last handle (handle 2 with N = 3) reaches it */
	disc_calls = 0;
	ll_put_le16(d, last);
	cmd(0x0406, d, 3);
	CHECK(is_cs(0x0406, LL_ST_SUCCESS) && disc_calls == 1 && got_handle == last);
	/* the first handle past the links, and an inactive link */
	ll_put_le16(d, n);
	cmd(0x0406, d, 3);
	CHECK(is_cs(0x0406, LL_ST_UNKNOWN_CONN_ID) && disc_calls == 1);
	valid_mask &= ~(1u << last);
	ll_put_le16(d, last);
	cmd(0x0406, d, 3);
	CHECK(is_cs(0x0406, LL_ST_UNKNOWN_CONN_ID) && disc_calls == 1);
	valid_mask |= 1u << last;

	/* LTK reply / negative reply on each handle */
	for (uint16_t k = 0; k < n; k++) {
		ll_put_le16(r, k);
		ltk_calls = 0;
		cmd(0x201A, r, 18);
		CHECK(is_cc(0x201A, LL_ST_SUCCESS) && ll_get_le16(&evt[7]) == k);
		CHECK(ltk_calls == 1 && got_handle == k);
		ll_put_le16(h, k);
		neg_calls = 0;
		cmd(0x201B, h, 2);
		CHECK(is_cc(0x201B, LL_ST_SUCCESS) && ll_get_le16(&evt[7]) == k);
		CHECK(neg_calls == 1 && got_handle == k);
	}
	ll_put_le16(r, n);
	ltk_calls = 0;
	cmd(0x201A, r, 18);
	CHECK(is_cc(0x201A, LL_ST_UNKNOWN_CONN_ID) && ltk_calls == 0);

	/* events carry the handle */
	{
		struct ll_connect_ind ci;
		struct ll_conn_params p = {.interval = 6, .latency = 30, .timeout = 300};
		static const uint8_t rnd8[8] = {0};

		memset(&ci, 0, sizeof(ci));
		ll_hci_evt_conn_complete(last, &ci);
		CHECK(evt_len == 22 && evt[3] == 0x01 && ll_get_le16(&evt[5]) == last);
		ll_hci_evt_disconn_complete(last, 0x13);
		CHECK(evt_len == 7 && ll_get_le16(&evt[4]) == last && evt[6] == 0x13);
		ll_hci_evt_num_completed(last, 2);
		CHECK(evt_len == 8 && evt[3] == 1 && ll_get_le16(&evt[4]) == last &&
		      ll_get_le16(&evt[6]) == 2);
		ll_hci_evt_ltk_req(last, rnd8, 0x1234);
		CHECK(evt_len == 16 && evt[3] == 0x05 && ll_get_le16(&evt[4]) == last);
		ll_hci_evt_enc_change(last, LL_ST_SUCCESS, true);
		CHECK(evt_len == 7 && ll_get_le16(&evt[4]) == last && evt[6] == 0x01);
		ll_hci_evt_conn_update(last, &p);
		CHECK(evt_len == 13 && evt[3] == 0x03 && ll_get_le16(&evt[5]) == last);
	}

	/* ACL: host -> LL keeps the handle; an invalid one is -ENOTCONN */
	for (uint16_t k = 0; k < n; k++) {
		a[0] = (uint8_t)k;
		memset(&pdu, 0xEE, sizeof(pdu));
		CHECK(ll_hci_acl_from_host(a, sizeof(a), &pdu) == 0 && pdu.handle == k);
	}
	a[0] = (uint8_t)n;
	CHECK(ll_hci_acl_from_host(a, sizeof(a), &pdu) == -ENOTCONN && pdu.handle == n);
	valid_mask &= ~(1u << last);
	a[0] = (uint8_t)last;
	CHECK(ll_hci_acl_from_host(a, sizeof(a), &pdu) == -ENOTCONN && pdu.handle == last);
	/* -EINVAL still reports the handle (credit back) */
	a[1] = 0x30;
	CHECK(ll_hci_acl_from_host(a, sizeof(a), &pdu) == -EINVAL && pdu.handle == last);
	a[1] = 0x00;
	valid_mask = 0x1;

	/* LL -> host on the last handle */
	CHECK(ll_hci_acl_to_host(out, last, LL_LLID_START, pl, 3) == 8);
	CHECK(out[0] == 0x02 && ll_get_le16(&out[1]) == (uint16_t)(last | 0x2000));
	CHECK(ll_hci_acl_to_host(out, last, LL_LLID_CONT, pl, 3) == 8);
	CHECK(ll_get_le16(&out[1]) == (uint16_t)(last | 0x1000));
}

/* ll_hci_init without handle_valid fails loudly (assert -> abort) */
static void test_init_requires_handle_valid(void)
{
	struct ll_hci_ops bad = ops;
	pid_t pid;
	int st = 0;

	bad.handle_valid = NULL;
	fflush(stdout);
	pid = fork();
	if (pid == 0) {
		/* silence the assert message in the test log */
		if (freopen("/dev/null", "w", stderr) == NULL) {
			_exit(1);
		}
		ll_hci_init(&bad, sink);
		_exit(0);
	}
	CHECK(pid > 0 && waitpid(pid, &st, 0) == pid);
	CHECK(WIFSIGNALED(st) && WTERMSIG(st) == SIGABRT);
}

int main(void)
{
	test_init_requires_handle_valid();
	ll_hci_init(&ops, sink);

	/* Reset */
	cmd(0x0C03, NULL, 0);
	CHECK(is_cc(0x0C03, LL_ST_SUCCESS));
	CHECK(evt_len == 7);
	CHECK(reset_calls == 1);

	/* Set Event Mask / LE Set Event Mask accept 8-byte masks */
	uint8_t mask[8] = {0xFF};
	cmd(0x0C01, mask, 8);
	CHECK(is_cc(0x0C01, LL_ST_SUCCESS));
	cmd(0x2001, mask, 8);
	CHECK(is_cc(0x2001, LL_ST_SUCCESS));
	cmd(0x2001, mask, 4);
	CHECK(is_cc(0x2001, LL_ST_INVALID_PARAM));

	/* Read Local Version Information */
	cmd(0x1001, NULL, 0);
	CHECK(is_cc(0x1001, LL_ST_SUCCESS));
	CHECK(evt_len == 7 + 8);
	CHECK(evt[7] == LL_HCI_VERSION);
	CHECK(evt[10] == LL_HCI_VERSION);
	CHECK(ll_get_le16(&evt[11]) == LL_COMPANY_ID);

	/* Read Local Supported Features: LE supported, BR/EDR not supported */
	cmd(0x1003, NULL, 0);
	CHECK(is_cc(0x1003, LL_ST_SUCCESS));
	CHECK(evt_len == 7 + 8);
	CHECK(evt[7 + 4] == 0x60);

	/* Read Local Supported Commands */
	cmd(0x1002, NULL, 0);
	CHECK(is_cc(0x1002, LL_ST_SUCCESS));
	CHECK(evt_len == 7 + 64);
	CHECK(evt[7 + 5] & 0x80);          /* Reset */
	CHECK(evt[7 + 26] & 0x02);         /* LE Set Advertising Enable */
	CHECK(evt[7 + 27] & 0x80);         /* LE Rand */
	CHECK(!(evt[7 + 10] & 0x20));      /* no controller-to-host flow control */
	CHECK(!(evt[7 + 28] & 0x08));      /* no LE Read Supported States */
	CHECK(evt[7 + 0] & 0x20);          /* Disconnect */
	CHECK(evt[7 + 28] & 0x02);         /* LE LTK Request Reply */
	CHECK(evt[7 + 28] & 0x04);         /* LE LTK Request Negative Reply */
	CHECK(!(evt[7 + 28] & 0x01));      /* no LE Start Encryption (central only) */
	CHECK(!(evt[7 + 27] & 0x04));      /* no LE Connection Update */
	CHECK(!(evt[7 + 27] & 0x20));      /* no LE Read Remote Features */
	CHECK(!(evt[7 + 2] & 0x80));       /* no Read Remote Version Information */
	CHECK(!(evt[7 + 27] & 0x40));      /* no LE Encrypt */

	/* Read BD_ADDR */
	cmd(0x1009, NULL, 0);
	CHECK(is_cc(0x1009, LL_ST_SUCCESS));
	CHECK(evt_len == 7 + 6);
	CHECK(evt[7] == 1 && evt[12] == 6);

	/* LE Read Buffer Size */
	cmd(0x2002, NULL, 0);
	CHECK(is_cc(0x2002, LL_ST_SUCCESS));
	CHECK(ll_get_le16(&evt[7]) == LL_ACL_MTU && evt[9] == LL_ACL_NUM);

	/* LE Read Local Supported Features: byte 0 = LE Encryption + Extended Reject Indication */
	cmd(0x2003, NULL, 0);
	CHECK(is_cc(0x2003, LL_ST_SUCCESS));
	CHECK(evt_len == 7 + 8);
	CHECK(evt[7] == LL_FEATURES_LOW && evt[7] == 0x05);
	for (int i = 1; i < 8; i++) CHECK(evt[7 + i] == 0);

	/* LE Rand */
	cmd(0x2018, NULL, 0);
	CHECK(is_cc(0x2018, LL_ST_SUCCESS));
	CHECK(evt_len == 7 + 8);
	CHECK(evt[7] == 0xA0 && evt[14] == 0xA7);

	/* LE Set Advertising Parameters: parsed and forwarded, op status returned */
	uint8_t ap[15] = {0x30, 0x00, 0x60, 0x00, 0x00, 0x00, 0x01,
			  0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x07, 0x00};
	next_status = LL_ST_SUCCESS;
	cmd(0x2006, ap, 15);
	CHECK(is_cc(0x2006, LL_ST_SUCCESS));
	CHECK(got_params.interval_min == 0x30 && got_params.interval_max == 0x60);
	CHECK(got_params.type == 0 && got_params.peer_addr_type == 1);
	CHECK(got_params.peer_addr[0] == 0x11 && got_params.peer_addr[5] == 0x66);
	CHECK(got_params.chan_map == 7 && got_params.filter_policy == 0);
	next_status = LL_ST_UNSUPPORTED;
	cmd(0x2006, ap, 15);
	CHECK(is_cc(0x2006, LL_ST_UNSUPPORTED));
	cmd(0x2006, ap, 14);
	CHECK(is_cc(0x2006, LL_ST_INVALID_PARAM));

	/* LE Set Advertising Data: 32-byte parameter, first byte = length */
	uint8_t ad[32] = {3, 0x02, 0x01, 0x06};
	cmd(0x2008, ad, 32);
	CHECK(is_cc(0x2008, LL_ST_SUCCESS));
	CHECK(got_data_len == 3 && got_data[0] == 0x02 && got_data[2] == 0x06);
	ad[0] = 32;
	data_calls = 0;
	cmd(0x2008, ad, 32);
	CHECK(is_cc(0x2008, LL_ST_INVALID_PARAM));
	CHECK(data_calls == 0);

	/* LE Set Scan Response Data */
	uint8_t sr[32] = {2, 0x01, 0x09};
	cmd(0x2009, sr, 32);
	CHECK(is_cc(0x2009, LL_ST_SUCCESS));
	CHECK(got_data_len == 2);

	/* LE Set Advertising Enable */
	uint8_t one = 1, two = 2;
	cmd(0x200A, &one, 1);
	CHECK(is_cc(0x200A, LL_ST_SUCCESS));
	CHECK(enable_arg == 1);
	cmd(0x200A, &two, 1);
	CHECK(is_cc(0x200A, LL_ST_INVALID_PARAM));
	/* the controller's status reaches the host unchanged: Connection
	 * Limit Exceeded when every link is taken (slice 6a Task 6) */
	enable_status = LL_ST_CONN_LIMIT;
	cmd(0x200A, &one, 1);
	CHECK(is_cc(0x200A, 0x09) && evt_len == 7);
	enable_status = LL_ST_SUCCESS;

	/* Unknown opcode: Command Complete with 0x01, hook called */
	cmd(0x2017, NULL, 0);
	CHECK(is_cc(0x2017, LL_ST_UNKNOWN_CMD));
	CHECK(unknown_calls == 1 && unknown_op == 0x2017);

	/* Parameter length mismatch with actual packet length */
	uint8_t bad[3] = {0x03, 0x0C, 0x05};
	evt_len = 0;
	ll_hci_cmd(bad, 3);
	CHECK(is_cc(0x0C03, LL_ST_INVALID_PARAM));

	/* Runt packet: ignored */
	evt_len = 0;
	ll_hci_cmd(bad, 2);
	CHECK(evt_len == 0);

	test_conn_cmds();
	test_events();
	test_event_masks();
	test_acl();
	test_handles();

	DONE();
}
