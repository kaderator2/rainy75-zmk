#include <string.h>
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
static uint8_t enable(bool e) { enable_arg = e; return 0; }
static void unknown(uint16_t op) { unknown_calls++; unknown_op = op; }

static const struct ll_hci_ops ops = {
	.get_bd_addr = get_addr, .rand = rnd, .reset = reset,
	.adv_set_params = set_params, .adv_set_data = set_data,
	.adv_set_scan_rsp = set_data, .adv_enable = enable, .unknown = unknown,
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

int main(void)
{
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

	/* Read BD_ADDR */
	cmd(0x1009, NULL, 0);
	CHECK(is_cc(0x1009, LL_ST_SUCCESS));
	CHECK(evt_len == 7 + 6);
	CHECK(evt[7] == 1 && evt[12] == 6);

	/* LE Read Buffer Size */
	cmd(0x2002, NULL, 0);
	CHECK(is_cc(0x2002, LL_ST_SUCCESS));
	CHECK(ll_get_le16(&evt[7]) == LL_ACL_MTU && evt[9] == LL_ACL_NUM);

	/* LE Read Local Supported Features: none in slice 1 */
	cmd(0x2003, NULL, 0);
	CHECK(is_cc(0x2003, LL_ST_SUCCESS));
	CHECK(evt_len == 7 + 8);
	for (int i = 0; i < 8; i++) CHECK(evt[7 + i] == 0);

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

	DONE();
}
