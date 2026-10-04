#include <string.h>
#include "test.h"
#include "../ll_pdu.h"
#include "../ll_defs.h"

static const uint8_t adva[6] = {0x01, 0x02, 0x03, 0x38, 0xC1, 0xA4};

/* Slice 6c: own address random (Own_Address_Type 1). AdvA is the random
 * address and TxAdd (header bit 6, Vol 6 Part B 2.3) is 1 in every PDU we
 * send; SCAN_REQ / CONNECT_IND are ours only with RxAdd (bit 7) 1 and that
 * AdvA. */
static const uint8_t rnda[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x4A}; /* RPA: top bits 01 */

static void test_random_adva(void)
{
	uint8_t out[LL_ADV_PDU_MAX];
	const uint8_t ad[3] = {0x02, 0x01, 0x06};
	uint8_t n;

	n = ll_pdu_build_adv(out, LL_PDU_ADV_IND, rnda, 1, ad, 3);
	CHECK(n == 11 && out[0] == (0x20 | 0x40) && out[1] == 9);   /* ChSel + TxAdd */
	CHECK(memcmp(&out[2], rnda, 6) == 0);
	n = ll_pdu_build_adv(out, LL_PDU_SCAN_RSP, rnda, 1, NULL, 0);
	CHECK(n == 8 && out[0] == (0x04 | 0x40));
	n = ll_pdu_build_adv(out, LL_PDU_ADV_NONCONN_IND, rnda, 1, ad, 3);
	CHECK(out[0] == (0x02 | 0x40));
	n = ll_pdu_build_adv(out, LL_PDU_ADV_SCAN_IND, rnda, 1, ad, 3);
	CHECK(out[0] == (0x06 | 0x40));
	/* any non-zero tx_add is 1 */
	n = ll_pdu_build_adv(out, LL_PDU_ADV_IND, rnda, 2, ad, 3);
	CHECK(out[0] == (0x20 | 0x40));

	/* SCAN_REQ: RxAdd must match our TxAdd */
	uint8_t req[14] = {0x43 | 0x80, 12, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
	memcpy(&req[8], rnda, 6);
	CHECK(ll_pdu_is_scan_req_for(req, 14, rnda, 1));
	CHECK(!ll_pdu_is_scan_req_for(req, 14, rnda, 0));   /* we advertise public */
	req[0] &= 0x7F;
	CHECK(!ll_pdu_is_scan_req_for(req, 14, rnda, 1));   /* RxAdd public */

	/* CONNECT_IND to the random AdvA (header 0x65 | RxAdd 0x80) */
	uint8_t ci_pdu[36] = {0xE5, 34, 0x11, 0x12, 0x13, 0x14, 0x15, 0xD6};
	struct ll_connect_ind ci;

	memcpy(&ci_pdu[8], rnda, 6);
	ci_pdu[2 + 22] = 24;
	memset(&ci, 0, sizeof(ci));
	CHECK(ll_pdu_parse_connect_ind(ci_pdu, 36, rnda, 1, &ci) == 0);
	CHECK(ci.interval == 24 && ci.init_addr_random == 1);
	CHECK(memcmp(ci.adv_a, rnda, 6) == 0 && ci.adv_addr_random == 1);
	CHECK(ll_pdu_parse_connect_ind(ci_pdu, 36, rnda, 0, &ci) == -1);  /* RxAdd 1, we public */
	CHECK(ll_pdu_parse_connect_ind(ci_pdu, 36, adva, 1, &ci) == -1);  /* other AdvA */
	ci_pdu[0] &= 0x7F;
	CHECK(ll_pdu_parse_connect_ind(ci_pdu, 36, rnda, 1, &ci) == -1);  /* RxAdd 0 */
	/* a CONNECT_IND to our public AdvA (RxAdd 0) while we advertise random */
	memcpy(&ci_pdu[8], adva, 6);
	CHECK(ll_pdu_parse_connect_ind(ci_pdu, 36, rnda, 1, &ci) == -1);
	(void)n;
}

int main(void)
{
	uint8_t out[LL_ADV_PDU_MAX];
	const uint8_t ad[3] = {0x02, 0x01, 0x06};

	/* ADV_IND: header type 0, TxAdd 0 (public), ChSel 1 (CSA#2,
	 * Vol 6 Part B 2.3.1); length 6 + 3 */
	uint8_t n = ll_pdu_build_adv(out, LL_PDU_ADV_IND, adva, 0, ad, 3);
	CHECK(n == 11);
	CHECK(out[0] == 0x20 && out[1] == 9);
	CHECK(memcmp(&out[2], adva, 6) == 0);
	CHECK(memcmp(&out[8], ad, 3) == 0);

	/* SCAN_RSP with empty data */
	n = ll_pdu_build_adv(out, LL_PDU_SCAN_RSP, adva, 0, NULL, 0);
	CHECK(n == 8 && out[0] == 0x04 && out[1] == 6);
	/* ChSel is RFU (0) in the other advertising PDUs */
	n = ll_pdu_build_adv(out, LL_PDU_ADV_NONCONN_IND, adva, 0, ad, 3);
	CHECK(n == 11 && out[0] == 0x02);
	n = ll_pdu_build_adv(out, LL_PDU_ADV_SCAN_IND, adva, 0, ad, 3);
	CHECK(n == 11 && out[0] == 0x06);

	/* SCAN_REQ: TxAdd=1 (random scanner), RxAdd=0, ScanA + AdvA */
	uint8_t req[14] = {0x43, 12, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
	memcpy(&req[8], adva, 6);
	CHECK(ll_pdu_is_scan_req_for(req, 14, adva, 0));
	req[13] ^= 1;
	CHECK(!ll_pdu_is_scan_req_for(req, 14, adva, 0));      /* other AdvA */
	req[13] ^= 1;
	req[0] |= 0x80;
	CHECK(!ll_pdu_is_scan_req_for(req, 14, adva, 0));      /* RxAdd random */
	req[0] &= 0x7F;
	req[1] = 11;
	CHECK(!ll_pdu_is_scan_req_for(req, 14, adva, 0));      /* bad length */
	req[1] = 12;
	req[0] = 0x45;
	CHECK(!ll_pdu_is_scan_req_for(req, 14, adva, 0));      /* not SCAN_REQ */

	/* CONNECT_IND: TxAdd=1, ChSel=1 -> header 0x65, length 34 */
	uint8_t ci_pdu[36] = {
		0x65, 34,
		0x11, 0x12, 0x13, 0x14, 0x15, 0xD6,   /* InitA */
		0, 0, 0, 0, 0, 0,                     /* AdvA (filled below) */
		0x78, 0x56, 0x34, 0x12,               /* AA */
		0xCC, 0xBB, 0xAA,                     /* CRCInit */
		0x02,                                 /* WinSize */
		0x05, 0x00,                           /* WinOffset */
		0x18, 0x00,                           /* Interval 24 = 30 ms */
		0x03, 0x00,                           /* Latency */
		0x48, 0x00,                           /* Timeout 72 = 720 ms */
		0xFF, 0xFF, 0xFF, 0xFF, 0x1F,         /* ChM */
		0xA7,                                 /* Hop 7, SCA 5 */
	};
	memcpy(&ci_pdu[8], adva, 6);
	struct ll_connect_ind ci;
	CHECK(ll_pdu_parse_connect_ind(ci_pdu, 36, adva, 0, &ci) == 0);
	CHECK(ci.init_a[0] == 0x11 && ci.init_a[5] == 0xD6);
	CHECK(ci.init_addr_random == 1 && ci.chsel == 1);
	CHECK(ci.aa == 0x12345678u);
	CHECK(ci.crc_init == 0xAABBCCu);
	CHECK(ci.win_size == 2 && ci.win_offset == 5);
	CHECK(ci.interval == 24 && ci.latency == 3 && ci.timeout == 72);
	CHECK(ci.chm[0] == 0xFF && ci.chm[4] == 0x1F);
	CHECK(ci.hop == 7 && ci.sca == 5);
	/* the local address the link was made to (slice 6c) */
	CHECK(memcmp(ci.adv_a, adva, 6) == 0 && ci.adv_addr_random == 0);

	ci_pdu[13] ^= 1;
	CHECK(ll_pdu_parse_connect_ind(ci_pdu, 36, adva, 0, &ci) == -1); /* not for us */
	ci_pdu[13] ^= 1;
	CHECK(ll_pdu_parse_connect_ind(ci_pdu, 35, adva, 0, &ci) == -1); /* truncated */

	test_random_adva();

	DONE();
}
