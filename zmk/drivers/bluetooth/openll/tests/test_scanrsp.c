/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_scanrsp host tests (slice 7 Task 2 item 1): which received PDU gets
 * the prepared SCAN_RSP, and the STX trigger tick for a first bit on air
 * T_IFS after the request.
 */
#include <string.h>
#include "test.h"
#include "../ll_defs.h"
#include "../ll_pdu.h"
#include "../ll_scanrsp.h"

static const uint8_t adva[6] = {0x01, 0x02, 0x03, 0x38, 0xC1, 0xA4};
static const uint8_t other[6] = {0x01, 0x02, 0x03, 0x38, 0xC1, 0xA5};
static const uint8_t scana[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

/* SCAN_REQ: header (TxAdd = scanner, RxAdd = advertiser), len 12, ScanA, AdvA */
static uint8_t scan_req(uint8_t *p, const uint8_t a[6], uint8_t rx_add)
{
	p[0] = LL_PDU_SCAN_REQ | 0x40 | (uint8_t)(rx_add << 7);
	p[1] = 12;
	memcpy(&p[2], scana, 6);
	memcpy(&p[8], a, 6);
	return 14;
}

static void test_match(void)
{
	static const uint8_t data[3] = {2, 0x0a, 0x00};
	uint8_t rsp_pub[2 + 37], rsp_rnd[2 + 37], adv[2 + 37], req[2 + 37];
	uint8_t lp = ll_pdu_build_adv(rsp_pub, LL_PDU_SCAN_RSP, adva, 0, data, sizeof(data));
	uint8_t lr = ll_pdu_build_adv(rsp_rnd, LL_PDU_SCAN_RSP, adva, 1, NULL, 0);
	uint8_t la = ll_pdu_build_adv(adv, LL_PDU_ADV_IND, adva, 0, data, sizeof(data));
	uint8_t n;

	/* a SCAN_REQ for our public AdvA */
	n = scan_req(req, adva, 0);
	CHECK(ll_scanrsp_for_us(rsp_pub, lp, req, n));
	/* RxAdd must match TxAdd of our SCAN_RSP */
	CHECK(!ll_scanrsp_for_us(rsp_rnd, lr, req, n));
	n = scan_req(req, adva, 1);
	CHECK(ll_scanrsp_for_us(rsp_rnd, lr, req, n));
	CHECK(!ll_scanrsp_for_us(rsp_pub, lp, req, n));
	/* another advertiser's SCAN_REQ (AdvA differs in one byte) */
	n = scan_req(req, other, 0);
	CHECK(!ll_scanrsp_for_us(rsp_pub, lp, req, n));
	/* a CONNECT_IND for us: never answered with a SCAN_RSP */
	memset(req, 0, sizeof(req));
	req[0] = LL_PDU_CONNECT_IND | 0x40;
	req[1] = 34;
	memcpy(&req[2], scana, 6);
	memcpy(&req[8], adva, 6);
	CHECK(!ll_scanrsp_for_us(rsp_pub, lp, req, 36));
	/* a SCAN_REQ with a wrong length field or a short buffer */
	n = scan_req(req, adva, 0);
	req[1] = 13;
	CHECK(!ll_scanrsp_for_us(rsp_pub, lp, req, n + 1));
	n = scan_req(req, adva, 0);
	CHECK(!ll_scanrsp_for_us(rsp_pub, lp, req, n - 1));
	/* other PDU types carrying our AdvA at the same place */
	for (uint8_t t = 0; t < 16; t++) {
		if (t == LL_PDU_SCAN_REQ) {
			continue;
		}
		n = scan_req(req, adva, 0);
		req[0] = (uint8_t)((req[0] & 0xF0) | t);
		CHECK(!ll_scanrsp_for_us(rsp_pub, lp, req, n));
	}
	/* nothing prepared, or the prepared PDU is not a SCAN_RSP (an
	 * ADV_IND), or too short to hold an AdvA */
	n = scan_req(req, adva, 0);
	CHECK(!ll_scanrsp_for_us(NULL, 0, req, n));
	CHECK(!ll_scanrsp_for_us(adv, la, req, n));
	CHECK(!ll_scanrsp_for_us(rsp_pub, 7, req, n));
	CHECK(!ll_scanrsp_for_us(rsp_pub, lp, NULL, 0));
}

#define US(x) ((uint32_t)(x) * LL_TICKS_PER_US)

static void test_trigger(void)
{
	const uint32_t lead = US(LL_T_IFS_US - LL_SCANRSP_SETTLE_US - LL_SCANRSP_TX_PATH_US);
	uint32_t end = 100000, t = 0xdeadbeef;

	/* 150 = (trigger - end) + settle + TX path */
#ifndef LL_TEST_SETTLE_OVERRIDE
	/* default settle: the smallest TX settle hal_telink publishes for
	 * BLE 1M (ext_rf.h LL_SCAN_TX_SETTLE / LL_TX_STL_TIFS_1M = 63) */
	CHECK(LL_SCANRSP_SETTLE_US == 63);
	CHECK(lead == US(28));
#else
	CHECK(lead == US(150 - LL_SCANRSP_SETTLE_US - 59));
#endif
	CHECK(LL_SCANRSP_TX_PATH_US == 59);
	/* decided 15 us after the end: trigger 41 us after the end */
	CHECK(ll_scanrsp_trigger(end, end + US(15), &t) && t == end + lead);
	/* exactly the minimum lead is accepted, one tick less is not */
	t = 0;
	CHECK(ll_scanrsp_trigger(end, end + lead - US(LL_SCANRSP_MIN_LEAD_US), &t) &&
	      t == end + lead);
	t = 0x1234;
	CHECK(!ll_scanrsp_trigger(end, end + lead - US(LL_SCANRSP_MIN_LEAD_US) + 1, &t));
	CHECK(t == 0x1234);   /* untouched when refused */
	/* the trigger is already past */
	CHECK(!ll_scanrsp_trigger(end, end + lead + 5, &t));
	CHECK(!ll_scanrsp_trigger(end, end + US(1000), &t));
	CHECK(t == 0x1234);
	/* 32-bit tick wrap between the end and the trigger, and between now
	 * and the trigger */
	end = 0xFFFFFFFFu - US(10);
	CHECK(ll_scanrsp_trigger(end, end + US(5), &t) && t == end + lead && t < end);
	end = 0xFFFFFFFFu - US(50);
	CHECK(ll_scanrsp_trigger(end, end + US(20), &t) && t == end + lead);
	CHECK(!ll_scanrsp_trigger(end, end + US(40), &t));
	/* a request that "ends" in the future (bogus timestamp): the trigger
	 * would lie more than T_IFS ahead; refused instead of arming the FSM
	 * far ahead */
	end = 500000;
	CHECK(!ll_scanrsp_trigger(end, end - US(200), &t));
	CHECK(!ll_scanrsp_trigger(end, end - US(1000000), &t));
	/* a request that ended just now (decision at once) is fine */
	CHECK(ll_scanrsp_trigger(end, end, &t) && t == end + lead);
}

int main(void)
{
	test_match();
	test_trigger();
	DONE();
}
