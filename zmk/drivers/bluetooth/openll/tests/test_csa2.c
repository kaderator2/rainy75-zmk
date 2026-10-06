/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_csa2: Channel Selection Algorithm #2 (Core Spec Vol 6 Part B 4.5.8.3).
 *
 * Sample data: Core Spec Vol 6 Part C 3 (3.1 all channels used, 3.2 nine
 * channels used), Access Address 0x8E89BED6, channel identifier 0x305F.
 * The published values were taken from the open test vectors that quote
 * that section: Apache NimBLE nimble/controller/test/src/ble_ll_csa2_test.c
 * ("based on sample data from CoreSpec 5.0 Vol 6 Part C 3.1 / 3.2") and
 * Zephyr subsys/bluetooth/controller/ll_sw/lll_chan.c lll_chan_sel_2_ut()
 * (both Apache-2.0; only the numbers are used, no code).
 */
#include <string.h>
#include "test.h"
#include "../ll_csa2.h"

static const uint8_t all37[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
/* 3.2: channels 9, 10, 21, 22, 23, 33, 34, 35, 36 */
static const uint8_t nine[5] = {0x00, 0x06, 0xE0, 0x00, 0x1E};
/* channels 1 and 2 only (the minimum of 2 used channels) */
static const uint8_t two[5] = {0x06, 0x00, 0x00, 0x00, 0x00};

static int used(const uint8_t chm[5], uint8_t ch)
{
	return ch < 37 && ((chm[ch >> 3] >> (ch & 7)) & 1);
}

/* every counter gives a used channel; returns how often each was hit */
static void sweep(const uint8_t chm[5], uint32_t hits[37])
{
	memset(hits, 0, 37 * sizeof(hits[0]));
	for (uint32_t n = 0; n < 65536; n++) {
		uint8_t ch = ll_csa2_channel(0x305F, (uint16_t)n, chm);

		CHECK(used(chm, ch));
		if (ch < 37) {
			hits[ch]++;
		}
	}
}

int main(void)
{
	uint32_t hits[37];
	uint8_t rfu[5];

	/* channel identifier: (AA >> 16) ^ (AA & 0xFFFF) */
	CHECK(ll_csa2_chan_id(0x8E89BED6u) == 0x305F);
	CHECK(ll_csa2_chan_id(0x12345678u) == (0x1234 ^ 0x5678));
	CHECK(ll_csa2_chan_id(0xFFFF0000u) == 0xFFFF);

	/* Vol 6 Part C 3.1, all 37 channels used */
	CHECK(ll_csa2_channel(0x305F, 0, all37) == 25);
	CHECK(ll_csa2_channel(0x305F, 1, all37) == 20);
	CHECK(ll_csa2_channel(0x305F, 2, all37) == 6);
	CHECK(ll_csa2_channel(0x305F, 3, all37) == 21);

	/* Vol 6 Part C 3.2, 9 channels used (remapping) */
	CHECK(ll_csa2_channel(0x305F, 6, nine) == 23);
	CHECK(ll_csa2_channel(0x305F, 7, nine) == 9);
	CHECK(ll_csa2_channel(0x305F, 8, nine) == 34);

	/* stateless: the order of the calls does not matter (latency skips,
	 * arbiter yields and re-plans just ask for another counter) */
	(void)ll_csa2_channel(0x305F, 40000, nine);
	(void)ll_csa2_channel(0x1234, 7, all37);
	CHECK(ll_csa2_channel(0x305F, 7, nine) == 9);
	CHECK(ll_csa2_channel(0x305F, 1, all37) == 20);

	/* bits 37..39 of the map are RFU and ignored */
	memcpy(rfu, nine, 5);
	rfu[4] |= 0xE0;
	CHECK(ll_csa2_channel(0x305F, 6, rfu) == 23);
	CHECK(ll_csa2_channel(0x305F, 7, rfu) == 9);
	CHECK(ll_csa2_channel(0x305F, 8, rfu) == 34);
	memcpy(rfu, two, 5);
	rfu[4] = 0xE0;
	for (uint32_t n = 0; n < 4096; n++) {
		uint8_t ch = ll_csa2_channel(0xBEEF, (uint16_t)n, rfu);

		CHECK(ch == 1 || ch == 2);
	}

	/* the whole counter space: always a used channel, every used channel
	 * is reached, roughly uniform (65536 / 37 = 1771 per channel) */
	sweep(all37, hits);
	for (int ch = 0; ch < 37; ch++) {
		CHECK(hits[ch] > 1400 && hits[ch] < 2150);
	}
	sweep(nine, hits);
	for (int ch = 0; ch < 37; ch++) {
		CHECK(used(nine, (uint8_t)ch) ? hits[ch] > 5000 : hits[ch] == 0);
	}
	sweep(two, hits);
	CHECK(hits[1] > 20000 && hits[2] > 20000 && hits[1] + hits[2] == 65536);

	/* a different channel identifier gives a different sequence */
	{
		int same = 0;

		for (uint16_t n = 0; n < 64; n++) {
			same += ll_csa2_channel(0x305F, n, all37) ==
				ll_csa2_channel(0x305E, n, all37);
		}
		CHECK(same < 16);
	}
	DONE();
}
