#include <string.h>
#include "test.h"
#include "../ll_csa1.h"

static const uint8_t all37[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
/* channels 0..9 unused, 10..36 used (27 channels) */
static const uint8_t no0to9[5] = {0x00, 0xFC, 0xFF, 0xFF, 0x1F};

/* Reference: Core Spec Vol 6 Part B 4.5.8.2 written out independently. */
static uint8_t ref_next(uint8_t *last, uint8_t hop, const uint8_t chm[5])
{
	uint8_t used[37], n = 0, unmapped;

	for (uint8_t ch = 0; ch < 37; ch++) {
		if (chm[ch >> 3] & (1u << (ch & 7))) {
			used[n++] = ch;
		}
	}
	unmapped = (uint8_t)((*last + hop) % 37);
	*last = unmapped;
	if (chm[unmapped >> 3] & (1u << (unmapped & 7))) {
		return unmapped;
	}
	return used[unmapped % n];
}

/* ll_csa1_skip(n) leaves the state n ll_csa1_next() calls would */
static void test_skip(void)
{
	static const uint32_t ns[] = {0, 1, 2, 36, 37, 38, 100, 1000, 4321, 65535};
	static const uint8_t no0to9[5] = {0x00, 0xFC, 0xFF, 0xFF, 0x1F};

	for (uint8_t hop = 5; hop <= 16; hop++) {
		for (unsigned int i = 0; i < sizeof(ns) / sizeof(ns[0]); i++) {
			struct ll_csa1 a, b;

			ll_csa1_init(&a, hop, no0to9);
			(void)ll_csa1_next(&a);
			(void)ll_csa1_next(&a);
			b = a;
			for (uint32_t k = 0; k < ns[i]; k++) {
				(void)ll_csa1_next(&a);
			}
			ll_csa1_skip(&b, ns[i]);
			CHECK(b.last_unmapped == a.last_unmapped);
			CHECK(ll_csa1_next(&b) == ll_csa1_next(&a));
		}
	}
}

int main(void)
{
	struct ll_csa1 c;
	uint8_t last;

	test_skip();

	/* all channels used: sequence is k*hop mod 37 */
	ll_csa1_init(&c, 5, all37);
	CHECK(c.n_used == 37);
	CHECK(ll_csa1_next(&c) == 5);
	CHECK(ll_csa1_next(&c) == 10);
	CHECK(ll_csa1_next(&c) == 15);
	for (int k = 4; k <= 100; k++) {
		CHECK(ll_csa1_next(&c) == (k * 5) % 37);
	}

	ll_csa1_init(&c, 13, all37);
	CHECK(ll_csa1_next(&c) == 13);
	CHECK(ll_csa1_next(&c) == 26);
	CHECK(ll_csa1_next(&c) == 2);   /* 39 mod 37 */
	CHECK(ll_csa1_next(&c) == 15);
	for (int k = 5; k <= 100; k++) {
		CHECK(ll_csa1_next(&c) == (k * 13) % 37);
	}

	/* channels 0..9 unused: remap = used[unmapped mod 27], used ascending 10..36 */
	ll_csa1_init(&c, 5, no0to9);
	CHECK(c.n_used == 27);
	CHECK(c.used[0] == 10 && c.used[26] == 36);
	CHECK(ll_csa1_next(&c) == 15);  /* unmapped 5 unused -> used[5] = 15 */
	CHECK(ll_csa1_next(&c) == 10);  /* unmapped 10 used */
	CHECK(ll_csa1_next(&c) == 15);
	CHECK(ll_csa1_next(&c) == 20);
	CHECK(ll_csa1_next(&c) == 25);
	CHECK(ll_csa1_next(&c) == 30);
	CHECK(ll_csa1_next(&c) == 35);
	CHECK(ll_csa1_next(&c) == 13);  /* unmapped 3 -> used[3] = 13 */
	CHECK(c.last_unmapped == 3);    /* last_unmapped is the unmapped channel */
	CHECK(ll_csa1_next(&c) == 18);  /* unmapped 8 -> used[8] = 18 */

	/* long run against the independent reference, hop 13 */
	ll_csa1_init(&c, 13, no0to9);
	last = 0;
	for (int k = 0; k < 500; k++) {
		CHECK(ll_csa1_next(&c) == ref_next(&last, 13, no0to9));
	}

	/* set_map mid-sequence keeps last_unmapped */
	ll_csa1_init(&c, 7, all37);
	CHECK(ll_csa1_next(&c) == 7);
	CHECK(ll_csa1_next(&c) == 14);
	ll_csa1_set_map(&c, no0to9);
	CHECK(c.last_unmapped == 14);
	CHECK(c.n_used == 27);
	CHECK(ll_csa1_next(&c) == 21);  /* unmapped 21 used */
	CHECK(ll_csa1_next(&c) == 28);
	CHECK(ll_csa1_next(&c) == 35);
	CHECK(ll_csa1_next(&c) == 15);  /* unmapped 5 -> used[5] = 15 */
	ll_csa1_set_map(&c, all37);
	CHECK(c.last_unmapped == 5);
	CHECK(ll_csa1_next(&c) == 12);
	CHECK(memcmp(c.chm, all37, 5) == 0);

	/* sparse map: only channels 1 and 36 used */
	{
		static const uint8_t two[5] = {0x02, 0x00, 0x00, 0x00, 0x10};

		ll_csa1_init(&c, 5, two);
		CHECK(c.n_used == 2);
		CHECK(ll_csa1_next(&c) == 36);  /* 5 mod 2 = 1 -> used[1] */
		CHECK(ll_csa1_next(&c) == 1);   /* 10 mod 2 = 0 -> used[0] */
	}

	/* bits 37..39 of the ChM are RFU and must be ignored */
	{
		static const uint8_t rfu[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

		ll_csa1_init(&c, 5, rfu);
		CHECK(c.n_used == 37);
	}

	DONE();
}
