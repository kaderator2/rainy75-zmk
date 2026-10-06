/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_credit host tests (slice 6a Task 6): the glue's per-handle Number Of
 * Completed Packets bookkeeping, credit back, connection generations.
 */
#include <string.h>
#include "test.h"
#include "../ll_credit.h"
#include "../ll_plat.h"

static int locks, lock_calls;
unsigned int ll_plat_lock(void) { locks++; lock_calls++; return 0; }
void ll_plat_unlock(unsigned int k) { (void)k; locks--; }

int main(void)
{
	uint32_t g0, g;

	ll_credit_init();
	for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
		CHECK(!ll_credit_up(k) && ll_credit_take(k) == 0 && ll_credit_gen(k) == 0);
	}
	/* out of range: nothing, no crash */
	ll_credit_open(LL_MAX_CONN);
	ll_credit_acked(LL_MAX_CONN);
	CHECK(!ll_credit_back(LL_MAX_CONN) && ll_credit_take(LL_MAX_CONN) == 0);
	CHECK(!ll_credit_up_gen(LL_MAX_CONN, &g) && g == 0);

	/* NOCP per handle: acks of one link never show up on another */
	for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
		ll_credit_open(k);
		CHECK(ll_credit_up(k) && ll_credit_gen(k) == 1);
	}
	for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
		for (uint8_t i = 0; i <= k; i++) {
			ll_credit_acked(k);
		}
	}
	for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
		CHECK(ll_credit_take(k) == (uint16_t)(k + 1));
		CHECK(ll_credit_take(k) == 0);
	}

	/* credit back: only while up, on that handle */
	CHECK(ll_credit_back(0));
	CHECK(ll_credit_take(0) == 1);
	if (LL_MAX_CONN > 1) {
		CHECK(ll_credit_close(1));
		CHECK(!ll_credit_back(1));
		CHECK(ll_credit_take(1) == 0);
		CHECK(ll_credit_back(0) && ll_credit_take(0) == 1);
	}

	/* no leak across connections on the same id: acks and credits of
	 * the old connection are dropped by close, and open clears what came
	 * after it (an ack racing the end) */
	ll_credit_acked(0);
	ll_credit_acked(0);
	CHECK(ll_credit_close(0));
	CHECK(!ll_credit_close(0));            /* second close: was not up */
	ll_credit_acked(0);                    /* late ISR ack of the old link */
	CHECK(ll_credit_take(0) == 0);         /* never reported while down */
	ll_credit_acked(0);
	g0 = ll_credit_gen(0);
	ll_credit_open(0);
	CHECK(ll_credit_gen(0) == g0 + 1);
	CHECK(ll_credit_take(0) == 0);

	/* up and generation are read in one lock section */
	lock_calls = 0;
	CHECK(ll_credit_up_gen(0, &g) && g == g0 + 1);
	CHECK(lock_calls == 1);
	ll_credit_close(0);
	CHECK(!ll_credit_up_gen(0, &g) && g == g0 + 1);

	/* the count saturates at 16 bits */
	ll_credit_open(0);
	for (uint32_t i = 0; i < 70000; i++) {
		ll_credit_acked(0);
	}
	CHECK(ll_credit_take(0) == UINT16_MAX);
	CHECK(locks == 0);
	DONE();
}
