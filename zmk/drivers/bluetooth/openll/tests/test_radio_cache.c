/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_radio_cache host tests (slice 7 Task 2): which per-event connection
 * register writes ll_radio may skip, and that every other writer of those
 * registers forces the next connection event to write them again.
 */
#include "test.h"
#include "../ll_radio_cache.h"

#define AA1  0x71963c5au
#define CRC1 0x123456u
#define AA2  0x50654a4bu
#define CRC2 0x8e89beu

/* Bring the cache into "everything written for link 1". */
static void warm(struct ll_radio_cache *c)
{
	ll_radio_cache_reset(c);
	(void)ll_radio_cache_conn_regs(c);
	(void)ll_radio_cache_aa_crc(c, AA1, CRC1);
}

int main(void)
{
	struct ll_radio_cache c;

	/* after reset nothing is known: write everything, also for an AA/CRC
	 * of 0 (the zeroed cache must not match it) */
	ll_radio_cache_reset(&c);
	CHECK(ll_radio_cache_conn_regs(&c));
	CHECK(ll_radio_cache_aa_crc(&c, 0, 0));
	ll_radio_cache_reset(&c);
	CHECK(ll_radio_cache_aa_crc(&c, AA1, CRC1));

	/* the same link again, nothing else in between: skip both */
	warm(&c);
	CHECK(!ll_radio_cache_conn_regs(&c));
	CHECK(!ll_radio_cache_aa_crc(&c, AA1, CRC1));
	CHECK(!ll_radio_cache_conn_regs(&c));
	CHECK(!ll_radio_cache_aa_crc(&c, AA1, CRC1));

	/* another link (multilink): AA and CRC differ, write them; the
	 * connection-mode registers are shared and stay */
	CHECK(ll_radio_cache_aa_crc(&c, AA2, CRC2));
	CHECK(!ll_radio_cache_conn_regs(&c));
	CHECK(!ll_radio_cache_aa_crc(&c, AA2, CRC2));
	CHECK(ll_radio_cache_aa_crc(&c, AA1, CRC1));
	/* only one of the two differs: still a write */
	CHECK(ll_radio_cache_aa_crc(&c, AA1, CRC2));
	CHECK(ll_radio_cache_aa_crc(&c, AA2, CRC2));
	CHECK(ll_radio_cache_aa_crc(&c, AA1, CRC2));

	/* every writer of those registers outside the per-event path
	 * invalidates both groups */
	static const enum ll_radio_writer inval[] = {
		LL_RADIO_W_INIT,        /* ll_radio_init: adv baseband setup */
		LL_RADIO_W_ADV_CHANNEL, /* ll_radio_set_adv_channel: adv AA/CRC */
		LL_RADIO_W_ADV_TX,      /* stx2rx / stx: DMA0 source moved */
		LL_RADIO_W_ADV_ENTER,   /* adv AA/CRC, ll_ctrl_1, maxlen, IRQ mask */
		LL_RADIO_W_RESTORE,     /* baseband reset (guard or return to adv) */
		LL_RADIO_W_CONN_INIT,   /* full connection setup */
		LL_RADIO_W_QUIESCE,     /* IRQ mask cleared */
	};
	for (unsigned int i = 0; i < sizeof(inval) / sizeof(inval[0]); i++) {
		warm(&c);
		ll_radio_cache_note(&c, inval[i]);
		CHECK(ll_radio_cache_conn_regs(&c));
		CHECK(ll_radio_cache_aa_crc(&c, AA1, CRC1));
		/* and valid again once written */
		CHECK(!ll_radio_cache_conn_regs(&c));
		CHECK(!ll_radio_cache_aa_crc(&c, AA1, CRC1));
	}

	/* writers that keep the cached state valid: the SN/NESN init bits
	 * (ll_txq, per event) are carried over by conn_regs and are not part
	 * of the cached state */
	warm(&c);
	ll_radio_cache_note(&c, LL_RADIO_W_SN_NESN);
	CHECK(!ll_radio_cache_conn_regs(&c));
	CHECK(!ll_radio_cache_aa_crc(&c, AA1, CRC1));

	/* an unknown writer is treated as one that invalidates */
	warm(&c);
	ll_radio_cache_note(&c, (enum ll_radio_writer)0x7f);
	CHECK(ll_radio_cache_conn_regs(&c));
	CHECK(ll_radio_cache_aa_crc(&c, AA1, CRC1));

	/* an AA/CRC write alone does not make the connection-mode registers
	 * valid (adv_enter then a link with a fresh AA) */
	ll_radio_cache_reset(&c);
	CHECK(ll_radio_cache_aa_crc(&c, AA1, CRC1));
	CHECK(ll_radio_cache_conn_regs(&c));
	DONE();
}
