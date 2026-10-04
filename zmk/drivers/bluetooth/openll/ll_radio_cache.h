/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Skip cache for the per-event connection register writes of ll_radio
 * (slice 7 Task 2, pure, host-tested).
 *
 * ll_radio writes two register groups before every BRX:
 * - the connection-mode registers (conn_regs: rxtcrcpkt with TX
 *   timestamps, ll_ctrl_1 connection value, DMA0 source = TX ring base,
 *   RX maxlen, IRQ mask), the same for every link;
 * - the link's access address and CRC init.
 * Each group is written only when the cache does not hold it as written.
 *
 * Correctness rule: the cache may only claim what ll_radio itself wrote,
 * and every other write of these registers invalidates it first
 * (ll_radio_cache_note). All writers except the per-event SN/NESN init bits
 * (which conn_regs carries over and which are not cached) invalidate both
 * groups; an unknown writer does too. A stale "valid" would leave an
 * advertising AA/CRC or adv-mode register on a connection event, so when in
 * doubt a writer invalidates.
 */
#ifndef LL_RADIO_CACHE_H_
#define LL_RADIO_CACHE_H_

#include <stdbool.h>
#include <stdint.h>

enum ll_radio_writer {
	LL_RADIO_W_INIT,         /* ll_radio_init: adv baseband setup */
	LL_RADIO_W_ADV_CHANNEL,  /* ll_radio_set_adv_channel: adv AA/CRC */
	LL_RADIO_W_ADV_TX,       /* stx2rx / stx: DMA0 source moved */
	LL_RADIO_W_ADV_ENTER,    /* ll_radio_adv_enter: adv registers */
	LL_RADIO_W_RESTORE,      /* baseband reset (adv guard or return to adv) */
	LL_RADIO_W_CONN_INIT,    /* full connection setup */
	LL_RADIO_W_QUIESCE,      /* IRQ mask cleared before deep sleep */
	LL_RADIO_W_SN_NESN,      /* ll_txq per-event SN/NESN init bits */
};

struct ll_radio_cache {
	bool conn;               /* connection-mode registers written */
	bool aa_crc;             /* aa_reg / crc_init written */
	uint32_t aa_reg;         /* as written to the register (byte-swapped) */
	uint32_t crc_init;
};

void ll_radio_cache_reset(struct ll_radio_cache *c);
/* Another writer touched the registers (see the enum). */
void ll_radio_cache_note(struct ll_radio_cache *c, enum ll_radio_writer w);
/* true: write the connection-mode registers now (they count as written
 * afterwards); false: they hold the connection values already. */
bool ll_radio_cache_conn_regs(struct ll_radio_cache *c);
/* true: write this AA/CRC now (cached afterwards); false: already there. */
bool ll_radio_cache_aa_crc(struct ll_radio_cache *c, uint32_t aa_reg, uint32_t crc_init);

#endif /* LL_RADIO_CACHE_H_ */
