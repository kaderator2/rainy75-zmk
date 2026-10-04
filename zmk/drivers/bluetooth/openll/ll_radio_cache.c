/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Skip cache for the per-event connection register writes (rules in
 * ll_radio_cache.h).
 */
#include <string.h>

#include "ll_radio_cache.h"

void ll_radio_cache_reset(struct ll_radio_cache *c)
{
	memset(c, 0, sizeof(*c));
}

void ll_radio_cache_note(struct ll_radio_cache *c, enum ll_radio_writer w)
{
	if (w == LL_RADIO_W_SN_NESN) {
		return;   /* carried over by conn_regs, not cached */
	}
	/* every other (or unknown) writer: write both groups again */
	c->conn = false;
	c->aa_crc = false;
}

bool ll_radio_cache_conn_regs(struct ll_radio_cache *c)
{
	bool write = !c->conn;

	c->conn = true;
	return write;
}

bool ll_radio_cache_aa_crc(struct ll_radio_cache *c, uint32_t aa_reg, uint32_t crc_init)
{
	bool write = !c->aa_crc || c->aa_reg != aa_reg || c->crc_init != crc_init;

	c->aa_crc = true;
	c->aa_reg = aa_reg;
	c->crc_init = crc_init;
	return write;
}
