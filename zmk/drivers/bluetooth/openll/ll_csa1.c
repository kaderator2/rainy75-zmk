/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Channel Selection Algorithm #1, Core Spec Vol 6 Part B 4.5.8.2:
 * unmappedChannel = (lastUnmappedChannel + hopIncrement) mod 37; if that
 * channel is unused, remappingIndex = unmappedChannel mod numUsedChannels
 * selects from the used channels in ascending order.
 */
#include "ll_csa1.h"

#define LL_DATA_CHANNELS 37

static int ch_used(const uint8_t chm[5], uint8_t ch)
{
	return (chm[ch >> 3] >> (ch & 7)) & 1;
}

void ll_csa1_set_map(struct ll_csa1 *c, const uint8_t chm[5])
{
	c->n_used = 0;
	for (uint8_t i = 0; i < 5; i++) {
		c->chm[i] = chm[i];
	}
	c->chm[4] &= 0x1F; /* bits 37..39 are RFU */
	for (uint8_t ch = 0; ch < LL_DATA_CHANNELS; ch++) {
		if (ch_used(c->chm, ch)) {
			c->used[c->n_used++] = ch;
		}
	}
}

void ll_csa1_skip(struct ll_csa1 *c, uint32_t n)
{
	c->last_unmapped = (uint8_t)((c->last_unmapped + (n % LL_DATA_CHANNELS) * c->hop) %
				     LL_DATA_CHANNELS);
}

void ll_csa1_init(struct ll_csa1 *c, uint8_t hop, const uint8_t chm[5])
{
	c->hop = hop;
	c->last_unmapped = 0;
	ll_csa1_set_map(c, chm);
}

uint8_t ll_csa1_next(struct ll_csa1 *c)
{
	uint8_t unmapped = (uint8_t)((c->last_unmapped + c->hop) % LL_DATA_CHANNELS);

	c->last_unmapped = unmapped;
	/* n_used == 0 is only a crash guard (no division by zero): the spec
	 * requires at least 2 used channels, and ll_conn / ll_llcp reject a
	 * CONNECT_IND or LL_CHANNEL_MAP_IND with fewer before a map gets here. */
	if (c->n_used == 0 || ch_used(c->chm, unmapped)) {
		return unmapped;
	}
	return c->used[unmapped % c->n_used];
}
