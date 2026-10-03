/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Channel Selection Algorithm #2, Core Spec Vol 6 Part B 4.5.8.3, written
 * from the spec text:
 *  - channelIdentifier = AA[31:16] XOR AA[15:0];
 *  - PERM: the bits of each octet of a 16-bit value are reversed;
 *  - MAM(a, b) = (17 * a + b) mod 2^16;
 *  - prn_e: start with counter XOR channelIdentifier, three rounds of
 *    PERM then MAM(., channelIdentifier), then XOR channelIdentifier;
 *  - unmappedChannel = prn_e mod 37; if that channel is unused,
 *    remappingIndex = floor(N * prn_e / 2^16) selects from the N used
 *    channels in ascending order.
 */
#include "ll_csa2.h"

#define LL_DATA_CHANNELS 37

static uint8_t rev8(uint8_t b)
{
	b = (uint8_t)(((b & 0xF0u) >> 4) | ((b & 0x0Fu) << 4));
	b = (uint8_t)(((b & 0xCCu) >> 2) | ((b & 0x33u) << 2));
	b = (uint8_t)(((b & 0xAAu) >> 1) | ((b & 0x55u) << 1));
	return b;
}

static uint16_t perm(uint16_t x)
{
	return (uint16_t)(((uint16_t)rev8((uint8_t)(x >> 8)) << 8) | rev8((uint8_t)x));
}

static uint16_t mam(uint16_t a, uint16_t b)
{
	return (uint16_t)(17u * a + b);
}

static int ch_used(const uint8_t chm[5], uint8_t ch)
{
	return (chm[ch >> 3] >> (ch & 7)) & 1;
}

uint16_t ll_csa2_chan_id(uint32_t aa)
{
	return (uint16_t)((aa >> 16) ^ (aa & 0xFFFFu));
}

uint8_t ll_csa2_channel(uint16_t chan_id, uint16_t counter, const uint8_t chm[5])
{
	uint16_t prn = (uint16_t)(counter ^ chan_id);
	uint8_t unmapped, n_used = 0, idx;

	for (int i = 0; i < 3; i++) {
		prn = mam(perm(prn), chan_id);
	}
	prn ^= chan_id;
	unmapped = (uint8_t)(prn % LL_DATA_CHANNELS);
	if (ch_used(chm, unmapped)) {
		return unmapped;
	}
	/* bits 37..39 are RFU: only channels 0..36 count */
	for (uint8_t ch = 0; ch < LL_DATA_CHANNELS; ch++) {
		n_used += (uint8_t)ch_used(chm, ch);
	}
	/* n_used == 0 is only a crash guard: the spec requires at least 2 used
	 * channels, and ll_conn / ll_llcp reject a map with fewer */
	if (n_used == 0) {
		return unmapped;
	}
	idx = (uint8_t)(((uint32_t)n_used * prn) >> 16);
	for (uint8_t ch = 0; ch < LL_DATA_CHANNELS; ch++) {
		if (ch_used(chm, ch) && idx-- == 0) {
			return ch;
		}
	}
	return unmapped; /* not reached */
}
