/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Channel Selection Algorithm #1, Core Spec Vol 6 Part B 4.5.8.2.
 */
#ifndef LL_CSA1_H_
#define LL_CSA1_H_

#include <stdint.h>

struct ll_csa1 {
	uint8_t hop;            /* hopIncrement from CONNECT_IND, 5..16 */
	uint8_t last_unmapped;  /* lastUnmappedChannel, 0 at connection start */
	uint8_t chm[5];         /* ChM, bit n = data channel n, LSB of chm[0] first */
	uint8_t used[37];       /* used channels in ascending order (remapping table) */
	uint8_t n_used;
};

/* Start a connection: last_unmapped = 0. Bits 37..39 of chm are ignored. */
void ll_csa1_init(struct ll_csa1 *c, uint8_t hop, const uint8_t chm[5]);
/* New channel map (LL_CHANNEL_MAP_IND at its instant); keeps last_unmapped. */
void ll_csa1_set_map(struct ll_csa1 *c, const uint8_t chm[5]);
/* Channel for the next connection event (0..36). Call once per event, in
 * order, including events skipped by latency. A map without used channels
 * (invalid, the spec requires at least 2) yields the unmapped channel. */
uint8_t ll_csa1_next(struct ll_csa1 *c);

#endif /* LL_CSA1_H_ */
