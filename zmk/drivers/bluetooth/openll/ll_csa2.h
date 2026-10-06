/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Channel Selection Algorithm #2, Core Spec Vol 6 Part B 4.5.8.3.
 *
 * Used on a connection when both the advertising PDU (our ADV_IND, ChSel 1)
 * and the CONNECT_IND have ChSel = 1. The channel of an event depends only
 * on the channel identifier, the event counter and the channel map, so
 * events skipped by peripheral latency or yielded to the arbiter need no
 * state stepping.
 */
#ifndef LL_CSA2_H_
#define LL_CSA2_H_

#include <stdint.h>

/* Channel Selection Algorithm #2 (Vol 6 Part B 4.5.8.3). chan_id = (AA >> 16) ^ (AA & 0xFFFF).
 * Stateless: returns the data channel (0..36) for event counter `counter` with channel map chm. */
uint16_t ll_csa2_chan_id(uint32_t aa);
uint8_t ll_csa2_channel(uint16_t chan_id, uint16_t counter, const uint8_t chm[5]);

#endif /* LL_CSA2_H_ */
