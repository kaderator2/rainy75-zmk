/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register-mode bookkeeping of ll_radio (pure, host-tested): the
 * advertising values of ll_ctrl_1 / rxtcrcpkt (snapshot) and whether the
 * one-time connection setup has run.
 *
 * - The snapshot is taken once, at the first connection setup, from the
 *   advertising registers, and is never taken again: later the registers
 *   may hold connection values (first-RX timeout, SN/NESN init bits).
 * - Connection mode is "set up" from ll_radio_conn_init() until the return
 *   to advertising after the last link (ll_radio_adv_restore). A baseband
 *   restore as stall recovery (adv guard) while links are live keeps it set
 *   up: the links' events re-select everything per event.
 * - Every adv channel after connection events writes the snapshot back
 *   (once there is one), so it never runs with connection values.
 */
#ifndef LL_RADIO_MODE_H_
#define LL_RADIO_MODE_H_

#include <stdbool.h>
#include <stdint.h>

struct ll_radio_mode {
	bool snap;          /* adv snapshot taken */
	bool conn;          /* connection setup done */
	uint8_t adv_ctrl1;
	uint8_t adv_rxtcrc;
};

void ll_radio_mode_reset(struct ll_radio_mode *m);
/* ll_radio_conn_init(): true if the full setup must run (then the
 * snapshot is taken from cur_* if there is none yet). */
bool ll_radio_mode_conn_init(struct ll_radio_mode *m, uint8_t cur_ctrl1, uint8_t cur_rxtcrc);
bool ll_radio_mode_conn_ready(const struct ll_radio_mode *m);
/* Baseband restore. keep_conn: stall recovery (connection setup stays if
 * it was done). Returns true with the adv values to write back. */
bool ll_radio_mode_restore(struct ll_radio_mode *m, bool keep_conn, uint8_t *ctrl1,
			   uint8_t *rxtcrc);
/* Adv channel after connection events: true with the adv values to write. */
bool ll_radio_mode_adv_regs(const struct ll_radio_mode *m, uint8_t *ctrl1, uint8_t *rxtcrc);

#endif /* LL_RADIO_MODE_H_ */
