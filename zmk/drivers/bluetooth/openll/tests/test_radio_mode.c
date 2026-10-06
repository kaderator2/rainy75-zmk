/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_radio_mode host tests (slice 6a Task 6 review): the advertising
 * register snapshot and the "connection mode set up" state of ll_radio
 * across connection setup, adv channels and baseband restores.
 */
#include <string.h>
#include "test.h"
#include "../ll_radio_mode.h"

#define ADV_CTRL1  0x40   /* advertising value of ll_ctrl_1 */
#define CONN_CTRL1 0x5b   /* connection value (first-RX timeout enabled, SN/NESN bits) */
#define ADV_RXTCRC 0x11
#define CONN_RXTCRC 0x31

int main(void)
{
	struct ll_radio_mode m;
	uint8_t c1, rx;

	ll_radio_mode_reset(&m);
	/* before any connection: nothing saved, adv_enter writes nothing
	 * (the register holds the adv value) */
	CHECK(!ll_radio_mode_adv_regs(&m, &c1, &rx));
	CHECK(!ll_radio_mode_restore(&m, false, &c1, &rx));

	/* first connection: full setup, snapshot of the adv values */
	CHECK(ll_radio_mode_conn_init(&m, ADV_CTRL1, ADV_RXTCRC));
	CHECK(ll_radio_mode_conn_ready(&m));
	/* a second link: no-op, no re-snapshot (the registers now hold
	 * connection values) */
	CHECK(!ll_radio_mode_conn_init(&m, CONN_CTRL1, CONN_RXTCRC));
	CHECK(ll_radio_mode_adv_regs(&m, &c1, &rx) && c1 == ADV_CTRL1 && rx == ADV_RXTCRC);

	/* adv guard fires while a link is live: the restore keeps the
	 * connection state and writes the adv snapshot back */
	CHECK(ll_radio_mode_restore(&m, true, &c1, &rx) && c1 == ADV_CTRL1 && rx == ADV_RXTCRC);
	CHECK(ll_radio_mode_conn_ready(&m));
	/* the next adv channel still writes the adv value (not the
	 * connection's first-RX timeout) */
	CHECK(ll_radio_mode_adv_regs(&m, &c1, &rx) && c1 == ADV_CTRL1);
	/* the next CONNECT_IND: conn_init stays a no-op, no re-snapshot */
	CHECK(!ll_radio_mode_conn_init(&m, CONN_CTRL1, CONN_RXTCRC));
	CHECK(ll_radio_mode_adv_regs(&m, &c1, &rx) && c1 == ADV_CTRL1 && rx == ADV_RXTCRC);

	/* the return to advertising after the last link (ll_radio_adv_restore):
	 * the next connection sets up again, the snapshot stays the adv one
	 * even if taken from connection-mode registers */
	CHECK(ll_radio_mode_restore(&m, false, &c1, &rx) && c1 == ADV_CTRL1);
	CHECK(!ll_radio_mode_conn_ready(&m));
	CHECK(ll_radio_mode_conn_init(&m, CONN_CTRL1, CONN_RXTCRC));
	CHECK(ll_radio_mode_adv_regs(&m, &c1, &rx) && c1 == ADV_CTRL1 && rx == ADV_RXTCRC);
	/* a guard restore without connection mode set up: nothing to keep */
	CHECK(ll_radio_mode_restore(&m, false, &c1, &rx));
	CHECK(ll_radio_mode_restore(&m, true, &c1, &rx));
	CHECK(!ll_radio_mode_conn_ready(&m));
	DONE();
}
