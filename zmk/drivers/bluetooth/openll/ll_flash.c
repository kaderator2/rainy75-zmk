/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Flash window (rules in ll_flash.h). The state is written in thread
 * context under ll_plat_lock() and read by the ISRs (ll_conn prepare,
 * ll_adv channel start), which the lock keeps out while it changes.
 */
#include "ll_conn.h"
#include "ll_flash.h"
#include "ll_radio.h"

static volatile bool active;
static struct ll_flash_stats stats;
/* ll_flash_open() has asked the current operation to wait at least once */
static bool waiting;

bool ll_flash_active(void)
{
	return active;
}

bool ll_flash_open(uint32_t now, uint32_t waited_us)
{
	bool ready = ll_conn_flash_ready(now);

	if (!ready && waited_us < LL_FLASH_WAIT_MAX_US) {
		stats.waits++;
		waiting = true;
		/* listen at the links' next events (no latency skip) */
		ll_conn_flash_kick();
		return false;
	}
	if (!ready) {
		stats.forced++;
	}
	if (waiting && waited_us > stats.wait_max_us) {
		stats.wait_max_us = waited_us;
	}
	waiting = false;
	stats.windows++;
	active = true;
	/* nothing may stay on air: end it now, with interrupts still off */
	ll_radio_flash_abort();
	return true;
}

void ll_flash_close(void)
{
	active = false;
}

void ll_flash_reset(void)
{
	active = false;
	waiting = false;
}

void ll_flash_get_stats(struct ll_flash_stats *s)
{
	*s = stats;
}
