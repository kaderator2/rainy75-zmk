/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * One-shot alarms on the B91 system timer capture interrupt (PLIC source 1).
 * Zephyr's kernel clock uses the RISC-V machine timer, so the system timer
 * capture is free once the blob is gone.
 *
 * Two slots (main, guard) are multiplexed on the single capture compare:
 * the compare always holds the earliest pending tick. The interrupt runs
 * every slot that is due, then reprograms the compare for what is left.
 */
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include "stimer.h"
#include "ll_defs.h"
#include "ll_sched.h"

#define STIMER_IRQ        (IRQ_TO_L2(1) | 11)
#define MIN_LEAD_TICKS    (30 * LL_TICKS_PER_US)

enum { SLOT_MAIN, SLOT_GUARD, SLOT_N };

struct slot {
	ll_sched_cb_t cb;   /* NULL: idle */
	uint32_t tick;
};

static struct slot slots[SLOT_N];

/* irq locked. Program the compare for the earliest pending slot. */
static void program(void)
{
	uint32_t now = stimer_get_tick();
	bool any = false;
	uint32_t best = 0;

	for (int i = 0; i < SLOT_N; i++) {
		if (slots[i].cb && (!any || (int32_t)(slots[i].tick - best) < 0)) {
			best = slots[i].tick;
			any = true;
		}
	}
	if (!any) {
		stimer_clr_irq_mask(FLD_SYSTEM_IRQ);
		return;
	}
	/* The capture compare would wait a full 268 s wrap for a past tick. */
	if ((int32_t)(best - now) < MIN_LEAD_TICKS) {
		best = now + MIN_LEAD_TICKS;
	}
	stimer_set_irq_capture(best);
	stimer_clr_irq_status(FLD_SYSTEM_IRQ);
	stimer_set_irq_mask(FLD_SYSTEM_IRQ);
}

static void stimer_isr(const void *arg)
{
	ARG_UNUSED(arg);
	stimer_clr_irq_mask(FLD_SYSTEM_IRQ);
	stimer_clr_irq_status(FLD_SYSTEM_IRQ);

	for (int i = 0; i < SLOT_N; i++) {
		ll_sched_cb_t cb = slots[i].cb;

		/* Due: the compare fires at the earliest tick (or at the
		 * MIN_LEAD clamp, which is later), so a slot not yet due here
		 * is strictly later and is reprogrammed below. */
		if (cb && (int32_t)(slots[i].tick - stimer_get_tick()) <= 0) {
			slots[i].cb = NULL;
			cb();   /* may re-arm any slot (program() runs inside) */
		}
	}
	unsigned int key = irq_lock();

	program();
	irq_unlock(key);
}

void ll_sched_init(void)
{
	IRQ_CONNECT(STIMER_IRQ, 1, stimer_isr, NULL, 0);
	irq_enable(STIMER_IRQ);
}

static void slot_at(int i, uint32_t tick, ll_sched_cb_t cb)
{
	unsigned int key = irq_lock();

	slots[i].cb = cb;
	slots[i].tick = tick;
	program();
	irq_unlock(key);
}

static void slot_cancel(int i)
{
	unsigned int key = irq_lock();

	slots[i].cb = NULL;
	program();
	irq_unlock(key);
}

void ll_sched_at(uint32_t tick, ll_sched_cb_t cb)
{
	slot_at(SLOT_MAIN, tick, cb);
}

void ll_sched_cancel(void)
{
	slot_cancel(SLOT_MAIN);
}

void ll_sched_guard_at(uint32_t tick, ll_sched_cb_t cb)
{
	slot_at(SLOT_GUARD, tick, cb);
}

void ll_sched_guard_cancel(void)
{
	slot_cancel(SLOT_GUARD);
}
