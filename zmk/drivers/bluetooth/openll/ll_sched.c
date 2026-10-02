/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * One-shot alarm on the B91 system timer capture interrupt (PLIC source 1).
 * Zephyr's kernel clock uses the RISC-V machine timer, so the system timer
 * capture is free once the blob is gone.
 */
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include "stimer.h"
#include "ll_defs.h"
#include "ll_sched.h"

#define STIMER_IRQ        (IRQ_TO_L2(1) | 11)
#define MIN_LEAD_TICKS    (30 * LL_TICKS_PER_US)

static volatile ll_sched_cb_t pending;

static void stimer_isr(const void *arg)
{
	ll_sched_cb_t cb;

	ARG_UNUSED(arg);
	stimer_clr_irq_mask(FLD_SYSTEM_IRQ);
	stimer_clr_irq_status(FLD_SYSTEM_IRQ);
	cb = pending;
	pending = NULL;
	if (cb) {
		cb();
	}
}

void ll_sched_init(void)
{
	IRQ_CONNECT(STIMER_IRQ, 1, stimer_isr, NULL, 0);
	irq_enable(STIMER_IRQ);
}

void ll_sched_at(uint32_t tick, ll_sched_cb_t cb)
{
	unsigned int key = irq_lock();
	uint32_t now = stimer_get_tick();

	/* The capture compare would wait a full 268 s wrap for a past tick. */
	if ((int32_t)(tick - now) < MIN_LEAD_TICKS) {
		tick = now + MIN_LEAD_TICKS;
	}
	pending = cb;
	stimer_set_irq_capture(tick);
	stimer_clr_irq_status(FLD_SYSTEM_IRQ);
	stimer_set_irq_mask(FLD_SYSTEM_IRQ);
	irq_unlock(key);
}

void ll_sched_cancel(void)
{
	unsigned int key = irq_lock();

	stimer_clr_irq_mask(FLD_SYSTEM_IRQ);
	pending = NULL;
	irq_unlock(key);
}
