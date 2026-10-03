/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * One-shot alarms on the system timer. Callbacks run in ISR context.
 *
 * Two independent slots share the one stimer compare: the main alarm
 * (ll_sched_at, used by ll_adv / ll_conn to start events) and the guard
 * alarm (ll_sched_guard_at, used by ll_radio to end a connection event whose
 * completion IRQ never arrived). The compare is programmed with the earlier
 * of the two; a slot whose tick is already due when the other fires runs in
 * the same interrupt.
 */
#ifndef LL_SCHED_H_
#define LL_SCHED_H_

#include <stdint.h>

typedef void (*ll_sched_cb_t)(void);

void ll_sched_init(void);
/* Main alarm: replaces a pending main alarm. */
void ll_sched_at(uint32_t tick, ll_sched_cb_t cb);
void ll_sched_cancel(void);
/* Guard alarm: independent of the main alarm (device only, not faked in
 * host tests). */
void ll_sched_guard_at(uint32_t tick, ll_sched_cb_t cb);
void ll_sched_guard_cancel(void);
/* Before SoC poweroff (device only): drop both slots, mask the stimer compare
 * IRQ and its PLIC line. Irq-lock safe, no re-enable path. */
void ll_sched_quiesce(void);

#endif /* LL_SCHED_H_ */
