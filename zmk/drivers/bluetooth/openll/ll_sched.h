/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Single one-shot alarm on the system timer. Callback runs in ISR context.
 */
#ifndef LL_SCHED_H_
#define LL_SCHED_H_

#include <stdint.h>

typedef void (*ll_sched_cb_t)(void);

void ll_sched_init(void);
void ll_sched_at(uint32_t tick, ll_sched_cb_t cb);
void ll_sched_cancel(void);

#endif /* LL_SCHED_H_ */
