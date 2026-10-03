/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Event arbiter (slice 6a): the only user of the main stimer alarm
 * (ll_sched_at / ll_sched_cancel). Every link (id 0..LL_MAX_CONN-1) and
 * advertising (LL_ARB_ADV) keep at most one request: their next planned
 * event. The arbiter keeps the accepted requests free of overlaps, arms the
 * main alarm for the earliest one, and calls the owner's start callback
 * when it fires. The guard alarm stays with ll_radio.
 *
 * Span of a request: [alarm_tick, open_tick + min_len_us]. Once started,
 * the request is "running" until its owner requests again or cancels
 * (ll_conn: at CONN_DONE; ll_adv: at the end of the adv event); a running
 * span ends at open_tick + max(min_len_us, cap_us), and it can neither be
 * displaced nor overlapped. If the alarm of a request fires while another
 * request is still running (an event overran its cap), that request is
 * displaced (bumped) instead of started.
 *
 * Collisions: a new request that overlaps accepted ones is refused if any
 * of them is running or has a higher priority, or has the same priority
 * while the requester did not yield at its last collision or the other one
 * did (round-robin: the one that yielded last wins ties). Otherwise all
 * the overlapped requests are displaced. At every collision the loser is
 * marked as having yielded, the winner as not.
 *
 * Context: ll_arb_request / ll_arb_cancel run with ll_plat_lock() held
 * (nesting ok, the callers may be in ISR context); the start callback runs
 * in the stimer ISR (without the lock), bumped in the context of the
 * displacing call. A bumped owner re-plans and requests again from inside
 * the callback; it never requests the event it was bumped from again.
 */
#ifndef LL_ARB_H_
#define LL_ARB_H_

#include <stdint.h>

#include "ll_defs.h"

#define LL_ARB_ADV LL_MAX_CONN                        /* requester id of advertising */
#define LL_ARB_IDS (LL_MAX_CONN + 1)

enum ll_arb_prio {                                    /* higher wins */
	LL_ARB_PRIO_ADV = 0,
	LL_ARB_PRIO_IDLE = 1,                         /* link without backlog */
	LL_ARB_PRIO_ACTIVE = 2,                       /* TX backlog or ops.busy */
	LL_ARB_PRIO_SUPERVISION = 3,                  /* next listen within 2 intervals of supervision timeout */
	LL_ARB_PRIO_MUST = 4,                         /* transmit-window event or instant event */
};

struct ll_arb_req {
	uint32_t alarm_tick;   /* when the owner must be called (RX open - lead) */
	uint32_t open_tick;    /* RX window open */
	uint32_t min_len_us;   /* from open_tick: first RX window + one exchange (adv: whole adv event) */
	uint32_t max_len_us;   /* owner's own cap (interval derived) */
	uint8_t prio;
};

struct ll_arb_ops {
	/* Main alarm of id's request fired (ISR). cap_us = max_len_us clipped so the event ends
	 * LL_CONN_EVENT_SAFETY_US + LL_CONN_ARM_LEAD_US before the next accepted request's open_tick. */
	void (*start)(uint8_t id, uint32_t cap_us);
	/* id's accepted request was displaced by a higher-priority one (same context as the displacing
	 * ll_arb_request call, ll_plat_lock held); the owner re-plans and requests again. */
	void (*bumped)(uint8_t id);
};

/* ops is copied. Drops every request and the main alarm. */
void ll_arb_init(const struct ll_arb_ops *ops);
/* Accept (0) or refuse (-EBUSY) id's next event. Overlap = [alarm_tick, open_tick + min_len_us]
 * intersects another accepted request's span. Refused if the other one has higher priority, or equal
 * priority and it did NOT yield at its last collision (round-robin: the one that yielded last wins
 * ties). Otherwise the other one is displaced (bumped). Replaces id's previous request. Arms the main
 * alarm (ll_sched_at) for the earliest accepted request. Caller holds ll_plat_lock. */
int ll_arb_request(uint8_t id, const struct ll_arb_req *r);
void ll_arb_cancel(uint8_t id);
/* Earliest tick >= from_tick at which a span [t - lead_us, t + len_us] overlaps no accepted request
 * (advertising uses it to slide into a gap). */
uint32_t ll_arb_gap(uint32_t from_tick, uint32_t lead_us, uint32_t len_us);

#endif /* LL_ARB_H_ */
