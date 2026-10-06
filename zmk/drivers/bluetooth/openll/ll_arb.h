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
 * span ends at open_tick + max(min_len_us, cap_us + LL_CONN_EVENT_SAFETY_US
 * + LL_CONN_ARM_LEAD_US) (the clipping reserve, so a request accepted while
 * it runs still opens its RX that long after the event's cap, the distance
 * the cap keeps before an accepted request), and it can neither be
 * displaced nor overlapped. A new request is tested against a running span
 * by its open_tick (its lead may lie in the reserve, as it does for a
 * request accepted before the event started, which can therefore be
 * re-requested while the event runs); ll_arb_gap() still uses the whole
 * span. If the alarm of a request fires while another request is still
 * running (an event overran its cap), that request is displaced (bumped)
 * instead of started.
 *
 * Collisions: a new request that overlaps accepted ones is refused if any
 * of them is running or has a higher priority, or has the same priority
 * unless the requester yielded at its last collision and the other one did
 * not (round-robin: the one that yielded last wins ties). Otherwise all the
 * overlapped requests are displaced.
 *
 * Fairness flags ("yielded"): a displaced request's owner is marked as
 * having yielded and the displacing requester as not. A refusal changes no
 * flag by itself: the owner may still find another event (dodge, kick
 * probes). Only when it actually gives up the event it was refused for does
 * it call ll_arb_yield(), which marks it yielded and the requests that
 * refused it (at its last refusal) as not.
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
	/* yielded LL_CONN_STARVE_YIELDS events in a row */
	LL_ARB_PRIO_STARVING = 3,
	/* next listen within 2 intervals of the supervision timeout */
	LL_ARB_PRIO_SUPERVISION = 4,
	LL_ARB_PRIO_MUST = 5,                         /* transmit-window event or instant event */
};

struct ll_arb_req {
	uint32_t alarm_tick;   /* when the owner must be called (RX open - lead) */
	uint32_t open_tick;    /* RX window open */
	/* from open_tick: first RX window + one exchange (adv: whole adv event) */
	uint32_t min_len_us;
	uint32_t max_len_us;   /* owner's own cap (interval derived) */
	uint8_t prio;
};

struct ll_arb_ops {
	/* Main alarm of id's request fired (ISR). cap_us = max_len_us clipped so the event
	 * ends LL_CONN_EVENT_SAFETY_US + LL_CONN_ARM_LEAD_US before the next accepted
	 * request's open_tick. */
	void (*start)(uint8_t id, uint32_t cap_us);
	/* id's accepted request was displaced by a higher-priority one (same context as the
	 * displacing ll_arb_request call, ll_plat_lock held); the owner re-plans and requests
	 * again. */
	void (*bumped)(uint8_t id);
};

/* ops is copied. Drops every request and the main alarm. */
void ll_arb_init(const struct ll_arb_ops *ops);
/* Accept (0) or refuse (-EBUSY) id's next event. Overlap = [alarm_tick, open_tick + min_len_us]
 * intersects another accepted request's span. Refused if the other one is running or has a higher
 * priority, or has the same priority unless id yielded at its last collision and the other one did
 * not (round-robin: the one that yielded last wins ties). Otherwise the other one is displaced
 * (bumped, marked yielded; id is marked not yielded). A refusal only records the refusing requests
 * for ll_arb_yield(). Replaces id's previous request (also when refused: id then has none). Arms
 * the main alarm (ll_sched_at) for the earliest accepted request. Caller holds ll_plat_lock. */
int ll_arb_request(uint8_t id, const struct ll_arb_req *r);
/* id gives up the event of its last refused request (a yield, not a probe): id is marked
 * yielded, the requests that refused it are marked not yielded (fairness, see the file
 * header). Without a recorded refusal only id is marked. Caller holds ll_plat_lock. */
void ll_arb_yield(uint8_t id);
void ll_arb_cancel(uint8_t id);
/* Per requester id, cumulative since boot (ll_arb_init keeps them).
 * lost[p]: arbitration losses to a winner of priority p, one per lost
 * request: displaced by it, refused by it and then given up via
 * ll_arb_yield (a request is refused by one slot, the first it loses to;
 * probes that are not yielded do not count), or bumped at its alarm by a
 * running event of priority p. This is not ll_conn's collision count:
 * a displaced link that dodges to another event of its latency window lost
 * a request but yielded no event, and a start with a cap below the floor
 * (ll_arb_yield without a recorded refusal) yields an event that is not a
 * loss here but counts in clipped. For a yield, p is the current priority
 * of the slot that refused id's last request, and that last request may
 * have been a dodge probe rather than the event first planned. clipped:
 * starts whose cap was cut below max_len_us for a following request. */
#define LL_ARB_PRIOS 6
struct ll_arb_stats {
	uint32_t lost[LL_ARB_PRIOS];
	uint32_t clipped;
};
void ll_arb_get_stats(uint8_t id, struct ll_arb_stats *s);

/* Earliest tick >= from_tick at which a span [t - lead_us, t + len_us] overlaps no accepted request
 * (advertising uses it to slide into a gap). */
uint32_t ll_arb_gap(uint32_t from_tick, uint32_t lead_us, uint32_t len_us);

#endif /* LL_ARB_H_ */
