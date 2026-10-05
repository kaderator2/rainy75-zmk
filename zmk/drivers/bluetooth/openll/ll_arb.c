/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Event arbiter (rules in ll_arb.h). One slot per requester id; the main
 * alarm always targets the earliest accepted request that has not started.
 * Tick arithmetic is modulo 2^32 with signed differences: all requests lie
 * within one supervision timeout (at most 32 s) of each other.
 */
#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include "ll_arb.h"
#include "ll_conn.h"
#include "ll_defs.h"
#include "ll_plat.h"
#include "ll_sched.h"

#define US(t) ((uint32_t)(t) * LL_TICKS_PER_US)
/* the event of a started request must end this long before the next
 * request's RX opens (ll_arb_ops.start) */
#define CLIP_RESERVE_US (LL_CONN_EVENT_SAFETY_US + LL_CONN_ARM_LEAD_US)

struct slot {
	bool used;       /* accepted request */
	bool running;    /* started, the owner has not requested again yet */
	bool yielded;    /* lost its last collision (displaced, or ll_arb_yield) */
	uint32_t refused_by; /* ids that refused its last request (for ll_arb_yield) */
	struct ll_arb_req r;
	/* running: open_tick + max(min_len_us, cap_us + CLIP_RESERVE_US) */
	uint32_t run_end;
};

static struct slot slots[LL_ARB_IDS];
static struct ll_arb_stats astats[LL_ARB_IDS];

static void lost_to(int id, uint8_t prio)
{
	if (prio < LL_ARB_PRIOS) {
		astats[id].lost[prio]++;
	}
}
static struct ll_arb_ops ops;
static bool armed;
/* Displaced ids whose bumped callback is still due, and whether a dispatch
 * loop is running (see dispatch_bumps). */
static uint32_t pending_bumps;
static bool dispatching;

static void alarm_fired(void);

static uint32_t span_end(const struct slot *s)
{
	return s->running ? s->run_end : s->r.open_tick + US(s->r.min_len_us);
}

/* [a0, a1] and [b0, b1] intersect (touching counts) */
static bool overlap(uint32_t a0, uint32_t a1, uint32_t b0, uint32_t b1)
{
	return (int32_t)(b1 - a0) >= 0 && (int32_t)(a1 - b0) >= 0;
}

/* Earliest accepted request that has not started, -1: none. */
static int earliest(void)
{
	int best = -1;

	for (int i = 0; i < LL_ARB_IDS; i++) {
		const struct slot *s = &slots[i];

		if (s->used && !s->running &&
		    (best < 0 || (int32_t)(s->r.alarm_tick - slots[best].r.alarm_tick) < 0)) {
			best = i;
		}
	}
	return best;
}

static void arm(void)
{
	int i = earliest();

	if (i >= 0) {
		ll_sched_at(slots[i].r.alarm_tick, alarm_fired);
		armed = true;
	} else if (armed) {
		armed = false;
		ll_sched_cancel();
	}
}

static int running_other(int id)
{
	for (int i = 0; i < LL_ARB_IDS; i++) {
		if (i != id && slots[i].used && slots[i].running) {
			return i;
		}
	}
	return -1;
}

/* cap of the started request id: its max, clipped before the next one */
static uint32_t clip_cap(int id)
{
	const struct slot *s = &slots[id];
	uint32_t cap = s->r.max_len_us;

	for (int i = 0; i < LL_ARB_IDS; i++) {
		const struct slot *o = &slots[i];
		int32_t d;
		uint32_t lim;

		if (i == id || !o->used || o->running) {
			continue;
		}
		d = (int32_t)(o->r.open_tick - s->r.open_tick);
		if (d < 0) {
			continue;   /* cannot happen with non-overlapping spans */
		}
		lim = (uint32_t)d / LL_TICKS_PER_US;
		lim = lim > CLIP_RESERVE_US ? lim - CLIP_RESERVE_US : 0;
		if (lim < cap) {
			cap = lim;
		}
	}
	return cap;
}

/* Call bumped for every displaced id. The owners re-plan and request
 * again from inside the callback, and such a nested request may displace
 * others: it only adds them to pending_bumps, the outermost call's loop
 * calls them. So there is no recursion through the arbiter (the stack
 * depth is one owner callback, whatever the chain), and the chain is
 * finite because a displaced owner never requests the event it was bumped
 * from again and every owner's re-plan is bounded (ll_conn: latency window,
 * then at most YIELD_MAX yields; ll_adv: ADV_PLAN_TRIES rounds). Caller
 * holds ll_plat_lock. */
static void dispatch_bumps(void)
{
	if (dispatching) {
		return;
	}
	dispatching = true;
	while (pending_bumps != 0) {
		int i = __builtin_ctz(pending_bumps);

		pending_bumps &= ~(1u << i);
		if (ops.bumped) {
			ops.bumped((uint8_t)i);
		}
	}
	dispatching = false;
}

/* stimer ISR (ll_sched main slot) */
static void alarm_fired(void)
{
	unsigned int key = ll_plat_lock();
	int id = earliest();
	uint32_t cap;
	struct slot *s;

	armed = false;
	if (id < 0) {
		ll_plat_unlock(key);
		return;
	}
	s = &slots[id];
	if (running_other(id) >= 0) {
		/* the previous event overran into this one: it yields */
		lost_to(id, slots[running_other(id)].r.prio);
		s->used = false;
		s->yielded = true;
		arm();
		pending_bumps |= 1u << id;
		dispatch_bumps();
		ll_plat_unlock(key);
		return;
	}
	cap = clip_cap(id);
	if (cap < s->r.max_len_us) {
		astats[id].clipped++;
	}
	s->running = true;
	/* The event may use the cap; a request accepted while it runs must
	 * still open its RX the clipping reserve after it (the same distance
	 * clip_cap() keeps before an accepted request; ll_arb_request() tests
	 * a running span against the request's open, not its alarm). */
	s->run_end = s->r.open_tick + US(cap + CLIP_RESERVE_US > s->r.min_len_us ?
					 cap + CLIP_RESERVE_US : s->r.min_len_us);
	arm();
	ll_plat_unlock(key);
	if (ops.start) {
		ops.start((uint8_t)id, cap);
	}
}

void ll_arb_init(const struct ll_arb_ops *o)
{
	unsigned int key = ll_plat_lock();

	memset(slots, 0, sizeof(slots));
	memset(&ops, 0, sizeof(ops));
	pending_bumps = 0;
	dispatching = false;
	if (o) {
		ops = *o;
	}
	if (armed) {
		armed = false;
		ll_sched_cancel();
	}
	ll_plat_unlock(key);
}

int ll_arb_request(uint8_t id, const struct ll_arb_req *r)
{
	struct slot *me;
	uint32_t a0, a1;
	uint32_t victims = 0;
	bool collided = false;

	if (id >= LL_ARB_IDS) {
		return -EINVAL;
	}
	me = &slots[id];
	me->used = false;
	me->running = false;
	a0 = r->alarm_tick;
	a1 = r->open_tick + US(r->min_len_us);

	for (int i = 0; i < LL_ARB_IDS; i++) {
		struct slot *o = &slots[i];
		bool win;

		if (i == id || !o->used) {
			continue;
		}
		if (o->running) {
			/* A running span ends at cap + CLIP_RESERVE_US, the
			 * distance clip_cap() keeps before the next request's
			 * open. Judged by the open as well (one tick before
			 * run_end, so a request clip_cap() made room for fits
			 * exactly): its alarm may lie in the reserve, as it did
			 * for an accepted request. Judged by the alarm, the
			 * lead would count twice and a request accepted before
			 * the event started could not be re-requested while it
			 * runs (a kick or instant re-plan would lose it). */
			if (!overlap(r->open_tick, a1, o->r.alarm_tick, o->run_end - 1)) {
				continue;
			}
		} else if (!overlap(a0, a1, o->r.alarm_tick, span_end(o))) {
			continue;
		}
		collided = true;
		if (o->running || o->r.prio > r->prio) {
			win = false;
		} else if (o->r.prio < r->prio) {
			win = true;
		} else {
			/* tie: the one that yielded last wins */
			win = me->yielded && !o->yielded;
		}
		if (!win) {
			/* no flag changes here: the owner may still find another
			 * event (a probe); ll_arb_yield() commits the loss */
			me->refused_by = 1u << i;
			arm();
			return -EBUSY;
		}
		victims |= 1u << i;
	}
	me->used = true;
	me->r = *r;
	me->refused_by = 0;
	if (collided) {
		me->yielded = false;
	}
	for (int i = 0; i < LL_ARB_IDS; i++) {
		if (victims & (1u << i)) {
			slots[i].used = false;
			slots[i].yielded = true;
			lost_to(i, r->prio);
		}
	}
	arm();
	pending_bumps |= victims;
	dispatch_bumps();
	return 0;
}

void ll_arb_yield(uint8_t id)
{
	if (id >= LL_ARB_IDS) {
		return;
	}
	slots[id].yielded = true;
	for (int i = 0; i < LL_ARB_IDS; i++) {
		if (slots[id].refused_by & (1u << i)) {
			slots[i].yielded = false;
			lost_to(id, slots[i].used ? slots[i].r.prio : LL_ARB_PRIOS);
		}
	}
	slots[id].refused_by = 0;
}

void ll_arb_cancel(uint8_t id)
{
	if (id >= LL_ARB_IDS || !slots[id].used) {
		return;
	}
	slots[id].used = false;
	slots[id].running = false;
	arm();
}

void ll_arb_get_stats(uint8_t id, struct ll_arb_stats *s)
{
	unsigned int key = ll_plat_lock();

	if (id < LL_ARB_IDS) {
		*s = astats[id];
	} else {
		memset(s, 0, sizeof(*s));
	}
	ll_plat_unlock(key);
}

uint32_t ll_arb_gap(uint32_t from_tick, uint32_t lead_us, uint32_t len_us)
{
	uint32_t t = from_tick;
	bool moved = true;

	/* each span can push t at most once (t only grows past its end) */
	while (moved) {
		moved = false;
		for (int i = 0; i < LL_ARB_IDS; i++) {
			const struct slot *o = &slots[i];

			if (o->used && overlap(t - US(lead_us), t + US(len_us), o->r.alarm_tick,
					       span_end(o))) {
				t = span_end(o) + US(lead_us) + 1;
				moved = true;
			}
		}
	}
	return t;
}
