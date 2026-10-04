/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Connection state machine, peripheral role, LL_MAX_CONN links (slice 6a),
 * CSA#1 / CSA#2 (4.5.8.2 / 4.5.8.3, per link from the CONNECT_IND ChSel),
 * peripheral latency (rules in ll_conn.h). Core Spec Vol 6 Part B: 4.5.1 connection events,
 * 4.5.2 supervision (6 events before the first packet), 4.5.3 transmit
 * window, 4.5.4 window widening, 4.5.5 connection setup, 5.1.1 connection
 * update, 5.1.2 channel map update, 5.1.3 termination.
 *
 * Timing model: event n has its anchor at ref_tick + (n - ref_counter) *
 * interval. After a packet was received, ref is the last received anchor
 * and win_us is 0. Before the first packet and after a connection update
 * until the first packet, ref is the transmit window start and win_us the
 * window size (a missed window repeats one interval later). Widening grows
 * with the time since sync_tick, the last anchor actually received (or the
 * CONNECT_IND end).
 *
 * Per event: alarm LL_CONN_ARM_LEAD_US before the RX opens -> prepare():
 * ll_radio_conn_select() (the link's AA and CRC init), ll_txq_event_start()
 * (ring rebuilt for this link), BRX via ll_radio_conn_event(); the link
 * becomes the event owner (ev_owner), and the radio callbacks of the event
 * go to it. Every planned event is a request to the arbiter (ll_arb.h),
 * which owns the main alarm and calls ll_conn_arb_start() with the event
 * cap (max_event_us, clipped before the next accepted request). Each
 * CONN_RX:
 * ll_txq_rx(), ll_rxq_isr_put(), the first one re-syncs the anchor and the
 * supervision timer, unless the event's first packet was not delivered
 * (bad CRC, or an acked retransmission without an RX entry) or the packet
 * starts after the RX window (it is then a chained packet, not the
 * anchor). A data PDU that ll_rxq cannot take (ring full) is lost, as the
 * hardware has acked it: the link then ends with 0x08 at CONN_DONE.
 * CONN_DONE: ll_txq_event_end(), counter++, termination
 * and supervision checks, plan the next event (applying instants).
 *
 * Peripheral latency: plan() decides how many events to skip (skip_n),
 * advances counter and CSA#1 over them and plans the listen at skip_base +
 * skip_n. skip_base and csa_base keep the state before the skip, so a
 * re-plan (ll_conn_kick, an instant for a skipped or the planned event)
 * can restore it and plan an earlier event of the window instead.
 *
 * Arbitration (slice 6a, owner re-plan rule): each planned event is
 * requested with a priority (MUST: transmit-window or instant event;
 * SUPERVISION: its RX opens later than last RX + timeout - 2 * interval;
 * ACTIVE: TX backlog or ops.busy; else IDLE). If the arbiter refuses it,
 * or displaces it later (bumped), the link first dodges: it tries the
 * other events of its latency window, latest first at planning, earliest
 * first for a kick (no collision counted). Without latency freedom, or
 * when every candidate is refused, it yields: the next event, and the
 * next, ... until one is accepted; each yielded event advances counter and
 * CSA#1 like a skip and counts in stats.collisions, and it does not clear
 * `anchored` (a yield is not a miss for the latency rule). An event
 * started with a cap too small for its first RX window plus one exchange
 * is yielded the same way.
 *
 * Everything runs in ISR context except the public calls documented as
 * thread calls, which take ll_plat_lock(). No allocation, no blocking.
 */
#include <errno.h>
#include <string.h>

#include "ll_arb.h"
#include "ll_conn.h"
#include "ll_csa1.h"
#include "ll_csa2.h"
#include "ll_defs.h"
#include "ll_plat.h"
#include "ll_radio.h"
#include "ll_rxq.h"
#include "ll_txq.h"

#define US(t)            ((uint32_t)(t) * LL_TICKS_PER_US)
#define UNIT_US          1250u     /* interval / window unit */
#define SUP_UNIT_US      10000u    /* supervision timeout unit */
#define NOT_ESTAB_EVENTS 6
#define OP_TERMINATE_IND 0x02
#define INSTANT_PAST     32767u   /* passed: (instant - counter) mod 65536 > this */
/* Yields in a row before the link is given up (the arbiter holds at most
 * LL_ARB_IDS - 1 other requests, each blocking one or two events). */
#define YIELD_MAX        64

/* SCA field -> worst-case ppm (Vol 6 Part B 2.3.3.1, Table 2.17) */
static const uint16_t sca_ppm[8] = {500, 250, 150, 100, 75, 50, 30, 20};

enum link_state { LINK_FREE, LINK_ACTIVE, LINK_ENDED };

struct ll_link {
	uint8_t id;
	uint8_t state;        /* enum link_state */
	bool active;          /* state == LINK_ACTIVE */
	bool planned;         /* alarm pending for event `counter` */
	bool in_event;        /* BRX issued, CONN_DONE pending */
	bool established;     /* a packet was received in this connection */
	bool rx_this_event;   /* a CRC-valid packet was received in this event */
	bool first_seen;      /* the event's first packet (valid or not) was seen */
	bool anchored;        /* the last closed event re-anchored (latency rule) */
	bool holdoff_done;    /* LL_CONN_LATENCY_HOLDOFF_MS has passed (latched, see skip_count) */
	/* effective maximum RX / TX times (ll_conn_set_dle_times): one
	 * exchange of such PDUs after the first RX window is the guard floor
	 * and part of the arbiter span (xchg_of) */
	uint16_t dle_rx_time;
	uint16_t dle_tx_time;
	/* xchg_of() at the last request: the planned event's prepare checks
	 * its cap against the span it was accepted with (no ll_txq walk in
	 * the time-critical prepare, which runs about 400 us of the 420 us
	 * before the RX trigger already) */
	uint32_t xchg_req;
	struct ll_connect_ind ci;
	struct ll_conn_params p;
	uint32_t interval_ticks;
	uint32_t sup_ticks;
	uint16_t ppm;         /* central SCA + own */
	uint32_t widen_max_us; /* interval / 2 - T_IFS (exceeds 16 bits) */
	/* CSA#2 (4.5.8.3) when the CONNECT_IND has ChSel 1 (our ADV_IND
	 * always has ChSel 1): the channel is a function of chan_id, the event
	 * counter and the map, so skips, yields and re-plans need no stepping.
	 * csa below is stepped for both algorithms; it holds the map in force. */
	bool csa2;
	uint16_t chan_id;
	struct ll_csa1 csa;
	/* CSA#1 state before the skipped events and the planned one; with
	 * skip_n == 0 after a re-plan (replan_to): the state after the
	 * instants applied at the planned event */
	struct ll_csa1 csa_base;
	struct ll_csa1 csa_evt; /* CSA#1 state before the planned event's channel */
	uint16_t counter;     /* next event not yet completed (planned: the listened one) */
	/* first event after the last closed one; with skip_n == 0 after a
	 * re-plan: the planned (target) event itself */
	uint16_t skip_base;
	uint16_t skip_n;      /* events skipped before the planned one */
	uint32_t ref_tick;
	uint16_t ref_counter;
	uint32_t win_us;
	uint32_t sync_tick;
	uint32_t sup_tick;    /* supervision timer start */
	uint32_t start_tick;  /* CONNECT_IND end: LL_CONN_LATENCY_HOLDOFF_MS counts from it */
	/* planned event */
	uint8_t ch;
	uint32_t open_tick;
	uint32_t fst_us;
	bool inst_evt;        /* an instant was applied at event inst_counter */
	uint16_t inst_counter;
	/* instants */
	bool upd_pending;
	uint16_t upd_instant;
	uint8_t upd_win_size;
	uint16_t upd_win_offset;
	struct ll_conn_params upd_p;
	bool chm_pending;
	uint16_t chm_instant;
	uint8_t chm[5];
	/* the map instant was applied at the planned event of a skip window
	 * (skip_n > 0, see skip_count): csa_base still holds the old map, so a
	 * re-plan to another event of the window first makes it pending again
	 * (chm_repend) and the instant is applied again only at its own event */
	bool chm_win;
	/* termination */
	bool term_local;
	bool term_acked;
	uint32_t term_tick;
	bool end_pending;
	uint8_t end_reason;
	uint8_t end_r;        /* reason reported with DISCONNECTED (arg storage) */
};

static struct ll_link links[LL_MAX_CONN];
static struct ll_conn_ops ops;
static struct ll_conn_stats stats[LL_MAX_CONN];
/* Link whose BRX is on air (radio callbacks go to it), -1: none. */
static int8_t ev_owner = -1;

#define ST(c) (&stats[(c)->id])

static struct ll_link *link_of(uint8_t link)
{
	return link < LL_MAX_CONN ? &links[link] : NULL;
}

static void report(struct ll_link *c, enum ll_conn_evt what, const void *arg)
{
	if (ops.evt) {
		ops.evt(c->id, what, arg);
	}
}

static void end(struct ll_link *c, uint8_t reason)
{
	c->active = false;
	c->state = LINK_ENDED;
	c->planned = false;
	c->in_event = false;
	c->end_pending = false;
	if (ev_owner == (int8_t)c->id) {
		ev_owner = -1;
	}
	{
		unsigned int key = ll_plat_lock();

		ll_arb_cancel(c->id);
		ll_plat_unlock(key);
	}
	c->end_r = reason;
	report(c, LL_CONN_EVT_DISCONNECTED, &c->end_r);
}

/* One exchange after the first RX window: the effective maximum times, and
 * our TX at least as long as the longest PDU still queued (slice 6b Task 4
 * review: a PDU queued under a larger effective TX length stays valid after
 * it shrank, 4.5.10, and is sent as it is until its ack). On-air time on
 * 1M: 8 us per octet of preamble 1 + AA 4 + header 2 + payload (MIC
 * included) + CRC 3. */
static uint32_t xchg_of(const struct ll_link *c)
{
	uint32_t q = 8u * (10u + ll_txq_max_len(c->id));
	uint16_t tx = c->dle_tx_time > q ? c->dle_tx_time : (uint16_t)q;

	return ll_conn_exchange_us(c->dle_rx_time, tx);
}

/* End now, or at CONN_DONE when an event is on air (first reason wins). */
static void request_end(struct ll_link *c, uint8_t reason)
{
	if (c->in_event) {
		if (!c->end_pending) {
			c->end_pending = true;
			c->end_reason = reason;
		}
		return;
	}
	end(c, reason);
}

/* 4.5.4 window widening. Above interval / 2 - T_IFS it is clamped (4.5.7
 * says the link is then lost; the supervision timeout ends it). */
static uint32_t widening_us(const struct ll_link *c, uint32_t dt_ticks)
{
	uint32_t dt_us = dt_ticks / LL_TICKS_PER_US;
	uint32_t w = (uint32_t)(((uint64_t)c->ppm * dt_us + 999999u) / 1000000u) + 16u;

	if (w > c->widen_max_us) {
		w = c->widen_max_us;
	}
	return w;
}

static uint32_t anchor_of(const struct ll_link *c, uint16_t counter)
{
	return c->ref_tick + (uint32_t)(uint16_t)(counter - c->ref_counter) * c->interval_ticks;
}

static void set_params(struct ll_link *c, const struct ll_conn_params *p)
{
	c->p = *p;
	c->interval_ticks = (uint32_t)p->interval * US(UNIT_US);
	c->sup_ticks = (uint32_t)p->timeout * US(SUP_UNIT_US);
	c->widen_max_us = (uint32_t)p->interval * UNIT_US / 2u - LL_T_IFS_US;
}

/* Instants of the event about to be planned (counter == instant). Returns
 * true if one was applied. */
static bool apply_instants(struct ll_link *c)
{
	bool applied = false;

	if (c->chm_pending && c->chm_instant == c->counter) {
		c->chm_pending = false;
		ll_csa1_set_map(&c->csa, c->chm);
		applied = true;
	}
	if (c->upd_pending && c->upd_instant == c->counter) {
		/* 5.1.1: the transmit window starts WinOffset after the anchor
		 * the instant event would have had with the old parameters. */
		uint32_t old_anchor = anchor_of(c, c->counter);
		bool changed = c->upd_p.interval != c->p.interval ||
			       c->upd_p.latency != c->p.latency ||
			       c->upd_p.timeout != c->p.timeout;

		c->upd_pending = false;
		applied = true;
		c->ref_tick = old_anchor + (uint32_t)c->upd_win_offset * US(UNIT_US);
		c->ref_counter = c->counter;
		c->win_us = (uint32_t)c->upd_win_size * UNIT_US;
		c->sup_tick = old_anchor;
		set_params(c, &c->upd_p);
		if (changed) {
			report(c, LL_CONN_EVT_UPDATED, &c->p);
		}
	}
	return applied;
}

/* RX open tick of event `counter` (current timing state); widening out. */
static uint32_t open_of(const struct ll_link *c, uint16_t counter, uint32_t *widen_out)
{
	uint32_t base = anchor_of(c, counter);
	uint32_t widen = widening_us(c, base + US(c->win_us) - c->sync_tick);
	uint32_t margin = c->win_us ? LL_CONN_WIN_MARGIN_US : LL_CONN_RX_MARGIN_US;

	if (widen_out) {
		*widen_out = widen;
	}
	return base - US(widen + margin);
}

/* Plan event `counter` (listened to): instants, channel, RX window. The
 * caller requests it from the arbiter. */
static void plan_event(struct ll_link *c)
{
	uint32_t widen, margin;
	bool chm_due = c->chm_pending && c->chm_instant == c->counter;

	if (apply_instants(c)) {
		c->inst_evt = true;
		c->inst_counter = c->counter;
		c->chm_win = chm_due && c->skip_n > 0;
	} else if (c->inst_evt && c->inst_counter != c->counter) {
		/* planned past the instant event (re-plans never go back
		 * before it, see replan_to): forget it, so a counter wrap
		 * (65536 events later) cannot make a stale counter MUST */
		c->inst_evt = false;
	}
	if (c->skip_n == 0) {
		/* re-plan point: the state after this event's instants */
		c->csa_base = c->csa;
	}
	c->csa_evt = c->csa;
	c->ch = ll_csa1_next(&c->csa);
	if (c->csa2) {
		c->ch = ll_csa2_channel(c->chan_id, c->counter, c->csa.chm);
	}
	c->open_tick = open_of(c, c->counter, &widen);
	if (widen > ST(c)->widen_max_us) {
		ST(c)->widen_max_us = widen;
	}
	margin = c->win_us ? LL_CONN_WIN_MARGIN_US : LL_CONN_RX_MARGIN_US;
	c->fst_us = c->win_us + 2u * (widen + margin) + LL_CONN_SYNC_US;
	c->planned = true;
}

/* Before re-planning to another event of the window (set_window,
 * replan_to): a map instant applied at the planned event is pending again,
 * as csa_base (restored next) holds the map before it. */
static void chm_repend(struct ll_link *c)
{
	if (c->chm_win) {
		c->chm_win = false;
		c->chm_pending = true;
	}
}

/* An instant in [counter, counter + n]: its event must be listened to. */
static bool instant_within(const struct ll_link *c, bool pending, uint16_t instant, uint16_t n)
{
	return pending && (uint16_t)(instant - c->counter) <= n;
}

/* Peripheral latency: events to skip before the next listened one (rules
 * in ll_conn.h). c->counter is the first candidate. */
static uint16_t skip_count(struct ll_link *c)
{
	uint16_t n = c->p.latency;
	uint32_t deadline;
	int32_t room;

	/* holdoff after connect: the first candidate's anchor must lie at
	 * least LL_CONN_LATENCY_HOLDOFF_MS after the connection start. Latched
	 * once passed: the 32-bit tick age wraps negative after 134 s, so it
	 * must never be tested again on a long link. Tested first, at
	 * every plan, so it latches within an event of the first second even
	 * while latency is 0 (an update to latency > 0 may come much later). */
	if (!c->holdoff_done) {
		if ((int32_t)(anchor_of(c, c->counter) - c->start_tick) <
		    (int32_t)US(LL_CONN_LATENCY_HOLDOFF_MS * 1000u)) {
			return 0;
		}
		c->holdoff_done = true;
	}
	if (n == 0 || !c->anchored || c->term_local || ll_txq_backlog(c->id) != 0 ||
	    (ops.busy && ops.busy(c->id))) {
		return 0;
	}
	if (instant_within(c, c->upd_pending, c->upd_instant, n)) {
		return 0;
	}
	if (instant_within(c, c->chm_pending, c->chm_instant, n)) {
		/* listen at the map instant, skip the events before it */
		n = (uint16_t)(c->chm_instant - c->counter);
	}
	/* supervision: listened anchor <= last RX + timeout - 2 * interval */
	deadline = c->sup_tick + c->sup_ticks - 2u * c->interval_ticks;
	room = (int32_t)(deadline - anchor_of(c, c->counter));
	if (room < 0) {
		return 0;
	}
	if ((uint32_t)room / c->interval_ticks < n) {
		n = (uint16_t)((uint32_t)room / c->interval_ticks);
	}
	return n;
}

/* Event length cap for the radio guard (see LL_CONN_EVENT_SAFETY_US): the
 * next event opens no earlier than one interval after this one, minus the
 * widening growth over that interval (no re-sync in this event). */
static uint32_t event_max_us(const struct ll_link *c, uint32_t xchg_us)
{
	uint32_t ival_us = (uint32_t)c->p.interval * UNIT_US;
	uint32_t growth = (uint32_t)(((uint64_t)c->ppm * ival_us + 999999u) / 1000000u);
	uint32_t reserve = growth + LL_CONN_ARM_LEAD_US + LL_CONN_EVENT_SAFETY_US;
	uint32_t floor_us = c->fst_us + xchg_us;
	uint32_t cap = ival_us > reserve ? ival_us - reserve : 0;

	return cap > floor_us ? cap : floor_us;
}

/* Arbiter priority of the planned event (rules at the top of the file). */
static uint8_t prio_of(struct ll_link *c)
{
	uint32_t deadline;

	if (c->win_us != 0 || (c->inst_evt && c->inst_counter == c->counter)) {
		return LL_ARB_PRIO_MUST;
	}
	deadline = c->sup_tick + c->sup_ticks - 2u * c->interval_ticks;
	if ((int32_t)(c->open_tick - deadline) > 0) {
		return LL_ARB_PRIO_SUPERVISION;
	}
	if (ll_txq_backlog(c->id) != 0 || (ops.busy && ops.busy(c->id))) {
		return LL_ARB_PRIO_ACTIVE;
	}
	return LL_ARB_PRIO_IDLE;
}

/* Request the planned event. Span: the alarm LL_CONN_ARM_LEAD_US before
 * the RX opens, to the first RX window + one exchange at the link's
 * effective times (xchg_of, at least LL_CONN_GUARD_MIN_TAIL_US) + the
 * arbiter's clipping reserve, so an accepted event is never started with
 * a cap below its floor. */
static int request(struct ll_link *c)
{
	uint32_t x = xchg_of(c);
	struct ll_arb_req r = {
		.alarm_tick = c->open_tick - US(LL_CONN_ARM_LEAD_US),
		.open_tick = c->open_tick,
		.min_len_us = c->fst_us + x + LL_CONN_EVENT_SAFETY_US + LL_CONN_ARM_LEAD_US,
		.max_len_us = event_max_us(c, x),
		.prio = prio_of(c),
	};
	unsigned int key;
	int ret;

	c->xchg_req = x;
	key = ll_plat_lock();
	ret = ll_arb_request(c->id, &r);

	ll_plat_unlock(key);
	return ret;
}

/* Plan event skip_base + k of the current window (k <= the window's
 * skip_n, so no instant lies before it in the window; one at k itself is
 * idempotent here). Not requested yet. */
static void set_window(struct ll_link *c, uint16_t k)
{
	chm_repend(c);
	c->csa = c->csa_base;
	for (uint16_t i = 0; i < k; i++) {
		(void)ll_csa1_next(&c->csa);
	}
	c->counter = (uint16_t)(c->skip_base + k);
	c->skip_n = k;
	plan_event(c);
}

/* The planned event becomes the new re-plan base (skip_n 0): used after a
 * yield or a kick, see replan_to(). */
static void rebase(struct ll_link *c)
{
	c->chm_win = false;
	c->skip_base = c->counter;
	c->skip_n = 0;
	c->csa_base = c->csa_evt;
}

/* Yield the planned event and the following ones until the arbiter
 * accepts one (each counts as a collision, advances like a skip). Every
 * step applies the instants of the event it plans. commit: the planned
 * event was refused by the arbiter at its last request (not displaced), so
 * giving it up is committed to the fairness flags (ll_arb_yield); a
 * displaced event was committed by the arbiter already. */
static void yield_on(struct ll_link *c, bool commit)
{
	for (int i = 0; i < YIELD_MAX; i++) {
		ST(c)->collisions++;
		if (commit) {
			unsigned int key = ll_plat_lock();

			ll_arb_yield(c->id);
			ll_plat_unlock(key);
		}
		commit = true;
		c->chm_win = false;
		c->counter++;
		c->skip_base = c->counter;
		c->skip_n = 0;
		plan_event(c);
		if (request(c) == 0) {
			return;
		}
	}
	/* never seen: the arbiter found no room for YIELD_MAX events */
	end(c, LL_ST_CONN_TIMEOUT);
}

/* Request the latest event of [skip_base + lo, skip_base + hi] the arbiter
 * accepts (a dodge inside the latency window), else yield after hi.
 * stats.skipped follows the event taken. On success nothing else is
 * touched afterwards: a bump of this link (dispatched before the request
 * returns) may already have re-planned. The probes do not count for
 * fairness; event hi is requested once more before it is given up, so the
 * yield commits the refusal of hi itself (with lo < hi the last probe was
 * lo). */
static void place_latest(struct ll_link *c, uint16_t lo, uint16_t hi)
{
	uint16_t orig = c->skip_n;

	for (int k = hi; k >= (int)lo; k--) {
		set_window(c, (uint16_t)k);
		if (request(c) == 0) {
			ST(c)->skipped -= (uint32_t)(orig - k);
			return;
		}
	}
	ST(c)->skipped -= (uint32_t)(orig - hi);
	if (lo < hi) {
		set_window(c, hi);
		if (request(c) == 0) {
			return;
		}
	}
	yield_on(c, true);
}

/* Plan the next event, skipping idle events where allowed. */
static void plan(struct ll_link *c)
{
	uint16_t n;

	c->chm_win = false;
	n = skip_count(c);
	c->skip_base = c->counter;
	c->csa_base = c->csa;
	c->skip_n = n;
	ST(c)->skipped += n;
	ST(c)->planned++;
	place_latest(c, 0, n);
}

/* Re-plan the planned (not yet issued) listen to event `target` in
 * [skip_base, counter]: restore the state before the skip, advance over
 * the events before target, plan it (applying its instants). Afterwards
 * target is the new skip base with skip_n = 0: the instants applied at it
 * changed the timing and the map, so no later re-plan may go back to an
 * earlier event (csa_base is the state after them, see plan_event). This
 * keeps the invariant that skip_n > 0 only while no timing instant is
 * applied at the planned event, which ll_conn_kick() / first_reachable()
 * rely on; the one exception, a map instant at the planned event of a
 * window (skip_count), changes no timing and is made pending again by
 * chm_repend() before any re-plan inside the window. */
static void replan_to(struct ll_link *c, uint16_t target)
{
	uint16_t k = (uint16_t)(target - c->skip_base);

	chm_repend(c);
	c->planned = false;
	c->csa = c->csa_base;
	for (uint16_t i = 0; i < k; i++) {
		(void)ll_csa1_next(&c->csa);
	}
	ST(c)->skipped -= (uint32_t)(c->skip_n - k);
	c->skip_n = 0;
	c->counter = target;
	c->skip_base = target;
	plan_event(c);
}

/* Index (from skip_base) of the first event of the planned window whose
 * alarm (LL_CONN_ARM_LEAD_US before its RX opens) is still ahead; skip_n
 * if none before the planned one is. Caller holds the lock, c->planned.
 * Only meaningful with skip_n > 0, i.e. on pre-instant timing state: plan()
 * never skips with an update instant pending in the window (a map instant
 * may end it, but changes no timing) and replan_to() sets skip_n = 0 once
 * it applied one, so the timing used here is that of every event in
 * [skip_base, counter]. */
static uint16_t first_reachable(const struct ll_link *c)
{
	uint32_t now = ll_radio_now();
	int32_t elapsed = (int32_t)(now - anchor_of(c, c->skip_base));
	uint32_t i = 0;

	/* events whose anchor already passed cannot be reached: start there */
	if (elapsed > 0) {
		i = (uint32_t)elapsed / c->interval_ticks;
	}
	while (i < c->skip_n &&
	       (int32_t)(open_of(c, (uint16_t)(c->skip_base + i), NULL) -
			 US(LL_CONN_ARM_LEAD_US) - now) <= 0) {
		i++;
	}
	return i < c->skip_n ? (uint16_t)i : c->skip_n;
}

/* An event ended (or was skipped): advance, check, plan the next one. */
static void event_closed(struct ll_link *c, uint32_t now)
{
	c->counter++;
	if (c->end_pending) {
		end(c, c->end_reason);
		return;
	}
	if (c->term_acked) {
		end(c, LL_ST_LOCAL_TERM);
		return;
	}
	if (!c->established) {
		if (c->counter >= NOT_ESTAB_EVENTS) {
			end(c, LL_ST_CONN_FAIL_EST);
			return;
		}
	} else if ((int32_t)(now - c->sup_tick) >= (int32_t)c->sup_ticks) {
		end(c, LL_ST_CONN_TIMEOUT);
		return;
	}
	if (c->term_local && (int32_t)(now - c->term_tick) >= (int32_t)c->sup_ticks) {
		/* 5.1.3: no ack of LL_TERMINATE_IND within the timeout */
		end(c, LL_ST_LOCAL_TERM);
		return;
	}
	plan(c);
}

/* Arbiter start: issue the BRX for the planned event of link c. */
static void prepare(struct ll_link *c, uint32_t cap_us)
{
	uint32_t now = ll_radio_now();

	c->planned = false;
	if (ev_owner >= 0 || cap_us < c->fst_us + c->xchg_req) {
		/* another event is on air (it overran its cap), or no room
		 * before the next request: yield this event (not a miss) */
		unsigned int key = ll_plat_lock();

		ll_arb_yield(c->id);
		ll_plat_unlock(key);
		ST(c)->collisions++;
		event_closed(c, now);
		return;
	}
	c->anchored = false;
	if ((int32_t)(c->open_tick - now) < (int32_t)US(LL_CONN_MIN_PREP_US)) {
		ST(c)->late++;
		ST(c)->missed++;
		event_closed(c, now);
		return;
	}
	ll_radio_conn_select(c->ci.aa, c->ci.crc_init);
	ll_txq_event_start(c->id);
	c->rx_this_event = false;
	c->first_seen = false;
	c->in_event = true;
	ev_owner = (int8_t)c->id;
	ST(c)->events++;
	ST(c)->listened++;
	ll_radio_conn_event(c->ch, c->open_tick, c->fst_us, cap_us);
}

void ll_conn_arb_start(uint8_t link, uint32_t cap_us)
{
	struct ll_link *c = link_of(link);

	if (!c || !c->active || !c->planned) {
		/* stale request (never expected): drop it */
		unsigned int key = ll_plat_lock();

		ll_arb_cancel(link);
		ll_plat_unlock(key);
		return;
	}
	prepare(c, cap_us);
}

void ll_conn_arb_bumped(uint8_t link)
{
	struct ll_link *c = link_of(link);
	unsigned int key;

	if (!c) {
		return;
	}
	key = ll_plat_lock();
	if (c->active && c->planned) {
		uint16_t orig = c->skip_n;
		uint16_t lo = orig > 0 ? first_reachable(c) : 0;
		bool placed = false;

		/* dodge: an earlier event of the window, latest first (the
		 * bumped one is never requested again) */
		for (int k = (int)orig - 1; k >= (int)lo; k--) {
			set_window(c, (uint16_t)k);
			if (request(c) == 0) {
				ST(c)->skipped -= (uint32_t)(orig - k);
				placed = true;
				break;
			}
		}
		if (!placed) {
			/* the bumped event itself was committed by the arbiter */
			set_window(c, orig);
			yield_on(c, false);
		}
	}
	ll_plat_unlock(key);
}

/* A packet with a bad CRC: if it is the event's first, it still was the
 * anchor packet, so a later valid packet of the event must not re-anchor. */
static void on_rx_bad(struct ll_link *c)
{
	if (!c->first_seen) {
		c->first_seen = true;
		ST(c)->first_bad++;
	}
}

/* A packet the hardware received but did not deliver (the central resent a
 * packet we have; Task 10): same as a bad first packet. With MD the event
 * continues with a new packet some 400..700 us after the anchor. An event
 * with only such retransmissions has no rx_this_event and so counts as
 * missed in the stats (it still proves the central is there, but it does
 * not refresh supervision; the next new packet does). */
static void on_rx_nodata(struct ll_link *c)
{
	if (!c->first_seen) {
		c->first_seen = true;
		ST(c)->first_nodata++;
	}
}

static void on_rx(struct ll_link *c, const uint8_t *pdu, uint16_t len, uint32_t tick)
{
	uint32_t anchor;
	bool first;

	if (len < 2) {
		return;
	}
	ST(c)->rx_pkts++;
	ll_txq_rx(c->id, pdu[0]);
	if (!ll_rxq_isr_put(c->id, pdu, len)) {
		/* The hardware has acked this data PDU already, so the central
		 * will never resend it: it is lost for good, and continuing
		 * would leave a hole in the L2CAP stream (and, encrypted, a
		 * CCM packet counter mismatch at the next PDU). End the link
		 * deterministically at the end of this event. 0x08: we stop
		 * following the link without an LL_TERMINATE_IND, so the
		 * central sees a supervision timeout too and both hosts treat
		 * it as an ordinary link loss (reconnect). */
		request_end(c, LL_ST_CONN_TIMEOUT);
	}
	c->rx_this_event = true;
	first = !c->first_seen;
	c->first_seen = true;
	/* The hardware syncs the event's first packet only inside the RX
	 * window (first-RX timeout); a packet whose access address ends after
	 * it (plus one sync time of slack) is a chained one whose anchor
	 * packet was not reported (e.g. both in one late ISR). */
	if (first && (int32_t)(tick - (c->open_tick + US(c->fst_us + LL_CONN_SYNC_US))) > 0) {
		ST(c)->first_outside++;
		first = false;
	}
	if (!first) {
		/* later packet of the event: proof of life only (4.5.2); it is
		 * not at the anchor, so timing stays as it was */
		c->sup_tick = tick - US(LL_CONN_SYNC_US);
		c->established = true;
		return;
	}
	/* first packet of the event: its start is the anchor point */
	anchor = tick - US(LL_CONN_SYNC_US);
	c->ref_tick = anchor;
	c->ref_counter = c->counter;
	c->win_us = 0;
	c->sync_tick = anchor;
	c->sup_tick = anchor;
	c->established = true;
	c->anchored = true;
}

void ll_conn_radio_evt(enum ll_radio_evt evt, const uint8_t *pdu, uint16_t len, uint32_t tick)
{
	struct ll_link *c;

	if (ev_owner < 0) {
		return;
	}
	c = &links[ev_owner];
	if (!c->active || !c->in_event) {
		return;
	}
	switch (evt) {
	case LL_RADIO_CONN_RX:
		on_rx(c, pdu, len, tick);
		break;
	case LL_RADIO_CONN_RX_CRC_ERR:
		on_rx_bad(c);
		break;
	case LL_RADIO_CONN_RX_NODATA:
		on_rx_nodata(c);
		break;
	case LL_RADIO_CONN_DONE:
		c->in_event = false;
		ev_owner = -1;
		ll_txq_event_end(c->id);
		if (c->rx_this_event) {
			ST(c)->rx_events++;
		} else {
			ST(c)->missed++;
		}
		event_closed(c, tick);
		break;
	default:
		break;
	}
}

static void txq_done(uint8_t link, enum ll_txq_kind kind, uint8_t ctrl_opcode, bool last)
{
	struct ll_link *c = link_of(link);

	if (c && kind == LL_TXQ_CTRL && ctrl_opcode == OP_TERMINATE_IND && c->term_local) {
		c->term_acked = true;   /* ended at this event's CONN_DONE */
	}
	if (ops.txq_done) {
		ops.txq_done(link, kind, ctrl_opcode, last);
	}
}

static void link_clear(uint8_t i)
{
	memset(&links[i], 0, sizeof(links[i]));
	links[i].id = i;
	links[i].state = LINK_FREE;
	/* 27 / 328 both ways for a new connection (4.5.10) */
	links[i].dle_rx_time = LL_DLE_MIN_TIME;
	links[i].dle_tx_time = LL_DLE_MIN_TIME;
}

void ll_conn_init(const struct ll_conn_ops *o)
{
	unsigned int key = ll_plat_lock();

	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		ll_arb_cancel(i);
		link_clear(i);
	}
	ll_plat_unlock(key);
	memset(&ops, 0, sizeof(ops));
	if (o) {
		ops = *o;
	}
	ev_owner = -1;
	ll_txq_init(txq_done);
}

/* Connection parameters and transmit window (Vol 6 Part B 2.3.3.1 and
 * 5.1.1), shared by CONNECT_IND and LL_CONNECTION_UPDATE_IND. */
static bool timing_valid(uint16_t interval, uint16_t latency, uint16_t timeout,
			 uint8_t win_size, uint16_t win_offset)
{
	uint32_t max_win;

	if (interval < 6 || interval > 3200) {
		return false;
	}
	max_win = interval - 1u;
	if (max_win > 8) {
		max_win = 8;
	}
	return timeout >= 10 && timeout <= 3200 && latency <= 499 &&
	       /* timeout > (1 + latency) * interval * 2 */
	       (uint32_t)timeout * SUP_UNIT_US >
		       (1u + latency) * (uint32_t)interval * UNIT_US * 2u &&
	       win_size >= 1 && win_size <= max_win && win_offset <= interval;
}

static bool params_valid(const struct ll_connect_ind *ci, const struct ll_csa1 *csa)
{
	return timing_valid(ci->interval, ci->latency, ci->timeout, ci->win_size,
			    ci->win_offset) &&
	       ci->hop >= 5 && ci->hop <= 16 && csa->n_used >= 2;
}

int ll_conn_start(const struct ll_connect_ind *ci, uint32_t connect_ind_end_tick)
{
	struct ll_conn_params p;
	struct ll_csa1 csa;
	struct ll_link *c = NULL;

	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		if (links[i].state == LINK_FREE) {
			c = &links[i];
			break;
		}
	}
	if (!c) {
		return -EBUSY;
	}
	ll_csa1_init(&csa, ci->hop, ci->chm);
	if (!params_valid(ci, &csa)) {
		return -EINVAL;
	}
	link_clear(c->id);
	c->ci = *ci;
	c->csa = csa;
	c->csa2 = ci->chsel != 0;
	c->chan_id = ll_csa2_chan_id(ci->aa);
	p.interval = ci->interval;
	p.latency = ci->latency;
	p.timeout = ci->timeout;
	set_params(c, &p);
	c->ppm = (uint16_t)(sca_ppm[ci->sca & 7] + LL_OWN_SCA_PPM);
	/* 4.5.3: transmitWindowDelay 1.25 ms + WinOffset, size WinSize */
	c->ref_tick = connect_ind_end_tick + US(UNIT_US) + (uint32_t)ci->win_offset * US(UNIT_US);
	c->ref_counter = 0;
	c->win_us = (uint32_t)ci->win_size * UNIT_US;
	c->sync_tick = connect_ind_end_tick;
	c->sup_tick = connect_ind_end_tick;
	c->start_tick = connect_ind_end_tick;
	c->active = true;
	c->state = LINK_ACTIVE;

	/* one-time radio setup; a no-op while other links are live */
	ll_radio_conn_init();
	ll_txq_reset(c->id);
	/* No ll_rxq_reset() here: this runs in ISR context while the controller
	 * thread may be inside ll_rxq_get(). The glue resets the link's ll_rxq
	 * in its thread when it handles LL_CONN_EVT_DISCONNECTED and releases
	 * the id only then (ll_conn_release). */
	plan(c);
	report(c, LL_CONN_EVT_CONNECTED, &c->ci);
	return c->id;
}

/* 0, or LL_ST_INSTANT_PASSED (connection ending). With a latency skip
 * planned, an instant for a skipped event whose anchor has gone by is
 * passed too (slice 7: the anchor, not the alarm time). A skipped instant
 * event whose alarm time is gone but whose anchor is not is still
 * honoured: instant_replan() plans it with a late alarm, prepare() issues
 * it if there is time left (LL_CONN_MIN_PREP_US), else it is a late miss;
 * either way the instant is applied when the event is planned. */
static int check_instant(struct ll_link *c, uint16_t instant)
{
	uint16_t cur = c->planned ? c->skip_base : c->counter;
	uint16_t d = (uint16_t)(instant - cur);

	/* d == 0 with the event already on air: its timing and channel were
	 * issued with the old values, so the instant cannot be honoured any
	 * more; treat it like a passed instant. */
	if (d > INSTANT_PAST || (d == 0 && c->in_event) ||
	    (c->planned && d < c->skip_n &&
	     (int32_t)(anchor_of(c, instant) - ll_radio_now()) <= 0)) {
		request_end(c, LL_ST_INSTANT_PASSED);
		return LL_ST_INSTANT_PASSED;
	}
	return 0;
}

/* A new instant for the planned event or one of the skipped events before
 * it: re-plan to the first reachable event of the window, or to the
 * instant if that comes first (check_instant() passed, so the instant's
 * anchor is still ahead; when its alarm time is gone too, the instant
 * event is planned with an alarm in the past). A target before the instant
 * applies no instant there; from it plan() listens to every event up to an
 * update instant, or skips to a map instant (skip_count). Re-planning to
 * the instant itself would leave skip_n = 0 at the instant event, so a
 * kick in between could not pull the listen earlier and TX would wait for
 * the instant. */
static void instant_replan(struct ll_link *c, uint16_t instant)
{
	uint16_t d = (uint16_t)(instant - c->skip_base);

	if (c->planned && d <= c->skip_n) {
		uint16_t i = c->skip_n > 0 ? first_reachable(c) : 0;

		replan_to(c, (uint16_t)(c->skip_base + (i < d ? i : d)));
		if (request(c) != 0) {
			yield_on(c, true);
		}
	}
}

int ll_conn_update_at(uint8_t link, uint16_t instant, uint8_t win_size, uint16_t win_offset,
		      const struct ll_conn_params *p)
{
	struct ll_link *c = link_of(link);
	unsigned int key = ll_plat_lock();
	int ret = LL_ST_DISALLOWED;

	if (!timing_valid(p->interval, p->latency, p->timeout, win_size, win_offset)) {
		ret = LL_ST_INVALID_LL_PARAM;
	} else if (c && c->active && !c->end_pending) {
		ret = check_instant(c, instant);
		if (ret == 0) {
			/* A second LL_CONNECTION_UPDATE_IND while one is
			 * pending replaces it (instant and parameters). The
			 * central may only start one procedure at a time, so
			 * this is a protocol violation for which the spec
			 * allows ending the link; following the latest one is
			 * accepted as the more forgiving choice. */
			c->upd_pending = true;
			c->upd_instant = instant;
			c->upd_win_size = win_size;
			c->upd_win_offset = win_offset;
			c->upd_p = *p;
			instant_replan(c, instant);
		}
	}
	ll_plat_unlock(key);
	return ret;
}

int ll_conn_chmap_at(uint8_t link, uint16_t instant, const uint8_t chm[5])
{
	struct ll_link *c = link_of(link);
	unsigned int key = ll_plat_lock();
	int ret = LL_ST_DISALLOWED;

	if (c && c->active && !c->end_pending) {
		ret = check_instant(c, instant);
		if (ret == 0) {
			c->chm_pending = true;
			c->chm_instant = instant;
			memcpy(c->chm, chm, sizeof(c->chm));
			instant_replan(c, instant);
		}
	}
	ll_plat_unlock(key);
	return ret;
}

void ll_conn_terminate(uint8_t link, uint8_t reason)
{
	const uint8_t pdu[2] = {OP_TERMINATE_IND, reason};
	struct ll_link *c = link_of(link);
	unsigned int key;
	bool go;

	if (!c) {
		return;
	}
	key = ll_plat_lock();
	go = c->active && !c->term_local && !c->end_pending;
	if (go) {
		c->term_local = true;
		c->term_acked = false;
		c->term_tick = ll_radio_now();
	}
	ll_plat_unlock(key);
	if (!go) {
		return;
	}
	/* ll_llcp's hook owes the PDU when the backlog is full and retries it
	 * (slice 7); the timeout still ends the link if it is never acked. */
	if (ops.ctrl_tx) {
		(void)ops.ctrl_tx(link, pdu, sizeof(pdu));
	} else {
		key = ll_plat_lock();
		(void)ll_txq_push(link, LL_TXQ_CTRL, LL_LLID_CTRL, pdu, sizeof(pdu),
				  OP_TERMINATE_IND, true);
		ll_plat_unlock(key);
	}
}

void ll_conn_end(uint8_t link, uint8_t reason)
{
	struct ll_link *c = link_of(link);
	unsigned int key;

	if (!c) {
		return;
	}
	key = ll_plat_lock();
	if (c->active) {
		request_end(c, reason);
	}
	ll_plat_unlock(key);
}

bool ll_conn_active(uint8_t link)
{
	struct ll_link *c = link_of(link);

	return c && c->active;
}

uint8_t ll_conn_end_all(uint8_t reason)
{
	unsigned int key = ll_plat_lock();
	uint8_t n = 0;

	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		if (links[i].active) {
			request_end(&links[i], reason);
			n++;
		}
	}
	ll_plat_unlock(key);
	return n;
}

uint8_t ll_conn_count(void)
{
	uint8_t n = 0;

	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		if (links[i].state != LINK_FREE) {
			n++;
		}
	}
	return n;
}

void ll_conn_release(uint8_t link)
{
	struct ll_link *c = link_of(link);
	unsigned int key;

	if (!c) {
		return;
	}
	key = ll_plat_lock();
	if (c->state == LINK_ENDED) {
		c->state = LINK_FREE;
	}
	ll_plat_unlock(key);
}

uint16_t ll_conn_event_counter(uint8_t link)
{
	struct ll_link *c = link_of(link);

	if (!c) {
		return 0;
	}
	return c->planned ? c->skip_base : c->counter;
}

uint8_t ll_conn_pending_instants(uint8_t link)
{
	struct ll_link *c = link_of(link);
	uint8_t r = 0;
	unsigned int key;

	if (!c) {
		return 0;
	}
	key = ll_plat_lock();
	if (c->active) {
		r = (c->upd_pending ? LL_CONN_PENDING_UPDATE : 0) |
		    (c->chm_pending || c->chm_win ? LL_CONN_PENDING_CHMAP : 0);
	}
	ll_plat_unlock(key);
	return r;
}

int ll_conn_event_owner(void)
{
	return ev_owner;
}

void ll_conn_kick(uint8_t link)
{
	struct ll_link *c = link_of(link);
	unsigned int key;

	if (!c) {
		return;
	}
	key = ll_plat_lock();
	if (c->active && c->planned && c->skip_n > 0) {
		uint16_t orig = c->skip_n;
		uint16_t i = first_reachable(c);

		bool placed = false;

		/* the earliest reachable event the arbiter accepts (the
		 * planned one at the latest); an earlier one becomes the
		 * re-plan base (replan_to semantics) */
		for (uint16_t k = i; k <= orig && i < orig && !placed; k++) {
			set_window(c, k);
			if (request(c) == 0) {
				placed = true;
				ST(c)->skipped -= (uint32_t)(orig - k);
				if (k < orig) {
					ST(c)->kicks++;
					if (c->planned &&
					    c->counter == (uint16_t)(c->skip_base + k)) {
						rebase(c);
					}
				}
			}
		}
		if (i < orig && !placed) {
			/* the last probe was orig itself: commit its refusal */
			set_window(c, orig);
			yield_on(c, true);
		}
	}
	ll_plat_unlock(key);
}

void ll_conn_get_stats(uint8_t link, struct ll_conn_stats *s)
{
	unsigned int key;

	if (link >= LL_MAX_CONN) {
		memset(s, 0, sizeof(*s));
		return;
	}
	key = ll_plat_lock();
	*s = stats[link];
	ll_plat_unlock(key);
}

void ll_conn_get_stats_total(struct ll_conn_stats *s)
{
	unsigned int key = ll_plat_lock();

	memset(s, 0, sizeof(*s));
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		const struct ll_conn_stats *t = &stats[i];

		s->events += t->events;
		s->rx_events += t->rx_events;
		s->missed += t->missed;
		s->late += t->late;
		s->rx_pkts += t->rx_pkts;
		if (t->widen_max_us > s->widen_max_us) {
			s->widen_max_us = t->widen_max_us;
		}
		s->first_bad += t->first_bad;
		s->first_nodata += t->first_nodata;
		s->first_outside += t->first_outside;
		s->planned += t->planned;
		s->listened += t->listened;
		s->skipped += t->skipped;
		s->kicks += t->kicks;
		s->collisions += t->collisions;
	}
	ll_plat_unlock(key);
}

uint32_t ll_conn_exchange_us(uint16_t max_rx_time, uint16_t max_tx_time)
{
	uint32_t x = (uint32_t)max_rx_time + LL_T_IFS_US + max_tx_time + LL_T_IFS_US;

	return x > LL_CONN_GUARD_MIN_TAIL_US ? x : LL_CONN_GUARD_MIN_TAIL_US;
}

void ll_conn_set_dle_times(uint8_t link, uint16_t max_rx_time, uint16_t max_tx_time)
{
	struct ll_link *c = link_of(link);
	unsigned int key;

	if (!c) {
		return;
	}
	key = ll_plat_lock();
	c->dle_rx_time = max_rx_time;
	c->dle_tx_time = max_tx_time;
	ll_plat_unlock(key);
}
