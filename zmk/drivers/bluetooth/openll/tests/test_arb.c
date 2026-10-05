/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_arb host tests (slice 6a Task 5).
 *
 * Part 1: the arbiter alone with recording owner callbacks: accept / refuse
 * by priority, bump, round-robin ties, running events, cap clipping,
 * ll_arb_gap, cancel, the main alarm always on the earliest request.
 *
 * Part 2: multilink scenarios with the real ll_conn (+ ll_txq, ll_rxq,
 * ll_csa1, ll_crypt) and the real ll_adv (+ ll_pdu) on a fake radio and a
 * fake one-shot alarm. A small simulator plays the centrals: every link has
 * a central with its own anchor sequence (optionally drifting); a BRX of a
 * link gets the central's packet when its anchor lies in the RX window.
 * The simulator checks after every step that no radio activity overlaps
 * (each alarm fires after the previous event has ended) and that every
 * connection event ends within its cap.
 */
#include <errno.h>
#include <string.h>
#include "test.h"
#include "../ll_adv.h"
#include "../ll_arb.h"
#include "../ll_conn.h"
#include "../ll_csa1.h"
#include "../ll_defs.h"
#include "../ll_flash.h"
#include "../ll_plat.h"
#include "../ll_rxq.h"
#include "../ll_sched.h"
#include "../ll_txq.h"

#define T(us)      ((uint32_t)(us) * LL_TICKS_PER_US)

void aes_ref_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

void ll_plat_aes_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
	aes_ref_encrypt(key, in, out);
}

/* ---------------- fakes ---------------- */

static uint32_t now;

static struct {
	bool selected;
	uint32_t aa, crc;
	int events;
	uint8_t ch;
	uint32_t open, fst, max_ev;
	uint8_t rptr, wptr;
	/* advertising */
	int txrx;
	uint8_t adv_ch;
	int stops;
	int adv_enters;
} rad;

static struct {
	uint32_t tick;
	ll_sched_cb_t cb;
	int ats, cancels;
} sch;

static int locks;
static uint32_t rnd = 12345;

uint32_t ll_radio_now(void) { return now; }
int ll_radio_init(ll_radio_cb_t cb) { (void)cb; return 0; }
void ll_radio_conn_init(void) {}
void ll_radio_conn_select(uint32_t aa, uint32_t crc_init)
{
	rad.selected = true;
	rad.aa = aa;
	rad.crc = crc_init;
}
void ll_radio_conn_event(uint8_t ch, uint32_t open_tick, uint32_t first_timeout_us,
			 uint32_t max_event_us)
{
	CHECK(rad.selected);
	rad.selected = false;
	rad.events++;
	rad.ch = ch;
	rad.open = open_tick;
	rad.fst = first_timeout_us;
	rad.max_ev = max_event_us;
}
void ll_radio_conn_stop(void) {}
void ll_radio_conn_set_sn_init(uint8_t sn) { (void)sn; }
void ll_radio_conn_set_nesn_init(uint8_t nesn) { (void)nesn; }
uint8_t ll_radio_fifo_rptr(void) { return rad.rptr; }
uint8_t ll_radio_fifo_wptr(void) { return rad.wptr; }
void ll_radio_fifo_write(uint8_t idx, uint8_t hdr0, const uint8_t *payload, uint8_t len)
{
	(void)idx; (void)hdr0; (void)payload; (void)len;
}
void ll_radio_fifo_set_wptr(uint8_t wptr) { rad.wptr = wptr; }
void ll_radio_set_adv_channel(uint8_t ch) { rad.adv_ch = ch; }
void ll_radio_tx_then_rx(const uint8_t *pdu, uint8_t len, uint32_t start_tick, uint32_t rx_us)
{
	(void)pdu; (void)len; (void)start_tick; (void)rx_us;
	rad.txrx++;
}
void ll_radio_prepare_rsp(const uint8_t *pdu, uint8_t len) { (void)pdu; (void)len; }
bool ll_radio_tx_rsp_at(uint32_t tick) { (void)tick; return true; }
void ll_radio_stop(void) { rad.stops++; }
void ll_radio_adv_restore(void) {}
void ll_radio_adv_enter(void) { rad.adv_enters++; }
/* the simulator plays every radio activity to its end inside sim_step(),
 * so nothing is ever on air when a flash window opens */
static int flash_aborts;
void ll_radio_flash_abort(void) { CHECK(locks > 0); flash_aborts++; }
void ll_sched_init(void) {}
void ll_sched_at(uint32_t tick, ll_sched_cb_t cb)
{
	sch.tick = tick;
	sch.cb = cb;
	sch.ats++;
}
void ll_sched_cancel(void)
{
	sch.cb = NULL;
	sch.cancels++;
}
uint32_t ll_plat_rand32(void)
{
	rnd = rnd * 1103515245u + 12345u;
	return rnd >> 8;
}
unsigned int ll_plat_lock(void) { locks++; return 0; }
void ll_plat_unlock(unsigned int k) { (void)k; locks--; }

static void fire_alarm(void)
{
	ll_sched_cb_t cb = sch.cb;

	CHECK(cb != NULL);
	if (!cb) {
		return;
	}
	sch.cb = NULL;
	if ((int32_t)(sch.tick - now) > 0) {
		now = sch.tick;
	}
	cb();
}

/* ---------------- part 1: the arbiter alone ---------------- */

static struct {
	int starts;
	uint8_t start_id;
	uint32_t start_cap;
	int bumps;
	uint8_t bump_id;
	uint32_t bumped_mask;
} rec;

static void rec_start(uint8_t id, uint32_t cap_us)
{
	rec.starts++;
	rec.start_id = id;
	rec.start_cap = cap_us;
}

/* chain test: a bumped owner requests chain_req[id] again (if set) */
static struct ll_arb_req chain_req[LL_ARB_IDS];
static bool chain_on[LL_ARB_IDS];
static int bump_depth, bump_depth_max;
static uint8_t bump_order[16];
static int bump_order_n;

static void rec_bumped(uint8_t id)
{
	CHECK(locks > 0);   /* same context as the displacing request */
	bump_depth++;
	if (bump_depth > bump_depth_max) {
		bump_depth_max = bump_depth;
	}
	rec.bumps++;
	rec.bump_id = id;
	rec.bumped_mask |= 1u << id;
	if (bump_order_n < 16) {
		bump_order[bump_order_n++] = id;
	}
	if (chain_on[id]) {
		chain_on[id] = false;
		CHECK(ll_arb_request(id, &chain_req[id]) == 0);
	}
	bump_depth--;
}

static const struct ll_arb_ops rec_ops = {.start = rec_start, .bumped = rec_bumped};

static void arb_reset(void)
{
	now = 1000000;
	memset(&sch, 0, sizeof(sch));
	memset(&rec, 0, sizeof(rec));
	memset(chain_on, 0, sizeof(chain_on));
	bump_depth = bump_depth_max = bump_order_n = 0;
	ll_arb_init(&rec_ops);
}

/* request [alarm, open + min] with alarm = open - lead */
static struct ll_arb_req mk(uint32_t open, uint32_t lead_us, uint32_t min_us, uint32_t max_us,
			    uint8_t prio)
{
	struct ll_arb_req r = {
		.alarm_tick = open - T(lead_us),
		.open_tick = open,
		.min_len_us = min_us,
		.max_len_us = max_us,
		.prio = prio,
	};
	return r;
}

static int req(uint8_t id, struct ll_arb_req r)
{
	unsigned int k = ll_plat_lock();
	int ret = ll_arb_request(id, &r);

	ll_plat_unlock(k);
	return ret;
}

static const uint8_t A = 0, B = LL_ARB_ADV;

static void yield(uint8_t id)
{
	unsigned int k = ll_plat_lock();

	ll_arb_yield(id);
	ll_plat_unlock(k);
}

static void test_accept_refuse_prio(void)
{
	const uint32_t t = 2000000;

	arb_reset();
	CHECK(sch.cb == NULL);
	/* B at [t-500, t+2000]; A far away: both accepted */
	CHECK(req(B, mk(t, 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(req(A, mk(t + T(10000), 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(rec.bumps == 0);
	/* A moves onto B with a lower priority: refused, A has no request */
	CHECK(req(A, mk(t + T(1000), 500, 2000, 10000, LL_ARB_PRIO_ADV)) == -EBUSY);
	CHECK(rec.bumps == 0);
	CHECK(sch.cb != NULL && sch.tick == t - T(500));
	fire_alarm();
	CHECK(rec.starts == 1 && rec.start_id == B);
	/* nothing else is pending: no alarm */
	CHECK(sch.cb == NULL);
	/* touching spans overlap; just after the end does not */
	arb_reset();
	CHECK(req(B, mk(t, 0, 1000, 1000, LL_ARB_PRIO_MUST)) == 0);
	CHECK(req(A, mk(t + T(1000), 0, 100, 100, LL_ARB_PRIO_IDLE)) == -EBUSY);
	CHECK(req(A, mk(t + T(1000) + 1, 0, 100, 100, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(req(A, mk(t - T(200), 0, 200, 100, LL_ARB_PRIO_IDLE)) == -EBUSY);
	CHECK(req(A, mk(t - T(200) - 1, 0, 200, 100, LL_ARB_PRIO_IDLE)) == 0);
	/* every priority level is ordered */
	for (uint8_t lo = LL_ARB_PRIO_ADV; lo < LL_ARB_PRIO_MUST; lo++) {
		arb_reset();
		CHECK(req(B, mk(t, 500, 2000, 2000, (uint8_t)(lo + 1))) == 0);
		CHECK(req(A, mk(t, 500, 2000, 2000, lo)) == -EBUSY);
		CHECK(rec.bumps == 0);
	}
}

static void test_bump(void)
{
	const uint32_t t = 2000000;

	arb_reset();
	CHECK(req(A, mk(t, 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(sch.tick == t - T(500));
	/* B with a higher priority, overlapping: A is bumped */
	CHECK(req(B, mk(t + T(1000), 500, 2000, 10000, LL_ARB_PRIO_ACTIVE)) == 0);
	CHECK(rec.bumps == 1 && rec.bump_id == A);
	/* A's request is gone: the alarm is B's */
	CHECK(sch.tick == t + T(500));
	fire_alarm();
	CHECK(rec.starts == 1 && rec.start_id == B);
	/* one request overlapping two: refused when either wins, else both
	 * bumped */
	if (LL_ARB_IDS >= 3) {
		const uint8_t C = 1;

		arb_reset();
		CHECK(req(A, mk(t, 0, 1000, 1000, LL_ARB_PRIO_IDLE)) == 0);
		CHECK(req(B, mk(t + T(2000), 0, 1000, 1000, LL_ARB_PRIO_SUPERVISION)) == 0);
		CHECK(req(C, mk(t + T(500), 0, 2000, 2000, LL_ARB_PRIO_ACTIVE)) == -EBUSY);
		CHECK(rec.bumps == 0);   /* nothing displaced by a refused request */
		CHECK(req(C, mk(t + T(500), 0, 2000, 2000, LL_ARB_PRIO_MUST)) == 0);
		CHECK(rec.bumps == 2 && rec.bumped_mask == ((1u << A) | (1u << B)));
		CHECK(sch.tick == t + T(500));
	}
}

/* A displaced owner's request that displaces another one does not nest:
 * the second bumped callback runs after the first returned (depth 1). */
static void test_bump_chain(void)
{
	const uint32_t t = 2000000;
	const uint8_t C = 1;

	if (LL_ARB_IDS < 3) {
		return;
	}
	arb_reset();
	CHECK(req(A, mk(t, 0, 1000, 1000, LL_ARB_PRIO_ACTIVE)) == 0);
	CHECK(req(C, mk(t + T(5000), 0, 1000, 1000, LL_ARB_PRIO_IDLE)) == 0);
	/* A, once bumped, moves onto C (ACTIVE beats IDLE) */
	chain_req[A] = mk(t + T(5000), 0, 1000, 1000, LL_ARB_PRIO_ACTIVE);
	chain_on[A] = true;
	CHECK(req(B, mk(t, 0, 1000, 1000, LL_ARB_PRIO_MUST)) == 0);
	CHECK(rec.bumps == 2 && bump_order_n == 2);
	CHECK(bump_order[0] == A && bump_order[1] == C);
	CHECK(bump_depth_max == 1);
	CHECK(sch.tick == t);
	fire_alarm();
	CHECK(rec.start_id == B);
	CHECK(sch.tick == t + T(5000));
	ll_arb_cancel(B);   /* B's event is over */
	fire_alarm();
	CHECK(rec.start_id == A);
}

static void test_round_robin(void)
{
	const uint32_t t = 2000000;

	arb_reset();
	CHECK(req(A, mk(t, 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);
	/* tie, B never yielded: refused */
	CHECK(req(B, mk(t + T(100), 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == -EBUSY);
	/* a refusal alone is a probe: no flag changes, B still loses */
	CHECK(req(B, mk(t + T(150), 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == -EBUSY);
	/* B gives the event up (a yield): now B yielded last, it wins the next
	 * tie, A is bumped (A yields) */
	yield(B);
	CHECK(req(B, mk(t + T(200), 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(rec.bumps == 1 && rec.bump_id == A);
	/* now A yielded last: A wins */
	CHECK(req(A, mk(t + T(300), 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(rec.bumps == 2 && rec.bump_id == B);
	/* B wins again, then A, ... strictly alternating */
	for (int i = 0; i < 6; i++) {
		uint8_t req_id = (i & 1) ? A : B;

		CHECK(req(req_id, mk(t + T(400 + 10 * i), 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);
		CHECK(rec.bump_id == ((i & 1) ? B : A));
	}
	/* a loss at a priority collision counts as a yield too once given up:
	 * A refused by a higher B, A yields, A wins the next tie */
	arb_reset();
	CHECK(req(B, mk(t, 500, 2000, 10000, LL_ARB_PRIO_ACTIVE)) == 0);
	CHECK(req(A, mk(t, 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == -EBUSY);
	yield(A);
	CHECK(req(B, mk(t, 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);   /* replaces itself */
	CHECK(req(A, mk(t, 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(rec.bumps == 1 && rec.bump_id == B);
	/* refused probes (dodge loop, kick probes) without a yield change no
	 * flag: A keeps losing ties against B, however often it probes */
	arb_reset();
	CHECK(req(B, mk(t, 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);
	for (int i = 0; i < 5; i++) {
		CHECK(req(A, mk(t + T(100 * i), 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == -EBUSY);
	}
	CHECK(rec.bumps == 0);
	/* the yield marks only the request that refused A's LAST probe as
	 * not yielded: B (refused A) is reset, a third request is untouched */
	if (LL_ARB_IDS >= 3) {
		const uint8_t C = 1;

		CHECK(req(C, mk(t + T(20000), 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);
		yield(C);   /* C has no recorded refusal: only C is marked */
		yield(A);   /* A's last refusal was by B */
		/* A now wins a tie against B (A yielded, B not) ... */
		CHECK(req(A, mk(t + T(100), 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);
		CHECK(rec.bumps == 1 && rec.bump_id == B);
		/* ... and C (yielded, never reset) wins a tie against A, which
		 * just won and is not yielded */
		CHECK(req(C, mk(t + T(200), 500, 2000, 10000, LL_ARB_PRIO_IDLE)) == 0);
		CHECK(rec.bumps == 2 && rec.bump_id == A);
	}
}

/* A started request is running until its owner requests again or
 * cancels: it cannot be displaced, and its span reaches open + cap. */
static void test_running(void)
{
	const uint32_t t = 2000000;
	const uint32_t reserve = LL_CONN_EVENT_SAFETY_US + LL_CONN_ARM_LEAD_US;

	arb_reset();
	CHECK(req(A, mk(t, 500, 2000, 12000, LL_ARB_PRIO_IDLE)) == 0);
	fire_alarm();
	CHECK(rec.starts == 1 && rec.start_id == A && rec.start_cap == 12000);
	/* MUST cannot displace a running event, also past min_len, up to
	 * the cap plus the clipping reserve (a request accepted while it runs
	 * keeps SAFETY + LEAD between the cap and its own span) */
	CHECK(req(B, mk(t + T(1000), 0, 1000, 1000, LL_ARB_PRIO_MUST)) == -EBUSY);
	CHECK(req(B, mk(t + T(11000), 0, 1000, 1000, LL_ARB_PRIO_MUST)) == -EBUSY);
	CHECK(req(B, mk(t + T(12000 + reserve), 0, 1000, 1000, LL_ARB_PRIO_MUST)) == -EBUSY);
	CHECK(rec.bumps == 0);
	CHECK(req(B, mk(t + T(12000 + reserve) + 1, 0, 1000, 1000, LL_ARB_PRIO_MUST)) == 0);
	CHECK(ll_arb_gap(t, 0, 100) == t + T(13000 + reserve) + 2);
	/* A's next request ends the running state */
	CHECK(req(A, mk(t + T(30000), 500, 2000, 12000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(ll_arb_gap(t, 0, 100) == t);

	/* an alarm that fires while another request is still running bumps
	 * the due request instead of starting it */
	arb_reset();
	CHECK(req(A, mk(t, 0, 1000, 1000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(req(B, mk(t + T(5000), 500, 1000, 1000, LL_ARB_PRIO_MUST)) == 0);
	fire_alarm();
	CHECK(rec.starts == 1 && rec.start_id == A);
	CHECK(sch.tick == t + T(4500));
	fire_alarm();   /* A's owner never requested again (event overran) */
	CHECK(rec.starts == 1 && rec.bumps == 1 && rec.bump_id == B);
	CHECK(sch.cb == NULL);
	ll_arb_cancel(A);
	CHECK(ll_arb_gap(t, 0, 100) == t);
}

static void test_cap_clip(void)
{
	const uint32_t t = 2000000;
	const uint32_t reserve = LL_CONN_EVENT_SAFETY_US + LL_CONN_ARM_LEAD_US;

	/* nothing after: the owner's max */
	arb_reset();
	CHECK(req(A, mk(t, 500, 2000, 14000, LL_ARB_PRIO_IDLE)) == 0);
	fire_alarm();
	CHECK(rec.start_cap == 14000);
	/* the next accepted request opens 9 ms after A's open */
	arb_reset();
	CHECK(req(A, mk(t, 500, 2000, 14000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(req(B, mk(t + T(9000), 500, 2000, 14000, LL_ARB_PRIO_IDLE)) == 0);
	fire_alarm();
	CHECK(rec.start_id == A && rec.start_cap == 9000 - reserve);
	/* the running span follows the clipped cap plus the reserve: it ends
	 * exactly where B's open is (B's own span starts with its lead) */
	CHECK(ll_arb_gap(t, 0, 1) == t + T(9000 + 2000) + 1);
	/* far after: max stays */
	arb_reset();
	CHECK(req(A, mk(t, 500, 2000, 14000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(req(B, mk(t + T(20000), 500, 2000, 14000, LL_ARB_PRIO_IDLE)) == 0);
	fire_alarm();
	CHECK(rec.start_cap == 14000);
	/* the next one only by its open (adv: alarm == open) */
	arb_reset();
	CHECK(req(A, mk(t, 500, 2000, 14000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(req(B, mk(t + T(4000), 0, 4000, 4000, LL_ARB_PRIO_ADV)) == 0);
	fire_alarm();
	CHECK(rec.start_cap == 4000 - reserve);
	/* a cap below the reserve is 0, never negative */
	arb_reset();
	CHECK(req(A, mk(t, 500, 200, 14000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(req(B, mk(t + T(600), 0, 100, 100, LL_ARB_PRIO_ADV)) == 0);
	fire_alarm();
	CHECK(rec.start_cap == 0);
}

static void test_gap(void)
{
	const uint32_t t = 2000000;

	arb_reset();
	CHECK(ll_arb_gap(t, 500, 4000) == t);
	CHECK(req(A, mk(t + T(1000), 500, 2000, 14000, LL_ARB_PRIO_IDLE)) == 0);   /* [t+500, t+3000] */
	/* [t-0, t+4000] overlaps: slide to just after A's span */
	CHECK(ll_arb_gap(t, 0, 4000) == t + T(3000) + 1);
	/* the lead counts too */
	CHECK(ll_arb_gap(t + T(3000), 200, 100) == t + T(3200) + 1);
	CHECK(ll_arb_gap(t + T(3100), 200, 100) == t + T(3200) + 1);   /* only the lead overlaps */
	/* a gap before A that is long enough is used */
	CHECK(ll_arb_gap(t - T(10000), 0, 4000) == t - T(10000));
	CHECK(ll_arb_gap(t - T(3000), 0, 4000) == t + T(3000) + 1);
	/* two spans with a gap that is too short between them */
	CHECK(req(B, mk(t + T(6000), 0, 2000, 2000, LL_ARB_PRIO_IDLE)) == 0);      /* [t+6000, t+8000] */
	CHECK(ll_arb_gap(t, 0, 4000) == t + T(8000) + 1);
	CHECK(ll_arb_gap(t, 0, 2000) == t + T(3000) + 1);
	ll_arb_cancel(B);
	CHECK(ll_arb_gap(t, 0, 4000) == t + T(3000) + 1);
}

static void test_cancel_and_alarm(void)
{
	const uint32_t t = 2000000;
	int c;

	arb_reset();
	CHECK(req(A, mk(t + T(10000), 500, 1000, 1000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(req(B, mk(t, 0, 1000, 1000, LL_ARB_PRIO_ADV)) == 0);
	CHECK(sch.cb != NULL && sch.tick == t);
	ll_arb_cancel(B);
	CHECK(sch.cb != NULL && sch.tick == t + T(9500));
	c = sch.cancels;
	ll_arb_cancel(A);
	CHECK(sch.cb == NULL && sch.cancels == c + 1);
	/* cancel of an id without a request: no alarm traffic */
	c = sch.cancels;
	ll_arb_cancel(A);
	CHECK(sch.cancels == c && sch.cb == NULL);
	/* a request moved later: the alarm follows */
	CHECK(req(A, mk(t, 500, 1000, 1000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(req(A, mk(t + T(5000), 500, 1000, 1000, LL_ARB_PRIO_IDLE)) == 0);
	CHECK(sch.tick == t + T(4500));
	/* a bumped request is gone */
	CHECK(req(B, mk(t + T(4000), 0, 2000, 2000, LL_ARB_PRIO_MUST)) == 0);
	CHECK(rec.bumps == 1);
	CHECK(sch.tick == t + T(4000));
	/* init drops everything */
	ll_arb_init(&rec_ops);
	CHECK(sch.cb == NULL);
	CHECK(ll_arb_gap(t, 0, 100000) == t);

	/* pseudo-random sequence: the alarm always is the earliest accepted
	 * request (the model follows the return values and the bumps) */
	{
		bool acc[LL_ARB_IDS];
		uint32_t al[LL_ARB_IDS];

		arb_reset();
		memset(acc, 0, sizeof(acc));
		for (int i = 0; i < 3000; i++) {
			uint8_t id = (uint8_t)(ll_plat_rand32() % LL_ARB_IDS);
			uint32_t open = t + T(ll_plat_rand32() % 60000);
			uint8_t prio = (uint8_t)(ll_plat_rand32() % 5);
			bool cancel = ll_plat_rand32() % 5 == 0;
			bool any = false;
			uint32_t best = 0;

			rec.bumped_mask = 0;
			if (cancel) {
				ll_arb_cancel(id);
				acc[id] = false;
			} else {
				acc[id] = req(id, mk(open, 500, 1500, 3000, prio)) == 0;
				al[id] = open - T(500);
			}
			for (uint8_t j = 0; j < LL_ARB_IDS; j++) {
				if (rec.bumped_mask & (1u << j)) {
					acc[j] = false;
				}
			}
			for (uint8_t j = 0; j < LL_ARB_IDS; j++) {
				if (acc[j] && (!any || (int32_t)(al[j] - best) < 0)) {
					best = al[j];
					any = true;
				}
			}
			CHECK(any == (sch.cb != NULL));
			if (any) {
				CHECK(sch.tick == best);
			}
			/* accepted requests never overlap */
			for (uint8_t j = 0; j < LL_ARB_IDS; j++) {
				for (uint8_t k = (uint8_t)(j + 1); k < LL_ARB_IDS; k++) {
					if (acc[j] && acc[k]) {
						int32_t d = (int32_t)(al[j] - al[k]);

						CHECK(d > (int32_t)T(2000) || d < -(int32_t)T(2000));
					}
				}
			}
		}
	}
}

/* ---------------- part 2: multilink scenarios ---------------- */

static bool busy[LL_MAX_CONN];
static uint32_t coll_base[LL_MAX_CONN];   /* stats are cumulative since boot */
static uint16_t sim_win_offset;           /* WinOffset of the next sim_connect */
static int disconnects[LL_MAX_CONN];
static bool auto_release = true;

static void on_evt(uint8_t link, enum ll_conn_evt what, const void *arg)
{
	(void)arg;
	if (what == LL_CONN_EVT_DISCONNECTED) {
		disconnects[link]++;
		if (auto_release) {
			ll_rxq_reset(link);
			ll_conn_release(link);
		}
	}
}

static bool hook_busy(uint8_t link) { return busy[link]; }

static void sim_start(uint8_t id, uint32_t cap_us)
{
	if (id == LL_ARB_ADV) {
		ll_adv_arb_start(cap_us);
	} else {
		ll_conn_arb_start(id, cap_us);
	}
}

static void sim_bumped(uint8_t id)
{
	if (id == LL_ARB_ADV) {
		ll_adv_arb_bumped();
	} else {
		ll_conn_arb_bumped(id);
	}
}

static const struct ll_arb_ops sim_ops = {.start = sim_start, .bumped = sim_bumped};

/* central of link k: event e has its anchor at base + e * ival +
 * e * drift_milli / 1000 (ticks) */
static struct central {
	bool present;
	uint32_t aa;
	uint32_t base;
	uint32_t ival;
	int32_t drift_milli;
} cen[LL_MAX_CONN];

static struct {
	uint32_t busy_until;
	int listened[LL_MAX_CONN];
	int rx[LL_MAX_CONN];
	int last_ev[LL_MAX_CONN];   /* counter of the last listened event, -1 none */
	int max_gap[LL_MAX_CONN];
	int adv_events;
	int adv_pdus;
	uint32_t adv_last, adv_max_gap;
	int overlaps, over_cap;
	/* flash window: an operation is running (sim_flash_op), and radio
	 * activity seen meanwhile (must stay 0) */
	bool in_op;
	int radio_in_op;
	/* the next adv channel receives this CONNECT_IND instead of nothing */
	const uint8_t *connect_pdu;
	uint32_t connect_end;
} sim;

static const uint8_t all37[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
static const uint8_t adva[6] = {0x01, 0x02, 0x03, 0x38, 0xC1, 0xA4};

static void sim_reset(void)
{
	struct ll_conn_ops ops = {.evt = on_evt, .busy = hook_busy};

	now = 1000000;
	memset(&rad, 0, sizeof(rad));
	memset(&sch, 0, sizeof(sch));
	memset(&sim, 0, sizeof(sim));
	memset(cen, 0, sizeof(cen));
	memset(busy, 0, sizeof(busy));
	memset(disconnects, 0, sizeof(disconnects));
	auto_release = true;
	ll_arb_init(&sim_ops);
	ll_conn_init(&ops);
	ll_adv_init(adva, NULL);
	ll_flash_reset();
	sim_win_offset = 0;
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		struct ll_conn_stats st;

		ll_rxq_reset(i);
		sim.last_ev[i] = -1;
		ll_conn_get_stats(i, &st);
		coll_base[i] = st.collisions;
	}
}

static void sim_run_until(uint32_t t);

/* Start link k (interval in 1.25 ms units) with its CONNECT_IND ending at
 * `end`; the central's event 0 anchor lies 100 us into the transmit
 * window (WinSize 1, WinOffset sim_win_offset). The simulation runs up to
 * `end` first (the CONNECT_IND is received while the radio is free). */
static int sim_connect(uint8_t k, uint32_t end, uint16_t interval, uint16_t latency,
		       uint16_t timeout, int32_t drift_milli)
{
	struct ll_connect_ind ci;
	int ret;

	memset(&ci, 0, sizeof(ci));
	ci.aa = 0x50000000u + 0x1111u * (k + 1);
	ci.crc_init = 0x555555u + k;
	ci.win_size = 1;
	ci.win_offset = sim_win_offset;
	ci.interval = interval;
	ci.latency = latency;
	ci.timeout = timeout;
	memcpy(ci.chm, all37, 5);
	ci.hop = 7;
	ci.sca = 1;
	sim_run_until(end);
	if ((int32_t)(now - end) > 0 && getenv("ARB_DBG")) {
		printf("connect late: now %u end %u\n", (unsigned)now, (unsigned)end);
	}
	CHECK((int32_t)(now - end) <= 0);
	now = end;
	ret = ll_conn_start(&ci, end);
	if (ret >= 0) {
		cen[ret].present = true;
		cen[ret].aa = ci.aa;
		cen[ret].base = end + T(1250 + 100) + T(1250u * sim_win_offset);
		cen[ret].ival = T(1250u * interval);
		cen[ret].drift_milli = drift_milli;
	}
	return ret;
}

static uint32_t cen_anchor(uint8_t k, int e)
{
	return cen[k].base + (uint32_t)e * cen[k].ival +
	       (uint32_t)(int32_t)(((int64_t)e * cen[k].drift_milli) / 1000);
}

static int link_of_aa(uint32_t aa)
{
	for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
		if (cen[k].aa == aa && ll_conn_active(k)) {
			return k;
		}
	}
	return -1;
}

/* Fire the next alarm and play whatever it starts. */
static void sim_step(void)
{
	int ev = rad.events, tx = rad.txrx;
	struct ll_adv_stats as0, as1;

	if (!sch.cb) {
		CHECK(sch.cb != NULL);
		return;
	}
	if ((int32_t)(sch.tick - sim.busy_until) < 0) {
		sim.overlaps++;
		if (getenv("ARB_DBG")) {
			printf("overlap: alarm %u busy_until %u\n", (unsigned)sch.tick,
			       (unsigned)sim.busy_until);
		}
	}
	ll_adv_get_stats(&as0);
	fire_alarm();
	ll_adv_get_stats(&as1);
	if (sim.in_op && (rad.events != ev || rad.txrx != tx)) {
		sim.radio_in_op++;
	}
	if (rad.events != ev) {
		int k = link_of_aa(rad.aa);
		int e;
		uint32_t a;

		CHECK(k >= 0);
		if (k < 0) {
			return;
		}
		e = ll_conn_event_counter((uint8_t)k);
		a = cen_anchor((uint8_t)k, e);
		sim.listened[k]++;
		if (sim.last_ev[k] >= 0 && e - sim.last_ev[k] > sim.max_gap[k]) {
			sim.max_gap[k] = e - sim.last_ev[k];
		}
		sim.last_ev[k] = e;
		if (cen[k].present && (int32_t)(a - rad.open) >= 0 &&
		    (int32_t)(rad.open + T(rad.fst) - (a + T(LL_CONN_SYNC_US))) >= 0) {
			uint8_t pdu[2] = {0x01, 0};
			uint32_t open = rad.open, max_ev = rad.max_ev;

			now = a + T(LL_CONN_SYNC_US);
			ll_conn_radio_evt(LL_RADIO_CONN_RX, pdu, 2, now);
			now += T(400);   /* our response, the event ends */
			if ((int32_t)(now - (open + T(max_ev))) > 0) {
				sim.over_cap++;
			}
			sim.rx[k]++;
			ll_conn_radio_evt(LL_RADIO_CONN_DONE, NULL, 1, now);
		} else {
			now = rad.open + T(rad.fst);
			ll_conn_radio_evt(LL_RADIO_CONN_DONE, NULL, 0, now);
		}
		sim.busy_until = now;
	} else if (rad.txrx != tx) {
		uint32_t start = now;

		if (as1.events != as0.events) {   /* an event's first channel */
			if (sim.adv_events > 0 && start - sim.adv_last > sim.adv_max_gap) {
				sim.adv_max_gap = start - sim.adv_last;
			}
			sim.adv_last = start;
			sim.adv_events++;
		}
		/* the channels the running request covers follow back to back
		 * (700 us each); a sliced event continues at a later alarm */
		for (;;) {
			int t0 = rad.txrx;

			sim.adv_pdus++;
			now += T(700);
			if (sim.connect_pdu) {
				const uint8_t *pdu = sim.connect_pdu;

				sim.connect_pdu = NULL;
				sim.connect_end = now;
				ll_adv_radio_evt(LL_RADIO_RX_OK, pdu, 36, now);
				break;
			}
			ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, now);
			if (rad.txrx == t0) {
				break;
			}
		}
		sim.busy_until = now;
	}
}

/* Run until link k's event counter reaches n (or the link ends). */
static void sim_run_link(uint8_t k, uint16_t n)
{
	for (int guard = 0; guard < 200000 && ll_conn_active(k) &&
	     (uint16_t)(ll_conn_event_counter(k)) < n; guard++) {
		sim_step();
	}
}

static void sim_run_until(uint32_t t)
{
	for (int guard = 0; guard < 200000 && sch.cb && (int32_t)(sch.tick - t) < 0; guard++) {
		sim_step();
	}
}

static uint32_t coll(uint8_t k)
{
	struct ll_conn_stats s;

	ll_conn_get_stats(k, &s);
	return s.collisions - coll_base[k];
}

/* Two links with the same interval whose anchors drift slowly through each
 * other (link 1 starts 3 ms before link 0 and gains 2 us per event, 133
 * ppm): over 2000 events each listens at least every second planned event,
 * neither times out, no radio overlap. A third link (N >= 3), far away in
 * phase, is never disturbed. */
static void test_drift_pair(void)
{
	const uint32_t t0 = 2000000;
	uint32_t c0, c1;

	if (LL_MAX_CONN < 2) {
		return;
	}
	sim_reset();
	CHECK(sim_connect(0, t0, 12, 0, 400, 0) == 0);
	CHECK(sim_connect(1, t0 + T(12000), 12, 0, 400, 32000) == 1);
	if (LL_MAX_CONN >= 3) {
		CHECK(sim_connect(2, t0 + T(22500), 12, 0, 400, 0) == 2);
	}
	sim_run_link(0, 2000);
	sim_run_link(1, 2000);
	c0 = coll(0);
	c1 = coll(1);
	printf("  drift_pair n%d: listened %d/%d, rx %d/%d, collisions %u/%u, max gap %d/%d\n",
	       LL_MAX_CONN, sim.listened[0], sim.listened[1], sim.rx[0], sim.rx[1],
	       (unsigned)c0, (unsigned)c1, sim.max_gap[0], sim.max_gap[1]);
	CHECK(ll_conn_active(0) && ll_conn_active(1));
	CHECK(disconnects[0] == 0 && disconnects[1] == 0);
	CHECK(ll_conn_event_counter(0) >= 2000 && ll_conn_event_counter(1) >= 2000);
	CHECK(sim.max_gap[0] <= 2 && sim.max_gap[1] <= 2);
	CHECK(c0 > 0 && c1 > 0);              /* the anchors did cross */
	CHECK(sim.rx[0] == sim.listened[0] && sim.rx[1] == sim.listened[1]);
	CHECK(sim.overlaps == 0 && sim.over_cap == 0);
	if (LL_MAX_CONN >= 3) {
		CHECK(ll_conn_active(2) && coll(2) == 0 && sim.max_gap[2] == 1);
	}
}

/* An idle link with latency 4 (15 ms) meets an active link (75 ms, busy)
 * whose events land on its latest allowed event: it dodges to an earlier
 * event inside its latency window, no collision counted, the active link
 * listens to every event. */
static void test_dodge(void)
{
	const uint32_t t0 = 2000000;
	struct ll_conn_stats s0;
	int dodges = 0, prev = -1;

	if (LL_MAX_CONN < 2) {
		return;
	}
	sim_reset();
	CHECK(sim_connect(0, t0, 12, 4, 400, 0) == 0);
	busy[1] = true;
	/* link 1's window event 0 lies 0.5 ms after a listened event of link 0
	 * (WinOffset 25 ms, so link 0 can still move), its events (75 ms) then
	 * land on every 5th event of link 0. Slice 7: link 0 skips only after
	 * its latency holdoff (events 0..h - 1 listened, then h + 4, h + 9,
	 * ...), so link 1 connects 5 events before event h + 9 (h = 67). */
	{
		uint32_t h = (LL_CONN_LATENCY_HOLDOFF_MS * 1000u - 1350u + 14999u) / 15000u;

		sim_win_offset = 20;
		CHECK(sim_connect(1, t0 + T(15000) * (h + 4) + T(50000 + 500), 60, 0, 400, 0) == 1);
	}
	sim_win_offset = 0;
	prev = sim.last_ev[0];
	for (int guard = 0; guard < 100000 && ll_conn_event_counter(0) < 600; guard++) {
		int ev = rad.events;

		sim_step();
		if (rad.events != ev && link_of_aa(rad.aa) == 0) {
			int e = sim.last_ev[0];

			if (prev >= 0 && e - prev < 5) {
				dodges++;
			}
			if (getenv("ARB_DBG")) {
				printf("link0 e=%d\n", e);
			}
			prev = e;
		}
	}
	ll_conn_get_stats(0, &s0);
	s0.collisions -= coll_base[0];
	printf("  dodge n%d: link 0 listened %d skipped %u dodges %d collisions %u; "
	       "link 1 listened %d gap %d\n", LL_MAX_CONN, sim.listened[0], (unsigned)s0.skipped,
	       dodges, (unsigned)s0.collisions, sim.listened[1], sim.max_gap[1]);
	CHECK(dodges > 0);
	CHECK(s0.collisions == 0 && coll(1) == 0);
	CHECK(sim.max_gap[0] <= 5);
	CHECK(sim.max_gap[1] == 1);
	CHECK(s0.skipped > 0);
	CHECK(sim.overlaps == 0 && sim.over_cap == 0);
	CHECK(disconnects[0] == 0 && disconnects[1] == 0);
}

/* A new link's transmit-window event (MUST) beats an active and an idle
 * link: it is issued in its window, the other link yields that event. */
static void test_window_wins(void)
{
	const uint32_t t0 = 2000000;
	uint32_t c0, c1;
	int ev2;

	if (LL_MAX_CONN < 3) {
		return;
	}
	sim_reset();
	CHECK(sim_connect(0, t0, 12, 0, 400, 0) == 0);              /* idle */
	busy[1] = true;
	CHECK(sim_connect(1, t0 + T(5000), 12, 0, 400, 0) == 1);    /* active */
	sim_run_link(0, 20);
	sim_run_link(1, 20);
	c0 = coll(0);
	c1 = coll(1);
	CHECK(c0 == 0 && c1 == 0);
	/* link 2's transmit window lies 0.8 ms before link 1's event after
	 * next: its window event wins, link 1 yields */
	{
		uint32_t a1 = cen_anchor(1, ll_conn_event_counter(1) + 1);

		CHECK(sim_connect(2, a1 - T(1350 + 800), 12, 0, 400, 0) == 2);
	}
	ev2 = sim.listened[2];
	while (sim.listened[2] == ev2 && ll_conn_active(2)) {
		sim_step();
	}
	CHECK(sim.last_ev[2] == 0 && sim.rx[2] == ev2 + 1);   /* window event not lost */
	sim_run_link(1, (uint16_t)(ll_conn_event_counter(1) + 2));
	CHECK(coll(1) > c1);                                  /* the active link yielded */
	ll_conn_end(2, 0x13);
	/* again, on the idle link 0 */
	{
		uint32_t a0 = cen_anchor(0, ll_conn_event_counter(0) + 1);

		CHECK(sim_connect(2, a0 - T(1350 + 800), 12, 0, 400, 0) == 2);
	}
	sim.last_ev[2] = -1;
	ev2 = sim.listened[2];
	while (sim.listened[2] == ev2 && ll_conn_active(2)) {
		sim_step();
	}
	CHECK(sim.last_ev[2] == 0 && sim.rx[2] == ev2 + 1);
	sim_run_link(0, (uint16_t)(ll_conn_event_counter(0) + 2));
	CHECK(coll(0) > c0);
	ll_conn_end(2, 0x13);
	printf("  window_wins n%d: collisions link0 %u link1 %u\n", LL_MAX_CONN,
	       (unsigned)coll(0), (unsigned)coll(1));
	CHECK(sim.overlaps == 0 && sim.over_cap == 0);
	CHECK(disconnects[0] == 0 && disconnects[1] == 0);
}

/* An idle link overlapping an always-active link on every event loses
 * every tie of priorities, but its supervision priority wins before the
 * timeout: it never times out. */
static void test_supervision_rescue(void)
{
	const uint32_t t0 = 2000000;
	/* timeout 1 s at 15 ms: 66 events; supervision priority from 2
	 * intervals before */
	const uint16_t timeout = 100;

	if (LL_MAX_CONN < 2) {
		return;
	}
	sim_reset();
	CHECK(sim_connect(0, t0, 12, 0, timeout, 0) == 0);
	busy[1] = true;
	CHECK(sim_connect(1, t0 + T(14000), 12, 0, timeout, 0) == 1);
	sim_run_link(1, 2000);
	printf("  supervision n%d: idle link listened %d (max gap %d), active link listened %d "
	       "(max gap %d), collisions %u/%u\n", LL_MAX_CONN, sim.listened[0], sim.max_gap[0],
	       sim.listened[1], sim.max_gap[1], (unsigned)coll(0), (unsigned)coll(1));
	CHECK(ll_conn_active(0) && ll_conn_active(1));
	CHECK(disconnects[0] == 0 && disconnects[1] == 0);
	CHECK(sim.listened[0] >= 2000 / 66);
	CHECK(sim.max_gap[0] <= 66);
	CHECK(sim.max_gap[0] > 2);       /* the active link really dominated */
	CHECK(coll(0) > 1000);
	CHECK(sim.overlaps == 0 && sim.over_cap == 0);
}

/* Advertising with idle links: slides into gaps when its nominal time
 * collides, never starves (at least one adv event per 3 adv intervals). */
static void test_adv_gaps(void)
{
	const uint32_t t0 = 2000000;
	struct ll_adv_params p = {.interval_min = 0x00A0, .interval_max = 0x00A0,
				  .type = 0, .chan_map = 7};
	struct ll_adv_stats as;
	const uint32_t ival = T(100000 + 10000);   /* interval + max advDelay */
	uint8_t links = LL_MAX_CONN < 3 ? LL_MAX_CONN : 3;

	sim_reset();
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	now = t0 - T(5000);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	/* idle links at 15 ms, 3 ms apart (the enable refusal while connected
	 * is Task 6; the links start after the enable) */
	for (uint8_t k = 0; k < links; k++) {
		CHECK(sim_connect(k, t0 + T(3000) * k, 12, 0, 400, 0) == k);
	}
	sim_run_until(t0 + T(5000000));
	ll_adv_get_stats(&as);
	printf("  adv_gaps n%d: adv events %d (slid %u, dropped %u), max gap %u us\n",
	       LL_MAX_CONN, sim.adv_events, (unsigned)as.slid, (unsigned)as.dropped,
	       (unsigned)(sim.adv_max_gap / LL_TICKS_PER_US));
	CHECK(sim.adv_events >= 5000 / 110 - 1);
	CHECK(sim.adv_max_gap <= 3 * ival);
	CHECK(as.slid > 0);
	CHECK(sim.overlaps == 0 && sim.over_cap == 0);
	for (uint8_t k = 0; k < links; k++) {
		CHECK(ll_conn_active(k) && sim.max_gap[k] <= 2);
	}
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
}

/* Advertising is dropped (not delayed beyond the next interval) while a
 * long request blocks the air, and resumes afterwards. */
static void test_adv_dropped(void)
{
	const uint32_t t0 = 2000000;
	struct ll_adv_params p = {.interval_min = 0x0020, .interval_max = 0x0020,   /* 20 ms */
				  .type = 0, .chan_map = 7};
	struct ll_adv_stats a0, a1;
	struct ll_arb_req r = {.alarm_tick = t0, .open_tick = t0, .min_len_us = 100000,
			       .max_len_us = 100000, .prio = LL_ARB_PRIO_SUPERVISION};
	int txrx;

	sim_reset();
	ll_adv_get_stats(&a0);
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	now = t0 - T(1000) - T(500);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);   /* first event at t0 - 500 us */
	/* a foreign 100 ms block from t0 (id 0, no link behind it): the adv
	 * request overlaps and is displaced; the gap after the block lies
	 * beyond the next adv interval, so events are dropped */
	{
		unsigned int k = ll_plat_lock();

		CHECK(ll_arb_request(0, &r) == 0);
		ll_plat_unlock(k);
	}
	ll_adv_get_stats(&a1);
	CHECK(a1.dropped - a0.dropped >= 1);
	txrx = rad.txrx;
	/* nothing transmits during the block */
	CHECK(sch.cb != NULL);
	CHECK((int32_t)(sch.tick - t0) >= 0);
	{
		unsigned int k = ll_plat_lock();

		ll_arb_cancel(0);   /* the block owner gives up before its alarm */
		ll_plat_unlock(k);
	}
	/* advertising resumes */
	sim_run_until(t0 + T(300000));
	CHECK(rad.txrx > txrx);
	ll_adv_get_stats(&a1);
	printf("  adv_dropped n%d: dropped %u, events %u\n", LL_MAX_CONN,
	       (unsigned)(a1.dropped - a0.dropped), (unsigned)(a1.events - a0.events));
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
	CHECK(sch.cb == NULL);
}

/* A starving advertiser asks at ACTIVE after ADV_STARVE_DROPS dropped
 * events: an idle (or active) block is displaced then, so advertising
 * resumes within a few intervals instead of waiting for the block. */
static void test_adv_starve_boost(void)
{
	const uint32_t t0 = 2000000;
	struct ll_adv_params p = {.interval_min = 0x0020, .interval_max = 0x0020,   /* 20 ms */
				  .type = 0, .chan_map = 7};
	struct ll_adv_stats a0, a1;
	struct ll_arb_req r = {.alarm_tick = t0, .open_tick = t0, .min_len_us = 200000,
			       .max_len_us = 200000, .prio = LL_ARB_PRIO_IDLE};
	int txrx;

	sim_reset();
	ll_adv_get_stats(&a0);
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	now = t0 - T(1000) - T(500);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	{
		unsigned int k = ll_plat_lock();

		CHECK(ll_arb_request(0, &r) == 0);   /* a foreign 200 ms idle block */
		ll_plat_unlock(k);
	}
	txrx = rad.txrx;
	/* the block's own alarm comes first; nothing transmits before an
	 * advertising event displaced it (alarm ~ the block's) */
	for (int i = 0; i < 50 && rad.txrx == txrx && sch.cb; i++) {
		if ((int32_t)(sch.tick - t0) >= 0 && (int32_t)(sch.tick - (t0 + T(1000))) < 0) {
			break;   /* the block's alarm: it was not displaced */
		}
		sim_step();
	}
	ll_adv_get_stats(&a1);
	printf("  adv_starve_boost n%d: dropped %u then adv tx %d at +%d us\n", LL_MAX_CONN,
	       (unsigned)(a1.dropped - a0.dropped), rad.txrx - txrx,
	       (int)((int32_t)(now - t0) / (int32_t)LL_TICKS_PER_US));
	CHECK(rad.txrx > txrx);
	CHECK(a1.dropped - a0.dropped >= 1 && a1.dropped - a0.dropped <= 4);
	CHECK((int32_t)(now - (t0 + T(100000))) < 0);   /* long before the block ends */
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
}

/* Three busy links at 7.5 ms (the air is full with their own events) plus
 * advertising every 100 ms: the links stay up and keep listening, and
 * advertising still gets out every few intervals (sliced channels and the
 * starvation boost), never overlapping a link's event. */
static void test_busy_links_adv(void)
{
	const uint32_t t0 = 2000000;
	/* non-connectable: with 3 of 3 links taken connectable advertising
	 * is refused (0x09); the air time is the same */
	struct ll_adv_params p = {.interval_min = 0x00A0, .interval_max = 0x00A0,
				  .type = 3, .chan_map = 7};
	struct ll_adv_stats a0, a1;
	const uint32_t ival = T(100000 + 10000);
	uint8_t links = LL_MAX_CONN < 3 ? LL_MAX_CONN : 3;
	int max_gap = 0;

	if (LL_MAX_CONN < 3) {
		return;
	}
	sim_reset();
	ll_adv_get_stats(&a0);
	for (uint8_t k = 0; k < links; k++) {
		busy[k] = true;
		CHECK(sim_connect(k, t0 + T(2500) * k, 6, 0, 100, 0) == k);
	}
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	sim_run_until(t0 + T(10000000));
	ll_adv_get_stats(&a1);
	for (uint8_t k = 0; k < links; k++) {
		if (sim.max_gap[k] > max_gap) {
			max_gap = sim.max_gap[k];
		}
	}
	printf("  busy_links_adv n%d: adv events %d pdus %d (slid %u dropped %u cut %u), "
	       "max adv gap %u us; links listened %d/%d/%d coll %u/%u/%u max gap %d\n",
	       LL_MAX_CONN, sim.adv_events, sim.adv_pdus, (unsigned)(a1.slid - a0.slid),
	       (unsigned)(a1.dropped - a0.dropped), (unsigned)(a1.cut - a0.cut),
	       (unsigned)(sim.adv_max_gap / LL_TICKS_PER_US), sim.listened[0], sim.listened[1],
	       sim.listened[2], (unsigned)coll(0), (unsigned)coll(1), (unsigned)coll(2), max_gap);
	CHECK(sim.overlaps == 0 && sim.over_cap == 0);
	for (uint8_t k = 0; k < links; k++) {
		CHECK(ll_conn_active(k) && disconnects[k] == 0);
		CHECK(sim.listened[k] > 10000 / 7.5 / 4);
	}
	CHECK(sim.adv_events >= 10000 / 110 / 4);
	CHECK(sim.adv_max_gap <= 4 * ival);
	CHECK(a1.stuck == a0.stuck);
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
}

/* Advertising while connected: enabling is allowed while a link is free,
 * connectable advertising is refused with 0x09 once every link is taken
 * (non-connectable still allowed); a CONNECT_IND during advertising while
 * connected creates the next link and stops advertising. */
static void test_adv_while_connected(void)
{
	const uint32_t t0 = 2000000;
	struct ll_adv_params p = {.interval_min = 0x00A0, .interval_max = 0x00A0,
				  .type = 0, .chan_map = 7};
	uint8_t ci_pdu[36] = {0x05, 34, 0x11, 0x12, 0x13, 0x14, 0x15, 0xD6};
	int ev;

	sim_reset();
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	CHECK(sim_connect(0, t0, 12, 0, 400, 0) == 0);
	sim_run_link(0, 3);
	if (LL_MAX_CONN == 1) {
		CHECK(ll_adv_enable(true) == LL_ST_CONN_LIMIT);
		CHECK(!ll_adv_is_enabled());
		p.type = 3;   /* non-connectable: allowed */
		CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
		CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
		CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
		ll_conn_end(0, 0x13);
		return;
	}
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	/* CONNECT_IND for us on the first adv channel */
	memcpy(&ci_pdu[8], adva, 6);
	ci_pdu[14] = 0x22; ci_pdu[15] = 0x22; ci_pdu[16] = 0x00; ci_pdu[17] = 0x50;   /* AA */
	ci_pdu[18] = 0x56; ci_pdu[19] = 0x55; ci_pdu[20] = 0x55;   /* CRCInit */
	ci_pdu[21] = 1;                       /* WinSize */
	ci_pdu[24] = 12;                      /* Interval */
	ci_pdu[28] = (uint8_t)(400 & 0xFF); ci_pdu[29] = (uint8_t)(400 >> 8);
	memcpy(&ci_pdu[30], all37, 5);
	ci_pdu[35] = 7 | (1 << 5);
	sim.connect_pdu = ci_pdu;
	for (int i = 0; i < 1000 && sim.connect_pdu; i++) {
		sim_step();
	}
	CHECK(sim.connect_pdu == NULL);
	CHECK(ll_conn_active(1) && ll_conn_count() == 2);
	CHECK(!ll_adv_is_enabled());
	cen[1].present = true;
	cen[1].aa = 0x50002222u;
	cen[1].base = sim.connect_end + T(1250 + 100);
	cen[1].ival = T(1250u * 12);
	ev = sim.rx[1];
	sim_run_link(1, 20);
	CHECK(sim.rx[1] > ev + 10);           /* link 1 follows its central */
	CHECK(ll_conn_active(0) && disconnects[0] == 0 && disconnects[1] == 0);
	CHECK(sim.overlaps == 0 && sim.over_cap == 0);
	/* fill the remaining links: connectable advertising is refused with
	 * 0x09, non-connectable and scannable ones are allowed */
	for (uint8_t k = 2; k < LL_MAX_CONN; k++) {
		CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
		CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
		CHECK(sim_connect(k, now + T(3000), 12, 0, 400, 0) == k);
	}
	CHECK(ll_conn_count() == LL_MAX_CONN);
	CHECK(ll_adv_enable(true) == LL_ST_CONN_LIMIT);
	CHECK(!ll_adv_is_enabled());
	p.type = 3;
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
	p.type = 2;
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
	/* an ended link awaiting release is still taken */
	auto_release = false;
	ll_conn_end(1, 0x13);
	p.type = 0;
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	CHECK(ll_adv_enable(true) == LL_ST_CONN_LIMIT);
	ll_rxq_reset(1);
	ll_conn_release(1);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
	auto_release = true;
	for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
		ll_conn_end(k, 0x13);
	}
}

/* ll_conn_init / ll_adv_reset leave the arbiter consistent: no request,
 * no alarm left behind. */
static void test_reset_consistent(void)
{
	struct ll_conn_ops ops = {.evt = on_evt, .busy = hook_busy};
	struct ll_adv_params p = {.interval_min = 0x00A0, .interval_max = 0x00A0,
				  .type = 0, .chan_map = 7};

	sim_reset();
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	CHECK(sim_connect(0, 3000000, 12, 0, 400, 0) == 0);
	CHECK(sch.cb != NULL);
	ll_adv_reset();
	CHECK(sch.cb != NULL);   /* the link's request stays */
	ll_conn_init(&ops);
	CHECK(sch.cb == NULL);
	CHECK(ll_arb_gap(3000000, 0, 1000000) == 3000000);
}

/* ---------------- flash window (ll_flash.h) ---------------- */

static uint32_t fw_wait_max_us;

static void sim_advance(uint32_t t)
{
	sim_run_until(t);
	if ((int32_t)(t - now) > 0) {
		now = t;
	}
}

/* One flash operation of op_us as the B91 glue runs it: ask for the
 * window, wait 1 ms between attempts (the radio keeps running), then the
 * operation (no radio activity allowed), then close. */
static void sim_flash_op(uint32_t op_us)
{
	uint32_t waited = 0;

	for (;;) {
		unsigned int k = ll_plat_lock();
		bool ok = ll_flash_open(now, waited);

		ll_plat_unlock(k);
		if (ok) {
			break;
		}
		sim_advance(now + T(1000));
		waited += 1000;
	}
	if (waited > fw_wait_max_us) {
		fw_wait_max_us = waited;
	}
	sim.in_op = true;
	sim_advance(now + T(op_us));
	sim.in_op = false;
	{
		unsigned int k = ll_plat_lock();

		ll_flash_close();
		ll_plat_unlock(k);
	}
}

/* A chain of back-to-back flash operations (26 ms erases and 2.5 ms page
 * writes, as an image upload or NVS GC does) against links at the
 * mcumgr parameters (7.5 ms, latency 0, 420 ms) and at an idle keyboard's
 * (15 ms, latency 30, 1 s and 940 ms), up to 3 links 2.5 ms apart: no radio activity
 * inside any operation, the links never time out, an event gets through
 * between operations whenever a link needs one (never forced), and the
 * waits stay within one interval per link. */
static void test_flash_chain(void)
{
	static const struct {
		uint16_t interval, latency, timeout;
	} cfg[] = {
		{6, 0, 42},
		{12, 30, 100},
		/* 940 ms: the latency window (465 ms) outlasts the ready limit
		 * (470 - 30 ms), so a wait needs the kicked event */
		{12, 30, 94},
	};
	uint8_t links = LL_MAX_CONN < 3 ? LL_MAX_CONN : 3;

	for (unsigned c = 0; c < sizeof(cfg) / sizeof(cfg[0]); c++) {
		const uint32_t t0 = 2000000;
		struct ll_flash_stats f0, f;
		struct ll_conn_stats st;
		uint32_t paused = 0;
		int rx0[LL_MAX_CONN];

		sim_reset();
		fw_wait_max_us = 0;
		ll_flash_get_stats(&f0);
		for (uint8_t k = 0; k < links; k++) {
			CHECK(sim_connect(k, t0 + T(2500) * k, cfg[c].interval, cfg[c].latency,
					  cfg[c].timeout, 0) == k);
		}
		sim_advance(t0 + T(1500000));      /* established, past the holdoff */
		for (uint8_t k = 0; k < links; k++) {
			rx0[k] = sim.rx[k];
		}
		for (int i = 0; i < 300; i++) {
			sim_flash_op(i % 3 == 0 ? 26000 : 2500);
		}
		ll_flash_get_stats(&f);
		for (uint8_t k = 0; k < links; k++) {
			ll_conn_get_stats(k, &st);
			paused += st.flash_paused;
		}
		printf("  flash_chain n%d cfg %u: windows %u waits %u forced %u wait max %u us, "
		       "paused %u, link 0 rx %d\n", LL_MAX_CONN, c,
		       (unsigned)(f.windows - f0.windows), (unsigned)(f.waits - f0.waits),
		       (unsigned)(f.forced - f0.forced), (unsigned)fw_wait_max_us,
		       (unsigned)paused, sim.rx[0] - rx0[0]);
		CHECK(sim.radio_in_op == 0);
		CHECK(f.windows - f0.windows == 300);
		CHECK(f.forced == f0.forced);
		/* a wait ends at the next event of every link that needs one:
		 * one interval for one link (the kicked event; without the
		 * kick an idle link would only listen at the end of its
		 * latency window), up to one per link with several (a link
		 * may yield its kicked event to another link's, and another
		 * link may run out of budget meanwhile) */
		CHECK(fw_wait_max_us <= links * 1250u * cfg[c].interval + 2000u);
		CHECK(paused > 0);
		for (uint8_t k = 0; k < links; k++) {
			CHECK(ll_conn_active(k) && disconnects[k] == 0);
			CHECK(sim.rx[k] > rx0[k]);
		}
		/* the links listen as before once the chain is over */
		for (uint8_t k = 0; k < links; k++) {
			rx0[k] = sim.rx[k];
		}
		sim_advance(now + T(1000000));
		for (uint8_t k = 0; k < links; k++) {
			CHECK(sim.rx[k] > rx0[k]);
		}
		CHECK(sim.overlaps == 0 && sim.over_cap == 0);
		for (uint8_t k = 0; k < links; k++) {
			ll_conn_end(k, 0x13);
		}
	}
}

/* A new link is not ready before its first packet: a flash operation
 * requested right after the CONNECT_IND waits for it (the 6-event
 * establishment rule is never put at risk by the window). */
static void test_flash_not_established(void)
{
	const uint32_t t0 = 2000000;

	sim_reset();
	fw_wait_max_us = 0;
	CHECK(sim_connect(0, t0, 12, 0, 400, 0) == 0);
	CHECK(sim.rx[0] == 0);
	sim_flash_op(26000);
	CHECK(sim.rx[0] >= 1);
	CHECK(fw_wait_max_us > 0);
	CHECK(sim.radio_in_op == 0);
	CHECK(ll_conn_active(0) && disconnects[0] == 0);
	ll_conn_end(0, 0x13);
}

/* Advertising sends nothing while the window is set (its events end,
 * stats.flash) and resumes afterwards. */
static void test_flash_adv(void)
{
	const uint32_t t0 = 2000000;
	struct ll_adv_params p = {.interval_min = 0x0020, .interval_max = 0x0020,   /* 20 ms */
				  .type = 0, .chan_map = 7};
	struct ll_adv_stats a0, a1;
	int ev;

	sim_reset();
	ll_adv_get_stats(&a0);
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	now = t0;
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	sim_advance(t0 + T(100000));
	CHECK(sim.adv_events > 0);
	for (int i = 0; i < 20; i++) {
		sim_flash_op(26000);
	}
	ll_adv_get_stats(&a1);
	CHECK(sim.radio_in_op == 0);
	CHECK(a1.flash - a0.flash > 0);
	ev = sim.adv_events;
	sim_advance(now + T(200000));
	CHECK(sim.adv_events > ev);
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
}

int main(void)
{
	test_accept_refuse_prio();
	test_bump();
	test_bump_chain();
	test_round_robin();
	test_running();
	test_cap_clip();
	test_gap();
	test_cancel_and_alarm();
	test_drift_pair();
	test_dodge();
	test_window_wins();
	test_supervision_rescue();
	test_adv_gaps();
	test_adv_dropped();
	test_adv_starve_boost();
	test_busy_links_adv();
	test_adv_while_connected();
	test_reset_consistent();
	test_flash_chain();
	test_flash_not_established();
	test_flash_adv();
	CHECK(locks == 0);
	DONE();
}
