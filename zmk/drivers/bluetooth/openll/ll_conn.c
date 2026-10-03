/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Connection state machine, peripheral role, one connection, CSA#1, no
 * peripheral latency. Core Spec Vol 6 Part B: 4.5.1 connection events,
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
 * ll_txq_event_start(), BRX via ll_radio_conn_event(). Each CONN_RX:
 * ll_txq_rx(), ll_rxq_isr_put(), the first one re-syncs the anchor and the
 * supervision timer, unless the event's first packet was not delivered
 * (bad CRC, or an acked retransmission without an RX entry) or the packet
 * starts after the RX window (it is then a chained packet, not the
 * anchor). A data PDU that ll_rxq cannot take (ring full) is lost, as the
 * hardware has acked it: the link then ends with 0x08 at CONN_DONE.
 * CONN_DONE: ll_txq_event_end(), counter++, termination
 * and supervision checks, plan the next event (applying instants).
 *
 * Everything runs in ISR context except the public calls documented as
 * thread calls, which take ll_plat_lock(). No allocation, no blocking.
 */
#include <errno.h>
#include <string.h>

#include "ll_conn.h"
#include "ll_csa1.h"
#include "ll_defs.h"
#include "ll_plat.h"
#include "ll_radio.h"
#include "ll_rxq.h"
#include "ll_sched.h"
#include "ll_txq.h"

#define US(t)            ((uint32_t)(t) * LL_TICKS_PER_US)
#define UNIT_US          1250u     /* interval / window unit */
#define SUP_UNIT_US      10000u    /* supervision timeout unit */
#define NOT_ESTAB_EVENTS 6
#define OP_TERMINATE_IND 0x02
#define INSTANT_PAST     32767u   /* passed: (instant - counter) mod 65536 > this */

/* SCA field -> worst-case ppm (Vol 6 Part B 2.3.3.1, Table 2.17) */
static const uint16_t sca_ppm[8] = {500, 250, 150, 100, 75, 50, 30, 20};

static struct {
	struct ll_conn_ops ops;
	bool active;
	bool planned;         /* alarm pending for event `counter` */
	bool in_event;        /* BRX issued, CONN_DONE pending */
	bool established;     /* a packet was received in this connection */
	bool rx_this_event;   /* a CRC-valid packet was received in this event */
	bool first_seen;      /* the event's first packet (valid or not) was seen */
	struct ll_connect_ind ci;
	struct ll_conn_params p;
	uint32_t interval_ticks;
	uint32_t sup_ticks;
	uint16_t ppm;         /* central SCA + own */
	uint32_t widen_max_us; /* interval / 2 - T_IFS (exceeds 16 bits) */
	struct ll_csa1 csa;
	struct ll_csa1 csa_prev; /* before the planned event's channel (re-plan) */
	uint16_t counter;     /* next event not yet completed */
	uint32_t ref_tick;
	uint16_t ref_counter;
	uint32_t win_us;
	uint32_t sync_tick;
	uint32_t sup_tick;    /* supervision timer start */
	/* planned event */
	uint8_t ch;
	uint32_t open_tick;
	uint32_t fst_us;
	/* instants */
	bool upd_pending;
	uint16_t upd_instant;
	uint8_t upd_win_size;
	uint16_t upd_win_offset;
	struct ll_conn_params upd_p;
	bool chm_pending;
	uint16_t chm_instant;
	uint8_t chm[5];
	/* termination */
	bool term_local;
	bool term_acked;
	uint32_t term_tick;
	bool end_pending;
	uint8_t end_reason;
} c;

static struct ll_conn_stats stats;

static void prepare(void);

static void report(enum ll_conn_evt what, const void *arg)
{
	if (c.ops.evt) {
		c.ops.evt(what, arg);
	}
}

static void end(uint8_t reason)
{
	static uint8_t r;

	c.active = false;
	c.planned = false;
	c.in_event = false;
	c.end_pending = false;
	ll_sched_cancel();
	r = reason;
	report(LL_CONN_EVT_DISCONNECTED, &r);
}

/* End now, or at CONN_DONE when an event is on air (first reason wins). */
static void request_end(uint8_t reason)
{
	if (c.in_event) {
		if (!c.end_pending) {
			c.end_pending = true;
			c.end_reason = reason;
		}
		return;
	}
	end(reason);
}

/* 4.5.4 window widening. Above interval / 2 - T_IFS it is clamped (4.5.7
 * says the link is then lost; the supervision timeout ends it). */
static uint32_t widening_us(uint32_t dt_ticks)
{
	uint32_t dt_us = dt_ticks / LL_TICKS_PER_US;
	uint32_t w = (uint32_t)(((uint64_t)c.ppm * dt_us + 999999u) / 1000000u) + 16u;

	if (w > c.widen_max_us) {
		w = c.widen_max_us;
	}
	return w;
}

static uint32_t anchor_of(uint16_t counter)
{
	return c.ref_tick + (uint32_t)(uint16_t)(counter - c.ref_counter) * c.interval_ticks;
}

static void set_params(const struct ll_conn_params *p)
{
	c.p = *p;
	c.interval_ticks = (uint32_t)p->interval * US(UNIT_US);
	c.sup_ticks = (uint32_t)p->timeout * US(SUP_UNIT_US);
	c.widen_max_us = (uint32_t)p->interval * UNIT_US / 2u - LL_T_IFS_US;
}

/* Instants of the event about to be planned (counter == instant). */
static void apply_instants(void)
{
	if (c.chm_pending && c.chm_instant == c.counter) {
		c.chm_pending = false;
		ll_csa1_set_map(&c.csa, c.chm);
	}
	if (c.upd_pending && c.upd_instant == c.counter) {
		/* 5.1.1: the transmit window starts WinOffset after the anchor
		 * the instant event would have had with the old parameters. */
		uint32_t old_anchor = anchor_of(c.counter);
		bool changed = c.upd_p.interval != c.p.interval ||
			       c.upd_p.latency != c.p.latency ||
			       c.upd_p.timeout != c.p.timeout;

		c.upd_pending = false;
		c.ref_tick = old_anchor + (uint32_t)c.upd_win_offset * US(UNIT_US);
		c.ref_counter = c.counter;
		c.win_us = (uint32_t)c.upd_win_size * UNIT_US;
		c.sup_tick = old_anchor;
		set_params(&c.upd_p);
		if (changed) {
			report(LL_CONN_EVT_UPDATED, &c.p);
		}
	}
}

/* Plan event `counter`: channel, RX window, alarm. */
static void plan(void)
{
	uint32_t base, widen, margin;

	apply_instants();
	c.csa_prev = c.csa;
	c.ch = ll_csa1_next(&c.csa);
	base = anchor_of(c.counter);
	widen = widening_us(base + US(c.win_us) - c.sync_tick);
	if (widen > stats.widen_max_us) {
		stats.widen_max_us = widen;
	}
	margin = c.win_us ? LL_CONN_WIN_MARGIN_US : LL_CONN_RX_MARGIN_US;
	c.open_tick = base - US(widen + margin);
	c.fst_us = c.win_us + 2u * (widen + margin) + LL_CONN_SYNC_US;
	c.planned = true;
	ll_sched_at(c.open_tick - US(LL_CONN_ARM_LEAD_US), prepare);
}

/* Re-plan the planned (not yet issued) event after a new instant for it. */
static void replan(void)
{
	ll_sched_cancel();
	c.csa = c.csa_prev;
	plan();
}

/* An event ended (or was skipped): advance, check, plan the next one. */
static void event_closed(uint32_t now)
{
	c.counter++;
	if (c.end_pending) {
		end(c.end_reason);
		return;
	}
	if (c.term_acked) {
		end(LL_ST_LOCAL_TERM);
		return;
	}
	if (!c.established) {
		if (c.counter >= NOT_ESTAB_EVENTS) {
			end(LL_ST_CONN_FAIL_EST);
			return;
		}
	} else if ((int32_t)(now - c.sup_tick) >= (int32_t)c.sup_ticks) {
		end(LL_ST_CONN_TIMEOUT);
		return;
	}
	if (c.term_local && (int32_t)(now - c.term_tick) >= (int32_t)c.sup_ticks) {
		/* 5.1.3: no ack of LL_TERMINATE_IND within the timeout */
		end(LL_ST_LOCAL_TERM);
		return;
	}
	plan();
}

/* Event length cap for the radio guard (see LL_CONN_EVENT_SAFETY_US): the
 * next event opens no earlier than one interval after this one, minus the
 * widening growth over that interval (no re-sync in this event). */
static uint32_t event_max_us(void)
{
	uint32_t ival_us = (uint32_t)c.p.interval * UNIT_US;
	uint32_t growth = (uint32_t)(((uint64_t)c.ppm * ival_us + 999999u) / 1000000u);
	uint32_t reserve = growth + LL_CONN_ARM_LEAD_US + LL_CONN_EVENT_SAFETY_US;
	uint32_t floor_us = c.fst_us + LL_CONN_GUARD_MIN_TAIL_US;
	uint32_t cap = ival_us > reserve ? ival_us - reserve : 0;

	return cap > floor_us ? cap : floor_us;
}

/* Alarm: issue the BRX for the planned event. */
static void prepare(void)
{
	uint32_t now;

	if (!c.active || !c.planned) {
		return;
	}
	c.planned = false;
	now = ll_radio_now();
	if ((int32_t)(c.open_tick - now) < (int32_t)US(LL_CONN_MIN_PREP_US)) {
		stats.late++;
		stats.missed++;
		event_closed(now);
		return;
	}
	ll_txq_event_start();
	c.rx_this_event = false;
	c.first_seen = false;
	c.in_event = true;
	stats.events++;
	ll_radio_conn_event(c.ch, c.open_tick, c.fst_us, event_max_us());
}

/* A packet with a bad CRC: if it is the event's first, it still was the
 * anchor packet, so a later valid packet of the event must not re-anchor. */
static void on_rx_bad(void)
{
	if (!c.first_seen) {
		c.first_seen = true;
		stats.first_bad++;
	}
}

/* A packet the hardware received but did not deliver (the central resent a
 * packet we have; Task 10): same as a bad first packet. With MD the event
 * continues with a new packet some 400..700 us after the anchor. An event
 * with only such retransmissions has no rx_this_event and so counts as
 * missed in the stats (it still proves the central is there, but it does
 * not refresh supervision; the next new packet does). */
static void on_rx_nodata(void)
{
	if (!c.first_seen) {
		c.first_seen = true;
		stats.first_nodata++;
	}
}

static void on_rx(const uint8_t *pdu, uint8_t len, uint32_t tick)
{
	uint32_t anchor;
	bool first;

	if (len < 2) {
		return;
	}
	stats.rx_pkts++;
	ll_txq_rx(pdu[0]);
	if (!ll_rxq_isr_put(pdu, len)) {
		/* The hardware has acked this data PDU already, so the central
		 * will never resend it: it is lost for good, and continuing
		 * would leave a hole in the L2CAP stream (and, encrypted, a
		 * CCM packet counter mismatch at the next PDU). End the link
		 * deterministically at the end of this event. 0x08: we stop
		 * following the link without an LL_TERMINATE_IND, so the
		 * central sees a supervision timeout too and both hosts treat
		 * it as an ordinary link loss (reconnect). */
		request_end(LL_ST_CONN_TIMEOUT);
	}
	c.rx_this_event = true;
	first = !c.first_seen;
	c.first_seen = true;
	/* The hardware syncs the event's first packet only inside the RX
	 * window (first-RX timeout); a packet whose access address ends after
	 * it (plus one sync time of slack) is a chained one whose anchor
	 * packet was not reported (e.g. both in one late ISR). */
	if (first && (int32_t)(tick - (c.open_tick + US(c.fst_us + LL_CONN_SYNC_US))) > 0) {
		stats.first_outside++;
		first = false;
	}
	if (!first) {
		/* later packet of the event: proof of life only (4.5.2); it is
		 * not at the anchor, so timing stays as it was */
		c.sup_tick = tick - US(LL_CONN_SYNC_US);
		c.established = true;
		return;
	}
	/* first packet of the event: its start is the anchor point */
	anchor = tick - US(LL_CONN_SYNC_US);
	c.ref_tick = anchor;
	c.ref_counter = c.counter;
	c.win_us = 0;
	c.sync_tick = anchor;
	c.sup_tick = anchor;
	c.established = true;
}

void ll_conn_radio_evt(enum ll_radio_evt evt, const uint8_t *pdu, uint8_t len, uint32_t tick)
{
	if (!c.active || !c.in_event) {
		return;
	}
	switch (evt) {
	case LL_RADIO_CONN_RX:
		on_rx(pdu, len, tick);
		break;
	case LL_RADIO_CONN_RX_CRC_ERR:
		on_rx_bad();
		break;
	case LL_RADIO_CONN_RX_NODATA:
		on_rx_nodata();
		break;
	case LL_RADIO_CONN_DONE:
		c.in_event = false;
		ll_txq_event_end();
		if (c.rx_this_event) {
			stats.rx_events++;
		} else {
			stats.missed++;
		}
		event_closed(tick);
		break;
	default:
		break;
	}
}

static void txq_done(enum ll_txq_kind kind, uint8_t ctrl_opcode)
{
	if (kind == LL_TXQ_CTRL && ctrl_opcode == OP_TERMINATE_IND && c.term_local) {
		c.term_acked = true;   /* ended at this event's CONN_DONE */
	}
	if (c.ops.txq_done) {
		c.ops.txq_done(kind, ctrl_opcode);
	}
}

void ll_conn_init(const struct ll_conn_ops *ops)
{
	memset(&c, 0, sizeof(c));
	if (ops) {
		c.ops = *ops;
	}
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

	if (c.active) {
		return -EBUSY;
	}
	ll_csa1_init(&csa, ci->hop, ci->chm);
	if (!params_valid(ci, &csa)) {
		return -EINVAL;
	}
	{
		struct ll_conn_ops ops = c.ops;

		memset(&c, 0, sizeof(c));
		c.ops = ops;
	}
	c.ci = *ci;
	c.csa = csa;
	p.interval = ci->interval;
	p.latency = ci->latency;
	p.timeout = ci->timeout;
	set_params(&p);
	c.ppm = (uint16_t)(sca_ppm[ci->sca & 7] + LL_OWN_SCA_PPM);
	/* 4.5.3: transmitWindowDelay 1.25 ms + WinOffset, size WinSize */
	c.ref_tick = connect_ind_end_tick + US(UNIT_US) + (uint32_t)ci->win_offset * US(UNIT_US);
	c.ref_counter = 0;
	c.win_us = (uint32_t)ci->win_size * UNIT_US;
	c.sync_tick = connect_ind_end_tick;
	c.sup_tick = connect_ind_end_tick;
	c.active = true;

	ll_radio_conn_setup(ci->aa, ci->crc_init);
	ll_txq_reset(txq_done);
	/* No ll_rxq_reset() here: this runs in ISR context while the controller
	 * thread may be inside ll_rxq_get(). The glue resets ll_rxq in its
	 * thread when it handles LL_CONN_EVT_DISCONNECTED, before advertising
	 * (and so a new connection) can be enabled again. */
	plan();
	report(LL_CONN_EVT_CONNECTED, &c.ci);
	return 0;
}

/* 0, or LL_ST_INSTANT_PASSED (connection ending) */
static int check_instant(uint16_t instant)
{
	uint16_t d = (uint16_t)(instant - c.counter);

	/* d == 0 with the event already on air: its timing and channel were
	 * issued with the old values, so the instant cannot be honoured any
	 * more; treat it like a passed instant. */
	if (d > INSTANT_PAST || (d == 0 && c.in_event)) {
		request_end(LL_ST_INSTANT_PASSED);
		return LL_ST_INSTANT_PASSED;
	}
	return 0;
}

int ll_conn_update_at(uint16_t instant, uint8_t win_size, uint16_t win_offset,
		      const struct ll_conn_params *p)
{
	unsigned int key = ll_plat_lock();
	int ret = LL_ST_DISALLOWED;

	if (!timing_valid(p->interval, p->latency, p->timeout, win_size, win_offset)) {
		ret = LL_ST_INVALID_LL_PARAM;
	} else if (c.active && !c.end_pending) {
		ret = check_instant(instant);
		if (ret == 0) {
			/* A second LL_CONNECTION_UPDATE_IND while one is
			 * pending replaces it (instant and parameters). The
			 * central may only start one procedure at a time, so
			 * this is a protocol violation for which the spec
			 * allows ending the link; following the latest one is
			 * accepted as the more forgiving choice. */
			c.upd_pending = true;
			c.upd_instant = instant;
			c.upd_win_size = win_size;
			c.upd_win_offset = win_offset;
			c.upd_p = *p;
			if (c.planned && instant == c.counter) {
				replan();
			}
		}
	}
	ll_plat_unlock(key);
	return ret;
}

int ll_conn_chmap_at(uint16_t instant, const uint8_t chm[5])
{
	unsigned int key = ll_plat_lock();
	int ret = LL_ST_DISALLOWED;

	if (c.active && !c.end_pending) {
		ret = check_instant(instant);
		if (ret == 0) {
			c.chm_pending = true;
			c.chm_instant = instant;
			memcpy(c.chm, chm, sizeof(c.chm));
			if (c.planned && instant == c.counter) {
				replan();
			}
		}
	}
	ll_plat_unlock(key);
	return ret;
}

void ll_conn_terminate(uint8_t reason)
{
	const uint8_t pdu[2] = {OP_TERMINATE_IND, reason};
	unsigned int key = ll_plat_lock();
	bool go = c.active && !c.term_local && !c.end_pending;

	if (go) {
		c.term_local = true;
		c.term_acked = false;
		c.term_tick = ll_radio_now();
	}
	ll_plat_unlock(key);
	if (!go) {
		return;
	}
	/* A failed push is not retried: the timeout then ends the link. */
	if (c.ops.ctrl_tx) {
		(void)c.ops.ctrl_tx(pdu, sizeof(pdu));
	} else {
		key = ll_plat_lock();
		(void)ll_txq_push(LL_TXQ_CTRL, LL_LLID_CTRL, pdu, sizeof(pdu), OP_TERMINATE_IND);
		ll_plat_unlock(key);
	}
}

void ll_conn_end(uint8_t reason)
{
	unsigned int key = ll_plat_lock();

	if (c.active) {
		request_end(reason);
	}
	ll_plat_unlock(key);
}

bool ll_conn_active(void)
{
	return c.active;
}

uint16_t ll_conn_event_counter(void)
{
	return c.counter;
}

void ll_conn_get_stats(struct ll_conn_stats *s)
{
	unsigned int key = ll_plat_lock();

	*s = stats;
	ll_plat_unlock(key);
}
