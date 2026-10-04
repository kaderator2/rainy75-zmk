/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Legacy advertising: one event = TX on each enabled channel 37..39, each
 * followed by an RX window for SCAN_REQ / CONNECT_IND. Events repeat every
 * advInterval + advDelay (0..10 ms random), Core Spec Vol 6 Part B 4.4.2.
 * Radio callbacks run in ISR context; HCI-facing calls take ll_plat_lock().
 *
 * Connection handover: a CONNECT_IND for us stops advertising and hands the
 * connection to ll_conn_start(), which takes the lowest free link. If
 * ll_conn accepts it, advertising stays disabled, as the controller must do
 * when a connection is created (Vol 4 Part E 7.8.9, Vol 6 Part B 4.4.2.2);
 * the host re-enables it with LE Set Advertising Enable (Zephyr / ZMK:
 * whenever the active profile is open or not connected). There is no
 * controller-side resume. If ll_conn refuses the CONNECT_IND, advertising
 * continues as before.
 *
 * Advertising while connected (slice 6a Task 6): enabling is allowed while a
 * link is free (ll_conn_count() < LL_MAX_CONN); connectable advertising
 * with every link taken is refused with Connection Limit Exceeded (0x09),
 * non-connectable advertising is always allowed. Vol 4 Part E 7.8.9 names
 * no error code for this case; 0x09 is the one whose definition fits (Vol 1
 * Part F 2.9: "an attempt to create another connection failed because the
 * Controller is already at its limit of the number of connections it can
 * support"). Radio: the first enable with no link left after a connection
 * restores the baseband (ll_radio_adv_restore); while links exist, every
 * adv channel that follows connection events starts with
 * ll_radio_adv_enter() (empty TX FIFO + adv registers, ml-spike S3), never
 * with a baseband reset.
 *
 * Arbitration (slice 6a): advertising is the requester LL_ARB_ADV with the
 * lowest priority. A request covers whole channels of ADV_CHAN_US each: the
 * whole event when it fits, else only the next channel (slicing; the
 * channels of one event then take separate requests, at most
 * ADV_PDU_GAP_MAX_US apart, Vol 6 Part B 4.4.2.3: "The time between the
 * beginning of two consecutive ADV_IND PDUs within an advertising event
 * shall be less than or equal to 10 ms"). A refused or displaced first
 * channel slides into the next gap (ll_arb_gap) before the next adv
 * interval, else the event is dropped; a continuation channel that finds
 * no gap within the 10 ms ends the event early (stats.cut). When no event
 * has completed all its channels for ADV_STARVE_INTERVALS intervals, or
 * that many events were dropped or cut in a row (no gap before the next
 * interval or within 10 ms, or displaced again and again by links that
 * keep the air full), advertising is starving: its channel requests (the
 * first and the continuation ones) ask at LL_ARB_PRIO_ACTIVE. They then
 * take part in the round-robin of the links' ties and may displace idle or
 * active link events: every link whose single planned event overlaps the
 * requested span, so up to one event per link for a whole-event request
 * (6 ms), and again for a continuation channel; never a
 * supervision-critical or transmit-window / instant event. So advertising
 * cannot starve for more than a few intervals.
 */
#include <string.h>
#ifdef __ZEPHYR__
#include <zephyr/logging/log.h>
#endif
#include "ll_adv.h"
#include "ll_arb.h"
#include "ll_conn.h"
#include "ll_defs.h"
#include "ll_plat.h"

#ifdef __ZEPHYR__
LOG_MODULE_DECLARE(openll, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);
#endif

#define ADV_RX_WINDOW_US     300                         /* SCAN_REQ/CONNECT_IND start within T_IFS */
#define ADV_START_LEAD_TICKS (1000 * LL_TICKS_PER_US)   /* first event 1 ms after enable */
#define ADV_TX_LEAD_TICKS    (100 * LL_TICKS_PER_US)    /* radio programming headroom */
#define ADV_DELAY_MAX_TICKS  (10000 * LL_TICKS_PER_US)
/* Air time reserved per adv channel, from the arbiter's start (alarm =
 * open) to the end of the channel's last packet, worst case:
 *   stimer ISR latency under USB load (up to about 270 us, see
 *   LL_CONN_ARM_LEAD_US)                                          300
 *   ll_radio_adv_enter() after connection events (S3: max 70 us)   70
 *   ADV_TX_LEAD (programming headroom)                            100
 *   TX settle (TX_SETTLE_ADV_US)                                   84
 *   ADV_IND, 37-byte payload: (1 + 4 + 2 + 37 + 3) * 8 us         376
 *   T_IFS + SCAN_REQ (12-byte payload, 176 us) or CONNECT_IND
 *   (34-byte payload, 352 us)                               150 + 352
 *   SCAN_RSP: measured 209 us after the request ends + 376 us     585
 *   sum 2017 (SCAN_REQ: 1841; RX window only, no request: 1230)
 * A CONNECT_IND ends the event (the link's first event lies at least
 * 1.25 ms later), so the SCAN_RSP case bounds the span: 2000 us
 * (ADV_RX_WINDOW_US 300 > T_IFS + the 40 us until a request's access
 * address is in, so a channel without any request also fits). */
#define ADV_CHAN_US          2000
/* Rule of Vol 6 Part B 4.4.2.3 (start of consecutive PDUs <= 10 ms) */
#define ADV_PDU_GAP_MAX_TICKS (10000 * LL_TICKS_PER_US)
/* A channel's PDU starts ADV_TX_LEAD after its start callback, which may
 * run this late after the request's tick: the stimer ISR latency and
 * ll_radio_adv_enter() of the ADV_CHAN_US budget (300 + 70 us). */
#define ADV_START_SLACK_TICKS ((300 + 70) * LL_TICKS_PER_US)
/* The next channel of a sliced event is asked for this long after now */
#define ADV_CHAN_LEAD_US     200
/* A whole-event gap is taken instead of a sliced start when it begins at
 * most this long after the earliest single-channel gap */
#define ADV_WHOLE_WAIT_TICKS (10000 * LL_TICKS_PER_US)
/* Starving: no event completed all its channels for this many adv
 * intervals (each with the maximum advDelay), or this many events dropped
 * or cut in a row; the channel requests (first and continuation) then ask
 * at ACTIVE (file header) */
#define ADV_STARVE_INTERVALS 2
/* slide / drop rounds per planned event (each drop moves one interval) */
#define ADV_PLAN_TRIES       8

#define HCI_ADV_IND          0
#define HCI_ADV_DIRECT_HIGH  1
#define HCI_ADV_SCAN_IND     2
#define HCI_ADV_NONCONN_IND  3
#define HCI_ADV_DIRECT_LOW   4

static struct {
	uint8_t pub_a[6];     /* public device address (BD_ADDR) */
	uint8_t rnd_a[6];     /* LE Set Random Address; valid when rnd_set */
	bool rnd_set;
	/* AdvA in use (Own_Address_Type 0: pub_a, 1: rnd_a) and its TxAdd;
	 * set with the PDUs, so every PDU we send and every SCAN_REQ /
	 * CONNECT_IND match of an event use the same address. It cannot
	 * change while advertising: Set Advertising Parameters is refused
	 * then, and so is Set Random Address with own type random. */
	uint8_t use_a[6];
	uint8_t use_tx_add;
	ll_adv_connect_cb_t on_connect;
	struct ll_adv_params prm;
	uint8_t data[LL_ADV_DATA_MAX];
	uint8_t data_len;
	uint8_t rsp[LL_ADV_DATA_MAX];
	uint8_t rsp_len;
	uint8_t pdu[LL_ADV_PDU_MAX];
	uint8_t pdu_len;
	uint8_t rsp_pdu[LL_ADV_PDU_MAX];
	uint8_t rsp_pdu_len;
	volatile bool enabled;
	volatile bool in_event; /* true from the first channel's start until the event ends */
	volatile bool on_air;   /* a channel's TX / RX (or SCAN_RSP) is running */
	/* a connection used the radio since the last baseband restore: every
	 * channel start after a gap enters adv mode (ll_radio_adv_enter) */
	volatile bool radio_dirty;
	uint8_t ch_idx;       /* 0..2 = channel 37..39 */
	uint8_t req_chans;    /* channels covered by the accepted request */
	uint8_t chans_left;   /* further channels of the running request */
	uint32_t last_full;   /* end of the last event with all its channels (or the enable) */
	uint8_t drops_in_row; /* events dropped or cut since then */
	uint32_t event_tick;  /* start of the current advertising event */
	uint32_t req_tick;    /* start of the accepted request */
	uint32_t pdu_tick;    /* start of the last channel's PDU (10 ms rule) */
	struct ll_adv_stats stats;
} adv;

static uint8_t pdu_type(void)
{
	switch (adv.prm.type) {
	case HCI_ADV_SCAN_IND:    return LL_PDU_ADV_SCAN_IND;
	case HCI_ADV_NONCONN_IND: return LL_PDU_ADV_NONCONN_IND;
	default:                  return LL_PDU_ADV_IND;
	}
}

static bool scannable(void)
{
	return adv.prm.type == HCI_ADV_IND || adv.prm.type == HCI_ADV_SCAN_IND;
}

static bool connectable(void)
{
	return adv.prm.type == HCI_ADV_IND;
}

/* Caller holds the lock (or advertising is off). */
static void build_pdus(void)
{
	if (adv.prm.own_addr_type == LL_OWN_ADDR_RANDOM) {
		memcpy(adv.use_a, adv.rnd_a, 6);
		adv.use_tx_add = 1;
	} else {
		memcpy(adv.use_a, adv.pub_a, 6);
		adv.use_tx_add = 0;
	}
	adv.pdu_len = ll_pdu_build_adv(adv.pdu, pdu_type(), adv.use_a, adv.use_tx_add,
				       adv.data, adv.data_len);
	adv.rsp_pdu_len = ll_pdu_build_adv(adv.rsp_pdu, LL_PDU_SCAN_RSP, adv.use_a,
					   adv.use_tx_add, adv.rsp, adv.rsp_len);
}

static void defaults(void)
{
	memset(&adv.prm, 0, sizeof(adv.prm));
	adv.prm.interval_min = 0x0800; /* 1.28 s, HCI default */
	adv.prm.interval_max = 0x0800;
	adv.prm.chan_map = 0x07;
	adv.data_len = 0;
	adv.rsp_len = 0;
	/* HCI Reset: no random address until the host sets one again */
	memset(adv.rnd_a, 0, 6);
	adv.rnd_set = false;
	build_pdus();
}

static uint8_t nchan(void)
{
	uint8_t m = adv.prm.chan_map & 0x07;

	return (uint8_t)((m & 1) + ((m >> 1) & 1) + ((m >> 2) & 1));
}

static void tx_current(void)
{
	adv.on_air = true;
	adv.pdu_tick = ll_radio_now();
	ll_radio_set_adv_channel(37 + adv.ch_idx);
	ll_radio_tx_then_rx(adv.pdu, adv.pdu_len, ll_radio_now() + ADV_TX_LEAD_TICKS,
			    ADV_RX_WINDOW_US);
}

static uint32_t interval_ticks(void)
{
	return (uint32_t)adv.prm.interval_min * 625u * LL_TICKS_PER_US;
}

static bool before(uint32_t a, uint32_t b)
{
	return (int32_t)(a - b) < 0;
}

/* Priority of the channel requests: ACTIVE while starving (file header). */
static uint8_t first_prio(void)
{
	uint32_t limit = ADV_STARVE_INTERVALS * (interval_ticks() + ADV_DELAY_MAX_TICKS);

	return adv.drops_in_row >= ADV_STARVE_INTERVALS ||
		       (uint32_t)(ll_radio_now() - adv.last_full) > limit
		       ? LL_ARB_PRIO_ACTIVE : LL_ARB_PRIO_ADV;
}

/* Request nch channels from t (alarm = open = t). Caller holds the lock. */
static int adv_request(uint32_t t, uint8_t nch, uint8_t prio)
{
	struct ll_arb_req r = {
		.alarm_tick = t,
		.open_tick = t,
		.min_len_us = (uint32_t)nch * ADV_CHAN_US,
		.max_len_us = (uint32_t)nch * ADV_CHAN_US,
		.prio = prio,
	};
	int ret = ll_arb_request(LL_ARB_ADV, &r);

	if (ret == 0) {
		adv.req_tick = t;
		adv.req_chans = nch;
	}
	return ret;
}

static void advance_event(void)
{
	adv.event_tick += interval_ticks() + ll_plat_rand32() % (ADV_DELAY_MAX_TICKS + 1);
}

/* First channel of the event: at event_tick (direct: a fresh plan; never
 * after a bump, which must not ask for the displaced placement again),
 * whole event first, else only its first channel; refused: slide into the
 * earliest gap before the next interval, the whole event if that gap is
 * not much later than a single channel's. Caller holds the lock. */
static bool place_first(bool direct)
{
	uint8_t n = nchan();
	uint8_t prio = first_prio();
	uint32_t limit = adv.event_tick + interval_ticks();
	uint32_t tf, t1;

	if (direct) {
		if (adv_request(adv.event_tick, n, prio) == 0 ||
		    (n > 1 && adv_request(adv.event_tick, 1, prio) == 0)) {
			return true;
		}
	}
	tf = ll_arb_gap(adv.event_tick, 0, (uint32_t)n * ADV_CHAN_US);
	t1 = ll_arb_gap(adv.event_tick, 0, ADV_CHAN_US);
	if (before(tf, limit) && (uint32_t)(tf - t1) <= ADV_WHOLE_WAIT_TICKS &&
	    adv_request(tf, n, prio) == 0) {
		adv.stats.slid++;
		adv.event_tick = tf;
		return true;
	}
	if (before(t1, limit) && adv_request(t1, 1, prio) == 0) {
		adv.stats.slid++;
		adv.event_tick = t1;
		return true;
	}
	return false;
}

/* Plan the event at event_tick (see place_first), dropping events that
 * find no room before their next interval. Caller holds the lock. */
static void plan_event(bool direct)
{
	for (int i = 0; i < ADV_PLAN_TRIES; i++) {
		if (place_first(direct)) {
			return;
		}
		adv.stats.dropped++;
		if (adv.drops_in_row < UINT8_MAX) {
			adv.drops_in_row++;
		}
		ll_arb_yield(LL_ARB_ADV);   /* gave the event up (fairness) */
		advance_event();
		direct = true;
	}
	/* Not expected (a gap always follows the few requests): the first
	 * channel after the gap, whatever its distance. ll_arb_gap() avoids
	 * every accepted span, so the request cannot be refused; if it still
	 * is, advertising is disabled rather than left enabled with no
	 * request and no alarm (counted in stats.stuck; the host enables it
	 * again on its next advertising update). */
	adv.event_tick = ll_arb_gap(adv.event_tick, 0, ADV_CHAN_US);
	if (adv_request(adv.event_tick, 1, LL_ARB_PRIO_ADV) != 0) {
#ifdef __ZEPHYR__
		LOG_ERR("advertising stuck: no arbiter request possible, disabled");
#endif
		adv.stats.stuck++;
		adv.enabled = false;
		adv.in_event = false;
		adv.on_air = false;
		ll_arb_cancel(LL_ARB_ADV);
	}
}

/* Event over (or abandoned): plan the next one. Any context. */
static void schedule_next_event(void)
{
	unsigned int key = ll_plat_lock();

	adv.in_event = false;
	adv.on_air = false;
	ll_arb_cancel(LL_ARB_ADV);   /* the event is over (no own span in the gap search) */
	advance_event();
	if ((int32_t)(adv.event_tick - ll_radio_now()) < 0) {
		/* missed the window for a long time (e.g. stalled radio); re-base
		 * on now instead of scheduling an event far in the past */
		adv.event_tick = ll_radio_now() + interval_ticks() +
				 ll_plat_rand32() % (ADV_DELAY_MAX_TICKS + 1);
	}
	plan_event(true);
	ll_plat_unlock(key);
}

/* The next channel of a sliced event, in the first gap from `from`, or the
 * event ends early when that lies beyond the 10 ms PDU spacing. Caller
 * holds the lock. */
static void place_next_channel(uint32_t from)
{
	uint32_t t;

	/* the channel just ended: its own (running) span must not push the
	 * gap search */
	ll_arb_cancel(LL_ARB_ADV);
	t = ll_arb_gap(from, 0, ADV_CHAN_US);

	/* pdu_tick + ADV_TX_LEAD was this channel's PDU start; the next
	 * one starts at the latest at t + slack + ADV_TX_LEAD */
	if ((int32_t)(t + ADV_START_SLACK_TICKS - adv.pdu_tick) <= (int32_t)ADV_PDU_GAP_MAX_TICKS &&
	    adv_request(t, 1, first_prio()) == 0) {
		return;
	}
	adv.stats.cut++;
	if (adv.drops_in_row < UINT8_MAX) {
		adv.drops_in_row++;   /* a cut event counts toward starvation */
	}
	schedule_next_event();
}

/* Advance to the next enabled channel; end the event after channel 39. */
static void next_channel(void)
{
	adv.on_air = false;
	while (++adv.ch_idx < 3) {
		if (adv.prm.chan_map & (1 << adv.ch_idx)) {
			if (adv.chans_left > 0) {
				adv.chans_left--;
				tx_current();
			} else {
				unsigned int key = ll_plat_lock();

				place_next_channel(ll_radio_now() + ADV_CHAN_LEAD_US * LL_TICKS_PER_US);
				ll_plat_unlock(key);
			}
			return;
		}
	}
	/* all channels sent: advertising is served */
	adv.last_full = ll_radio_now();
	adv.drops_in_row = 0;
	schedule_next_event();
}

/* Arbiter start: the event's first channel, or the next channel of a
 * sliced event. */
void ll_adv_arb_start(uint32_t cap_us)
{
	(void)cap_us;   /* fixed length, the span already covers it */
	if (!adv.enabled) {
		unsigned int key = ll_plat_lock();

		ll_arb_cancel(LL_ARB_ADV);
		ll_plat_unlock(key);
		return;
	}
	if (!adv.in_event) {
		adv.stats.events++;
		if (scannable()) {
			ll_radio_prepare_rsp(adv.rsp_pdu, adv.rsp_pdu_len);
		}
		adv.ch_idx = 0;
		while (!(adv.prm.chan_map & (1 << adv.ch_idx))) {
			adv.ch_idx++;
		}
		adv.in_event = true;
	}
	adv.chans_left = adv.req_chans > 0 ? (uint8_t)(adv.req_chans - 1) : 0;
	if (adv.radio_dirty) {
		/* connection events may have used the radio since the last
		 * channel: empty TX FIFO + adv registers (S3) */
		ll_radio_adv_enter();
	}
	tx_current();
}

/* Displaced (lock held by the arbiter): the first gap from the displaced
 * placement, never that placement again. */
void ll_adv_arb_bumped(void)
{
	if (!adv.enabled || adv.on_air) {
		return;
	}
	if (adv.in_event) {
		place_next_channel(adv.req_tick);
	} else {
		plan_event(false);
	}
}

void ll_adv_get_stats(struct ll_adv_stats *s)
{
	unsigned int key = ll_plat_lock();

	*s = adv.stats;
	ll_plat_unlock(key);
}

void ll_adv_init(const uint8_t adva[6], ll_adv_connect_cb_t on_connect)
{
	struct ll_adv_stats st = adv.stats;

	memset(&adv, 0, sizeof(adv));
	adv.stats = st;   /* cumulative since boot */
	memcpy(adv.pub_a, adva, 6);
	adv.on_connect = on_connect;
	defaults();
}

uint8_t ll_adv_enable(bool enable)
{
	unsigned int key = ll_plat_lock();
	uint8_t st = LL_ST_SUCCESS;

	if (enable && !adv.enabled) {
		uint8_t taken = ll_conn_count();

		if (adv.prm.own_addr_type == LL_OWN_ADDR_RANDOM && !adv.rnd_set) {
			/* Vol 4 Part E 7.8.9: own type random and the random
			 * address not initialized with LE Set Random Address */
			st = LL_ST_INVALID_PARAM;
		} else if (connectable() && taken >= LL_MAX_CONN) {
			/* every link taken (active or awaiting release): see
			 * the file header for the code */
			st = LL_ST_CONN_LIMIT;
		} else {
			if (adv.radio_dirty && taken == 0) {
				/* back from connection mode, no link left:
				 * baseband restore once */
				adv.radio_dirty = false;
				ll_radio_adv_restore();
			}
			build_pdus();   /* AdvA in force from this enable */
			adv.enabled = true;
			adv.in_event = false;
			adv.on_air = false;
			adv.last_full = ll_radio_now();
			adv.drops_in_row = 0;
			adv.event_tick = ll_radio_now() + ADV_START_LEAD_TICKS;
			plan_event(true);
		}
	} else if (!enable && adv.enabled) {
		adv.enabled = false;
		adv.in_event = false;
		adv.on_air = false;
		ll_arb_cancel(LL_ARB_ADV);
		ll_radio_stop();
	}
	ll_plat_unlock(key);
	return st;
}
bool ll_adv_is_enabled(void)
{
	return adv.enabled;
}

void ll_adv_reset(void)
{
	ll_adv_enable(false);
	unsigned int key = ll_plat_lock();

	defaults();
	ll_plat_unlock(key);
}

uint8_t ll_adv_set_params(const struct ll_adv_params *p)
{
	if (adv.enabled) {
		return LL_ST_DISALLOWED;
	}
	if (p->type > HCI_ADV_DIRECT_LOW || p->filter_policy > 3 || p->own_addr_type > 3 ||
	    p->chan_map == 0 || p->chan_map > 0x07 ||
	    p->interval_min < 0x0020 || p->interval_max > 0x4000 ||
	    p->interval_min > p->interval_max) {
		return LL_ST_INVALID_PARAM;
	}
	/* own types 2/3 (controller-generated RPA) need a resolving list */
	if (p->type == HCI_ADV_DIRECT_HIGH || p->type == HCI_ADV_DIRECT_LOW ||
	    p->own_addr_type > LL_OWN_ADDR_RANDOM || p->filter_policy != 0) {
		return LL_ST_UNSUPPORTED;
	}
	unsigned int key = ll_plat_lock();

	adv.prm = *p;
	build_pdus();
	ll_plat_unlock(key);
	return LL_ST_SUCCESS;
}

uint8_t ll_adv_set_random_addr(const uint8_t addr[6])
{
	unsigned int key = ll_plat_lock();
	uint8_t st = LL_ST_SUCCESS;

	if (adv.enabled && adv.prm.own_addr_type == LL_OWN_ADDR_RANDOM) {
		/* Vol 4 Part E 7.8.4: not while legacy advertising uses the
		 * random address (Zephyr stops advertising to rotate the RPA) */
		st = LL_ST_DISALLOWED;
	} else {
		memcpy(adv.rnd_a, addr, 6);
		adv.rnd_set = true;
		if (!adv.enabled) {
			/* while enabled (public AdvA) nothing changes on air;
			 * the next enable rebuilds the PDUs */
			build_pdus();
		}
	}
	ll_plat_unlock(key);
	return st;
}

static uint8_t set_buf(uint8_t *dst, uint8_t *dst_len, const uint8_t *data, uint8_t len)
{
	if ((data == NULL && len != 0) || len > LL_ADV_DATA_MAX) {
		return LL_ST_INVALID_PARAM;
	}
	unsigned int key = ll_plat_lock();

	if (len) {
		memcpy(dst, data, len);
	}
	*dst_len = len;
	build_pdus();
	ll_plat_unlock(key);
	return LL_ST_SUCCESS;
}

uint8_t ll_adv_set_data(const uint8_t *data, uint8_t len)
{
	return set_buf(adv.data, &adv.data_len, data, len);
}

uint8_t ll_adv_set_scan_rsp(const uint8_t *data, uint8_t len)
{
	return set_buf(adv.rsp, &adv.rsp_len, data, len);
}

void ll_adv_radio_evt(enum ll_radio_evt evt, const uint8_t *pdu, uint8_t len,
		      uint32_t end_tick)
{
	struct ll_connect_ind ci;

	/* only the advertising radio events of a channel on air (connection
	 * events between sliced channels reach this dispatcher too) */
	if (!adv.enabled || !adv.in_event || !adv.on_air ||
	    (evt != LL_RADIO_RX_OK && evt != LL_RADIO_RX_TIMEOUT && evt != LL_RADIO_RX_CRC_ERR &&
	     evt != LL_RADIO_TX_DONE)) {
		return;
	}
	if (evt == LL_RADIO_RX_OK) {
		if (scannable() && ll_pdu_is_scan_req_for(pdu, len, adv.use_a, adv.use_tx_add)) {
			if (ll_radio_tx_rsp_at(end_tick + LL_T_IFS_US * LL_TICKS_PER_US)) {
				return; /* continue on LL_RADIO_TX_DONE */
			}
			/* too late to answer; no TX_DONE will come, so move on now */
		}
		if (connectable() &&
		    ll_pdu_parse_connect_ind(pdu, len, adv.use_a, adv.use_tx_add, &ci) == 0) {
			if (adv.on_connect) {
				adv.on_connect(&ci);
			}
			/* Stop advertising first: ll_conn_start() takes over the
			 * radio and may report CONNECTED synchronously. No
			 * advertising alarm is pending inside an event. */
			adv.in_event = false;
			adv.on_air = false;
			adv.enabled = false;
			{
				unsigned int key = ll_plat_lock();

				ll_arb_cancel(LL_ARB_ADV);
				ll_plat_unlock(key);
			}
			ll_radio_stop();
			if (ll_conn_start(&ci, end_tick) >= 0) {   /* link id */
				adv.radio_dirty = true;
				return;
			}
			/* refused (invalid parameters / busy): nothing touched
			 * the radio, keep advertising */
			adv.enabled = true;
			schedule_next_event();
			return;
		}
	}
	next_channel();
}
