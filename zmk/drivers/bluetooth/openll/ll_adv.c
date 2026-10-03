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
 * connection to ll_conn_start(). If ll_conn accepts it, advertising stays
 * disabled, as the controller must do when a connection is created
 * (Vol 4 Part E 7.8.9, Vol 6 Part B 4.4.2.2); the host re-enables it with
 * LE Set Advertising Enable after Disconnection Complete (Zephyr:
 * bt_le_adv_resume() when the peripheral connection object is released).
 * There is therefore no controller-side resume. Enabling is refused while
 * the connection is active (we support no advertising + connection state
 * combination), and the first enable after a connection calls
 * ll_radio_adv_restore() to bring the baseband back from connection mode.
 * If ll_conn refuses the CONNECT_IND, advertising continues as before.
 *
 * Arbitration (slice 6a): every adv event is a request to ll_arb (id
 * LL_ARB_ADV, lowest priority, span ADV_EVENT_US from its start). When it
 * is refused or displaced, the event slides to the next gap
 * (ll_arb_gap()), or is dropped when that gap lies beyond the next adv
 * interval (the next event is then planned as usual, with a new advDelay).
 */
#include <string.h>
#include "ll_adv.h"
#include "ll_arb.h"
#include "ll_conn.h"
#include "ll_defs.h"
#include "ll_plat.h"

#define ADV_RX_WINDOW_US     300                         /* SCAN_REQ/CONNECT_IND start within T_IFS */
#define ADV_START_LEAD_TICKS (1000 * LL_TICKS_PER_US)   /* first event 1 ms after enable */
#define ADV_TX_LEAD_TICKS    (100 * LL_TICKS_PER_US)    /* radio programming headroom */
#define ADV_DELAY_MAX_TICKS  (10000 * LL_TICKS_PER_US)
/* Air time reserved per adv event: 3 channels with TX, RX window, a
 * possible SCAN_RSP or CONNECT_IND, and the radio programming. */
#define ADV_EVENT_US         4000
/* slide / drop rounds per planned event (each drop moves one interval) */
#define ADV_PLAN_TRIES       8

#define HCI_ADV_IND          0
#define HCI_ADV_DIRECT_HIGH  1
#define HCI_ADV_SCAN_IND     2
#define HCI_ADV_NONCONN_IND  3
#define HCI_ADV_DIRECT_LOW   4

static struct {
	uint8_t adva[6];
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
	volatile bool in_event; /* true from the first tx_current() until the event ends */
	volatile bool radio_dirty; /* a connection used the radio since advertising ran */
	uint8_t ch_idx;       /* 0..2 = channel 37..39 */
	uint32_t event_tick;  /* start of the current advertising event */
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

static void build_pdus(void)
{
	adv.pdu_len = ll_pdu_build_adv(adv.pdu, pdu_type(), adv.adva, adv.data, adv.data_len);
	adv.rsp_pdu_len = ll_pdu_build_adv(adv.rsp_pdu, LL_PDU_SCAN_RSP, adv.adva,
					   adv.rsp, adv.rsp_len);
}

static void defaults(void)
{
	memset(&adv.prm, 0, sizeof(adv.prm));
	adv.prm.interval_min = 0x0800; /* 1.28 s, HCI default */
	adv.prm.interval_max = 0x0800;
	adv.prm.chan_map = 0x07;
	adv.data_len = 0;
	adv.rsp_len = 0;
	build_pdus();
}

static void tx_current(void)
{
	ll_radio_set_adv_channel(37 + adv.ch_idx);
	ll_radio_tx_then_rx(adv.pdu, adv.pdu_len, ll_radio_now() + ADV_TX_LEAD_TICKS,
			    ADV_RX_WINDOW_US);
}

static uint32_t interval_ticks(void)
{
	return (uint32_t)adv.prm.interval_min * 625u * LL_TICKS_PER_US;
}

static int adv_request(uint32_t t)
{
	struct ll_arb_req r = {
		.alarm_tick = t,
		.open_tick = t,
		.min_len_us = ADV_EVENT_US,
		.max_len_us = ADV_EVENT_US,
		.prio = LL_ARB_PRIO_ADV,
	};
	unsigned int key = ll_plat_lock();
	int ret = ll_arb_request(LL_ARB_ADV, &r);

	ll_plat_unlock(key);
	return ret;
}

static void advance_event(void)
{
	adv.event_tick += interval_ticks() + ll_plat_rand32() % (ADV_DELAY_MAX_TICKS + 1);
}

/* Request the event at event_tick; refused: slide into the next gap, or
 * drop it when that gap lies beyond the next interval. */
static void plan_event(void)
{
	for (int i = 0; i < ADV_PLAN_TRIES; i++) {
		uint32_t t;

		if (adv_request(adv.event_tick) == 0) {
			return;
		}
		t = ll_arb_gap(adv.event_tick, 0, ADV_EVENT_US);
		if ((int32_t)(t - (adv.event_tick + interval_ticks())) < 0) {
			adv.stats.slid++;
			adv.event_tick = t;
			if (adv_request(t) == 0) {
				return;
			}
		}
		adv.stats.dropped++;
		advance_event();
	}
	/* not expected (a gap always follows the few requests): try again
	 * after the gap, whatever its distance */
	adv.event_tick = ll_arb_gap(adv.event_tick, 0, ADV_EVENT_US);
	(void)adv_request(adv.event_tick);
}

static void schedule_next_event(void)
{
	advance_event();
	if ((int32_t)(adv.event_tick - ll_radio_now()) < 0) {
		/* missed the window for a long time (e.g. stalled radio); re-base
		 * on now instead of scheduling an event far in the past */
		adv.event_tick = ll_radio_now() + interval_ticks() +
				 ll_plat_rand32() % (ADV_DELAY_MAX_TICKS + 1);
	}
	plan_event();
}

/* Advance to the next enabled channel; end the event after channel 39. */
static void next_channel(void)
{
	while (++adv.ch_idx < 3) {
		if (adv.prm.chan_map & (1 << adv.ch_idx)) {
			tx_current();
			return;
		}
	}
	adv.in_event = false;
	schedule_next_event();
}

static void start_event(void)
{
	if (!adv.enabled) {
		unsigned int key = ll_plat_lock();

		ll_arb_cancel(LL_ARB_ADV);
		ll_plat_unlock(key);
		return;
	}
	adv.stats.events++;
	if (scannable()) {
		ll_radio_prepare_rsp(adv.rsp_pdu, adv.rsp_pdu_len);
	}
	adv.ch_idx = 0;
	while (!(adv.prm.chan_map & (1 << adv.ch_idx))) {
		adv.ch_idx++;
	}
	adv.in_event = true;
	tx_current();
}

void ll_adv_arb_start(uint32_t cap_us)
{
	(void)cap_us;   /* fixed length, the span already covers it */
	start_event();
}

void ll_adv_arb_bumped(void)
{
	if (adv.enabled && !adv.in_event) {
		plan_event();
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
	memcpy(adv.adva, adva, 6);
	adv.on_connect = on_connect;
	defaults();
}

uint8_t ll_adv_enable(bool enable)
{
	unsigned int key = ll_plat_lock();

	if (enable && !adv.enabled) {
		/* any link taken (active or awaiting release): no
		 * advertising + connection combination yet (slice 6a Task 6
		 * allows it while a link is free) */
		if (ll_conn_count() != 0) {
			ll_plat_unlock(key);
			return LL_ST_DISALLOWED;
		}
		if (adv.radio_dirty) {
			adv.radio_dirty = false;
			ll_radio_adv_restore();
		}
		adv.enabled = true;
		adv.event_tick = ll_radio_now() + ADV_START_LEAD_TICKS;
		plan_event();
	} else if (!enable && adv.enabled) {
		adv.enabled = false;
		adv.in_event = false;
		ll_arb_cancel(LL_ARB_ADV);
		ll_radio_stop();
	}
	ll_plat_unlock(key);
	return LL_ST_SUCCESS;
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
	if (p->type == HCI_ADV_DIRECT_HIGH || p->type == HCI_ADV_DIRECT_LOW ||
	    p->own_addr_type != 0 || p->filter_policy != 0) {
		return LL_ST_UNSUPPORTED;
	}
	unsigned int key = ll_plat_lock();

	adv.prm = *p;
	build_pdus();
	ll_plat_unlock(key);
	return LL_ST_SUCCESS;
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

	if (!adv.enabled || !adv.in_event) {
		return;
	}
	if (evt == LL_RADIO_RX_OK) {
		if (scannable() && ll_pdu_is_scan_req_for(pdu, len, adv.adva)) {
			if (ll_radio_tx_rsp_at(end_tick + LL_T_IFS_US * LL_TICKS_PER_US)) {
				return; /* continue on LL_RADIO_TX_DONE */
			}
			/* too late to answer; no TX_DONE will come, so move on now */
		}
		if (connectable() && ll_pdu_parse_connect_ind(pdu, len, adv.adva, &ci) == 0) {
			if (adv.on_connect) {
				adv.on_connect(&ci);
			}
			/* Stop advertising first: ll_conn_start() takes over the
			 * radio and may report CONNECTED synchronously. No
			 * advertising alarm is pending inside an event. */
			adv.in_event = false;
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
