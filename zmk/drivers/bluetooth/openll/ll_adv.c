/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Legacy advertising: one event = TX on each enabled channel 37..39, each
 * followed by an RX window for SCAN_REQ / CONNECT_IND. Events repeat every
 * advInterval + advDelay (0..10 ms random), Core Spec Vol 6 Part B 4.4.2.
 * Radio callbacks run in ISR context; HCI-facing calls take ll_plat_lock().
 */
#include <string.h>
#include "ll_adv.h"
#include "ll_defs.h"
#include "ll_plat.h"
#include "ll_sched.h"

#define ADV_RX_WINDOW_US     300                         /* SCAN_REQ/CONNECT_IND start within T_IFS */
#define ADV_START_LEAD_TICKS (1000 * LL_TICKS_PER_US)   /* first event 1 ms after enable */
#define ADV_TX_LEAD_TICKS    (100 * LL_TICKS_PER_US)    /* radio programming headroom */
#define ADV_DELAY_MAX_TICKS  (10000 * LL_TICKS_PER_US)

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
	uint8_t ch_idx;       /* 0..2 = channel 37..39 */
	uint32_t event_tick;  /* start of the current advertising event */
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

static void start_event(void);

static void schedule_next_event(void)
{
	uint32_t interval = (uint32_t)adv.prm.interval_min * 625u * LL_TICKS_PER_US;
	uint32_t delay = ll_plat_rand32() % (ADV_DELAY_MAX_TICKS + 1);

	adv.event_tick += interval + delay;
	ll_sched_at(adv.event_tick, start_event);
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
	schedule_next_event();
}

static void start_event(void)
{
	if (!adv.enabled) {
		return;
	}
	if (scannable()) {
		ll_radio_prepare_rsp(adv.rsp_pdu, adv.rsp_pdu_len);
	}
	adv.ch_idx = 0;
	while (!(adv.prm.chan_map & (1 << adv.ch_idx))) {
		adv.ch_idx++;
	}
	tx_current();
}

void ll_adv_init(const uint8_t adva[6], ll_adv_connect_cb_t on_connect)
{
	memset(&adv, 0, sizeof(adv));
	memcpy(adv.adva, adva, 6);
	adv.on_connect = on_connect;
	defaults();
}

uint8_t ll_adv_enable(bool enable)
{
	unsigned int key = ll_plat_lock();

	if (enable && !adv.enabled) {
		adv.enabled = true;
		adv.event_tick = ll_radio_now() + ADV_START_LEAD_TICKS;
		ll_sched_at(adv.event_tick, start_event);
	} else if (!enable && adv.enabled) {
		adv.enabled = false;
		ll_sched_cancel();
		ll_radio_stop();
	}
	ll_plat_unlock(key);
	return LL_ST_SUCCESS;
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
	if (len > LL_ADV_DATA_MAX) {
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

	if (!adv.enabled) {
		return;
	}
	if (evt == LL_RADIO_RX_OK) {
		if (scannable() && ll_pdu_is_scan_req_for(pdu, len, adv.adva)) {
			ll_radio_tx_rsp_at(end_tick + LL_T_IFS_US * LL_TICKS_PER_US);
			return; /* continue on LL_RADIO_TX_DONE */
		}
		if (connectable() && ll_pdu_parse_connect_ind(pdu, len, adv.adva, &ci) == 0) {
			if (adv.on_connect) {
				adv.on_connect(&ci);
			}
			/* Slice 1 does not follow the connection: end this event and
			 * keep advertising. Slice 2 replaces this with the conn state. */
			schedule_next_event();
			return;
		}
	}
	next_channel();
}
