/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Legacy advertising state machine (ADV_IND / ADV_SCAN_IND / ADV_NONCONN_IND).
 */
#ifndef LL_ADV_H_
#define LL_ADV_H_

#include <stdbool.h>
#include <stdint.h>
#include "ll_pdu.h"
#include "ll_radio.h"

/* Own_Address_Type (Vol 4 Part E 7.8.5). 2/3 (controller-generated RPA)
 * need a resolving list and are refused with Unsupported Feature (0x11). */
#define LL_OWN_ADDR_PUBLIC 0
#define LL_OWN_ADDR_RANDOM 1

struct ll_adv_params {
	uint16_t interval_min;   /* units of 625 us */
	uint16_t interval_max;
	uint8_t type;            /* HCI adv type: 0 ADV_IND, 1 DIRECT_HIGH, 2 SCAN_IND, 3 NONCONN, 4 DIRECT_LOW */
	uint8_t own_addr_type;   /* LL_OWN_ADDR_PUBLIC / LL_OWN_ADDR_RANDOM; 2/3 unsupported */
	uint8_t peer_addr_type;
	uint8_t peer_addr[6];
	uint8_t chan_map;        /* bit0 = ch37, bit1 = ch38, bit2 = ch39 */
	uint8_t filter_policy;
};

/* adva: the public device address. Every advertising PDU carries the AdvA
 * of Own_Address_Type (public, or the address of LE Set Random Address)
 * with the matching TxAdd; only a SCAN_REQ / CONNECT_IND to that AdvA and
 * address type is ours (ci->adv_a / adv_addr_random record it). */
/* Called (ISR) for every CONNECT_IND addressed to us, before it is handed
 * to ll_conn_start(); informational (logging). May be NULL. */
typedef void (*ll_adv_connect_cb_t)(const struct ll_connect_ind *ci);

void ll_adv_init(const uint8_t adva[6], ll_adv_connect_cb_t on_connect);
void ll_adv_reset(void);
/* LL_ST_UNSUPPORTED (0x11) for own types 2/3, directed types and filter
 * policies != 0; LL_ST_DISALLOWED while enabled. */
uint8_t ll_adv_set_params(const struct ll_adv_params *p);
uint8_t ll_adv_set_data(const uint8_t *data, uint8_t len);
uint8_t ll_adv_set_scan_rsp(const uint8_t *data, uint8_t len);
/* Enabling is allowed while a link is free (ll_conn_count() <
 * LL_MAX_CONN); connectable advertising (ADV_IND) with every link taken
 * (active or awaiting ll_conn_release()) returns LL_ST_CONN_LIMIT (0x09),
 * non-connectable advertising is always allowed. A CONNECT_IND accepted by
 * ll_conn_start() disables advertising (the host re-enables it; no
 * controller-side resume). The first enable with no link left after a
 * connection calls ll_radio_adv_restore(); while links exist, adv channels
 * enter adv mode with ll_radio_adv_enter() instead. */
/* Enabling with own type random and no random address set (since init or
 * Reset) returns LL_ST_INVALID_PARAM (0x12, Vol 4 Part E 7.8.9). The AdvA
 * is fixed at the enable. */
uint8_t ll_adv_enable(bool enable);
/* LE Set Random Address (Vol 4 Part E 7.8.4): stores addr (LSB first) and
 * returns 0, or LL_ST_DISALLOWED (0x0C) while advertising is enabled with
 * own type random. Allowed while advertising public (the AdvA on air
 * stays public). ll_adv_reset() forgets it. */
uint8_t ll_adv_set_random_addr(const uint8_t addr[6]);
bool ll_adv_is_enabled(void);

/* ll_arb owner callbacks for advertising (id LL_ARB_ADV): start runs the
 * adv event's first channel, or the next channel of a sliced event (cap
 * ignored, a channel has a fixed length), bumped re-plans it into the
 * next gap. */
void ll_adv_arb_start(uint32_t cap_us);
void ll_adv_arb_bumped(void);

struct ll_adv_stats {
	uint32_t events;    /* adv events started */
	uint32_t slid;      /* events moved into a gap by the arbiter */
	uint32_t dropped;   /* events dropped: no gap before the next interval */
	uint32_t cut;       /* events ended early: next channel found no gap within 10 ms */
	uint32_t stuck;     /* advertising disabled: no request possible (never expected) */
};
/* Cumulative since boot. */
void ll_adv_get_stats(struct ll_adv_stats *s);

/* Radio completion handler; registered with ll_radio_init(). ISR context. */
void ll_adv_radio_evt(enum ll_radio_evt evt, const uint8_t *pdu, uint8_t len,
		      uint32_t end_tick);

#endif /* LL_ADV_H_ */
