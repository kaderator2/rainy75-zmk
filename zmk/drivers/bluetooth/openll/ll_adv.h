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

struct ll_adv_params {
	uint16_t interval_min;   /* units of 625 us */
	uint16_t interval_max;
	uint8_t type;            /* HCI adv type: 0 ADV_IND, 1 DIRECT_HIGH, 2 SCAN_IND, 3 NONCONN, 4 DIRECT_LOW */
	uint8_t own_addr_type;
	uint8_t peer_addr_type;
	uint8_t peer_addr[6];
	uint8_t chan_map;        /* bit0 = ch37, bit1 = ch38, bit2 = ch39 */
	uint8_t filter_policy;
};

typedef void (*ll_adv_connect_cb_t)(const struct ll_connect_ind *ci);

void ll_adv_init(const uint8_t adva[6], ll_adv_connect_cb_t on_connect);
void ll_adv_reset(void);
uint8_t ll_adv_set_params(const struct ll_adv_params *p);
uint8_t ll_adv_set_data(const uint8_t *data, uint8_t len);
uint8_t ll_adv_set_scan_rsp(const uint8_t *data, uint8_t len);
uint8_t ll_adv_enable(bool enable);

/* Radio completion handler; registered with ll_radio_init(). ISR context. */
void ll_adv_radio_evt(enum ll_radio_evt evt, const uint8_t *pdu, uint8_t len,
		      uint32_t end_tick);

#endif /* LL_ADV_H_ */
