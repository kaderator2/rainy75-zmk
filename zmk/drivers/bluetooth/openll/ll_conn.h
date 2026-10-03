/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Connection state (peripheral, one connection): transmit window, window
 * widening, anchor re-sync, CSA#1, event counter, instants, supervision
 * timeout, termination. Drives the radio via ll_radio.h.
 */
#ifndef LL_CONN_H_
#define LL_CONN_H_

#include <stdbool.h>
#include <stdint.h>

#include "ll_pdu.h"
#include "ll_radio.h"

struct ll_conn_params {
	uint16_t interval;   /* 1.25 ms units */
	uint16_t latency;
	uint16_t timeout;    /* 10 ms units */
};

enum ll_conn_evt {
	/* arg: const struct ll_connect_ind * (the CONNECT_IND that started it).
	 * Reported when the connection is created (CONNECT_IND accepted). */
	LL_CONN_EVT_CONNECTED,
	/* arg: const uint8_t * pointing at the HCI reason code */
	LL_CONN_EVT_DISCONNECTED,
	/* arg: const struct ll_conn_params * (new parameters, applied at instant) */
	LL_CONN_EVT_UPDATED,
};

/* ISR context (called from the radio/stimer path). */
typedef void (*ll_conn_evt_cb_t)(enum ll_conn_evt what, const void *arg);

void ll_conn_init(ll_conn_evt_cb_t cb);
/* Start following a connection. connect_ind_end_tick = end of the
 * CONNECT_IND packet (LL_RADIO_RX_OK end_tick). */
void ll_conn_start(const struct ll_connect_ind *ci, uint32_t connect_ind_end_tick);
/* Radio callback in connection mode (same signature as ll_radio_cb_t). ISR. */
void ll_conn_radio_evt(enum ll_radio_evt evt, const uint8_t *pdu, uint8_t len, uint32_t tick);
/* Schedule LL_CONNECTION_UPDATE_IND / LL_CHANNEL_MAP_IND at instant.
 * Return 0, or LL_ST_INSTANT_PASSED (connection is then terminated). */
int ll_conn_update_at(uint16_t instant, uint8_t win_size, uint16_t win_offset,
		      const struct ll_conn_params *p);
int ll_conn_chmap_at(uint16_t instant, const uint8_t chm[5]);
/* Local termination: LL_TERMINATE_IND, then disconnect after ack/timeout. */
void ll_conn_terminate(uint8_t reason);
bool ll_conn_active(void);
uint16_t ll_conn_event_counter(void);

#endif /* LL_CONN_H_ */
