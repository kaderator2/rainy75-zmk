/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Connection state (peripheral, one connection): transmit window, window
 * widening, anchor re-sync, CSA#1, event counter, instants, supervision
 * timeout, termination. Drives the radio via ll_radio.h, the alarm via
 * ll_sched.h, and calls the ll_txq / ll_rxq per-event hooks.
 *
 * Timing uses stimer ticks only (LL_TICKS_PER_US). Peripheral latency is
 * accepted (CONNECT_IND, connection update) but not used before slice 5:
 * every connection event is listened to. Window widening above
 * connInterval / 2 - T_IFS is clamped; the supervision timeout then ends
 * the link.
 */
#ifndef LL_CONN_H_
#define LL_CONN_H_

#include <stdbool.h>
#include <stdint.h>

#include "ll_pdu.h"
#include "ll_radio.h"
#include "ll_txq.h"

/* Timing constants (us). The RX window of an event opens at
 * anchor - widening - margin (window events: transmit window start -
 * widening - LL_CONN_WIN_MARGIN_US) and lasts until the access address of a
 * packet starting at anchor + widening + margin (window end + widening +
 * margin) has been received. */
#define LL_CONN_SYNC_US        40    /* preamble + access address at 1M */
#define LL_CONN_RX_MARGIN_US   60    /* synced events (spike-proven) */
#define LL_CONN_WIN_MARGIN_US  200   /* transmit window events */
/* Alarm this long before the RX opens. Task 10: under traffic the stimer
 * ISR often starts 150..270 us late (another ISR still running, often the
 * USB ISR, which has a higher PLIC priority); with 300 us (180 us of
 * tolerance above LL_CONN_MIN_PREP_US) about 0.4 % of the events at a 7.5
 * ms interval were skipped as late, with 500 us none in 10 min. */
#define LL_CONN_ARM_LEAD_US    500
#define LL_CONN_MIN_PREP_US    120   /* alarm later than this before RX open: skip */
/* Event length cap, passed to ll_radio_conn_event() as max_event_us (the
 * radio's guard alarm fires at open + max_event_us and stops the BRX). It
 * is derived from the interval: interval - the widening growth over one
 * interval - LL_CONN_ARM_LEAD_US - LL_CONN_EVENT_SAFETY_US, so a long MD
 * burst of the central (SMP image upload, rgb_mgmt writes) may use almost
 * the whole interval but the guard always ends it before the next event's
 * alarm. The safety covers the larger margin of a transmit-window event at
 * an update instant (LL_CONN_WIN_MARGIN_US - LL_CONN_RX_MARGIN_US = 140 us)
 * and the guard ISR's own latency. The cap never cuts into the first RX
 * window: it is at least first_timeout_us + LL_CONN_GUARD_MIN_TAIL_US (one
 * exchange of maximum-length PDUs is about 0.8 ms). */
#define LL_CONN_EVENT_SAFETY_US   300
#define LL_CONN_GUARD_MIN_TAIL_US 1000

struct ll_conn_params {
	uint16_t interval;   /* 1.25 ms units */
	uint16_t latency;
	uint16_t timeout;    /* 10 ms units */
};

enum ll_conn_evt {
	/* arg: const struct ll_connect_ind * (the CONNECT_IND that started it).
	 * Reported by ll_conn_start() when the connection is created. */
	LL_CONN_EVT_CONNECTED,
	/* arg: const uint8_t * pointing at the HCI reason code. The
	 * connection is already torn down (no alarm, radio idle). */
	LL_CONN_EVT_DISCONNECTED,
	/* arg: const struct ll_conn_params * (new parameters, applied at the
	 * instant). Only reported when interval, latency or timeout changed. */
	LL_CONN_EVT_UPDATED,
};

/* ISR context (radio/stimer path), or thread context with ll_plat_lock()
 * held when the event results from a thread call (ll_conn_start,
 * ll_conn_end, ll_conn_update_at/chmap_at with an instant in the past). */
typedef void (*ll_conn_evt_cb_t)(enum ll_conn_evt what, const void *arg);

struct ll_conn_ops {
	ll_conn_evt_cb_t evt;
	/* ll_txq completions (ISR), forwarded unchanged after ll_conn's own
	 * LL_TERMINATE_IND tracking. May be NULL. */
	ll_txq_done_cb_t txq_done;
	/* Queue one own LL control PDU (plaintext payload, opcode first),
	 * encrypting it when the link is encrypted, via ll_txq_push(LL_TXQ_CTRL,
	 * LL_LLID_CTRL, ..., ctrl_opcode = payload[0]). Thread context; takes
	 * ll_plat_lock() itself. Returns 0 or a negative errno. NULL: ll_conn
	 * pushes the plaintext PDU itself (fine while unencrypted). The glue's
	 * hook (ll_llcp_ctrl_tx) also takes the ll_plat_tx_lock() mutex, so it
	 * may block: never call it from an ISR or with ll_plat_lock() held. */
	int (*ctrl_tx)(const uint8_t *payload, uint8_t len);
};

/* ops is copied. */
void ll_conn_init(const struct ll_conn_ops *ops);
/* Start following a connection: ll_radio_conn_setup(), ll_txq_reset(),
 * plan the first event in the transmit window, report
 * LL_CONN_EVT_CONNECTED. ll_rxq is NOT reset here (ISR context, the
 * consumer thread may be inside ll_rxq_get()): the owner of the consumer
 * thread resets it when it handles LL_CONN_EVT_DISCONNECTED, before
 * advertising can be enabled again (boot state is reset already). connect_ind_end_tick = end of the CONNECT_IND
 * packet (LL_RADIO_RX_OK end_tick); the caller has stopped advertising.
 * Returns 0, or -EINVAL for unusable parameters (nothing started, the
 * caller keeps advertising) or -EBUSY (a connection is active). ISR. */
int ll_conn_start(const struct ll_connect_ind *ci, uint32_t connect_ind_end_tick);
/* Radio callback in connection mode (same signature as ll_radio_cb_t). ISR.
 * Handles LL_RADIO_CONN_RX / LL_RADIO_CONN_DONE, ignores the rest. */
void ll_conn_radio_evt(enum ll_radio_evt evt, const uint8_t *pdu, uint8_t len, uint32_t tick);
/* Schedule LL_CONNECTION_UPDATE_IND / LL_CHANNEL_MAP_IND parameters for the
 * event with counter == instant. Return 0, or LL_ST_INSTANT_PASSED when
 * (instant - counter) mod 65536 > 32767, or when instant is the event
 * already on air (the connection is then terminated with 0x28). Without an
 * active connection: LL_ST_DISALLOWED. ll_conn_update_at checks the
 * parameters first (interval 6..3200, latency <= 499, timeout 10..3200 and
 * > (1 + latency) * interval * 2, WinSize 1..min(8, interval - 1), WinOffset
 * <= interval) and returns LL_ST_INVALID_LL_PARAM for invalid ones without
 * changing anything (the caller, ll_llcp, decides how to end the link).
 * Thread. */
int ll_conn_update_at(uint16_t instant, uint8_t win_size, uint16_t win_offset,
		      const struct ll_conn_params *p);
int ll_conn_chmap_at(uint16_t instant, const uint8_t chm[5]);
/* Local termination (HCI Disconnect via ll_llcp): queue LL_TERMINATE_IND
 * with reason (via ops.ctrl_tx), then end with LL_ST_LOCAL_TERM once it is
 * acked, or after connSupervisionTimeout without ack. Thread. */
void ll_conn_terminate(uint8_t reason);
/* Immediate end with an HCI reason, no PDU sent: remote LL_TERMINATE_IND
 * (reason from the PDU; our response in the receiving event already acked
 * it), MIC failure (0x3D), LLCP response timeout (0x22). Deferred to the
 * end of the current event if one is on air. Thread or ISR. */
void ll_conn_end(uint8_t reason);
bool ll_conn_active(void);
/* Counter of the next connection event not yet completed (the one on air,
 * if any). Instants are relative to this. */
uint16_t ll_conn_event_counter(void);

struct ll_conn_stats {
	uint32_t events;      /* events issued to the radio */
	uint32_t rx_events;   /* events with at least one CRC-valid packet */
	uint32_t missed;      /* events without a valid packet (incl. late) */
	uint32_t late;        /* events skipped: alarm too late to issue BRX */
	uint32_t rx_pkts;     /* CRC-valid packets */
	uint32_t widen_max_us;
	uint32_t first_bad;   /* events whose first packet had a bad CRC (no re-anchor) */
	uint32_t first_nodata; /* events whose first packet was not delivered (no re-anchor) */
	uint32_t first_outside; /* first delivered packet after the RX window (no re-anchor) */
};
/* Cumulative since boot. */
void ll_conn_get_stats(struct ll_conn_stats *s);

#endif /* LL_CONN_H_ */
