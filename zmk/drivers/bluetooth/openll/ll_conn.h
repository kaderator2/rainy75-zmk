/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Connection state (peripheral, LL_MAX_CONN links, slice 6a): transmit
 * window, window widening, anchor re-sync, CSA#1, event counter, instants,
 * supervision timeout, termination, per link. Drives the radio via
 * ll_radio.h, requests its events from the arbiter (ll_arb.h), and calls
 * the ll_txq / ll_rxq
 * per-event hooks of the link that owns the event.
 *
 * Link ids: uint8_t link, 0 <= link < LL_MAX_CONN (== the HCI connection
 * handle). A link is free, active, or ended and awaiting
 * ll_conn_release() (the consumer thread resets the link's ll_rxq and
 * ll_llcp state first). Calls with an id out of range or of a link that is
 * not active do nothing (or return LL_ST_DISALLOWED / false / 0).
 *
 * Events are arbitrated (slice 6a Task 5): every planned event is an
 * ll_arb request with a priority (MUST > SUPERVISION > ACTIVE > IDLE, see
 * ll_conn.c); a refused or displaced event is moved to another event of
 * the latency window (dodge) or yielded (counter and CSA#1 advance as for
 * a skip, stats.collisions counts it, not a miss for the latency rule).
 *
 * Timing uses stimer ticks only (LL_TICKS_PER_US). Window widening above
 * connInterval / 2 - T_IFS is clamped; the supervision timeout then ends
 * the link.
 *
 * Peripheral latency (Vol 6 Part B 4.5.1, 4.5.7; slice 5): after an event
 * the next one is planned up to connPeripheralLatency events ahead (the
 * latency from CONNECT_IND or the last applied connection update) when all
 * of these hold, else the next event is listened to:
 *  - ll_txq_backlog(link) == 0 (nothing queued, nothing unacked);
 *  - ops.busy(link) is false (no LLCP procedure waiting) and no local
 *    termination is running;
 *  - no channel map / connection update instant is pending in the
 *    candidate window [next, next + latency] (the instant event is always
 *    listened to, so no skip at all until it was);
 *  - the event just closed re-anchored (its first packet was received in
 *    the RX window): never skip after a miss, a late alarm or a first
 *    packet that was not the anchor;
 *  - the listened event's anchor stays <= last RX + connSupervisionTimeout
 *    - 2 * connInterval (defensive: with spec-valid parameters, timeout >
 *    (1 + latency) * interval * 2, this never limits the skip).
 * Skipped events still advance the event counter and CSA#1; window
 * widening uses the real time since the last anchor (at most 500 intervals
 * of growth, never reaching the clamp). New TX data calls ll_conn_kick().
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
 * ll_conn_end, ll_conn_update_at/chmap_at with an instant in the past).
 * link: the link the event belongs to. */
typedef void (*ll_conn_evt_cb_t)(uint8_t link, enum ll_conn_evt what, const void *arg);

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
	int (*ctrl_tx)(uint8_t link, const uint8_t *payload, uint8_t len);
	/* Peripheral latency: true while an LL control procedure waits on us
	 * or on the central (encryption start, a response not queued yet).
	 * Called from ISR context when the next event is planned, so it must
	 * not block or take ll_plat_tx_lock(); reading a flag is enough (a
	 * stale answer costs at most one skip window, the response itself
	 * kicks via ll_conn_kick()). NULL: never busy. */
	bool (*busy)(uint8_t link);
};

/* ops is copied; also ll_txq_init() (completions go to ops->txq_done).
 * Frees every link; the stats stay cumulative. */
void ll_conn_init(const struct ll_conn_ops *ops);
/* Start following a connection on the lowest free link:
 * ll_radio_conn_init() (one-time radio setup, a no-op while it is done),
 * ll_txq_reset(link), plan the first event in the transmit window, report
 * LL_CONN_EVT_CONNECTED. ll_rxq is NOT reset here (ISR context, the
 * consumer thread may be inside ll_rxq_get()): the owner of the consumer
 * thread resets the link's ll_rxq / ll_llcp when it handles
 * LL_CONN_EVT_DISCONNECTED and then calls ll_conn_release(), and only then
 * is the id free again (boot state is reset already).
 * connect_ind_end_tick = end of the CONNECT_IND packet (LL_RADIO_RX_OK
 * end_tick); the caller has stopped advertising. Returns the new link id
 * (>= 0), or -EINVAL for unusable parameters (nothing started, the caller
 * keeps advertising) or -EBUSY (no free link; checked first). ISR. */
int ll_conn_start(const struct ll_connect_ind *ci, uint32_t connect_ind_end_tick);
/* Radio callback in connection mode (same signature as ll_radio_cb_t). ISR.
 * Handles LL_RADIO_CONN_RX / _RX_CRC_ERR / _RX_NODATA / _DONE for the link
 * whose event is on air (the owner recorded when the BRX was issued),
 * ignores the rest and everything while no event is on air. */
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
int ll_conn_update_at(uint8_t link, uint16_t instant, uint8_t win_size, uint16_t win_offset,
		      const struct ll_conn_params *p);
int ll_conn_chmap_at(uint8_t link, uint16_t instant, const uint8_t chm[5]);
/* Local termination (HCI Disconnect via ll_llcp): queue LL_TERMINATE_IND
 * with reason (via ops.ctrl_tx), then end with LL_ST_LOCAL_TERM once it is
 * acked, or after connSupervisionTimeout without ack. Thread. */
void ll_conn_terminate(uint8_t link, uint8_t reason);
/* Immediate end with an HCI reason, no PDU sent: remote LL_TERMINATE_IND
 * (reason from the PDU; our response in the receiving event already acked
 * it), MIC failure (0x3D), LLCP response timeout (0x22). Deferred to the
 * end of the current event if one is on air. Thread or ISR. */
void ll_conn_end(uint8_t link, uint8_t reason);
bool ll_conn_active(uint8_t link);
/* ll_conn_end(link, reason) for every active link (one radio: a wedged
 * radio ends all links, the glue's guard-streak rule). Returns the number of
 * links ended (or whose end is pending). Thread or ISR. */
uint8_t ll_conn_end_all(uint8_t reason);
/* Links not free (active, or ended and awaiting ll_conn_release()). */
uint8_t ll_conn_count(void);
/* Thread: the consumer has reset the link's rxq and llcp after
 * LL_CONN_EVT_DISCONNECTED; the id may be reused. No-op for a link that is
 * not awaiting release. Takes ll_plat_lock(). */
void ll_conn_release(uint8_t link);
/* Counter of the next connection event not yet completed (the one on air,
 * if any). Instants are relative to this. While a latency skip is planned
 * this is the first skipped event (conservative: an instant for a skipped
 * event re-plans the listen to the first reachable event, or to the
 * instant if that comes first, and is passed if the instant event is no
 * longer reachable). After a re-plan (kick, instant, or a yield to the
 * arbiter) it is the planned event, so an instant for an earlier event is
 * treated as passed (0x28); a conforming central never sends one (the
 * instant is >= 6 events after the PDU, which arrives in a listened event;
 * after yields it may be passed in rare multilink overlaps). */
uint16_t ll_conn_event_counter(uint8_t link);
/* New TX data was queued (call after a successful ll_txq_push; the
 * ll_plat_lock() it takes nests, so the caller may hold it): if the planned
 * event lies beyond the next regular event that can still be prepared
 * (alarm LL_CONN_ARM_LEAD_US before its RX opens), re-plan to that event.
 * No-op without a connection, during an event (the next plan sees the
 * backlog) or when that event is already the planned one. An instant in
 * the skip window re-plans to the first reachable event (see
 * ll_conn_event_counter), and no skip is planned while an instant is
 * pending within the latency window, so a kick never waits for an instant.
 * ISR-safe; takes ll_plat_lock(). */
void ll_conn_kick(uint8_t link);

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
	/* Peripheral latency (slice 5). planned: listen alarms armed (a
	 * re-plan of the same listen counts once); planned - listened = late
	 * events + plans ended by the link end. listened: alias of events
	 * (events issued to the radio), named for the power counters. skipped:
	 * events skipped by latency, net of kick / instant re-plans; counted
	 * when planned, so it may overstate by up to latency when the link ends
	 * before the planned event. kicks: ll_conn_kick() calls that
	 * re-planned. */
	uint32_t planned;
	uint32_t listened;
	uint32_t skipped;
	uint32_t kicks;
	/* Slice 6a: events yielded to the arbiter (refused when planned,
	 * displaced, or started with no room); counted neither in listened
	 * nor missed nor skipped. planned - listened includes the yields at
	 * start. */
	uint32_t collisions;
};
/* Per link, cumulative since boot (not reset per connection). Out-of-range
 * link: all zero. */
void ll_conn_get_stats(uint8_t link, struct ll_conn_stats *s);
/* Sum over all links (widen_max_us: the maximum), as reported by the
 * aggregate group 66 fields and the stats log. */
void ll_conn_get_stats_total(struct ll_conn_stats *s);

/* ll_arb owner callbacks for the links (the glue's ll_arb_ops dispatch ids
 * < LL_MAX_CONN here). start: issue the BRX of the link's planned event
 * with the arbiter's cap as max_event_us (stimer ISR). bumped: the planned
 * event was displaced; re-plan (dodge, else yield) and request again. */
void ll_conn_arb_start(uint8_t link, uint32_t cap_us);
void ll_conn_arb_bumped(uint8_t link);

#endif /* LL_CONN_H_ */
