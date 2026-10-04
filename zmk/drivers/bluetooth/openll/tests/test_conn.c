/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_conn host tests with a fake radio (connection mode + a passive TX FIFO
 * whose rptr the test moves) and a fake one-shot alarm. The real ll_txq,
 * ll_rxq, ll_csa1, ll_csa2 (and ll_crypt, linked by ll_rxq) are used.
 *
 * Expected timing values are worked out by hand from the Core Spec Vol 6
 * Part B 4.5.3 (transmit window), 4.5.4 (window widening), 4.5.2/4.5.5
 * (supervision), 5.1.1 (connection update), 5.1.2 (channel map). Window
 * widening is rounded up to whole us: ceil((SCA_ppm + 50) * dt / 1e6) + 16.
 */
#include <errno.h>
#include <string.h>
#include "test.h"
#include "../ll_arb.h"
#include "../ll_conn.h"
#include "../ll_csa1.h"
#include "../ll_csa2.h"
#include "../ll_defs.h"
#include "../ll_plat.h"
#include "../ll_rxq.h"
#include "../ll_sched.h"
#include "../ll_txq.h"

#define T(us)      ((uint32_t)(us) * LL_TICKS_PER_US)
#define HDR_NESN   0x04
#define HDR_SN     0x08

void aes_ref_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

void ll_plat_aes_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
	aes_ref_encrypt(key, in, out);
}

/* ---------------- fakes ---------------- */

static uint32_t now;

static struct {
	int setups;          /* ll_radio_conn_init calls */
	int selects;
	bool selected;       /* ll_radio_conn_select since the last event */
	uint32_t aa, crc;
	int events;
	uint8_t ch;
	uint32_t open;
	uint32_t fst;
	uint32_t max_ev;
	uint8_t sn_init;
	uint8_t rptr, wptr;
	uint8_t fifo[4][LL_DATA_PDU_MAX + LL_MIC_LEN];
	uint8_t fifo_len[4];
	uint8_t fifo_hdr[4];
} rad;

static struct {
	uint32_t tick;
	ll_sched_cb_t cb;
	int cancels;
} sch;

static int locks;

uint32_t ll_radio_now(void) { return now; }
void ll_radio_conn_init(void)
{
	rad.setups++;
}
void ll_radio_conn_select(uint32_t aa, uint32_t crc_init)
{
	rad.selects++;
	rad.selected = true;
	rad.aa = aa;
	rad.crc = crc_init;
}
void ll_radio_conn_event(uint8_t ch, uint32_t open_tick, uint32_t first_timeout_us,
			 uint32_t max_event_us)
{
	CHECK(rad.selected);   /* the link's AA/CRC are selected for every event */
	rad.selected = false;
	rad.events++;
	rad.ch = ch;
	rad.open = open_tick;
	rad.fst = first_timeout_us;
	rad.max_ev = max_event_us;
}
void ll_radio_conn_set_sn_init(uint8_t sn) { rad.sn_init = sn; }
void ll_radio_conn_set_nesn_init(uint8_t nesn) { (void)nesn; }
uint8_t ll_radio_fifo_rptr(void) { return rad.rptr; }
uint8_t ll_radio_fifo_wptr(void) { return rad.wptr; }
void ll_radio_fifo_write(uint8_t idx, uint8_t hdr0, const uint8_t *payload, uint8_t len)
{
	rad.fifo_hdr[idx & 3] = hdr0;
	rad.fifo_len[idx & 3] = len;
	if (len) {
		memcpy(rad.fifo[idx & 3], payload, len);
	}
}
void ll_radio_fifo_set_wptr(uint8_t wptr) { rad.wptr = wptr; }
void ll_sched_init(void) {}
void ll_sched_at(uint32_t tick, ll_sched_cb_t cb)
{
	sch.tick = tick;
	sch.cb = cb;
}
void ll_sched_cancel(void)
{
	sch.cb = NULL;
	sch.cancels++;
}
uint32_t ll_plat_rand32(void) { return 42; }
unsigned int ll_plat_lock(void) { locks++; return 0; }
void ll_plat_unlock(unsigned int k) { (void)k; locks--; }

/* ---------------- callbacks ---------------- */

static struct {
	int connected, disconnected, updated;
	uint8_t link;          /* link of the last event callback */
	uint8_t reason;
	struct ll_conn_params p;
	int done_ctrl;
	uint8_t done_op;
	uint8_t done_link;
	int ctrl_tx_calls;
	uint8_t ctrl_tx_link;
	uint8_t ctrl_tx_pdu[4];
	uint8_t ctrl_tx_len;
	int busy_calls;
	uint8_t busy_link;
} cbs;

/* The glue releases a link once it has reset that link's rxq and llcp
 * (thread context). The tests release it from the DISCONNECTED callback,
 * unless a test holds the id back to check the release rule. */
static bool auto_release;

static void on_evt(uint8_t link, enum ll_conn_evt what, const void *arg)
{
	CHECK(link < LL_MAX_CONN);
	cbs.link = link;
	switch (what) {
	case LL_CONN_EVT_CONNECTED:
		cbs.connected++;
		CHECK(ll_conn_active(link));
		break;
	case LL_CONN_EVT_DISCONNECTED:
		cbs.disconnected++;
		cbs.reason = *(const uint8_t *)arg;
		CHECK(!ll_conn_active(link));
		/* not free before the release */
		CHECK(ll_conn_count() >= 1);
		if (ll_conn_count() == 1) {
			CHECK(sch.cb == NULL);   /* no other link left: no alarm */
		}
		if (auto_release) {
			ll_conn_release(link);
		}
		break;
	case LL_CONN_EVT_UPDATED:
		cbs.updated++;
		cbs.p = *(const struct ll_conn_params *)arg;
		break;
	}
}

static void on_txq_done(uint8_t link, enum ll_txq_kind kind, uint8_t op)
{
	CHECK(link < LL_MAX_CONN);
	if (kind == LL_TXQ_CTRL) {
		cbs.done_ctrl++;
		cbs.done_op = op;
		cbs.done_link = link;
	}
}

static bool busy_flag;

static bool hook_busy(uint8_t link)
{
	cbs.busy_calls++;
	cbs.busy_link = link;
	return busy_flag;
}

static int hook_ctrl_tx(uint8_t link, const uint8_t *payload, uint8_t len)
{
	cbs.ctrl_tx_calls++;
	cbs.ctrl_tx_link = link;
	cbs.ctrl_tx_len = len;
	memcpy(cbs.ctrl_tx_pdu, payload, len);
	return 0;
}

/* the arbiter (real ll_arb) dispatches to ll_conn as the glue does; no
 * advertising in these tests */
static void arb_start(uint8_t id, uint32_t cap_us)
{
	CHECK(id < LL_MAX_CONN);
	ll_conn_arb_start(id, cap_us);
}

static void arb_bumped(uint8_t id)
{
	CHECK(id < LL_MAX_CONN);
	ll_conn_arb_bumped(id);
}

static const struct ll_arb_ops arb_ops = {.start = arb_start, .bumped = arb_bumped};

/* ---------------- helpers ---------------- */

static const uint8_t all37[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
static const uint8_t no0to9[5] = {0x00, 0xFC, 0xFF, 0xFF, 0x1F};
static const uint8_t no35[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x17};

static void reset_all(bool hook)
{
	struct ll_conn_ops ops = {
		.evt = on_evt,
		.txq_done = on_txq_done,
		.ctrl_tx = hook ? hook_ctrl_tx : NULL,
		.busy = hook_busy,
	};

	now = 0;
	busy_flag = false;
	memset(&rad, 0, sizeof(rad));
	memset(&sch, 0, sizeof(sch));
	memset(&cbs, 0, sizeof(cbs));
	auto_release = true;
	ll_arb_init(&arb_ops);
	ll_conn_init(&ops);
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		ll_rxq_reset(i);   /* the glue's job (controller thread), not ll_conn's */
	}
}

static struct ll_connect_ind mk_ci(uint16_t interval, uint16_t timeout, uint8_t sca,
				   uint8_t win_size, uint16_t win_offset)
{
	struct ll_connect_ind ci;

	memset(&ci, 0, sizeof(ci));
	ci.aa = 0x8e89bed6u ^ 0x12345678u;
	ci.crc_init = 0x555555;
	ci.win_size = win_size;
	ci.win_offset = win_offset;
	ci.interval = interval;
	ci.latency = 0;
	ci.timeout = timeout;
	memcpy(ci.chm, all37, 5);
	ci.hop = 7;
	ci.sca = sca;
	return ci;
}

static void fire_alarm(void)
{
	ll_sched_cb_t cb = sch.cb;

	CHECK(cb != NULL);
	if (!cb) {
		return;
	}
	sch.cb = NULL;
	if ((int32_t)(sch.tick - now) > 0) {
		now = sch.tick;   /* time never runs backwards */
	}
	cb();
}

/* the alarm of the planned event fires LL_CONN_ARM_LEAD_US before its RX opens */
static void check_alarm_lead(void)
{
	CHECK(sch.tick == rad.open - T(LL_CONN_ARM_LEAD_US));
}

static void rx(uint32_t anchor, uint8_t hdr0, uint8_t paylen)
{
	uint8_t pdu[2 + LL_DATA_PDU_MAX] = {hdr0, paylen};

	for (uint8_t i = 0; i < paylen; i++) {
		pdu[2 + i] = (uint8_t)(0xA0 + i);
	}
	now = anchor + T(LL_CONN_SYNC_US);
	ll_conn_radio_evt(LL_RADIO_CONN_RX, pdu, (uint8_t)(2 + paylen), now);
}

/* CRC-bad packet whose access address ends at anchor + sync */
static void rx_bad(uint32_t anchor)
{
	now = anchor + T(LL_CONN_SYNC_US);
	ll_conn_radio_evt(LL_RADIO_CONN_RX_CRC_ERR, NULL, 0, now);
}

/* the hardware received a CRC-valid packet but wrote no RX entry (an
 * acked retransmission of the central), with its access address ending at
 * anchor + sync */
static void rx_nodata(uint32_t anchor)
{
	now = anchor + T(LL_CONN_SYNC_US);
	ll_conn_radio_evt(LL_RADIO_CONN_RX_NODATA, NULL, 0, now);
}

static void done(uint8_t n_rx)
{
	if ((int32_t)(rad.open + T(rad.fst) - now) > 0) {
		now = rad.open + T(rad.fst);
	}
	ll_conn_radio_evt(LL_RADIO_CONN_DONE, NULL, n_rx, now);
}

/* one event: alarm, one empty central packet with its anchor at anchor */
static void ev_rx(uint32_t anchor)
{
	fire_alarm();
	rx(anchor, 0x01, 0);
	done(1);
}

static void ev_miss(void)
{
	fire_alarm();
	done(0);
}

static uint32_t widen(uint32_t ppm, uint32_t dt_us)
{
	return (uint32_t)(((uint64_t)ppm * dt_us + 999999u) / 1000000u) + 16;
}

/* ---------------- tests ---------------- */

/* 4.5.3 transmit window, 4.5.4 widening, anchor re-sync, CSA#1, counter */
static void test_first_events(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 2, 3);
	const uint32_t t0 = 1000000;
	uint32_t a1, a2;

	reset_all(false);
	CHECK(!ll_conn_active(0));
	CHECK(ll_conn_start(&ci, t0) == 0);
	CHECK(ll_conn_active(0));
	CHECK(cbs.connected == 1);
	CHECK(rad.setups == 1);
	CHECK(ll_conn_event_counter(0) == 0);
	if (LL_MAX_CONN == 1) {
		CHECK(ll_conn_start(&ci, t0) == -EBUSY);   /* N > 1: test_link_ids */
	}

	/* window start = t0 + 1.25 ms + 3 * 1.25 ms = t0 + 80000 ticks, size 2.5 ms.
	 * widening: SCA 1 = 250 ppm + 50 own = 300 ppm over 7500 us (to window
	 * end) = 2.25 -> 3, + 16 = 19 us. open = 1080000 - (19 + 200) * 16. */
	fire_alarm();
	CHECK(rad.events == 1);
	CHECK(rad.selects == 1 && rad.aa == ci.aa && rad.crc == ci.crc_init);
	CHECK(rad.ch == 7);
	CHECK(rad.open == 1076496);
	CHECK(rad.fst == 2500 + 2 * (19 + 200) + 40);
	CHECK(sch.tick == 0 || sch.cb == NULL);
	CHECK(rad.sn_init == 0);

	/* first packet 1 ms into the window, a second chained packet later
	 * must not move the anchor */
	a1 = 1080000 + T(1000);
	rx(a1, 0x01, 0);
	rx(a1 + T(400), 0x05 | HDR_SN, 0);
	done(2);
	CHECK(ll_conn_event_counter(0) == 1);
	/* next anchor a1 + 15 ms, widening 300 ppm * 15000 us = 4.5 -> 5 + 16 */
	CHECK(sch.cb != NULL);
	fire_alarm();
	CHECK(rad.ch == 14);
	CHECK(rad.open == a1 + T(15000) - T(21 + LL_CONN_RX_MARGIN_US));
	CHECK(rad.fst == 2 * (21 + 60) + 40);
	check_alarm_lead();
	/* SN_INIT = NESN of the central's last packet (via ll_txq_rx) */
	CHECK(rad.sn_init == 1);
	done(0);
	CHECK(ll_conn_event_counter(0) == 2);

	/* missed event: widening over 2 intervals: 9 + 16 = 25 */
	fire_alarm();
	CHECK(rad.ch == 21);
	CHECK(rad.open == a1 + T(30000) - T(25 + 60));
	CHECK(rad.fst == 2 * (25 + 60) + 40);
	/* central 2 us late: re-anchor on it */
	a2 = a1 + T(30002);
	rx(a2, 0x01, 0);
	done(1);
	fire_alarm();
	CHECK(rad.ch == 28);
	CHECK(rad.open == a2 + T(15000) - T(21 + 60));
	done(0);
	CHECK(ll_conn_event_counter(0) == 4);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
	CHECK(cbs.disconnected == 1 && cbs.reason == LL_ST_REMOTE_TERM);
}

/* RX path: every CONN_RX goes to ll_rxq in order */
static void test_rx_path(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	struct ll_rx_pdu out;

	reset_all(false);
	CHECK(ll_conn_start(&ci, 500000) == 0);
	fire_alarm();
	rx(500000 + T(1250 + 100), LL_LLID_START, 5);
	rx(500000 + T(1250 + 500), LL_LLID_CTRL | HDR_SN, 3);
	done(2);
	CHECK(ll_rxq_get(0, &out) == LL_RXQ_OK);
	CHECK(out.len == 5 && out.data[0] == 0xA0 && (out.hdr0 & 3) == LL_LLID_START);
	CHECK(ll_rxq_get(0, &out) == LL_RXQ_OK);
	CHECK(out.len == 3 && (out.hdr0 & 3) == LL_LLID_CTRL);
	CHECK(ll_rxq_get(0, &out) == LL_RXQ_EMPTY);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* A data PDU that does not fit into ll_rxq is lost for good: the hardware
 * has acked it already, the central will not resend it. The link must end
 * deterministically (0x08, at the end of the event) instead of continuing
 * with a hole in the L2CAP stream or the CCM packet counter. Empty PDUs
 * never fill the ring, so a full ring of data PDUs alone is fine. */
static void test_rxq_overflow_ends_link(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	struct ll_rx_pdu out;
	uint32_t a1;
	uint8_t sn = 0;

	reset_all(false);
	CHECK(ll_conn_start(&ci, 500000) == 0);
	a1 = 500000 + T(1250 + 300);
	/* 16 data PDUs (the ring depth) over 4 events, nothing drained */
	for (uint32_t k = 0; k < 4; k++) {
		fire_alarm();
		for (uint32_t i = 0; i < 4; i++) {
			rx(a1 + T(15000 * k + 400 * i), LL_LLID_START | sn, 4);
			sn ^= HDR_SN;
		}
		rx(a1 + T(15000 * k + 1600), 0x01 | sn, 0);   /* empty: not queued */
		sn ^= HDR_SN;
		done(5);
		CHECK(ll_conn_active(0));
	}
	CHECK(cbs.disconnected == 0);
	/* the 17th data PDU overflows: the event completes, then the link ends */
	fire_alarm();
	rx(a1 + T(60000), LL_LLID_START | sn, 4);
	CHECK(ll_conn_active(0));
	done(1);
	CHECK(!ll_conn_active(0));
	CHECK(cbs.disconnected == 1 && cbs.reason == LL_ST_CONN_TIMEOUT);
	CHECK(sch.cb == NULL);
	CHECK(ll_rxq_overflow_count(0) == 1);
	/* the 16 queued PDUs are still delivered in order */
	for (int i = 0; i < 16; i++) {
		CHECK(ll_rxq_get(0, &out) == LL_RXQ_OK && out.len == 4);
	}
	CHECK(ll_rxq_get(0, &out) == LL_RXQ_EMPTY);
}

/* ll_conn_start runs in ISR context and must not reset ll_rxq (the
 * controller thread may be inside ll_rxq_get); the glue resets it in its
 * thread on DISCONNECTED. A PDU already in the ring survives the start. */
static void test_start_keeps_rxq(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	const uint8_t pdu[3] = {LL_LLID_START, 1, 0x5A};
	struct ll_rx_pdu out;

	reset_all(false);
	CHECK(ll_rxq_isr_put(0, pdu, sizeof(pdu)));
	CHECK(ll_conn_start(&ci, 500000) == 0);
	CHECK(ll_rxq_get(0, &out) == LL_RXQ_OK);
	CHECK(out.len == 1 && out.data[0] == 0x5A);
	CHECK(ll_rxq_get(0, &out) == LL_RXQ_EMPTY);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* Only the first packet of an event marks its anchor. When that packet has a
 * bad CRC, a later valid (MD) packet of the same event, about 400 us later,
 * must not re-anchor; it only refreshes the supervision timer. */
static void test_first_packet_bad_crc(void)
{
	struct ll_connect_ind ci = mk_ci(12, 10, 1, 1, 0);   /* 15 ms, 100 ms timeout */
	struct ll_conn_stats st0, st;
	uint32_t a1;

	reset_all(false);
	ll_conn_get_stats(0, &st0);
	CHECK(ll_conn_start(&ci, 500000) == 0);
	a1 = 500000 + T(1250 + 300);
	ev_rx(a1);
	/* event 1: bad first packet at the true anchor, valid one 400 us later */
	fire_alarm();
	rx_bad(a1 + T(15000));
	rx(a1 + T(15400), 0x01, 0);
	done(1);
	/* not re-anchored: event 2 still from a1, widening over 2 intervals
	 * (300 ppm * 30000 us = 9, + 16 = 25 us) */
	fire_alarm();
	CHECK(rad.open == a1 + T(30000) - T(25 + LL_CONN_RX_MARGIN_US));
	rx_bad(a1 + T(30000));
	rx(a1 + T(30400), 0x01, 0);
	done(1);
	/* supervision refreshed by the later valid packets: keep this up for
	 * more than 100 ms after the last clean anchor a1 */
	for (uint32_t k = 3; k <= 8; k++) {
		fire_alarm();
		rx_bad(a1 + T(15000 * k));
		rx(a1 + T(15000 * k + 400), 0x01, 0);
		done(1);
		CHECK(ll_conn_active(0));
	}
	ll_conn_get_stats(0, &st);
	CHECK(st.first_bad - st0.first_bad == 8);
	CHECK(st.rx_events - st0.rx_events == 9);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* Task 10 (device + sniffer, forced NACKs): the central resends its packet
 * at the anchor, the hardware acks it without an RX entry, and with MD the
 * event continues with a new packet about 400..700 us later. That packet
 * is not the anchor packet: re-anchoring on it moved the RX window late,
 * and the link was deaf for 40..130 events until widening caught up. */
static void test_first_packet_retransmission(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	struct ll_rx_pdu out;
	uint32_t a1;

	reset_all(false);
	CHECK(ll_conn_start(&ci, 500000) == 0);
	a1 = 500000 + T(1250 + 300);
	ev_rx(a1);
	/* event 1: retransmitted anchor packet, then a new chained one */
	fire_alarm();
	rx_nodata(a1 + T(15000));
	rx(a1 + T(15000 + 600), LL_LLID_START | HDR_SN, 4);
	done(1);
	/* not re-anchored: event 2 still planned from a1 (2 intervals of
	 * widening: 25 us); the chained PDU still reaches ll_rxq */
	fire_alarm();
	CHECK(rad.open == a1 + T(30000) - T(25 + LL_CONN_RX_MARGIN_US));
	CHECK(ll_rxq_get(0, &out) == LL_RXQ_OK && out.len == 4);
	/* a normal event re-anchors again */
	rx(a1 + T(30001), 0x01, 0);
	done(1);
	fire_alarm();
	CHECK(rad.open == a1 + T(45001) - T(21 + LL_CONN_RX_MARGIN_US));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* The same situation when the radio reports only the chained packet (the
 * retransmission not seen, e.g. both handled in one late ISR): a first
 * valid packet whose access address ends after the RX window closed cannot
 * be the anchor packet (the hardware syncs the first packet only inside
 * the window). It refreshes the supervision timer but does not re-anchor. */
static void test_first_packet_after_window(void)
{
	struct ll_connect_ind ci = mk_ci(12, 10, 1, 1, 0);   /* 100 ms timeout */
	uint32_t a1;

	reset_all(false);
	CHECK(ll_conn_start(&ci, 500000) == 0);
	a1 = 500000 + T(1250 + 300);
	ev_rx(a1);
	/* window of event 1: open = a1 + 15 ms - (21 + 60), fst = 2 * 81 + 40,
	 * so it closes at a1 + 15 ms + 81 + 40 us */
	fire_alarm();
	CHECK(rad.fst == 2 * (21 + 60) + 40);
	rx(a1 + T(15000 + 500), 0x01, 0);
	done(1);
	fire_alarm();
	CHECK(rad.open == a1 + T(30000) - T(25 + LL_CONN_RX_MARGIN_US));
	done(0);
	/* a packet at the window edge is still the anchor packet */
	fire_alarm();
	rx(a1 + T(45000 + 29 + 60), 0x01, 0);
	done(1);
	fire_alarm();
	CHECK(rad.open == a1 + T(60000 + 29 + 60) - T(21 + LL_CONN_RX_MARGIN_US));
	done(0);
	/* supervision: only late (non-anchor) packets for > 100 ms keep the
	 * link up */
	for (uint32_t k = 5; k <= 12; k++) {
		fire_alarm();
		rx(rad.open + T(rad.fst + 300), 0x01, 0);
		done(1);
		CHECK(ll_conn_active(0));
	}
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* 4.5.2: not established within 6 connection events -> 0x3E; a missed
 * transmit window repeats one interval later with the same window */
static void test_six_interval_rule(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 2, 3);
	const uint32_t ws = 1000000 + 80000;

	reset_all(false);
	CHECK(ll_conn_start(&ci, 1000000) == 0);
	ev_miss();
	/* event 1: window at ws + 15 ms, widening over 22.5 ms: 6.75 -> 7 + 16 */
	fire_alarm();
	CHECK(rad.open == ws + T(15000) - T(23 + LL_CONN_WIN_MARGIN_US));
	CHECK(rad.fst == 2500 + 2 * (23 + 200) + 40);
	done(0);
	for (int i = 2; i < 5; i++) {
		ev_miss();
	}
	CHECK(ll_conn_active(0) && cbs.disconnected == 0);
	CHECK(ll_conn_event_counter(0) == 5);
	ev_miss();
	CHECK(!ll_conn_active(0));
	CHECK(cbs.disconnected == 1 && cbs.reason == LL_ST_CONN_FAIL_EST);
	CHECK(sch.cb == NULL);
}

/* 4.5.5: supervision timeout after established -> 0x08 */
static void test_supervision(void)
{
	struct ll_connect_ind ci = mk_ci(12, 10, 1, 1, 0);   /* 100 ms */
	const uint32_t a0 = 2000000 + T(1250 + 200);

	reset_all(false);
	CHECK(ll_conn_start(&ci, 2000000) == 0);
	ev_rx(a0);
	/* events 1..6 end at about a0 + k * 15 ms + 0.1 ms < 100 ms */
	for (int k = 1; k <= 6; k++) {
		ev_miss();
		CHECK(ll_conn_active(0));
	}
	/* event 7 ends at a0 + 105 ms */
	ev_miss();
	CHECK(!ll_conn_active(0));
	CHECK(cbs.disconnected == 1 && cbs.reason == LL_ST_CONN_TIMEOUT);
	CHECK(sch.cb == NULL);
	/* nothing more happens on a late radio callback */
	done(0);
	CHECK(cbs.disconnected == 1);
}

/* widening is clamped to connInterval / 2 - T_IFS */
static void test_widening_clamp(void)
{
	struct ll_connect_ind ci = mk_ci(6, 3200, 0, 1, 0);  /* 7.5 ms, 32 s, 500 ppm */
	struct ll_conn_stats st;

	reset_all(false);
	CHECK(ll_conn_start(&ci, 3000000) == 0);
	ev_rx(3000000 + T(1250 + 100));
	for (int k = 0; k < 1000; k++) {
		ev_miss();
	}
	CHECK(ll_conn_active(0));
	fire_alarm();
	CHECK(rad.fst == 2 * (3600 + 60) + 40);
	ll_conn_get_stats(0, &st);
	CHECK(st.widen_max_us == 3600);
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* 5.1.1 connection update: at the instant, the window starts at the old
 * anchor of the instant event + WinOffset, size WinSize */
static void test_conn_update(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	struct ll_conn_params np = {.interval = 6, .latency = 2, .timeout = 50};
	uint32_t a, ws, a_new;
	uint16_t c;

	reset_all(false);
	CHECK(ll_conn_start(&ci, 4000000) == 0);
	a = 4000000 + T(1250 + 300);
	ev_rx(a);
	c = ll_conn_event_counter(0);
	CHECK(c == 1);
	CHECK(ll_conn_update_at(0, c + 6, 1, 2, &np) == 0);
	for (int k = 1; k <= 5; k++) {
		a += T(15000);
		ev_rx(a);
	}
	CHECK(ll_conn_event_counter(0) == c + 5);
	CHECK(cbs.updated == 0);
	a += T(15000);
	ev_rx(a);
	/* applied when the instant event c + 6 is planned */
	CHECK(cbs.updated == 1);
	/* instant event c + 6: old anchor a + 15 ms, window + 2.5 ms, 1.25 ms
	 * wide; widening 300 ppm over 15 + 2.5 + 1.25 ms = 5.625 -> 6 + 16 */
	ws = a + T(15000) + T(2500);
	fire_alarm();
	CHECK(cbs.p.interval == 6 && cbs.p.latency == 2 && cbs.p.timeout == 50);
	CHECK(rad.open == ws - T(22 + LL_CONN_WIN_MARGIN_US));
	CHECK(rad.fst == 1250 + 2 * (22 + 200) + 40);
	CHECK(ll_conn_event_counter(0) == c + 6);
	/* window missed: next event one new interval later, same window,
	 * widening over 7.5 + 2.5 + 15 + 1.25 = 26.25 ms -> 7.875 -> 8 + 16 */
	done(0);
	fire_alarm();
	CHECK(rad.open == ws + T(7500) - T(24 + 200));
	CHECK(rad.fst == 1250 + 2 * (24 + 200) + 40);
	/* found it 300 us into the window: synced, new interval 7.5 ms */
	a_new = ws + T(7500) + T(300);
	rx(a_new, 0x01, 0);
	done(1);
	/* slice 7: the new latency 2 is not used yet, the link is up for less
	 * than LL_CONN_LATENCY_HOLDOFF_MS (test_latency_from_update: honoured
	 * after it): event +1 is listened to, widening over 7.5 ms: 2.25 -> 3
	 * + 16 */
	fire_alarm();
	CHECK(rad.open == a_new + T(7500) - T(widen(300, 7500) + 60));
	CHECK(rad.fst == 2 * (19 + 60) + 40);
	done(0);
	/* new supervision timeout (500 ms) in force: 66 * 7.5 ms = 495 ms ok;
	 * after the miss every event is listened to */
	for (int k = 2; k <= 66; k++) {
		ev_miss();
	}
	CHECK(ll_conn_active(0));
	ev_miss();
	CHECK(!ll_conn_active(0) && cbs.reason == LL_ST_CONN_TIMEOUT);
}

/* 5.1.1: the supervision timer restarts at the instant (old anchor of the
 * instant event) */
static void test_update_restarts_supervision(void)
{
	struct ll_connect_ind ci = mk_ci(12, 10, 1, 1, 0);   /* 100 ms */
	struct ll_conn_params np = {.interval = 12, .latency = 0, .timeout = 10};
	uint32_t a = 4000000 + T(1250 + 300);

	reset_all(false);
	CHECK(ll_conn_start(&ci, 4000000) == 0);
	ev_rx(a);
	CHECK(ll_conn_update_at(0, 1, 1, 0, &np) == 0);
	/* event k ends at about a + k * 15 ms + 1.5 ms; timer from a + 15 ms */
	for (int k = 1; k <= 7; k++) {
		ev_miss();
		CHECK(ll_conn_active(0));
	}
	ev_miss();
	CHECK(!ll_conn_active(0) && cbs.reason == LL_ST_CONN_TIMEOUT);
}

/* long intervals: the widening clamp (interval / 2 - T_IFS) must not wrap */
static void test_long_interval(void)
{
	struct ll_connect_ind ci = mk_ci(210, 3200, 1, 1, 0);   /* 262.5 ms */
	struct ll_conn_stats st;
	uint32_t a = 1000000 + T(1250 + 100);

	reset_all(false);
	CHECK(ll_conn_start(&ci, 1000000) == 0);
	ev_rx(a);
	for (int k = 0; k < 3; k++) {
		ev_miss();
	}
	/* 4 intervals = 1.05 s at 300 ppm: 315 + 16 = 331 us */
	fire_alarm();
	CHECK(rad.open == a + T(4 * 262500) - T(331 + 60));
	CHECK(rad.fst == 2 * (331 + 60) + 40);
	ll_conn_get_stats(0, &st);
	CHECK(st.widen_max_us >= 331);
	done(0);
	ll_conn_end(0, 0x13);
}

/* Event length cap (guard): interval minus the widening growth over one
 * interval, the alarm lead and LL_CONN_EVENT_SAFETY_US, so a long MD burst
 * may run but never overruns the next event's alarm; never below the first
 * RX window plus LL_CONN_GUARD_MIN_TAIL_US. */
static void test_event_cap(void)
{
	struct ll_connect_ind ci = mk_ci(6, 3200, 1, 1, 0);   /* 7.5 ms, 300 ppm */
	struct ll_connect_ind ci2 = mk_ci(12, 400, 1, 1, 0);  /* 15 ms */
	struct ll_connect_ind ci3 = mk_ci(6, 3200, 0, 1, 0);  /* 7.5 ms, 500 ppm */
	uint32_t a = 1000000 + T(1250 + 100);

	/* 300 ppm * 7500 us = 2.25 -> 3 us growth */
	reset_all(false);
	CHECK(ll_conn_start(&ci, 1000000) == 0);
	ev_rx(a);
	fire_alarm();
	CHECK(rad.max_ev == 7500 - 3 - LL_CONN_ARM_LEAD_US - LL_CONN_EVENT_SAFETY_US);
	CHECK(rad.max_ev > rad.fst + LL_CONN_GUARD_MIN_TAIL_US);
	/* the guard (open + max_ev) ends before the next event's alarm */
	CHECK(rad.open + T(rad.max_ev) < rad.open + T(7500) - T(LL_CONN_ARM_LEAD_US));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* 15 ms: 300 ppm * 15000 us = 4.5 -> 5 us growth */
	reset_all(false);
	CHECK(ll_conn_start(&ci2, 1000000) == 0);
	ev_rx(a);
	fire_alarm();
	CHECK(rad.max_ev == 15000 - 5 - LL_CONN_ARM_LEAD_US - LL_CONN_EVENT_SAFETY_US);
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* clamped widening: the first RX window alone exceeds the interval,
	 * the guard still leaves it plus the minimum tail */
	reset_all(false);
	CHECK(ll_conn_start(&ci3, 3000000) == 0);
	ev_rx(3000000 + T(1250 + 100));
	for (int k = 0; k < 1000; k++) {
		ev_miss();
	}
	fire_alarm();
	CHECK(rad.fst == 2 * (3600 + 60) + 40);
	CHECK(rad.max_ev == rad.fst + LL_CONN_GUARD_MIN_TAIL_US);
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* LL_CONNECTION_UPDATE_IND parameters are validated; invalid ones are
 * refused with LL_ST_INVALID_LL_PARAM and change nothing */
static void test_update_validation(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	uint32_t a = 4000000 + T(1250 + 300);
	static const struct { uint16_t iv, lat, to; uint8_t ws; uint16_t wo; } bad[] = {
		{0, 0, 400, 1, 0}, {5, 0, 400, 1, 0}, {3201, 0, 3200, 1, 0},
		{12, 500, 3200, 1, 0}, {12, 0, 9, 1, 0}, {12, 0, 3201, 1, 0},
		{12, 0, 3, 1, 0},       /* 30 ms <= 2 * 15 ms */
		{12, 4, 15, 1, 0},      /* 150 ms <= 5 * 15 ms * 2 */
		{12, 0, 400, 0, 0}, {12, 0, 400, 9, 0}, {6, 0, 400, 6, 0},
		{12, 0, 400, 1, 13},
	};
	struct ll_conn_params p;
	uint16_t c;

	reset_all(false);
	CHECK(ll_conn_start(&ci, 4000000) == 0);
	ev_rx(a);
	c = ll_conn_event_counter(0);
	for (unsigned int i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		p.interval = bad[i].iv;
		p.latency = bad[i].lat;
		p.timeout = bad[i].to;
		CHECK(ll_conn_update_at(0, c + 2, bad[i].ws, bad[i].wo, &p) == LL_ST_INVALID_LL_PARAM);
	}
	CHECK(ll_conn_active(0) && cbs.disconnected == 0);
	/* nothing was scheduled: the events stay on the old 15 ms grid */
	for (int k = 1; k <= 4; k++) {
		a += T(15000);
		ev_rx(a);
	}
	fire_alarm();
	CHECK(rad.open == a + T(15000) - T(widen(300, 15000) + 60));
	CHECK(cbs.updated == 0);
	done(0);
	/* boundary values are accepted */
	p.interval = 6;
	p.latency = 0;
	p.timeout = 10;
	CHECK(ll_conn_update_at(0, ll_conn_event_counter(0) + 2, 5, 6, &p) == 0);
	ll_conn_end(0, 0x13);
}

/* an update that keeps the parameters is applied but not reported */
static void test_conn_update_same_params(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	struct ll_conn_params same = {.interval = 12, .latency = 0, .timeout = 400};
	uint32_t a = 4000000 + T(1250 + 300);

	reset_all(false);
	CHECK(ll_conn_start(&ci, 4000000) == 0);
	ev_rx(a);
	CHECK(ll_conn_update_at(0, ll_conn_event_counter(0) + 1, 1, 0, &same) == 0);
	a += T(15000);
	ev_rx(a);
	fire_alarm();
	/* window at the old anchor + 0, 1.25 ms wide */
	CHECK(rad.open == a + T(15000) - T(widen(300, 16250) + LL_CONN_WIN_MARGIN_US));
	CHECK(cbs.updated == 0);
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* 5.1.2 channel map at the instant; CSA#1 keeps lastUnmappedChannel */
static void test_chmap(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	struct ll_csa1 ref;
	uint32_t a = 5000000 + T(1250 + 100);
	uint16_t inst;

	reset_all(false);
	ll_csa1_init(&ref, 7, all37);
	CHECK(ll_conn_start(&ci, 5000000) == 0);
	fire_alarm();
	CHECK(rad.ch == ll_csa1_next(&ref));
	rx(a, 0x01, 0);
	done(1);
	inst = ll_conn_event_counter(0) + 6;
	CHECK(ll_conn_chmap_at(0, inst, no0to9) == 0);
	for (int k = 1; k < 30; k++) {
		if (ll_conn_event_counter(0) == inst) {
			ll_csa1_set_map(&ref, no0to9);
		}
		a += T(15000);
		fire_alarm();
		CHECK(rad.ch == ll_csa1_next(&ref));
		if (k >= 7) {
			CHECK(rad.ch >= 10);
		}
		rx(a, 0x01, 0);
		done(1);
	}
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* instant == counter of the planned (not yet issued) event: re-planned
 * with the new map, the CSA#1 sequence is not advanced twice */
static void test_instant_replan(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	struct ll_csa1 ref;
	uint32_t a = 5000000 + T(1250 + 100);
	uint8_t ch;

	reset_all(false);
	ll_csa1_init(&ref, 7, all37);
	CHECK(ll_conn_start(&ci, 5000000) == 0);
	for (int k = 0; k < 4; k++) {
		fire_alarm();
		CHECK(rad.ch == ll_csa1_next(&ref));
		rx(a, 0x01, 0);
		done(1);
		a += T(15000);
	}
	/* event 4 is planned on unmapped channel 5 * 7 mod 37 = 35; the new
	 * map drops 35, so the re-planned event must use the remapped 36 */
	CHECK(ll_conn_chmap_at(0, ll_conn_event_counter(0), no35) == 0);
	ll_csa1_set_map(&ref, no35);
	fire_alarm();
	ch = ll_csa1_next(&ref);
	CHECK(ch == 36);
	CHECK(rad.ch == ch);
	CHECK(rad.open == a - T(widen(300, 15000) + 60));
	rx(a, 0x01, 0);
	done(1);
	a += T(15000);
	fire_alarm();
	CHECK(rad.ch == ll_csa1_next(&ref));
	done(0);

	/* instant == the event on air: too late -> 0x28 after the event */
	CHECK(ll_conn_chmap_at(0, ll_conn_event_counter(0), all37) == 0);  /* planned */
	fire_alarm();
	CHECK(ll_conn_chmap_at(0, ll_conn_event_counter(0), all37) == LL_ST_INSTANT_PASSED);
	CHECK(cbs.disconnected == 0);
	done(0);
	CHECK(cbs.disconnected == 1 && cbs.reason == LL_ST_INSTANT_PASSED);
}

/* instant in the past: (instant - counter) mod 65536 > 32767 -> 0x28 */
static void test_instant_passed(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	struct ll_conn_params np = {.interval = 6, .latency = 0, .timeout = 50};
	uint32_t a = 6000000 + T(1250 + 100);
	uint16_t c;

	reset_all(false);
	CHECK(ll_conn_start(&ci, 6000000) == 0);
	for (int k = 0; k < 10; k++) {
		ev_rx(a);
		a += T(15000);
	}
	c = ll_conn_event_counter(0);
	CHECK(c == 10);
	CHECK(ll_conn_update_at(0, (uint16_t)(c + 32767), 1, 0, &np) == 0);
	CHECK(ll_conn_active(0));
	CHECK(ll_conn_chmap_at(0, (uint16_t)(c + 32768), no0to9) == LL_ST_INSTANT_PASSED);
	CHECK(!ll_conn_active(0));
	CHECK(cbs.disconnected == 1 && cbs.reason == LL_ST_INSTANT_PASSED);
	CHECK(sch.cb == NULL);

	reset_all(false);
	CHECK(ll_conn_update_at(0, 1, 1, 0, &np) == LL_ST_DISALLOWED);
	CHECK(ll_conn_start(&ci, 6000000) == 0);
	a = 6000000 + T(1250 + 100);
	for (int k = 0; k < 10; k++) {
		ev_rx(a);
		a += T(15000);
	}
	CHECK(ll_conn_update_at(0, 5, 1, 0, &np) == LL_ST_INSTANT_PASSED);
	CHECK(cbs.disconnected == 1 && cbs.reason == LL_ST_INSTANT_PASSED);
}

/* 16-bit event counter wraps; instants across the wrap */
static void test_counter_wrap(void)
{
	struct ll_connect_ind ci = mk_ci(6, 3200, 7, 1, 0);
	struct ll_csa1 ref;
	uint32_t a = 7000000 + T(1250 + 100);

	reset_all(false);
	ll_csa1_init(&ref, 7, all37);
	CHECK(ll_conn_start(&ci, 7000000) == 0);
	for (uint32_t k = 0; k < 65534; k++) {
		fire_alarm();
		(void)ll_csa1_next(&ref);
		rx(a, 0x01, 0);
		done(1);
		a += T(7500);
	}
	CHECK(ll_conn_event_counter(0) == 65534);
	CHECK(ll_conn_chmap_at(0, 2, no0to9) == 0);
	for (int k = 0; k < 6; k++) {
		if (ll_conn_event_counter(0) == 2) {
			ll_csa1_set_map(&ref, no0to9);
		}
		fire_alarm();
		CHECK(rad.ch == ll_csa1_next(&ref));
		rx(a, 0x01, 0);
		done(1);
		a += T(7500);
	}
	CHECK(ll_conn_event_counter(0) == 4);
	CHECK(ll_conn_active(0));
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* local terminate: LL_TERMINATE_IND via txq, end with 0x16 once acked */
static void test_local_terminate_ack(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	uint32_t a = 8000000 + T(1250 + 100);
	int slot = -1;

	reset_all(false);
	CHECK(ll_conn_start(&ci, 8000000) == 0);
	ev_rx(a);
	a += T(15000);
	ll_conn_terminate(0, 0x13);
	CHECK(locks == 0);
	CHECK(ll_conn_active(0));
	fire_alarm();
	/* the ring holds a placeholder (base sent last) and the PDU */
	for (uint8_t i = rad.rptr; i != rad.wptr; i++) {
		if (rad.fifo_len[i & 3] == 2 && rad.fifo[i & 3][0] == 0x02) {
			slot = i & 3;
		}
	}
	CHECK(slot >= 0);
	if (slot >= 0) {
		CHECK(rad.fifo[slot][1] == 0x13 && rad.fifo_hdr[slot] == LL_LLID_CTRL);
	}
	/* not acked in this event */
	rx(a, 0x01 | HDR_NESN, 0);
	done(1);
	CHECK(ll_conn_active(0));
	a += T(15000);
	fire_alarm();
	rx(a, 0x01, 0);
	rad.rptr = rad.wptr;   /* hardware popped everything: acked */
	done(1);
	CHECK(cbs.done_ctrl == 1 && cbs.done_op == 0x02);
	CHECK(!ll_conn_active(0));
	CHECK(cbs.disconnected == 1 && cbs.reason == LL_ST_LOCAL_TERM);
	CHECK(sch.cb == NULL);
}

/* local terminate without ack: end after connSupervisionTimeout; the PDU
 * goes through ops.ctrl_tx when given */
static void test_local_terminate_timeout(void)
{
	struct ll_connect_ind ci = mk_ci(12, 10, 1, 1, 0);   /* 100 ms */
	uint32_t a = 9000000 + T(1250 + 100);
	int k;

	reset_all(true);
	CHECK(ll_conn_start(&ci, 9000000) == 0);
	ev_rx(a);
	ll_conn_terminate(0, 0x15);
	CHECK(cbs.ctrl_tx_calls == 1 && cbs.ctrl_tx_len == 2);
	CHECK(cbs.ctrl_tx_pdu[0] == 0x02 && cbs.ctrl_tx_pdu[1] == 0x15);
	CHECK(ll_txq_backlog(0) == 0);   /* ll_conn did not push itself */
	/* the central keeps talking but never acks: end once 100 ms passed */
	for (k = 0; k < 20 && ll_conn_active(0); k++) {
		a += T(15000);
		ev_rx(a);
	}
	CHECK(!ll_conn_active(0));
	/* terminate called ~1.6 ms after a0; the 7th event ends at a0 + 105 ms */
	CHECK(k == 7);
	CHECK(cbs.disconnected == 1 && cbs.reason == LL_ST_LOCAL_TERM);
}

/* ll_conn_end: immediate between events, deferred during an event */
static void test_end(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	uint32_t a = 10000000 + T(1250 + 100);
	int ev;

	reset_all(false);
	CHECK(ll_conn_start(&ci, 10000000) == 0);
	ev_rx(a);
	CHECK(sch.cb != NULL);
	ll_conn_end(0, LL_ST_MIC_FAILURE);
	CHECK(!ll_conn_active(0) && sch.cb == NULL);
	CHECK(cbs.disconnected == 1 && cbs.reason == LL_ST_MIC_FAILURE);
	ll_conn_end(0, LL_ST_MIC_FAILURE);
	CHECK(cbs.disconnected == 1);

	reset_all(false);
	CHECK(ll_conn_start(&ci, 10000000) == 0);
	fire_alarm();
	ll_conn_end(0, 0x13);
	CHECK(ll_conn_active(0) && cbs.disconnected == 0);
	ev = rad.events;
	rx(a, 0x01, 0);
	done(1);
	CHECK(!ll_conn_active(0) && cbs.disconnected == 1 && cbs.reason == 0x13);
	CHECK(sch.cb == NULL && rad.events == ev);
	/* a new connection can start afterwards */
	CHECK(ll_conn_start(&ci, 20000000) == 0);
	CHECK(rad.setups == 2 && cbs.connected == 2);
	ll_conn_end(0, 0x13);
}

/* the alarm came too late to issue the BRX: event skipped, counted missed */
static void test_late_alarm(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	struct ll_conn_stats s0, s1;
	uint32_t a = 11000000 + T(1250 + 100);
	uint32_t open1;
	int ev;

	reset_all(false);
	ll_conn_get_stats(0, &s0);
	CHECK(ll_conn_start(&ci, 11000000) == 0);
	ev_rx(a);
	open1 = a + T(15000) - T(widen(300, 15000) + 60);
	ev = rad.events;
	CHECK(sch.tick == open1 - T(LL_CONN_ARM_LEAD_US));
	sch.tick = open1 - T(LL_CONN_MIN_PREP_US) + 1;   /* alarm delivered late */
	fire_alarm();
	CHECK(rad.events == ev);
	CHECK(ll_conn_event_counter(0) == 2);
	CHECK(sch.cb != NULL);
	fire_alarm();
	CHECK(rad.events == ev + 1);
	CHECK(rad.ch == 21);
	CHECK(rad.open == a + T(30000) - T(widen(300, 30000) + 60));
	done(0);
	ll_conn_get_stats(0, &s1);
	CHECK(s1.late - s0.late == 1);
	CHECK(s1.missed - s0.missed == 2);
	CHECK(s1.events - s0.events == 2);
	ll_conn_end(0, 0x13);
}

static void test_start_validation(void)
{
	struct ll_connect_ind ci;

	reset_all(false);
	ci = mk_ci(0, 400, 1, 1, 0);
	CHECK(ll_conn_start(&ci, 1) == -EINVAL);
	ci = mk_ci(3201, 3200, 1, 1, 0);
	CHECK(ll_conn_start(&ci, 1) == -EINVAL);
	ci = mk_ci(12, 400, 1, 0, 0);       /* WinSize 0 */
	CHECK(ll_conn_start(&ci, 1) == -EINVAL);
	ci = mk_ci(12, 3, 1, 1, 0);         /* timeout 30 ms <= 2 * 15 ms */
	CHECK(ll_conn_start(&ci, 1) == -EINVAL);
	ci = mk_ci(12, 400, 1, 1, 0);
	memset(ci.chm, 0, 5);
	ci.chm[0] = 0x01;                   /* one channel */
	CHECK(ll_conn_start(&ci, 1) == -EINVAL);
	ci = mk_ci(12, 400, 1, 1, 0);
	ci.hop = 4;
	CHECK(ll_conn_start(&ci, 1) == -EINVAL);
	CHECK(!ll_conn_active(0) && cbs.connected == 0 && rad.setups == 0);
}

/* ---------------- peripheral latency (slice 5) ---------------- */

/* open tick of the event k intervals (15 ms, 300 ppm) after the anchor a */
static uint32_t open_at(uint32_t a, uint32_t k)
{
	return a + T(15000 * k) - T(widen(300, 15000 * k) + LL_CONN_RX_MARGIN_US);
}

/* First event whose anchor lies LL_CONN_LATENCY_HOLDOFF_MS or more after
 * the connection start, for a 15 ms link whose event 0 anchor is
 * first_us after the CONNECT_IND end: before it no event is skipped. */
static uint16_t holdoff_first(uint32_t first_us)
{
	uint32_t h = LL_CONN_LATENCY_HOLDOFF_MS * 1000u;

	return (uint16_t)((h - first_us + 14999u) / 15000u);
}

/* Slice 7: no event is skipped during the first LL_CONN_LATENCY_HOLDOFF_MS
 * of a link. The latency tests run their scenario after it: their "event k"
 * is the link's event EV(k), with event EV(0) = e0 the last event of the
 * holdoff (received, so the scenario starts synced as before). */
static uint16_t e0;
#define EV(k) ((uint16_t)(e0 + (k)))

/* The only active link received its event 0 at *a (15 ms, 300 ppm): listen
 * to and receive every further event of the holdoff, each one interval
 * after the last (ref follows CSA#1 when not NULL). On return *a is the
 * anchor of event e0, the last one before the first skippable event. */
static void hold_events(uint8_t link, uint32_t *a, struct ll_csa1 *ref, uint32_t first_us)
{
	uint16_t h = holdoff_first(first_us);

	for (uint16_t e = 1; e < h; e++) {
		fire_alarm();
		CHECK(ll_conn_event_counter(link) == e);
		CHECK(rad.open == open_at(*a, 1));
		if (ref) {
			CHECK(rad.ch == ll_csa1_next(ref));
		}
		*a += T(15000);
		rx(*a, 0x01, 0);
		done(1);
	}
	e0 = (uint16_t)(h - 1);
}

/* Start a 15 ms connection with the given latency and timeout (SCA 1, 300
 * ppm), receive event 0 (window event) and every event of the latency
 * holdoff; return the anchor of event e0 (EV(0)). ref follows the CSA#1
 * sequence (events up to e0 consumed). A skip from EV(1) is planned. */
static uint32_t start_lat(uint16_t latency, uint16_t timeout, struct ll_csa1 *ref, bool hook)
{
	struct ll_connect_ind ci = mk_ci(12, timeout, 1, 1, 0);
	const uint32_t t0 = 1000000;
	uint32_t a0 = t0 + T(1250 + 300);

	ci.latency = latency;
	reset_all(hook);
	ll_csa1_init(ref, 7, all37);
	CHECK(ll_conn_start(&ci, t0) == 0);
	fire_alarm();
	CHECK(rad.ch == ll_csa1_next(ref));
	rx(a0, 0x01, 0);
	done(1);
	hold_events(0, &a0, ref, 1550);
	return a0;
}

/* advance ref by n events, return the channel of the last */
static uint8_t ref_skip(struct ll_csa1 *ref, uint32_t n)
{
	uint8_t ch = 0;

	for (uint32_t i = 0; i < n; i++) {
		ch = ll_csa1_next(ref);
	}
	return ch;
}

/* Idle and synced (after the holdoff): EV(1)..EV(4) are skipped, EV(5) is
 * listened to; the counter and CSA#1 advance over the skipped events,
 * widening grows over the real time since the anchor. Stats count it. */
static void test_latency_skip(void)
{
	struct ll_conn_stats s0, s1;
	struct ll_csa1 ref;
	uint32_t a0;

	ll_conn_get_stats(0, &s0);
	a0 = start_lat(4, 400, &ref, false);
	/* instants are judged against the first skipped event */
	CHECK(ll_conn_event_counter(0) == EV(1));
	fire_alarm();
	CHECK(rad.ch == ref_skip(&ref, 5));
	CHECK(rad.open == open_at(a0, 5));
	CHECK(rad.fst == 2 * (widen(300, 75000) + 60) + 40);
	check_alarm_lead();
	CHECK(ll_conn_event_counter(0) == EV(5));
	rx(a0 + T(75000), 0x01, 0);
	done(1);
	CHECK(ll_conn_event_counter(0) == EV(6));
	fire_alarm();
	CHECK(rad.ch == ref_skip(&ref, 5));
	CHECK(rad.open == open_at(a0 + T(75000), 5));
	CHECK(ll_conn_event_counter(0) == EV(10));
	done(0);
	ll_conn_get_stats(0, &s1);
	CHECK(s1.skipped - s0.skipped == 8);
	CHECK(s1.listened - s0.listened == 3u + e0);   /* + the holdoff */
	CHECK(s1.events - s0.events == 3u + e0);
	/* events EV(0), EV(5), EV(10), the armed (never fired) EV(11) after the
	 * miss, and the holdoff events before EV(0) */
	CHECK(s1.planned - s0.planned == 4u + e0);
	CHECK(s1.kicks == s0.kicks);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* Data queued or unacked: no skip. The next event is listened to; once
 * everything is acked skipping resumes. */
static void test_latency_refused_txq(void)
{
	const uint8_t pdu[3] = {0x01, 0x02, 0x03};
	struct ll_csa1 ref;
	uint32_t a0, a1;

	a0 = start_lat(4, 400, &ref, false);
	/* the skip to EV(5) is planned; the host queues data during it */
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(5));
	CHECK(rad.ch == ref_skip(&ref, 5));
	a1 = a0 + T(75000);
	rx(a1, 0x01, 0);
	CHECK(ll_txq_push(0, LL_TXQ_ACL, LL_LLID_START, pdu, sizeof(pdu), 0) == 0);
	done(1);
	fire_alarm();
	CHECK(rad.ch == ref_skip(&ref, 1));
	CHECK(rad.open == open_at(a1, 1));
	a1 += T(15000);
	/* EV(6): not acked yet (central's NESN unchanged): still listening */
	rx(a1, 0x01, 0);
	done(1);
	CHECK(ll_txq_backlog(0) > 0);
	fire_alarm();
	CHECK(rad.ch == ref_skip(&ref, 1));
	CHECK(rad.open == open_at(a1, 1));
	/* EV(7): acked */
	rx(a1 + T(15000), 0x01 | HDR_NESN, 0);
	rad.rptr = rad.wptr;
	done(1);
	CHECK(ll_txq_backlog(0) == 0);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(12));
	CHECK(rad.ch == ref_skip(&ref, 5));
	CHECK(rad.open == open_at(a1 + T(15000), 5));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* LLCP busy (ops.busy) and a local termination in progress: no skip */
static void test_latency_refused_busy_term(void)
{
	struct ll_csa1 ref;
	uint32_t a0, a;

	a0 = start_lat(4, 400, &ref, true);
	/* event 1..4 skipped already planned; busy from now on */
	busy_flag = true;
	fire_alarm();
	CHECK(rad.open == open_at(a0, 5));
	rx(a0 + T(75000), 0x01, 0);
	done(1);
	a = a0 + T(75000);
	for (int k = 1; k <= 3; k++) {
		fire_alarm();
		CHECK(rad.open == open_at(a, 1));
		a += T(15000);
		rx(a, 0x01, 0);
		done(1);
	}
	busy_flag = false;
	fire_alarm();
	CHECK(rad.open == open_at(a, 1));
	a += T(15000);
	rx(a, 0x01, 0);
	done(1);
	/* not busy any more: skip again */
	fire_alarm();
	CHECK(rad.open == open_at(a, 5));
	a += T(75000);
	rx(a, 0x01, 0);
	/* local termination (the hook queues nothing): no skip while it runs */
	ll_conn_terminate(0, 0x13);
	CHECK(cbs.ctrl_tx_calls == 1 && ll_txq_backlog(0) == 0);
	done(1);
	fire_alarm();
	CHECK(rad.open == open_at(a, 1));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* Never skip right after an event without a re-anchor: a missed event, a
 * late (skipped) alarm, an event whose first packet had a bad CRC. */
static void test_latency_no_skip_unsynced(void)
{
	struct ll_csa1 ref;
	uint32_t a0, a;

	a0 = start_lat(4, 400, &ref, false);
	fire_alarm();
	CHECK(rad.open == open_at(a0, 5));
	CHECK(rad.ch == ref_skip(&ref, 5));
	done(0);                       /* event 5 missed */
	fire_alarm();
	CHECK(rad.open == open_at(a0, 6));
	CHECK(rad.ch == ref_skip(&ref, 1));
	a = a0 + T(90000);
	rx(a, 0x01, 0);                /* event 6 synced again */
	done(1);
	fire_alarm();
	CHECK(rad.open == open_at(a, 5));   /* event 11 */
	CHECK(rad.ch == ref_skip(&ref, 5));
	/* event 11: bad first packet, a valid chained one: no re-anchor */
	rx_bad(a + T(75000));
	rx(a + T(75400), 0x01, 0);
	done(1);
	fire_alarm();
	CHECK(rad.open == open_at(a, 6));   /* event 12 */
	CHECK(rad.ch == ref_skip(&ref, 1));
	a += T(90000);
	rx(a, 0x01, 0);
	done(1);
	/* event 17 planned; its alarm is delivered too late: event 18 next,
	 * not skipped */
	CHECK(sch.tick == open_at(a, 5) - T(LL_CONN_ARM_LEAD_US));
	sch.tick = open_at(a, 5) - T(LL_CONN_MIN_PREP_US) + 1;
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(18));
	fire_alarm();
	CHECK(rad.open == open_at(a, 6));
	(void)ref_skip(&ref, 5);
	CHECK(rad.ch == ref_skip(&ref, 1));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* A pending instant in or right after the skip window: no skip until the
 * instant event was listened to (it is applied there), then skipping
 * resumes with the new map. */
static void test_latency_instant_pending(void)
{
	struct ll_csa1 ref;
	uint32_t a;

	a = start_lat(4, 400, &ref, false);
	/* LL_CHANNEL_MAP_IND received in event EV(5), instant EV(8) */
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(5));
	CHECK(rad.ch == ref_skip(&ref, 5));
	a += T(75000);
	rx(a, 0x01, 0);
	CHECK(ll_conn_chmap_at(0, EV(8), no0to9) == 0);
	done(1);
	for (uint16_t e = 6; e <= 8; e++) {
		if (e == 8) {
			ll_csa1_set_map(&ref, no0to9);
		}
		fire_alarm();
		CHECK(ll_conn_event_counter(0) == EV(e));
		CHECK(rad.open == open_at(a, 1));
		CHECK(rad.ch == ll_csa1_next(&ref));
		a += T(15000);
		rx(a, 0x01, 0);
		done(1);
	}
	/* instant passed: skip again (EV(9..12)), listen at EV(13) */
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(13));
	CHECK(rad.open == open_at(a, 5));
	CHECK(rad.ch == ref_skip(&ref, 5));
	CHECK(rad.ch >= 10);
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* an instant exactly at the end of the window (EV(10)) also blocks */
	a = start_lat(4, 400, &ref, false);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(5));
	a += T(75000);
	rx(a, 0x01, 0);
	CHECK(ll_conn_chmap_at(0, EV(10), no0to9) == 0);
	done(1);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(6));
	CHECK(rad.open == open_at(a, 1));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* An instant that arrives while a skip is planned and falls into the
 * skipped part: re-planned to listen at the first reachable event of the
 * window, or at the instant if that comes first (slice 5 final fix; the
 * earlier-reachable case is test_latency_instant_replan_reachable); once
 * the instant event's anchor has passed, the instant has passed (0x28;
 * slice 7: by the anchor, not the alarm, see
 * test_instant_alarm_passed_anchor_not). */
static void test_latency_instant_in_window(void)
{
	struct ll_csa1 ref;
	struct ll_conn_stats s0, s1;
	uint32_t a0;

	ll_conn_get_stats(0, &s0);
	a0 = start_lat(4, 400, &ref, false);
	/* skip to event 5 planned; 31 ms later (events 1 and 2 gone) the
	 * thread handles a map update for instant 3, now the first reachable
	 * event (an earlier reachable one: test_latency_instant_replan_reachable) */
	now = a0 + T(31000);
	CHECK(ll_conn_chmap_at(0, EV(3), no0to9) == 0);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(3));
	CHECK(rad.open == open_at(a0, 3));
	(void)ref_skip(&ref, 2);
	ll_csa1_set_map(&ref, no0to9);
	CHECK(rad.ch == ll_csa1_next(&ref));
	done(0);
	ll_conn_get_stats(0, &s1);
	CHECK(s1.skipped - s0.skipped == 2);
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* the instant event lies in the past already */
	a0 = start_lat(4, 400, &ref, false);
	now = a0 + T(31000);   /* events 1 and 2 have passed */
	CHECK(ll_conn_chmap_at(0, EV(2), no0to9) == LL_ST_INSTANT_PASSED);
	CHECK(!ll_conn_active(0) && cbs.disconnected == 1);
	CHECK(cbs.reason == LL_ST_INSTANT_PASSED);

	/* an instant before the first skipped event is passed as before */
	a0 = start_lat(4, 400, &ref, false);
	CHECK(ll_conn_chmap_at(0, EV(0), no0to9) == LL_ST_INSTANT_PASSED);
	CHECK(cbs.disconnected == 1);

	/* an instant after the planned event: nothing re-planned */
	a0 = start_lat(4, 400, &ref, false);
	CHECK(ll_conn_chmap_at(0, EV(6), no0to9) == 0);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(5));
	CHECK(rad.open == open_at(a0, 5));
	rx(a0 + T(75000), 0x01, 0);
	done(1);
	/* instant 6 pending: listened */
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(6));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* Supervision: with any valid parameter set (timeout > (1 + latency) *
 * interval * 2), skipping latency events from a synced event leaves at
 * least latency + 1 listened events before the timeout; a miss after the
 * skip is followed by listening on every event. Latency 4, 15 ms, timeout
 * 160 ms (the smallest valid one). */
static void test_latency_supervision(void)
{
	struct ll_csa1 ref;
	uint32_t a0;
	int listened = 0;

	a0 = start_lat(4, 16, &ref, false);
	fire_alarm();
	CHECK(rad.open == open_at(a0, 5));
	/* listened anchor + 2 intervals stays within the timeout */
	CHECK(rad.open + T(2 * 15000) < a0 + T(160000));
	done(0);
	listened++;
	for (uint32_t k = 6; ll_conn_active(0) && k < 20; k++) {
		fire_alarm();
		CHECK(rad.open == open_at(a0, k));
		done(0);
		listened++;
	}
	CHECK(!ll_conn_active(0) && cbs.reason == LL_ST_CONN_TIMEOUT);
	/* events 5..11 issued: 10 ends at about 150 ms, 11 at 165 ms ends
	 * the link */
	CHECK(listened == 7);
}

/* ll_conn_kick: new TX data re-plans to the next regular event that can
 * still be prepared; no-op when that is already the planned one */
static void test_kick(void)
{
	struct ll_csa1 ref;
	struct ll_conn_stats s0, s1;
	uint32_t a0, a1, tick;
	int cancels;

	/* no connection: nothing happens */
	reset_all(false);
	ll_conn_kick(0);
	CHECK(locks == 0 && sch.cb == NULL);

	ll_conn_get_stats(0, &s0);
	a0 = start_lat(4, 400, &ref, false);
	ll_conn_kick(0);
	CHECK(locks == 0);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(1));
	CHECK(rad.open == open_at(a0, 1));
	CHECK(rad.ch == ref_skip(&ref, 1));
	/* kick while the event is on air: nothing */
	cancels = sch.cancels;
	ll_conn_kick(0);
	CHECK(sch.cancels == cancels && sch.cb == NULL);
	a1 = a0 + T(15000);
	rx(a1, 0x01, 0);
	done(1);
	ll_conn_get_stats(0, &s1);
	CHECK(s1.kicks - s0.kicks == 1);
	/* 4 skipped, given back by the kick, 4 skipped again (events 2..5) */
	CHECK(s1.skipped - s0.skipped == 4);
	/* re-planned: counted once; + the holdoff events */
	CHECK(s1.planned - s0.planned == 3u + e0);

	/* skip planned (base 2, listen 6); 31 ms later events 2 and 3 have
	 * passed: kick -> event 4 */
	now = a1 + T(31000);
	ll_conn_kick(0);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(4));
	CHECK(rad.open == open_at(a1, 3));
	CHECK(rad.ch == ref_skip(&ref, 3));
	ll_conn_get_stats(0, &s1);
	CHECK(s1.skipped - s0.skipped == 2);   /* events 2, 3 only */
	rx(a1 + T(45000), 0x01, 0);
	done(1);
	a1 += T(45000);

	/* kick exactly when event 5's alarm is due: event 6 */
	now = open_at(a1, 1) - T(LL_CONN_ARM_LEAD_US) + 1;
	ll_conn_kick(0);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(6));
	CHECK(rad.open == open_at(a1, 2));
	CHECK(rad.ch == ref_skip(&ref, 2));
	rx(a1 + T(30000), 0x01, 0);
	done(1);
	a1 += T(30000);

	/* the first kick re-plans to event 7, the second is a no-op */
	ll_conn_kick(0);
	tick = sch.tick;
	cancels = sch.cancels;
	ll_conn_kick(0);
	CHECK(sch.cancels == cancels && sch.tick == tick);
	fire_alarm();
	CHECK(rad.open == open_at(a1, 1));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* latency 0: always a no-op */
	a0 = start_lat(0, 400, &ref, false);
	cancels = sch.cancels;
	ll_conn_kick(0);
	CHECK(sch.cancels == cancels);
	fire_alarm();
	CHECK(rad.open == open_at(a0, 1));
	done(0);
	ll_conn_get_stats(0, &s1);
	CHECK(s1.kicks - s0.kicks == 4);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* The latency in force comes from the last applied connection update */
static void test_latency_from_update(void)
{
	struct ll_conn_params p3 = {.interval = 12, .latency = 3, .timeout = 400};
	struct ll_conn_params p0 = {.interval = 12, .latency = 0, .timeout = 400};
	struct ll_csa1 ref;
	uint32_t a0, a, ws;

	/* 0 -> 3 */
	a0 = start_lat(0, 400, &ref, false);
	CHECK(ll_conn_update_at(0, EV(3), 1, 0, &p3) == 0);
	a = a0;
	for (int e = 1; e <= 2; e++) {
		fire_alarm();
		CHECK(rad.open == open_at(a, 1));
		a += T(15000);
		rx(a, 0x01, 0);
		done(1);
	}
	/* instant 3: window at the old anchor, packet 100 us into it */
	fire_alarm();
	CHECK(cbs.updated == 1 && cbs.p.latency == 3);
	ws = a + T(15000);
	rx(ws + T(100), 0x01, 0);
	done(1);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(7));
	CHECK(rad.open == open_at(ws + T(100), 4));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* 4 -> 0 */
	a0 = start_lat(4, 400, &ref, false);
	CHECK(ll_conn_update_at(0, EV(8), 1, 0, &p0) == 0);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(5));
	a = a0 + T(75000);
	rx(a, 0x01, 0);
	done(1);
	/* 6, 7 listened (instant pending), 8 = instant window */
	for (int e = 6; e <= 7; e++) {
		fire_alarm();
		CHECK(ll_conn_event_counter(0) == EV(e));
		a += T(15000);
		rx(a, 0x01, 0);
		done(1);
	}
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(8) && cbs.p.latency == 0);
	ws = a + T(15000);
	rx(ws + T(100), 0x01, 0);
	done(1);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(9));
	CHECK(rad.open == open_at(ws + T(100), 1));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* Slice 5 review fix: once an instant re-plan applied the instant to the
 * planned event, nothing may re-plan to an earlier event (it would use the
 * new timing / the old map, and the instant would never be applied again). */
static void test_latency_instant_replan_then_kick(void)
{
	struct ll_conn_params p24 = {.interval = 24, .latency = 0, .timeout = 400};
	struct ll_conn_params p12 = {.interval = 12, .latency = 0, .timeout = 400};
	struct ll_csa1 ref;
	uint32_t a0;
	int cancels;
	uint8_t unm3[5], ch3;

	/* channel map instant 3 re-planned (events 1 and 2 gone, so event 3
	 * is the first reachable one), then a kick: still event 3, new map */
	a0 = start_lat(4, 400, &ref, false);
	now = a0 + T(31000);
	CHECK(ll_conn_chmap_at(0, EV(3), no0to9) == 0);
	cancels = sch.cancels;
	ll_conn_kick(0);
	CHECK(sch.cancels == cancels);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(3));
	CHECK(rad.open == open_at(a0, 3));
	(void)ref_skip(&ref, 2);
	ll_csa1_set_map(&ref, no0to9);
	CHECK(rad.ch == ll_csa1_next(&ref));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* connection update instant 3 re-planned, then a kick: still the
	 * transmit window of event 3 (old anchor + 0, 1.25 ms, widening over
	 * 46.25 ms: 13.875 -> 14 + 16 = 30 us) */
	a0 = start_lat(4, 400, &ref, false);
	now = a0 + T(31000);
	CHECK(ll_conn_update_at(0, EV(3), 1, 0, &p24) == 0);
	ll_conn_kick(0);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(3) && cbs.updated == 1);
	CHECK(rad.open == a0 + T(45000) - T(30 + LL_CONN_WIN_MARGIN_US));
	CHECK(rad.fst == 1250 + 2 * (30 + 200) + 40);
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* a second instant for the event whose map instant is applied: the
	 * applied map stays (re-plan restores the state after it) */
	a0 = start_lat(4, 400, &ref, false);
	/* a map without EV(3)'s unmapped CSA#1 channel 7 * (EV(3) + 1) mod 37 */
	ch3 = (uint8_t)((7u * (EV(3) + 1u)) % 37u);
	memcpy(unm3, all37, sizeof(unm3));
	unm3[ch3 / 8] &= (uint8_t)~(1u << (ch3 % 8));
	now = a0 + T(31000);
	CHECK(ll_conn_chmap_at(0, EV(3), unm3) == 0);
	CHECK(ll_conn_update_at(0, EV(3), 1, 0, &p12) == 0);
	ll_conn_kick(0);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(3));
	CHECK(rad.open == a0 + T(45000) - T(30 + LL_CONN_WIN_MARGIN_US));
	(void)ref_skip(&ref, 2);
	ll_csa1_set_map(&ref, unm3);
	CHECK(rad.ch == ll_csa1_next(&ref));
	CHECK(rad.ch != ch3);   /* EV(3)'s unmapped channel, dropped by unm3 */
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* Slice 5 final review fix: an instant inside the planned skip window
 * re-plans to the first reachable event (not to the instant), so a kick
 * right after it still reaches the next event; every event up to and
 * including the instant is then listened to, the instant is applied at
 * its own event and the channel sequence follows CSA#1. */
static void test_latency_instant_replan_reachable(void)
{
	struct ll_conn_params p24 = {.interval = 24, .latency = 0, .timeout = 400};
	struct ll_csa1 ref;
	struct ll_conn_stats s0, s1;
	uint32_t a0, a;
	int cancels;

	/* channel map instant 4 while the skip to event 5 is planned:
	 * listen at event 1 (old map), 2, 3, then 4 with the new map */
	ll_conn_get_stats(0, &s0);
	a0 = start_lat(4, 400, &ref, false);
	CHECK(ll_conn_chmap_at(0, EV(4), no0to9) == 0);
	cancels = sch.cancels;
	ll_conn_kick(0);   /* already the next event: no-op */
	CHECK(sch.cancels == cancels);
	a = a0;
	for (uint16_t e = 1; e <= 4; e++) {
		fire_alarm();
		CHECK(ll_conn_event_counter(0) == EV(e));
		CHECK(rad.open == open_at(a, 1));
		if (e == 4) {
			ll_csa1_set_map(&ref, no0to9);
		}
		CHECK(rad.ch == ll_csa1_next(&ref));
		a += T(15000);
		rx(a, 0x01, 0);
		done(1);
	}
	CHECK(rad.ch >= 10);
	/* instant passed: skipping resumes (events 5..8, listen at 9) */
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(9));
	CHECK(rad.open == open_at(a, 5));
	CHECK(rad.ch == ref_skip(&ref, 5));
	done(0);
	ll_conn_get_stats(0, &s1);
	CHECK(s1.skipped - s0.skipped == 4);   /* events 5..8 only */
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* event 1 already gone: the re-plan lands on event 2, and queued data
	 * (kick) does not wait for the instant either */
	a0 = start_lat(4, 400, &ref, false);
	now = a0 + T(16000);
	CHECK(ll_conn_chmap_at(0, EV(4), no0to9) == 0);
	ll_conn_kick(0);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(2));
	CHECK(rad.open == open_at(a0, 2));
	CHECK(rad.ch == ref_skip(&ref, 2));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* connection update instant 3: events 1, 2 with the old timing, the
	 * transmit window at event 3 (old anchor + 0, 1.25 ms) */
	a0 = start_lat(4, 400, &ref, false);
	CHECK(ll_conn_update_at(0, EV(3), 1, 0, &p24) == 0);
	a = a0;
	for (uint16_t e = 1; e <= 2; e++) {
		fire_alarm();
		CHECK(ll_conn_event_counter(0) == EV(e) && cbs.updated == 0);
		CHECK(rad.open == open_at(a, 1));
		CHECK(rad.ch == ll_csa1_next(&ref));
		a += T(15000);
		rx(a, 0x01, 0);
		done(1);
	}
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(3) && cbs.updated == 1);
	CHECK(rad.open == a + T(15000) - T(widen(300, 15000) + LL_CONN_WIN_MARGIN_US));
	CHECK(rad.ch == ll_csa1_next(&ref));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* ---------------- multilink (slice 6a) ---------------- */

static struct ll_connect_ind mk_ci_link(uint8_t k)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);

	ci.aa ^= (uint32_t)(k + 1) << 8;
	ci.crc_init = 0x555555u + k;
	return ci;
}

/* start returns the lowest free id and -EBUSY when all are taken; an ended
 * link stays taken until ll_conn_release(); out-of-range ids are refused */
static void test_link_ids(void)
{
	struct ll_connect_ind ci;
	const uint8_t n = LL_MAX_CONN;

	reset_all(true);
	auto_release = false;
	CHECK(ll_conn_count() == 0);
	for (uint8_t k = 0; k < n; k++) {
		ci = mk_ci_link(k);
		CHECK(ll_conn_start(&ci, 1000000 + T(100000) * k) == k);
		CHECK(cbs.link == k && cbs.connected == k + 1);
		CHECK(ll_conn_active(k) && ll_conn_count() == k + 1);
		CHECK(ll_conn_event_counter(k) == 0);
	}
	CHECK(rad.setups == n);
	ci = mk_ci_link(0);
	CHECK(ll_conn_start(&ci, 2000000) == -EBUSY);
	CHECK(cbs.connected == n);

	/* the last link ends: inactive, but its id is taken until released */
	ll_conn_end(n - 1, 0x13);
	CHECK(cbs.disconnected == 1 && cbs.link == n - 1 && cbs.reason == 0x13);
	CHECK(!ll_conn_active(n - 1) && ll_conn_count() == n);
	CHECK(ll_conn_start(&ci, 2000000) == -EBUSY);
	for (uint8_t k = 0; k + 1 < n; k++) {
		CHECK(ll_conn_active(k));
	}
	ll_conn_release(n - 1);
	CHECK(ll_conn_count() == n - 1);
	CHECK(ll_conn_start(&ci, 2000000) == n - 1);
	CHECK(ll_conn_active(n - 1) && ll_conn_count() == n);

	/* two gaps: the lowest is reused first */
	if (n >= 3) {
		ll_conn_end(n - 1, 0x13);
		ll_conn_end(1, 0x13);
		CHECK(cbs.disconnected == 3 && cbs.link == 1);
		ll_conn_release(n - 1);
		ll_conn_release(1);
		CHECK(ll_conn_count() == n - 2);
		CHECK(ll_conn_start(&ci, 3000000) == 1);
		CHECK(ll_conn_start(&ci, 3000000) == n - 1);
		CHECK(ll_conn_count() == n);
	}

	/* release of an active link, ids out of range: no effect */
	ll_conn_release(0);
	CHECK(ll_conn_active(0) && ll_conn_count() == n);
	CHECK(!ll_conn_active(n));
	ll_conn_end(n, 0x13);
	ll_conn_release(n);
	ll_conn_kick(n);
	{
		struct ll_conn_params p = {.interval = 12, .latency = 0, .timeout = 400};
		int calls = cbs.ctrl_tx_calls;

		CHECK(ll_conn_update_at(n, 5, 1, 0, &p) == LL_ST_DISALLOWED);
		CHECK(ll_conn_chmap_at(n, 5, no0to9) == LL_ST_DISALLOWED);
		ll_conn_terminate(n, 0x13);
		CHECK(cbs.ctrl_tx_calls == calls);
	}
	CHECK(ll_conn_event_counter(n) == 0);
	CHECK(ll_conn_count() == n);
	CHECK(cbs.disconnected == (n >= 3 ? 3 : 1));

	/* all end; an ended link is free once released */
	auto_release = true;
	for (uint8_t k = 0; k < n; k++) {
		ll_conn_end(k, 0x13);
		CHECK(!ll_conn_active(k));
	}
	CHECK(ll_conn_count() == 0 && sch.cb == NULL);
	CHECK(ll_conn_start(&ci, 4000000) == 0);
	ll_conn_end(0, 0x13);
}

/* Two links, 5 ms apart: the alarm follows the earliest planned event of
 * any link, every event selects its owner's AA / CRC init, and the radio
 * callbacks reach the owner of the event on air only. One link ends, the
 * other continues. Stats are per link. */
static void test_links_interleaved(void)
{
	struct ll_connect_ind ci0 = mk_ci_link(0), ci1 = mk_ci_link(1);
	const uint32_t t0 = 1000000, t1 = t0 + T(5000);
	uint32_t a0 = t0 + T(1350), a1 = t1 + T(1350);
	struct ll_conn_stats s0a, s1a, s0b, s1b;

	if (LL_MAX_CONN < 2) {
		return;
	}
	reset_all(false);
	ll_conn_get_stats(0, &s0a);
	ll_conn_get_stats(1, &s1a);
	CHECK(ll_conn_start(&ci0, t0) == 0);
	CHECK(ll_conn_start(&ci1, t1) == 1);

	/* nothing on air: a stray radio callback reaches nobody */
	ll_conn_radio_evt(LL_RADIO_CONN_DONE, NULL, 0, now);
	CHECK(ll_conn_event_counter(0) == 0 && ll_conn_event_counter(1) == 0);

	fire_alarm();   /* link 0, event 0 */
	CHECK(rad.aa == ci0.aa && rad.crc == ci0.crc_init);
	rx(a0, 0x01, 0);
	done(1);
	CHECK(ll_conn_event_counter(0) == 1 && ll_conn_event_counter(1) == 0);

	fire_alarm();   /* link 1, event 0 (before link 0's event 1) */
	CHECK(rad.aa == ci1.aa && rad.crc == ci1.crc_init);
	rx(a1, 0x01, 0);
	done(1);
	CHECK(ll_conn_event_counter(0) == 1 && ll_conn_event_counter(1) == 1);

	fire_alarm();   /* link 0, event 1 */
	CHECK(rad.aa == ci0.aa);
	CHECK(rad.open == open_at(a0, 1));
	done(0);        /* missed on link 0 only */
	CHECK(ll_conn_event_counter(0) == 2 && ll_conn_event_counter(1) == 1);

	fire_alarm();   /* link 1, event 1 */
	CHECK(rad.aa == ci1.aa);
	CHECK(rad.open == open_at(a1, 1));
	rx(a1 + T(15000), 0x01, 0);
	done(1);
	CHECK(ll_conn_event_counter(1) == 2);

	/* link 1 ends; link 0 keeps its alarm and follows on */
	ll_conn_end(1, 0x13);
	CHECK(cbs.disconnected == 1 && cbs.link == 1);
	CHECK(ll_conn_active(0) && !ll_conn_active(1) && sch.cb != NULL);
	fire_alarm();
	CHECK(rad.aa == ci0.aa);
	CHECK(rad.open == open_at(a0, 2));
	rx(a0 + T(30000), 0x01, 0);
	done(1);
	CHECK(ll_conn_event_counter(0) == 3);

	ll_conn_get_stats(0, &s0b);
	ll_conn_get_stats(1, &s1b);
	CHECK(s0b.events - s0a.events == 3 && s0b.missed - s0a.missed == 1);
	CHECK(s0b.rx_events - s0a.rx_events == 2);
	CHECK(s1b.events - s1a.events == 2 && s1b.missed == s1a.missed);
	CHECK(s1b.rx_events - s1a.rx_events == 2);
	CHECK(s0b.collisions == s0a.collisions && s1b.collisions == s1a.collisions);
	ll_conn_end(0, 0x13);
	CHECK(ll_conn_count() == 0 && sch.cb == NULL);
}

/* With the arbiter (Task 5): a link whose window event overlaps another
 * link's accepted one (same priority, it never yielded) is refused and
 * yields that event: counter + 1, collisions + 1, neither missed nor
 * issued; link 0's event runs undisturbed, then link 1's next window
 * event. An event started with a cap below its floor yields as well. */
static void test_collision_skips(void)
{
	struct ll_connect_ind ci0 = mk_ci_link(0), ci1 = mk_ci_link(1);
	const uint32_t t0 = 1000000;
	struct ll_conn_stats s1a, s1b;
	int ev;

	if (LL_MAX_CONN < 2) {
		return;
	}
	reset_all(false);
	ll_conn_get_stats(1, &s1a);
	{
		struct ll_conn_stats s0;

		ll_conn_get_stats(0, &s0);
		CHECK(s0.collisions == 0);
	}
	CHECK(ll_conn_start(&ci0, t0) == 0);
	CHECK(ll_conn_start(&ci1, t0 + T(500)) == 1);
	CHECK(ll_conn_event_counter(1) == 1);
	ll_conn_get_stats(1, &s1b);
	CHECK(s1b.collisions - s1a.collisions == 1);
	CHECK(s1b.missed == s1a.missed && s1b.events == s1a.events);
	fire_alarm();   /* link 0 on air */
	CHECK(rad.aa == ci0.aa);
	ev = rad.events;
	rx(t0 + T(1350), 0x01, 0);
	done(1);
	/* link 0's (idle) event 1 overlaps link 1's window event 1 (MUST):
	 * link 0 yields it */
	CHECK(ll_conn_event_counter(0) == 2);
	{
		struct ll_conn_stats s0;

		ll_conn_get_stats(0, &s0);
		CHECK(s0.collisions == 1);
	}
	fire_alarm();   /* link 1, event 1: its window one interval later */
	CHECK(rad.events == ev + 1 && rad.aa == ci1.aa);
	done(0);
	CHECK(ll_conn_active(0) && ll_conn_active(1));
	ll_conn_end(0, 0x13);
	ll_conn_end(1, 0x13);

	/* started with a cap too small: yield, not a miss */
	reset_all(false);
	CHECK(ll_conn_start(&ci0, t0) == 0);
	ll_conn_get_stats(0, &s1a);
	ev = rad.events;
	ll_conn_arb_start(0, 10);   /* as if the arbiter clipped it to 10 us */
	ll_conn_get_stats(0, &s1b);
	CHECK(rad.events == ev);
	CHECK(s1b.collisions - s1a.collisions == 1 && s1b.missed == s1a.missed);
	CHECK(ll_conn_event_counter(0) == 1 && sch.cb != NULL);
	ll_conn_end(0, 0x13);
}

/* The single-link behaviour on the highest id alone (the others ended):
 * hooks get that link, latency skips and the TX backlog check use it. */
static void test_last_link_alone(void)
{
	const uint8_t pdu[2] = {0x01, 0x02};
	const uint8_t last = LL_MAX_CONN - 1;
	struct ll_connect_ind ci = mk_ci_link(0);
	struct ll_csa1 ref;
	const uint32_t t0 = 1000000;
	uint32_t a0 = t0 + T(1250 + 300);

	ci.latency = 4;
	reset_all(true);
	/* the others 5 ms apart before it (no overlap of the window events) */
	for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
		CHECK(ll_conn_start(&ci, t0 - T(5000) * (uint32_t)(last - k)) == k);
	}
	for (uint8_t k = 0; k < last; k++) {
		ll_conn_end(k, 0x13);
	}
	CHECK(ll_conn_count() == 1 && ll_conn_active(last));
	ll_csa1_init(&ref, 7, all37);
	fire_alarm();
	CHECK(rad.ch == ll_csa1_next(&ref));
	rx(a0, 0x01, 0);
	cbs.busy_calls = 0;
	done(1);
	CHECK(cbs.busy_calls >= 1 && cbs.busy_link == last);
	hold_events(last, &a0, &ref, 1550);
	fire_alarm();
	CHECK(ll_conn_event_counter(last) == EV(5));
	CHECK(rad.open == open_at(a0, 5));
	CHECK(rad.ch == ref_skip(&ref, 5));
	/* backlog on this link: the next event is listened to */
	CHECK(ll_txq_push(last, LL_TXQ_ACL, LL_LLID_START, pdu, sizeof(pdu), 0) == 0);
	rx(a0 + T(75000), 0x01, 0);
	done(1);
	fire_alarm();
	CHECK(ll_conn_event_counter(last) == EV(6));
	done(0);
	/* local termination queues LL_TERMINATE_IND through the link's hook */
	ll_conn_terminate(last, 0x13);
	CHECK(cbs.ctrl_tx_calls == 1 && cbs.ctrl_tx_link == last);
	CHECK(cbs.ctrl_tx_pdu[0] == 0x02 && cbs.ctrl_tx_pdu[1] == 0x13);
	ll_conn_end(last, 0x13);
	CHECK(ll_conn_count() == 0);
}

/* ll_conn_end_all: every active link ends with the reason (the glue's
 * radio-wedge rule); a link with its event on air ends at CONN_DONE. */
static void test_end_all(void)
{
	struct ll_connect_ind ci = mk_ci_link(0);
	const uint32_t t0 = 1000000;

	reset_all(false);
	CHECK(ll_conn_end_all(LL_ST_CONN_TIMEOUT) == 0);
	for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
		CHECK(ll_conn_start(&ci, t0 + T(5000) * k) == k);
	}
	fire_alarm();   /* link 0's window event on air */
	CHECK(rad.aa == ci.aa);
	cbs.disconnected = 0;
	CHECK(ll_conn_end_all(LL_ST_CONN_TIMEOUT) == LL_MAX_CONN);
	/* the others ended at once, link 0 at the end of its event */
	CHECK(cbs.disconnected == LL_MAX_CONN - 1);
	CHECK(ll_conn_active(0));   /* its end is pending until CONN_DONE */
	done(0);
	CHECK(cbs.disconnected == LL_MAX_CONN && cbs.reason == LL_ST_CONN_TIMEOUT);
	for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
		CHECK(!ll_conn_active(k));
	}
	CHECK(ll_conn_count() == 0 && sch.cb == NULL);
	CHECK(ll_conn_end_all(LL_ST_CONN_TIMEOUT) == 0);
}

/* The MUST priority of an instant event is forgotten once the link has
 * planned past it: 65536 events later the same counter is an ordinary
 * (idle) event that an ACTIVE request can displace. */
static void test_instant_prio_after_wrap(void)
{
	struct ll_connect_ind ci = mk_ci(6, 3200, 7, 1, 0);
	uint32_t a = 7000000 + T(1250 + 100);
	const uint16_t x = 10;
	struct ll_arb_req r;
	unsigned int key;
	int ret;

	reset_all(false);
	CHECK(ll_conn_start(&ci, 7000000) == 0);
	CHECK(ll_conn_chmap_at(0, x, no0to9) == 0);
	for (uint32_t k = 0; k < 65536u + x; k++) {
		fire_alarm();
		rx(a, 0x01, 0);
		done(1);
		a += T(7500);
	}
	CHECK(ll_conn_event_counter(0) == x && sch.cb != NULL);
	/* an ACTIVE request over link 0's planned (idle) event */
	r.alarm_tick = sch.tick + T(600);
	r.open_tick = r.alarm_tick;
	r.min_len_us = 500;
	r.max_len_us = 500;
	r.prio = LL_ARB_PRIO_ACTIVE;
	key = ll_plat_lock();
	ret = ll_arb_request(LL_ARB_ADV, &r);
	ll_arb_cancel(LL_ARB_ADV);
	ll_plat_unlock(key);
	CHECK(ret == 0);
	{
		struct ll_conn_stats st;

		ll_conn_get_stats(0, &st);
		CHECK(st.collisions >= 1);   /* link 0 yielded that event */
	}
	CHECK(ll_conn_active(0));
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* ---------------- CSA#2 (Vol 6 Part B 4.5.8.3) ---------------- */

/* the Core Spec sample data access address: channel identifier 0x305F */
#define CSA2_AA    0x8E89BED6u
#define CSA2_CHID  0x305F
/* Vol 6 Part C 3.2: 9 used channels */
static const uint8_t nine[5] = {0x00, 0x06, 0xE0, 0x00, 0x1E};

static uint8_t csa2_ch(uint16_t chid, uint16_t counter, const uint8_t chm[5])
{
	return ll_csa2_channel(chid, counter, chm);
}

/* ChSel 1 in the CONNECT_IND: the link hops with CSA#2. With the sample
 * data AA the first events give the published channels (Part C 3.1), and
 * after a channel map update to the 9-channel map at instant 6 the
 * published 3.2 channels for events 6, 7, 8. */
static void test_csa2_sequence(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	uint32_t a = 5000000 + T(1250 + 100);
	static const uint8_t exp[9] = {25, 20, 6, 21, 0, 0, 23, 9, 34};

	ci.aa = CSA2_AA;
	ci.chsel = 1;
	reset_all(false);
	CHECK(ll_conn_start(&ci, 5000000) == 0);
	for (uint16_t k = 0; k < 9; k++) {
		fire_alarm();
		CHECK(ll_conn_event_counter(0) == k);
		if (k == 4 || k == 5) {
			CHECK(rad.ch == csa2_ch(CSA2_CHID, k, all37));
		} else {
			CHECK(rad.ch == exp[k]);
		}
		rx(a, 0x01, 0);
		done(1);
		if (k == 2) {
			CHECK(ll_conn_chmap_at(0, 6, nine) == 0);
		}
		a += T(15000);
	}
	/* later events keep following CSA#2 on the new map */
	for (uint16_t k = 9; k < 200; k++) {
		fire_alarm();
		CHECK(rad.ch == csa2_ch(CSA2_CHID, k, nine));
		rx(a, 0x01, 0);
		done(1);
		a += T(15000);
	}
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* ChSel 0 (a central without CSA#2, or any link before this slice): the
 * same access address still hops with CSA#1 (hop 7: 7, 14, 21, 28) */
static void test_csa1_when_chsel0(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	uint32_t a = 5000000 + T(1250 + 100);
	struct ll_csa1 ref;

	ci.aa = CSA2_AA;
	ci.chsel = 0;
	reset_all(false);
	ll_csa1_init(&ref, 7, all37);
	CHECK(ll_conn_start(&ci, 5000000) == 0);
	for (int k = 0; k < 100; k++) {
		fire_alarm();
		CHECK(rad.ch == ll_csa1_next(&ref));
		if (k < 4) {
			CHECK(rad.ch == 7 * (k + 1));
		}
		rx(a, 0x01, 0);
		done(1);
		a += T(15000);
	}
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* Latency skips, kicks back to an earlier event and instants inside a
 * skipped window keep the CSA#2 channel exact: it is always the channel of
 * the event counter that goes on air. */
static void test_csa2_latency(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	const uint32_t t0 = 1000000;
	uint32_t a0 = t0 + T(1250 + 300), a1;

	ci.aa = CSA2_AA;
	ci.chsel = 1;
	ci.latency = 4;
	reset_all(false);
	CHECK(ll_conn_start(&ci, t0) == 0);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == 0 && rad.ch == 25);
	rx(a0, 0x01, 0);
	done(1);
	hold_events(0, &a0, NULL, 1550);
	/* skip EV(1..4), listen to EV(5), then EV(10) */
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(5));
	CHECK(rad.ch == csa2_ch(CSA2_CHID, EV(5), all37));
	CHECK(rad.open == open_at(a0, 5));
	rx(a0 + T(75000), 0x01, 0);
	done(1);
	a1 = a0 + T(75000);
	/* skip planned (6..9, listen 10); a kick takes the next reachable
	 * event 6 back, with event 6's channel */
	ll_conn_kick(0);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(6));
	CHECK(rad.ch == csa2_ch(CSA2_CHID, EV(6), all37));
	CHECK(rad.open == open_at(a1, 1));
	rx(a1 + T(15000), 0x01, 0);
	done(1);
	a1 += T(15000);
	/* a channel map instant at event 9: the events up to the instant are
	 * listened to (no skip over an instant), event 9 and later use the
	 * new map, then skipping resumes (14) */
	CHECK(ll_conn_chmap_at(0, EV(9), nine) == 0);
	for (uint16_t k = 7; k <= 9; k++) {
		fire_alarm();
		CHECK(ll_conn_event_counter(0) == EV(k));
		CHECK(rad.ch == csa2_ch(CSA2_CHID, EV(k), k >= 9 ? nine : all37));
		a1 += T(15000);
		rx(a1, 0x01, 0);
		done(1);
	}
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == EV(14));
	CHECK(rad.ch == csa2_ch(CSA2_CHID, EV(14), nine));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* Arbiter yields (collision with another link's event) skip the event
 * like latency: the next listened event has its own CSA#2 channel. Each
 * link uses its own channel identifier; one link may use CSA#1 next to a
 * CSA#2 link. */
static void test_csa2_yield(void)
{
	struct ll_connect_ind ci0 = mk_ci_link(0), ci1 = mk_ci_link(1);
	const uint32_t t0 = 1000000;
	uint16_t id0, last1 = 0, seen0 = 0, seen1 = 0;
	struct ll_conn_stats s0;
	struct ll_csa1 ref1;
	bool gap0 = false;
	uint16_t prev0 = 0;

	if (LL_MAX_CONN < 2) {
		return;
	}
	ci0.chsel = 1;   /* link 0: CSA#2, link 1: CSA#1 */
	id0 = ll_csa2_chan_id(ci0.aa);
	reset_all(false);
	ll_csa1_init(&ref1, 7, all37);
	CHECK(ll_conn_start(&ci0, t0) == 0);
	/* link 1's window overlaps link 0's events: they keep colliding */
	CHECK(ll_conn_start(&ci1, t0 + T(500)) == 1);
	for (int i = 0; i < 60; i++) {
		uint8_t ch;

		fire_alarm();
		if (rad.aa == ci0.aa) {
			uint16_t n = ll_conn_event_counter(0);

			CHECK(rad.ch == csa2_ch(id0, n, all37));
			if (seen0 && (uint16_t)(n - prev0) > 1) {
				gap0 = true;
			}
			prev0 = n;
			seen0++;
		} else {
			uint16_t n = ll_conn_event_counter(1);

			CHECK(rad.aa == ci1.aa);
			ch = 0;
			for (uint16_t k = last1; k <= n; k++) {
				ch = ll_csa1_next(&ref1);
			}
			last1 = (uint16_t)(n + 1);
			CHECK(rad.ch == ch);
			seen1++;
		}
		/* the central's packet at the middle of the RX window */
		rx(rad.open + T(rad.fst / 2) - T(LL_CONN_SYNC_US), 0x01, 0);
		done(1);
	}
	ll_conn_get_stats(0, &s0);
	CHECK(seen0 > 10 && seen1 > 10);
	/* link 0 yielded events and kept the exact channel after the gaps */
	CHECK(s0.collisions > 0 && gap0);
	CHECK(ll_conn_active(0) && ll_conn_active(1));
	ll_conn_end(0, 0x13);
	ll_conn_end(1, 0x13);
}

/* ---------------- latency holdoff after connect (slice 7) ---------------- */

/* Latency 4 from the CONNECT_IND: every event is listened to until the
 * link has been up LL_CONN_LATENCY_HOLDOFF_MS (the central's feature
 * exchange, encryption and connection update are answered at once), then
 * skipping starts as before. A busy LLCP at the end of the holdoff keeps
 * listening until it is idle. */
static void test_latency_holdoff(void)
{
	struct ll_connect_ind ci = mk_ci(12, 400, 1, 1, 0);
	const uint32_t t0 = 1000000;
	const uint16_t h = holdoff_first(1550);
	struct ll_conn_stats s0, s1;
	struct ll_csa1 ref;
	uint32_t a = t0 + T(1250 + 300);

	CHECK(LL_CONN_LATENCY_HOLDOFF_MS == 1000);
	CHECK(h == 67);
	ci.latency = 4;
	reset_all(false);
	ll_conn_get_stats(0, &s0);
	ll_csa1_init(&ref, 7, all37);
	CHECK(ll_conn_start(&ci, t0) == 0);
	for (uint16_t e = 0; e < h; e++) {
		fire_alarm();
		CHECK(ll_conn_event_counter(0) == e);
		CHECK(rad.ch == ll_csa1_next(&ref));
		if (e > 0) {
			CHECK(rad.open == open_at(a, 1));
			a += T(15000);
		}
		rx(a, 0x01, 0);
		done(1);
	}
	/* event h is the first one at or after 1 s: skipped with h + 1..h + 3 */
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == h + 4);
	CHECK(rad.open == open_at(a, 5));
	CHECK(rad.ch == ref_skip(&ref, 5));
	a += T(75000);
	rx(a, 0x01, 0);
	done(1);
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == h + 9);
	CHECK(rad.open == open_at(a, 5));
	done(0);
	ll_conn_get_stats(0, &s1);
	CHECK(s1.skipped - s0.skipped == 8);
	CHECK(s1.listened - s0.listened == (uint32_t)h + 2);
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* LLCP busy when the holdoff ends: listening goes on until it is
	 * idle, then skipping starts */
	reset_all(false);
	CHECK(ll_conn_start(&ci, t0) == 0);
	a = t0 + T(1250 + 300);
	for (uint16_t e = 0; e < h + 3; e++) {
		if (e == h - 5) {
			busy_flag = true;
		}
		if (e == h + 2) {
			busy_flag = false;
		}
		fire_alarm();
		CHECK(ll_conn_event_counter(0) == e);
		if (e > 0) {
			a += T(15000);
		}
		rx(a, 0x01, 0);
		done(1);
	}
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == h + 7);
	CHECK(rad.open == open_at(a, 5));
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* the holdoff counts from the connection start, not from the first
	 * packet: a link synced late (event 3) skips from the same event */
	reset_all(false);
	CHECK(ll_conn_start(&ci, t0) == 0);
	ev_miss();
	ev_miss();
	ev_miss();
	a = t0 + T(1250 + 300) + T(45000);
	for (uint16_t e = 3; e < h; e++) {
		fire_alarm();
		CHECK(ll_conn_event_counter(0) == e);
		if (e > 3) {
			a += T(15000);
		}
		rx(a, 0x01, 0);
		done(1);
	}
	fire_alarm();
	CHECK(ll_conn_event_counter(0) == h + 4);
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);
}

/* Per link: links started 300 ms apart (30 ms interval, latency 4, anchors
 * 5 ms apart so no event collides) each listen to every event for their
 * own first LL_CONN_LATENCY_HOLDOFF_MS, and skip afterwards, while the
 * older links already skip. */
static void test_latency_holdoff_per_link(void)
{
	const uint32_t t0 = 1000000;
	uint32_t start[LL_MAX_CONN];
	int last[LL_MAX_CONN];
	uint32_t last_anchor[LL_MAX_CONN];
	int skips[LL_MAX_CONN], early_gaps = 0, late_ones = 0;
	uint32_t coll0[LL_MAX_CONN];
	uint8_t started = 0;

	reset_all(false);
	for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
		struct ll_conn_stats s;

		ll_conn_get_stats(k, &s);   /* cumulative since boot */
		coll0[k] = s.collisions;
		start[k] = t0 + T(305000) * k;
		last[k] = -1;
		skips[k] = 0;
	}
	while (now < start[LL_MAX_CONN - 1] + T(2500000)) {
		if (started < LL_MAX_CONN && (sch.cb == NULL ||
		    (int32_t)(sch.tick - start[started]) > 0)) {
			struct ll_connect_ind ci = mk_ci_link(started);

			ci.interval = 24;
			ci.latency = 4;
			if ((int32_t)(start[started] - now) > 0) {
				now = start[started];
			}
			CHECK(ll_conn_start(&ci, start[started]) == started);
			started++;
			continue;
		}
		fire_alarm();
		{
			int k = -1;

			for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
				struct ll_connect_ind ci = mk_ci_link(i);

				if (ci.aa == rad.aa && ll_conn_active(i)) {
					k = i;
				}
			}
			CHECK(k >= 0);
			if (k < 0) {
				break;
			}
			/* the central's packet in the middle of the RX window */
			uint32_t anchor = rad.open + T((rad.fst - LL_CONN_SYNC_US) / 2);
			int e = ll_conn_event_counter((uint8_t)k);

			if (last[k] >= 0 && e - last[k] > 1) {
				/* a skip: its first skipped event lies after the
				 * link's own holdoff */
				skips[k]++;
				if ((int32_t)(last_anchor[k] + T(30000) - start[k]) <
				    (int32_t)T(LL_CONN_LATENCY_HOLDOFF_MS * 1000u)) {
					early_gaps++;
				}
				CHECK(e - last[k] == 5);
			} else if (last[k] >= 0 &&
				   (int32_t)(last_anchor[k] + T(30000) - start[k]) >=
					   (int32_t)(T(LL_CONN_LATENCY_HOLDOFF_MS * 1000u) + T(60000))) {
				late_ones++;   /* listened although idle after the holdoff */
			}
			last[k] = e;
			last_anchor[k] = anchor;
			rx(anchor, 0x01, 0);
			done(1);
		}
	}
	CHECK(started == LL_MAX_CONN);
	CHECK(early_gaps == 0 && late_ones == 0);
	for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
		struct ll_conn_stats s;

		ll_conn_get_stats(k, &s);
		CHECK(skips[k] > 0 && s.collisions == coll0[k]);
		CHECK(ll_conn_active(k));
		ll_conn_end(k, LL_ST_REMOTE_TERM);
	}
}

/* Slice 7: "instant passed" is judged by the instant event's anchor, not
 * by its alarm. An instant for a skipped event whose alarm time has gone
 * by but whose anchor has not is still honoured: the event is planned with
 * a late alarm and issued if there is still time to prepare it (else it
 * counts as a late miss), the instant is applied, the link lives on. Once
 * the anchor has passed, it is 0x28 as before. */
static void test_instant_alarm_passed_anchor_not(void)
{
	struct ll_conn_params p24 = {.interval = 24, .latency = 0, .timeout = 400};
	struct ll_conn_stats s0, s1;
	struct ll_csa1 ref;
	uint32_t a0;
	int ev;

	/* inside the alarm lead of EV(3): still issued, with the new map */
	a0 = start_lat(4, 400, &ref, false);
	now = open_at(a0, 3) - T(LL_CONN_ARM_LEAD_US) + T(100);
	CHECK(ll_conn_chmap_at(0, EV(3), no0to9) == 0);
	CHECK(ll_conn_active(0) && cbs.disconnected == 0);
	ev = rad.events;
	fire_alarm();
	CHECK(rad.events == ev + 1);
	CHECK(ll_conn_event_counter(0) == EV(3));
	CHECK(rad.open == open_at(a0, 3));
	(void)ref_skip(&ref, 2);
	ll_csa1_set_map(&ref, no0to9);
	CHECK(rad.ch == ll_csa1_next(&ref));
	rx(a0 + T(45000), 0x01, 0);
	done(1);
	CHECK(ll_conn_active(0));
	fire_alarm();
	CHECK(ll_conn_event_counter(0) > EV(3) && rad.ch >= 10);
	done(0);
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* closer than LL_CONN_MIN_PREP_US to EV(3)'s RX: a late miss, but the
	 * map is applied and the link follows on */
	a0 = start_lat(4, 400, &ref, false);
	ll_conn_get_stats(0, &s0);
	now = open_at(a0, 3) - T(LL_CONN_MIN_PREP_US) + T(10);
	CHECK(ll_conn_chmap_at(0, EV(3), no0to9) == 0);
	ev = rad.events;
	fire_alarm();
	CHECK(rad.events == ev);
	ll_conn_get_stats(0, &s1);
	CHECK(s1.late - s0.late == 1);
	CHECK(ll_conn_active(0));
	fire_alarm();
	CHECK(rad.events == ev + 1);
	CHECK(ll_conn_event_counter(0) == EV(4));
	CHECK(rad.open == open_at(a0, 4));
	(void)ref_skip(&ref, 2);
	ll_csa1_set_map(&ref, no0to9);
	(void)ll_csa1_next(&ref);
	CHECK(rad.ch == ll_csa1_next(&ref));
	done(0);
	CHECK(ll_conn_active(0));
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* a connection update for EV(3) inside its alarm lead: the transmit
	 * window event of the instant is issued (old anchor + 0, 1.25 ms;
	 * widening 300 ppm over 46.25 ms: 13.875 -> 14 + 16 = 30 us) */
	a0 = start_lat(4, 400, &ref, false);
	now = open_at(a0, 3) - T(LL_CONN_ARM_LEAD_US) + T(100);   /* old timing's alarm */
	CHECK(ll_conn_update_at(0, EV(3), 1, 0, &p24) == 0);
	ev = rad.events;
	fire_alarm();
	CHECK(rad.events == ev + 1 && cbs.updated == 1);
	CHECK(ll_conn_event_counter(0) == EV(3));
	CHECK(rad.open == a0 + T(45000) - T(30 + LL_CONN_WIN_MARGIN_US));
	rx(a0 + T(45000) + T(200), 0x01, 0);
	done(1);
	CHECK(ll_conn_active(0));
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* the anchor of EV(3) one tick ahead: not passed (a late miss) */
	a0 = start_lat(4, 400, &ref, false);
	now = a0 + T(45000) - 1;
	CHECK(ll_conn_chmap_at(0, EV(3), no0to9) == 0);
	CHECK(ll_conn_active(0));
	ll_conn_end(0, LL_ST_REMOTE_TERM);

	/* the anchor of EV(3) reached: passed, 0x28 */
	a0 = start_lat(4, 400, &ref, false);
	now = a0 + T(45000);
	CHECK(ll_conn_chmap_at(0, EV(3), no0to9) == LL_ST_INSTANT_PASSED);
	CHECK(!ll_conn_active(0) && cbs.disconnected == 1);
	CHECK(cbs.reason == LL_ST_INSTANT_PASSED);
}

int main(void)
{
	test_first_events();
	test_rx_path();
	test_start_keeps_rxq();
	test_rxq_overflow_ends_link();
	test_first_packet_bad_crc();
	test_first_packet_retransmission();
	test_first_packet_after_window();
	test_six_interval_rule();
	test_supervision();
	test_widening_clamp();
	test_conn_update();
	test_conn_update_same_params();
	test_update_restarts_supervision();
	test_chmap();
	test_instant_replan();
	test_instant_passed();
	test_counter_wrap();
	test_local_terminate_ack();
	test_local_terminate_timeout();
	test_end();
	test_late_alarm();
	test_start_validation();
	test_long_interval();
	test_update_validation();
	test_event_cap();
	test_latency_skip();
	test_latency_refused_txq();
	test_latency_refused_busy_term();
	test_latency_no_skip_unsynced();
	test_latency_instant_pending();
	test_latency_instant_in_window();
	test_latency_supervision();
	test_kick();
	test_latency_from_update();
	test_latency_instant_replan_then_kick();
	test_latency_instant_replan_reachable();
	test_link_ids();
	test_links_interleaved();
	test_collision_skips();
	test_last_link_alone();
	test_end_all();
	test_instant_prio_after_wrap();
	test_csa2_sequence();
	test_csa1_when_chsel0();
	test_csa2_latency();
	test_csa2_yield();
	test_latency_holdoff();
	test_latency_holdoff_per_link();
	test_instant_alarm_passed_anchor_not();
	DONE();
}
