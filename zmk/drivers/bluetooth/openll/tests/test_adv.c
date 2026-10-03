#include <errno.h>
#include <string.h>
#include "test.h"
#include "../ll_adv.h"
#include "../ll_arb.h"
#include "../ll_conn.h"
#include "../ll_defs.h"
#include "../ll_sched.h"
#include "../ll_plat.h"

/* ---- fakes ---- */
static uint32_t now_tick = 1000000;
static uint32_t sched_tick; static ll_sched_cb_t sched_cb; static int sched_cancels;
static int sched_calls;
static uint8_t radio_ch; static uint8_t tx_pdu[64]; static uint8_t tx_len;
static uint32_t tx_start; static int txrx_calls;
static uint8_t rsp_pdu[64]; static uint8_t rsp_len; static uint32_t rsp_tick; static int rsp_calls;
static int radio_stops; static bool rsp_ok = true;
static int conn_calls; static struct ll_connect_ind conn;

int ll_radio_init(ll_radio_cb_t cb) { (void)cb; return 0; }
uint32_t ll_radio_now(void) { return now_tick; }
void ll_radio_set_adv_channel(uint8_t ch) { radio_ch = ch; }
void ll_radio_tx_then_rx(const uint8_t *p, uint8_t l, uint32_t t, uint32_t w)
{ (void)w; memcpy(tx_pdu, p, l); tx_len = l; tx_start = t; txrx_calls++; }
void ll_radio_prepare_rsp(const uint8_t *p, uint8_t l) { memcpy(rsp_pdu, p, l); rsp_len = l; }
bool ll_radio_tx_rsp_at(uint32_t t) { rsp_tick = t; rsp_calls++; return rsp_ok; }
void ll_radio_stop(void) { radio_stops++; }
void ll_sched_init(void) {}
void ll_sched_at(uint32_t t, ll_sched_cb_t cb) { sched_tick = t; sched_cb = cb; sched_calls++; }
void ll_sched_cancel(void) { sched_cancels++; sched_cb = NULL; }
uint32_t ll_plat_rand32(void) { return 1234567; }
unsigned int ll_plat_lock(void) { return 0; }
void ll_plat_unlock(unsigned int k) { (void)k; }
static void on_conn(const struct ll_connect_ind *ci) { conn = *ci; conn_calls++; }
static int start_ret = -EINVAL, start_calls, restores; static bool conn_is_active;
static struct ll_connect_ind start_ci; static uint32_t start_tick;
static bool start_saw_enabled = true, start_saw_sched = true; static int start_saw_stops;
/* links taken (active or awaiting release): 0..LL_MAX_CONN */
static uint8_t taken;
int ll_conn_start(const struct ll_connect_ind *ci, uint32_t t)
{
	start_calls++; start_ci = *ci; start_tick = t;
	/* advertising must already be stopped when the connection starts */
	start_saw_enabled = ll_adv_is_enabled(); start_saw_sched = sched_cb != NULL;
	start_saw_stops = radio_stops;
	if (start_ret >= 0) {
		conn_is_active = true;
		taken++;
	}
	return start_ret;
}
uint8_t ll_conn_count(void) { return taken; }
void ll_radio_adv_restore(void) { restores++; }
static int adv_enters;
void ll_radio_adv_enter(void) { adv_enters++; }

static const uint8_t adva[6] = {0x01, 0x02, 0x03, 0x38, 0xC1, 0xA4};

static struct ll_adv_params params(uint8_t type, uint8_t map)
{
	struct ll_adv_params p = {.interval_min = 0x00A0, .interval_max = 0x00F0,
				  .type = type, .chan_map = map};
	return p;
}

/* the real ll_arb; advertising is its only real owner, id 0 is a foreign
 * requester driven by the test (starts and bumps recorded) */
static int foreign_starts, foreign_bumps;
static void arb_start(uint8_t id, uint32_t cap_us)
{
	if (id == LL_ARB_ADV) ll_adv_arb_start(cap_us); else foreign_starts++;
}
static void arb_bumped(uint8_t id)
{
	if (id == LL_ARB_ADV) ll_adv_arb_bumped(); else foreign_bumps++;
}
static const struct ll_arb_ops arb_ops = {.start = arb_start, .bumped = arb_bumped};

static void fire_sched(void) { ll_sched_cb_t cb = sched_cb; sched_cb = NULL; CHECK(cb != NULL); if (cb) cb(); }

static void put_ci_pdu(uint8_t *ci_pdu)
{
	static const uint8_t base[8] = {0x05, 34, 0x11, 0x12, 0x13, 0x14, 0x15, 0xD6};

	memset(ci_pdu, 0, 36);
	memcpy(ci_pdu, base, 8);
	memcpy(&ci_pdu[8], adva, 6);
	ci_pdu[2 + 22] = 24;  /* interval */
}

/* Slice 6a Task 6: enable while links are up. */
static void test_while_connected(void)
{
	struct ll_adv_params p;
	uint8_t ci_pdu[36];

	ll_adv_reset();
	/* counts 0..N: connectable allowed below N, 0x09 at N */
	for (uint8_t n = 0; n <= LL_MAX_CONN; n++) {
		taken = n;
		for (uint8_t type = 0; type <= 3; type++) {
			uint8_t want;

			if (type == 1) {
				continue;   /* directed: unsupported */
			}
			p = params(type, 7);
			CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
			/* only connectable (ADV_IND) is limited */
			want = (type == 0 && n >= LL_MAX_CONN) ? LL_ST_CONN_LIMIT : LL_ST_SUCCESS;
			CHECK(ll_adv_enable(true) == want);
			CHECK(ll_adv_is_enabled() == (want == LL_ST_SUCCESS));
			CHECK((sched_cb != NULL) == (want == LL_ST_SUCCESS));
			CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
		}
	}
	/* after a handover with links left: no restore, ll_radio_adv_enter
	 * before every channel start that follows a gap */
	if (LL_MAX_CONN >= 2) {
		int r0 = restores, e0;

		taken = 0;
		p = params(0, 7);
		CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
		CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
		fire_sched();
		put_ci_pdu(ci_pdu);
		start_ret = 0;
		ll_adv_radio_evt(LL_RADIO_RX_OK, ci_pdu, 36, now_tick);
		CHECK(!ll_adv_is_enabled() && taken == 1);
		e0 = adv_enters;
		CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
		CHECK(restores == r0);
		fire_sched();
		CHECK(adv_enters == e0 + 1 && radio_ch == 37);
		/* the whole event is one request: no further enter */
		ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
		ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
		CHECK(radio_ch == 39 && adv_enters == e0 + 1);
		ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
		/* connection radio events between adv events are ignored */
		{
			int tx0 = txrx_calls, sc0 = sched_calls;

			ll_adv_radio_evt(LL_RADIO_CONN_DONE, NULL, 0, 0);
			ll_adv_radio_evt(LL_RADIO_CONN_RX, ci_pdu, 2, 0);
			CHECK(txrx_calls == tx0 && sched_calls == sc0);
		}
		fire_sched();
		CHECK(adv_enters == e0 + 2);
		CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
		/* the last link gone: the next enable restores once, no enters */
		taken = 0;
		CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
		CHECK(restores == r0 + 1);
		e0 = adv_enters;
		fire_sched();
		CHECK(adv_enters == e0);
		CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
	}
	taken = 0;
	conn_is_active = false;
}

static int foreign_req(uint32_t t, uint32_t len_us, uint8_t prio)
{
	struct ll_arb_req r = {.alarm_tick = t, .open_tick = t, .min_len_us = len_us,
			       .max_len_us = len_us, .prio = prio};

	return ll_arb_request(0, &r);
}

/* A displaced adv event goes straight to the next gap: it never asks for
 * the displaced placement again (which, at a tie it would win as the one
 * that yielded last, would displace the displacer back). */
static void test_bumped_goes_to_gap(void)
{
	struct ll_adv_params p = params(0, 7);
	uint32_t ev0;

	ll_adv_reset();
	foreign_starts = foreign_bumps = 0;
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	ev0 = sched_tick;
	/* the foreign requester loses once and gives up (yield), so it wins
	 * the next tie against advertising and displaces it */
	CHECK(foreign_req(ev0 + 1000 * LL_TICKS_PER_US, 1000, LL_ARB_PRIO_ADV) == -EBUSY);
	ll_arb_yield(0);
	CHECK(foreign_req(ev0 + 1000 * LL_TICKS_PER_US, 1000, LL_ARB_PRIO_ADV) == 0);
	CHECK(foreign_bumps == 0);   /* advertising did not take it back */
	CHECK(sched_tick == ev0 + 1000 * LL_TICKS_PER_US);
	fire_sched();
	CHECK(foreign_starts == 1);
	ll_arb_cancel(0);
	/* advertising after the foreign span */
	CHECK(sched_cb != NULL && (int32_t)(sched_tick - (ev0 + 2000 * LL_TICKS_PER_US)) > 0);
	fire_sched();
	CHECK(radio_ch == 37);
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
}

static int foreign_req_id(uint8_t id, uint32_t t, uint32_t len_us, uint8_t prio)
{
	struct ll_arb_req r = {.alarm_tick = t, .open_tick = t, .min_len_us = len_us,
			       .max_len_us = len_us, .prio = prio};

	return ll_arb_request(id, &r);
}

#define TU(us) ((uint32_t)(us) * LL_TICKS_PER_US)

/* Run one adv channel at the pending alarm: fire, 700 us of air. */
static void adv_channel(void)
{
	now_tick = sched_tick;
	fire_sched();
	now_tick += TU(700);
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
}

/* Slicing: the whole event is moved behind a short block when it fits
 * there within 10 ms; else only the first channel goes in front of it,
 * and the next channels into later gaps, each within 10 ms of the last
 * PDU (Vol 6 Part B 4.4.2.3), or the event is cut. */
static void test_sliced(void)
{
	struct ll_adv_params p = params(3, 7);
	struct ll_adv_stats a0, a1;
	uint32_t ev0;

	ll_adv_reset();
	foreign_starts = foreign_bumps = 0;
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	ll_adv_get_stats(&a0);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	ev0 = sched_tick;
	/* A: a 3 ms MUST block 2.5 ms after the start: the whole event moves
	 * behind it (5.5 ms later) and runs its 3 channels back to back */
	CHECK(foreign_req(ev0 + TU(2500), 3000, LL_ARB_PRIO_MUST) == 0);
	CHECK(sched_tick == ev0 + TU(2500));
	now_tick = sched_tick;
	fire_sched();
	CHECK(foreign_starts == 1);
	ll_arb_cancel(0);
	CHECK(sched_tick == ev0 + TU(5500) + 1);
	now_tick = sched_tick;
	fire_sched();
	CHECK(radio_ch == 37);
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	CHECK(radio_ch == 38);
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	CHECK(radio_ch == 39);
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	ll_adv_get_stats(&a1);
	CHECK(a1.events == a0.events + 1 && a1.slid == a0.slid + 1 && a1.cut == a0.cut);

	/* B: a 12 ms block: only channel 37 fits in front of it; channel 38
	 * would start more than 10 ms after it: the event is cut */
	ev0 = sched_tick;
	CHECK(foreign_req(ev0 + TU(2500), 12000, LL_ARB_PRIO_MUST) == 0);
	CHECK(sched_tick == ev0);
	adv_channel();
	CHECK(radio_ch == 37);
	ll_adv_get_stats(&a1);
	CHECK(a1.cut == a0.cut + 1 && a1.events == a0.events + 2);
	CHECK(sched_tick == ev0 + TU(2500));   /* the block, then the next event */
	now_tick = sched_tick;
	fire_sched();
	ll_arb_cancel(0);
	CHECK(sched_cb != NULL && (int32_t)(sched_tick - (ev0 + TU(100000))) > 0);

	/* C (two foreign requesters, N >= 2): blocks [2.5, 5.5] and [10, 20]
	 * ms: sliced, 37 at the start, 38 after the first block, 39 right
	 * behind it, all within 10 ms of each other */
	if (LL_MAX_CONN >= 2) {
		uint32_t t37, t38;

		ev0 = sched_tick;
		CHECK(foreign_req_id(1, ev0 + TU(10000), 10000, LL_ARB_PRIO_MUST) == 0);
		CHECK(foreign_req_id(0, ev0 + TU(2500), 3000, LL_ARB_PRIO_MUST) == 0);
		CHECK(sched_tick == ev0);
		t37 = ev0;
		adv_channel();
		CHECK(radio_ch == 37);
		CHECK(sched_tick == ev0 + TU(2500));
		/* waiting for channel 38: radio events of other users (a link's
		 * connection event in the gap, a stray timeout) are ignored */
		{
			int tx0 = txrx_calls, sc0 = sched_calls;

			ll_adv_radio_evt(LL_RADIO_CONN_DONE, NULL, 0, 0);
			ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
			ll_adv_radio_evt(LL_RADIO_TX_DONE, NULL, 0, 0);
			CHECK(txrx_calls == tx0 && sched_calls == sc0 && radio_ch == 37);
		}
		now_tick = sched_tick;
		fire_sched();               /* block 1 */
		ll_arb_cancel(0);
		t38 = sched_tick;
		CHECK(t38 == ev0 + TU(5500) + 1);
		adv_channel();
		CHECK(radio_ch == 38);
		CHECK((int32_t)(sched_tick - t38) > 0 &&
		      (int32_t)(sched_tick - (ev0 + TU(10000))) < 0);
		CHECK((int32_t)(t38 - t37) <= (int32_t)TU(10000));
		adv_channel();
		CHECK(radio_ch == 39);
		ll_adv_get_stats(&a1);
		CHECK(a1.cut == a0.cut + 1 && a1.events == a0.events + 3);
		now_tick = sched_tick;
		fire_sched();               /* block 2 */
		ll_arb_cancel(1);

		/* D: the next channel's gap would start 9.7 ms after the last
		 * PDU: with the start slack (stimer ISR latency + adv_enter,
		 * 370 us) it could begin after 10 ms, so the event is cut */
		ll_adv_get_stats(&a0);
		ev0 = sched_tick;
		CHECK(foreign_req_id(1, ev0 + TU(12000), 10000, LL_ARB_PRIO_MUST) == 0);
		CHECK(foreign_req_id(0, ev0 + TU(2500), 7200, LL_ARB_PRIO_MUST) == 0);
		CHECK(sched_tick == ev0);
		adv_channel();
		CHECK(radio_ch == 37);
		ll_adv_get_stats(&a1);
		CHECK(a1.cut == a0.cut + 1);
		now_tick = sched_tick;
		fire_sched();               /* block 1 */
		ll_arb_cancel(0);
		now_tick = sched_tick;
		fire_sched();               /* block 2 */
		ll_arb_cancel(1);
	}

	/* E: a complete event, then two events cut after their first
	 * channel: advertising is starving, its next event is requested at
	 * ACTIVE, so an idle request over it is refused */
	adv_channel();              /* a whole event: 38 and 39 follow at once */
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	CHECK(radio_ch == 39);
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	for (int i = 0; i < 2; i++) {
		ev0 = sched_tick;
		CHECK(foreign_req(ev0 + TU(2500), 12000, LL_ARB_PRIO_MUST) == 0);
		CHECK(sched_tick == ev0);
		adv_channel();
		now_tick = sched_tick;
		fire_sched();               /* the block */
		ll_arb_cancel(0);
	}
	ev0 = sched_tick;
	CHECK(foreign_req(ev0 + TU(500), 1000, LL_ARB_PRIO_IDLE) == -EBUSY);
	CHECK(foreign_req(ev0 + TU(500), 1000, LL_ARB_PRIO_SUPERVISION) == 0);
	ll_arb_cancel(0);
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
}

int main(void)
{
	struct ll_adv_params p;

	ll_arb_init(&arb_ops);
	ll_adv_init(adva, on_conn);

	/* parameter validation */
	p = params(1, 7); CHECK(ll_adv_set_params(&p) == LL_ST_UNSUPPORTED);   /* directed */
	p = params(5, 7); CHECK(ll_adv_set_params(&p) == LL_ST_INVALID_PARAM);
	p = params(0, 0); CHECK(ll_adv_set_params(&p) == LL_ST_INVALID_PARAM); /* no channels */
	p = params(0, 7); p.interval_min = 0x10; CHECK(ll_adv_set_params(&p) == LL_ST_INVALID_PARAM);
	p = params(0, 7); p.interval_min = 0x200; CHECK(ll_adv_set_params(&p) == LL_ST_INVALID_PARAM); /* min > max */
	p = params(0, 7); p.own_addr_type = 1; CHECK(ll_adv_set_params(&p) == LL_ST_UNSUPPORTED);
	p = params(0, 7); p.filter_policy = 1; CHECK(ll_adv_set_params(&p) == LL_ST_UNSUPPORTED);
	CHECK(ll_adv_set_data(NULL, 32) == LL_ST_INVALID_PARAM);
	CHECK(ll_adv_set_data(NULL, 5) == LL_ST_INVALID_PARAM);  /* data NULL, len != 0 */

	/* valid ADV_IND on all channels */
	p = params(0, 7);
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	const uint8_t ad[3] = {0x02, 0x01, 0x06};
	CHECK(ll_adv_set_data(ad, 3) == LL_ST_SUCCESS);
	const uint8_t sr[3] = {0x02, 0x09, 'R'};
	CHECK(ll_adv_set_scan_rsp(sr, 3) == LL_ST_SUCCESS);

	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	CHECK(ll_adv_is_enabled());
	CHECK(sched_cb != NULL);
	uint32_t ev0 = sched_tick;
	CHECK(ev0 > now_tick);
	p = params(0, 7);
	CHECK(ll_adv_set_params(&p) == LL_ST_DISALLOWED);  /* while enabled */

	/* event: 37 -> 38 -> 39 on RX timeouts */
	fire_sched();
	CHECK(radio_ch == 37 && txrx_calls == 1);
	CHECK(tx_len == 11 && tx_pdu[0] == (LL_PDU_ADV_IND | 0x20) && tx_pdu[1] == 9); /* ChSel 1 */
	CHECK(memcmp(&tx_pdu[2], adva, 6) == 0);
	CHECK(rsp_len == 11 && rsp_pdu[0] == LL_PDU_SCAN_RSP);  /* prepared */
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	CHECK(radio_ch == 38 && txrx_calls == 2);
	ll_adv_radio_evt(LL_RADIO_RX_CRC_ERR, NULL, 0, 0);
	CHECK(radio_ch == 39 && txrx_calls == 3);
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	/* next event: interval_min * 625 us + advDelay (rand % 10 ms window) */
	CHECK(sched_cb != NULL);
	CHECK(sched_tick == ev0 + 0x00A0 * 625 * LL_TICKS_PER_US +
	      1234567u % (10000 * LL_TICKS_PER_US + 1));

	/* SCAN_REQ for us -> SCAN_RSP at end + T_IFS, then next channel on TX done */
	fire_sched();
	CHECK(radio_ch == 37);
	uint8_t req[14] = {0x43, 12, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
	memcpy(&req[8], adva, 6);
	ll_adv_radio_evt(LL_RADIO_RX_OK, req, 14, 5000000);
	CHECK(rsp_calls == 1);
	CHECK(rsp_tick == 5000000 + LL_T_IFS_US * LL_TICKS_PER_US);
	CHECK(radio_ch == 37);
	ll_adv_radio_evt(LL_RADIO_TX_DONE, NULL, 0, 0);
	CHECK(radio_ch == 38);

	/* unrelated packet on 38 -> just move on */
	req[13] ^= 1;
	ll_adv_radio_evt(LL_RADIO_RX_OK, req, 14, 6000000);
	CHECK(rsp_calls == 1 && radio_ch == 39);
	req[13] ^= 1;

	/* CONNECT_IND for us on 39 that ll_conn refuses (-EINVAL): callback,
	 * advertising continues next event */
	uint8_t ci_pdu[36] = {0x65, 34, 0x11, 0x12, 0x13, 0x14, 0x15, 0xD6};
	memcpy(&ci_pdu[8], adva, 6);
	ci_pdu[2 + 22] = 24;  /* interval */
	start_ret = -EINVAL;
	ll_adv_radio_evt(LL_RADIO_RX_OK, ci_pdu, 36, 7000000);
	CHECK(conn_calls == 1 && conn.interval == 24);
	CHECK(start_calls == 1 && start_ci.interval == 24 && start_tick == 7000000);
	CHECK(sched_cb != NULL);
	CHECK(ll_adv_is_enabled());

	/* data update while enabled is used on the next TX */
	const uint8_t ad2[4] = {0x03, 0x19, 0xC1, 0x03};
	CHECK(ll_adv_set_data(ad2, 4) == LL_ST_SUCCESS);
	fire_sched();
	CHECK(tx_len == 12 && tx_pdu[1] == 10 && tx_pdu[8] == 0x03);

	/* disable: cancel + stop, later radio events ignored */
	int before = txrx_calls;
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
	CHECK(!ll_adv_is_enabled());
	/* disabled inside an event: no alarm was pending (ll_arb arms none
	 * while the adv event runs), the arbiter drops the request */
	CHECK(sched_cb == NULL && radio_stops >= 1);
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	CHECK(txrx_calls == before);

	/* channel map 37 + 39 only, non-connectable: SCAN_REQ ignored, no CONNECT_IND */
	p = params(3, 0x05);
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	fire_sched();
	CHECK(radio_ch == 37 && tx_pdu[0] == LL_PDU_ADV_NONCONN_IND);
	ll_adv_radio_evt(LL_RADIO_RX_OK, req, 14, 8000000);
	CHECK(rsp_calls == 1);
	CHECK(radio_ch == 39);
	ll_adv_radio_evt(LL_RADIO_RX_OK, ci_pdu, 36, 9000000);
	CHECK(conn_calls == 1);
	CHECK(sched_cb != NULL);  /* event ended after 39 */

	/* in_event guard: a radio event arriving after the event already ended
	 * (e.g. a late/extra RX timeout) must not schedule or transmit again */
	int sc_before = sched_calls;
	int txrx_before = txrx_calls;
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	CHECK(sched_calls == sc_before);
	CHECK(txrx_calls == txrx_before);

	/* catch-up after a stall: if the MCU missed the window for a long time,
	 * the next event_tick is re-based on now instead of staying far in the past */
	uint32_t ev_prev = sched_tick;
	fire_sched();
	CHECK(radio_ch == 37);
	uint32_t interval = (uint32_t)0x00A0 * 625u * LL_TICKS_PER_US;
	now_tick = ev_prev + 10 * interval;
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	CHECK(radio_ch == 39);
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	CHECK(sched_tick == now_tick + interval + 1234567u % (10000 * LL_TICKS_PER_US + 1));

	/* in_event guard on the RX-timeout end path: an extra timeout after the
	 * last channel neither reschedules nor transmits */
	sc_before = sched_calls;
	txrx_before = txrx_calls;
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	CHECK(sched_calls == sc_before);
	CHECK(txrx_calls == txrx_before);

	/* a refused CONNECT_IND keeps advertising: the next event transmits */
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
	p = params(0, 7);
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	fire_sched();
	CHECK(radio_ch == 37);
	start_ret = -EBUSY;
	ll_adv_radio_evt(LL_RADIO_RX_OK, ci_pdu, 36, 7100000);
	CHECK(start_calls == 2 && ll_adv_is_enabled() && sched_cb != NULL);
	txrx_before = txrx_calls;
	fire_sched();
	CHECK(txrx_calls == txrx_before + 1 && radio_ch == 37);

	/* accepted CONNECT_IND (on 37): advertising is stopped before
	 * ll_conn_start (Vol 4 Part E 7.8.9), stays disabled, nothing scheduled */
	int stops_before = radio_stops;
	start_ret = 2;   /* a link id: any id >= 0 means accepted (slice 6a) */
	sc_before = sched_calls;
	txrx_before = txrx_calls;
	ll_adv_radio_evt(LL_RADIO_RX_OK, ci_pdu, 36, 7200000);
	CHECK(start_calls == 3 && start_tick == 7200000);
	CHECK(!start_saw_enabled && !start_saw_sched);
	CHECK(start_saw_stops == stops_before + 1);
	CHECK(!ll_adv_is_enabled());
	CHECK(sched_cb == NULL && sched_calls == sc_before && txrx_calls == txrx_before);
	/* late radio events of the old advertising event are ignored */
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	ll_adv_radio_evt(LL_RADIO_TX_DONE, NULL, 0, 0);
	CHECK(sched_calls == sc_before && txrx_calls == txrx_before);
	/* with one link only, connectable advertising cannot be re-enabled
	 * during the connection (0x09); with more links it can, without a
	 * baseband restore. Disable is a harmless no-op. */
	if (LL_MAX_CONN == 1) {
		CHECK(ll_adv_enable(true) == LL_ST_CONN_LIMIT);
		CHECK(!ll_adv_is_enabled() && sched_cb == NULL && restores == 0);
	} else {
		CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
		CHECK(ll_adv_is_enabled() && restores == 0);
		CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
	}
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
	/* parameters and data may still be changed (host prepares resume) */
	p = params(0, 7);
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	/* after the disconnect the host re-enables advertising via HCI: the
	 * radio is restored once (baseband left connection mode), then events run */
	conn_is_active = false;
	taken = 0;
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	CHECK(restores == 1 && ll_adv_is_enabled() && sched_cb != NULL);
	fire_sched();
	CHECK(radio_ch == 37 && tx_pdu[0] == (LL_PDU_ADV_IND | 0x20));
	/* no further restore without another connection */
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	CHECK(restores == 1);
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);

	/* reset disables and restores defaults */
	ll_adv_reset();
	p = params(0, 7);
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);

	/* SCAN_RSP trigger too late: the radio refuses it (returns false) and no
	 * TX_DONE will follow, so the next channel must be used immediately */
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	fire_sched();
	CHECK(radio_ch == 37);
	int rsp_before = rsp_calls;
	txrx_before = txrx_calls;
	rsp_ok = false;
	ll_adv_radio_evt(LL_RADIO_RX_OK, req, 14, 9500000);
	rsp_ok = true;
	CHECK(rsp_calls == rsp_before + 1);
	CHECK(radio_ch == 38);
	CHECK(txrx_calls == txrx_before + 1);
	/* a refused response on the last channel ends the event */
	ll_adv_radio_evt(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	CHECK(radio_ch == 39);
	sc_before = sched_calls;
	rsp_ok = false;
	ll_adv_radio_evt(LL_RADIO_RX_OK, req, 14, 9600000);
	rsp_ok = true;
	CHECK(sched_calls == sc_before + 1 && sched_cb != NULL);
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);

	test_while_connected();
	test_bumped_goes_to_gap();
	test_sliced();
	DONE();
}
