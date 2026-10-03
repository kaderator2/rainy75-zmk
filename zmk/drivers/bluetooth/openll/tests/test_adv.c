#include <errno.h>
#include <string.h>
#include "test.h"
#include "../ll_adv.h"
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
int ll_conn_start(const struct ll_connect_ind *ci, uint32_t t)
{
	start_calls++; start_ci = *ci; start_tick = t;
	/* advertising must already be stopped when the connection starts */
	start_saw_enabled = ll_adv_is_enabled(); start_saw_sched = sched_cb != NULL;
	start_saw_stops = radio_stops;
	if (start_ret >= 0) conn_is_active = true;
	return start_ret;
}
/* links taken (active or awaiting release) */
uint8_t ll_conn_count(void) { return conn_is_active ? 1 : 0; }
void ll_radio_adv_restore(void) { restores++; }

static const uint8_t adva[6] = {0x01, 0x02, 0x03, 0x38, 0xC1, 0xA4};

static struct ll_adv_params params(uint8_t type, uint8_t map)
{
	struct ll_adv_params p = {.interval_min = 0x00A0, .interval_max = 0x00F0,
				  .type = type, .chan_map = map};
	return p;
}

static void fire_sched(void) { ll_sched_cb_t cb = sched_cb; sched_cb = NULL; CHECK(cb != NULL); if (cb) cb(); }

int main(void)
{
	struct ll_adv_params p;

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
	CHECK(tx_len == 11 && tx_pdu[0] == LL_PDU_ADV_IND && tx_pdu[1] == 9);
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
	CHECK(sched_cancels >= 1 && radio_stops >= 1);
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
	/* the host may not re-enable advertising during the connection;
	 * disable is a harmless no-op */
	CHECK(ll_adv_enable(true) == LL_ST_DISALLOWED);
	CHECK(!ll_adv_is_enabled() && sched_cb == NULL && restores == 0);
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
	/* parameters and data may still be changed (host prepares resume) */
	p = params(0, 7);
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	/* after the disconnect the host re-enables advertising via HCI: the
	 * radio is restored once (baseband left connection mode), then events run */
	conn_is_active = false;
	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
	CHECK(restores == 1 && ll_adv_is_enabled() && sched_cb != NULL);
	fire_sched();
	CHECK(radio_ch == 37 && tx_pdu[0] == LL_PDU_ADV_IND);
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

	DONE();
}
