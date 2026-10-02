#include <string.h>
#include "test.h"
#include "../ll_adv.h"
#include "../ll_defs.h"
#include "../ll_sched.h"
#include "../ll_plat.h"

/* ---- fakes ---- */
static uint32_t now_tick = 1000000;
static uint32_t sched_tick; static ll_sched_cb_t sched_cb; static int sched_cancels;
static uint8_t radio_ch; static uint8_t tx_pdu[64]; static uint8_t tx_len;
static uint32_t tx_start; static int txrx_calls;
static uint8_t rsp_pdu[64]; static uint8_t rsp_len; static uint32_t rsp_tick; static int rsp_calls;
static int radio_stops;
static int conn_calls; static struct ll_connect_ind conn;

int ll_radio_init(ll_radio_cb_t cb) { (void)cb; return 0; }
uint32_t ll_radio_now(void) { return now_tick; }
void ll_radio_set_adv_channel(uint8_t ch) { radio_ch = ch; }
void ll_radio_tx_then_rx(const uint8_t *p, uint8_t l, uint32_t t, uint32_t w)
{ (void)w; memcpy(tx_pdu, p, l); tx_len = l; tx_start = t; txrx_calls++; }
void ll_radio_prepare_rsp(const uint8_t *p, uint8_t l) { memcpy(rsp_pdu, p, l); rsp_len = l; }
void ll_radio_tx_rsp_at(uint32_t t) { rsp_tick = t; rsp_calls++; }
void ll_radio_stop(void) { radio_stops++; }
void ll_sched_init(void) {}
void ll_sched_at(uint32_t t, ll_sched_cb_t cb) { sched_tick = t; sched_cb = cb; }
void ll_sched_cancel(void) { sched_cancels++; sched_cb = NULL; }
uint32_t ll_plat_rand32(void) { return 1234567; }
unsigned int ll_plat_lock(void) { return 0; }
void ll_plat_unlock(unsigned int k) { (void)k; }
static void on_conn(const struct ll_connect_ind *ci) { conn = *ci; conn_calls++; }

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

	/* valid ADV_IND on all channels */
	p = params(0, 7);
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);
	const uint8_t ad[3] = {0x02, 0x01, 0x06};
	CHECK(ll_adv_set_data(ad, 3) == LL_ST_SUCCESS);
	const uint8_t sr[3] = {0x02, 0x09, 'R'};
	CHECK(ll_adv_set_scan_rsp(sr, 3) == LL_ST_SUCCESS);

	CHECK(ll_adv_enable(true) == LL_ST_SUCCESS);
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

	/* CONNECT_IND for us on 39 -> callback, advertising continues next event */
	uint8_t ci_pdu[36] = {0x65, 34, 0x11, 0x12, 0x13, 0x14, 0x15, 0xD6};
	memcpy(&ci_pdu[8], adva, 6);
	ci_pdu[2 + 22] = 24;  /* interval */
	ll_adv_radio_evt(LL_RADIO_RX_OK, ci_pdu, 36, 7000000);
	CHECK(conn_calls == 1 && conn.interval == 24);
	CHECK(sched_cb != NULL);

	/* data update while enabled is used on the next TX */
	const uint8_t ad2[4] = {0x03, 0x19, 0xC1, 0x03};
	CHECK(ll_adv_set_data(ad2, 4) == LL_ST_SUCCESS);
	fire_sched();
	CHECK(tx_len == 12 && tx_pdu[1] == 10 && tx_pdu[8] == 0x03);

	/* disable: cancel + stop, later radio events ignored */
	int before = txrx_calls;
	CHECK(ll_adv_enable(false) == LL_ST_SUCCESS);
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

	/* reset disables and restores defaults */
	ll_adv_reset();
	p = params(0, 7);
	CHECK(ll_adv_set_params(&p) == LL_ST_SUCCESS);

	DONE();
}
