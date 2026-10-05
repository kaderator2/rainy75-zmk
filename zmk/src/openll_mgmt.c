/*
 * Custom mcumgr group for power counters of the open BLE controller.
 *
 * Group ID 66 (MGMT_GROUP_ID_PERUSER + 2), 2 commands:
 *   0: stats (read) -> {"rc":0, "up": uptime ms, "idle": idle-thread ms,
 *                       "plan": uint, "listen": uint,
 *                       "skip": uint, "kick": uint, "ev": uint, "miss": uint,
 *                       "wake": uint, "mv": uint (battery, 0 if unavailable),
 *                       "links": uint (links up),
 *                       "link": [[up 0/1, listen, skip, coll, miss], ...]
 *                       (one positional list per link id),
 *                       "adv": {"ev": uint, "slid": uint, "drop": uint,
 *                               "cut": uint, "stuck": uint},
 *                       "flash": {"win": uint, "wait": uint, "force": uint,
 *                                 "wmax": uint, "hmax": uint, "pause": uint,
 *                                 "cut": uint, "fkick": uint,
 *                                 "abort": uint, "pskip": uint}}
 *   1: arbiter (read) -> {"rc":0, "arb": [[gmax, gus, gx, elen, clip,
 *                         lost x LL_ARB_PRIOS], ...] (one list per link id)}
 *   All counters are cumulative since boot and uint32 (wrap after 49 days).
 *   plan/listen/skip/kick/ev/miss are sums over all links of ll_conn_get_stats
 *   (ev = events issued to the radio, miss = events without any CRC-valid
 *   packet plus late alarms), wake counts controller-thread passes. With
 *   peripheral latency active and idle, skip grows much faster than listen.
 *   "link" and "arb" have one list per link id 0..
 *   CONFIG_BT_HCI_B91_OPENLL_MAX_CONN-1 (coll: events yielded to the
 *   arbiter; gmax / gus / gx: longest listen gap in events / us / events
 *   beyond latency + 1, elen: longest event in us, all maxima since boot;
 *   clip: starts with a clipped cap; lost: events given up to a winner of
 *   arbiter priority ADV, IDLE, ACTIVE, STARVING, SUPERVISION, MUST, see
 *   ll_arb.h). Positional lists keep the replies within one mcumgr buffer
 *   for every MAX_CONN (build-time bound below); firmware before the
 *   multi-host fix sent "link" as maps with named keys and has no
 *   command 1. "adv" are the advertising
 *   arbitration counters (ll_adv_get_stats); "flash" the flash window
 *   (ll_flash.h: windows, waits for the links, forced opens, longest wait
 *   and longest window in us, connection events paused / cut / pulled in,
 *   radio aborts) and the RX DMA
 *   ring overruns (pskip, ll_radio_stats.rx_ptr_skip).
 *
 * "idle" is the CPU idle time (k_thread_runtime_stats_all_get); needs
 * CONFIG_THREAD_RUNTIME_STATS (about +290 B ROM, +240 B RAM, set in
 * conf/openll.conf) and is left out of the reply without it.
 *
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/handlers.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zcbor_common.h>
#include <zcbor_encode.h>
#include <zephyr/logging/log.h>

#include "b91_bt.h"
#include "ll_adv.h"
#include "ll_arb.h"
#include "ll_conn.h"
#include "ll_defs.h"
#include "ll_flash.h"

LOG_MODULE_REGISTER(openll_mgmt, LOG_LEVEL_INF);

#define OPENLL_MGMT_GROUP_ID  (MGMT_GROUP_ID_PERUSER + 2)
#define OPENLL_MGMT_ID_STATS  0
#define OPENLL_MGMT_ID_ARB    1

#if DT_HAS_CHOSEN(zmk_battery)
#define BATT_NODE DT_CHOSEN(zmk_battery)
#endif

/*
 * The battery sensor is shared with ZMK's own battery reporting. There is no
 * lock around the sample: a rare overlap with a ZMK read would at worst yield
 * one odd millivolt value in a diagnostic reply, never a stuck ADC. Since the
 * driver's voltage / state-of-charge fields are shared too, such an overlap
 * can also overwrite them under ZMK's battery work, so ZMK's next reported
 * battery level (BLE battery service) may be odd once as well.
 */
static uint32_t battery_mv(void)
{
#ifdef BATT_NODE
	const struct device *dev = DEVICE_DT_GET(BATT_NODE);
	struct sensor_value v;

	if (!device_is_ready(dev) ||
	    sensor_sample_fetch_chan(dev, SENSOR_CHAN_GAUGE_VOLTAGE) != 0 ||
	    sensor_channel_get(dev, SENSOR_CHAN_GAUGE_VOLTAGE, &v) != 0) {
		return 0;
	}
	return (uint32_t)(v.val1 * 1000 + v.val2 / 1000);
#else
	return 0;
#endif
}

#ifdef CONFIG_SCHED_THREAD_USAGE_ALL
static uint32_t idle_ms(void)
{
	k_thread_runtime_stats_t rs;

	if (k_thread_runtime_stats_all_get(&rs) != 0) {
		return 0;
	}
	return (uint32_t)(rs.idle_cycles * 1000ULL / sys_clock_hw_cycles_per_sec());
}
#endif

/*
 * Reply layout and its worst-case size, from one list of keys per part
 * (X-macros), so the encoder and the build-time bound cannot drift apart.
 * zcbor encodes a container as indefinite length (start byte + break byte,
 * ZCBOR_CANONICAL is off), a key of fewer than 24 characters as 1 + length
 * bytes (sizeof of the literal), a uint32 in at most 5 bytes.
 */
#define CB_UINT_MAX 5u
#define CB_CONT     2u
#define SZ_KV(k, v) + (sizeof(k) + CB_UINT_MAX)
#define SZ_V(v)     + CB_UINT_MAX
#define PUT_KV(k, v) && zcbor_tstr_put_lit(zse, k) && zcbor_uint32_put(zse, (v))
#define PUT_V(v)     && zcbor_uint32_put(zse, (v))

#define STATS_TOP(X)                                                                   \
	X("up", (uint32_t)k_uptime_get())                                              \
	STATS_IDLE(X)                                                                  \
	X("plan", s.planned) X("listen", s.listened) X("skip", s.skipped)              \
	X("kick", s.kicks) X("ev", s.events) X("miss", s.missed)                       \
	X("wake", b91_bt_controller_wakeups()) X("mv", battery_mv()) X("links", up)
#ifdef CONFIG_SCHED_THREAD_USAGE_ALL
#define STATS_IDLE(X) X("idle", idle_ms())
#else
#define STATS_IDLE(X)
#endif
/* per link id: [up, listen, skip, coll, miss] */
#define STATS_LINK(X)                                                                  \
	X(ll_conn_active(i) ? 1u : 0u) X(l.listened) X(l.skipped) X(l.collisions)      \
	X(l.missed)
#define STATS_ADV(X)                                                                   \
	X("ev", as.events) X("slid", as.slid) X("drop", as.dropped) X("cut", as.cut)   \
	X("stuck", as.stuck)
#define STATS_FLASH(X)                                                                 \
	X("win", fs.windows) X("wait", fs.waits) X("force", fs.forced)                 \
	X("wmax", fs.wait_max_us) X("hmax", fs.hold_max_us) X("pause", s.flash_paused) \
	X("cut", s.flash_cut) X("fkick", s.flash_kicks) X("abort", rs.flash_aborts)    \
	X("pskip", rs.rx_ptr_skip)
/* command 1, per link id: [gmax, gus, gx, elen, clip, lost[0..LL_ARB_PRIOS-1]] */
#define ARB_LINK(X)                                                                    \
	X(l.gap_max) X(l.gap_max_us) X(l.gap_excess_max) X(l.ev_len_max_us) X(a.clipped)

/* the SMP header (struct smp_hdr, internal to Zephyr's mcumgr) */
#define SMP_HDR_SZ 8u
/* the reply map is opened and closed by the SMP core */
#define RC_SZ (CB_CONT + sizeof("rc") + 1u)
#define STATS_MAX_SZ                                                                   \
	(RC_SZ STATS_TOP(SZ_KV) + sizeof("link") + CB_CONT +                           \
	 LL_MAX_CONN * (CB_CONT STATS_LINK(SZ_V)) + sizeof("adv") + CB_CONT STATS_ADV(SZ_KV) + \
	 sizeof("flash") + CB_CONT STATS_FLASH(SZ_KV))
#define ARB_MAX_SZ                                                                     \
	(RC_SZ + sizeof("arb") + CB_CONT +                                             \
	 LL_MAX_CONN * (CB_CONT ARB_LINK(SZ_V) + LL_ARB_PRIOS * CB_UINT_MAX))
/* Both must fit one mcumgr buffer with the SMP header, for every
 * BT_HCI_B91_OPENLL_MAX_CONN (1..5) with every counter at UINT32_MAX (the
 * per-link maps with named keys of the first multilink version did not:
 * up to 951 bytes at MAX_CONN 5). */
BUILD_ASSERT(STATS_MAX_SZ + SMP_HDR_SZ <= CONFIG_MCUMGR_TRANSPORT_NETBUF_SIZE,
	     "group 66 stats reply may exceed the mcumgr buffer");
BUILD_ASSERT(ARB_MAX_SZ + SMP_HDR_SZ <= CONFIG_MCUMGR_TRANSPORT_NETBUF_SIZE,
	     "group 66 arbiter reply may exceed the mcumgr buffer");

static int openll_mgmt_stats(struct smp_streamer *ctxt)
{
	zcbor_state_t *zse = ctxt->writer->zs;
	struct ll_conn_stats s;
	struct ll_adv_stats as;
	struct ll_flash_stats fs;
	struct ll_radio_stats rs;
	uint32_t up = 0;
	bool ok;

	ll_conn_get_stats_total(&s);   /* aggregates: sums over the links */
	ll_adv_get_stats(&as);
	/* flash window (ll_flash.h) and the RX ring overrun it prevents */
	ll_flash_get_stats(&fs);
	ll_radio_get_stats(&rs);
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		up += ll_conn_active(i) ? 1 : 0;
	}

	ok = zcbor_tstr_put_lit(zse, "rc") && zcbor_int32_put(zse, 0)
	     STATS_TOP(PUT_KV) &&
	     zcbor_tstr_put_lit(zse, "link") && zcbor_list_start_encode(zse, LL_MAX_CONN);
	for (uint8_t i = 0; ok && i < LL_MAX_CONN; i++) {
		struct ll_conn_stats l;

		ll_conn_get_stats(i, &l);
		ok = zcbor_list_start_encode(zse, 5) STATS_LINK(PUT_V) &&
		     zcbor_list_end_encode(zse, 5);
	}
	ok = ok && zcbor_list_end_encode(zse, LL_MAX_CONN) &&
	     zcbor_tstr_put_lit(zse, "adv") && zcbor_map_start_encode(zse, 5)
	     STATS_ADV(PUT_KV) && zcbor_map_end_encode(zse, 5) &&
	     zcbor_tstr_put_lit(zse, "flash") && zcbor_map_start_encode(zse, 10)
	     STATS_FLASH(PUT_KV) && zcbor_map_end_encode(zse, 10);
	return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

static int openll_mgmt_arb(struct smp_streamer *ctxt)
{
	zcbor_state_t *zse = ctxt->writer->zs;
	bool ok = zcbor_tstr_put_lit(zse, "rc") && zcbor_int32_put(zse, 0) &&
		  zcbor_tstr_put_lit(zse, "arb") && zcbor_list_start_encode(zse, LL_MAX_CONN);

	for (uint8_t i = 0; ok && i < LL_MAX_CONN; i++) {
		struct ll_conn_stats l;
		struct ll_arb_stats a;

		ll_conn_get_stats(i, &l);
		ll_arb_get_stats(i, &a);
		ok = zcbor_list_start_encode(zse, 5 + LL_ARB_PRIOS) ARB_LINK(PUT_V);
		for (uint8_t k = 0; ok && k < LL_ARB_PRIOS; k++) {
			ok = zcbor_uint32_put(zse, a.lost[k]);
		}
		ok = ok && zcbor_list_end_encode(zse, 5 + LL_ARB_PRIOS);
	}
	ok = ok && zcbor_list_end_encode(zse, LL_MAX_CONN);
	return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

static const struct mgmt_handler openll_mgmt_handlers[] = {
	[OPENLL_MGMT_ID_STATS] = { .mh_read = openll_mgmt_stats, .mh_write = NULL },
	[OPENLL_MGMT_ID_ARB] = { .mh_read = openll_mgmt_arb, .mh_write = NULL },
};

static struct mgmt_group openll_mgmt_group = {
	.mg_handlers = openll_mgmt_handlers,
	.mg_handlers_count = ARRAY_SIZE(openll_mgmt_handlers),
	.mg_group_id = OPENLL_MGMT_GROUP_ID,
};

static void openll_mgmt_register(void)
{
	mgmt_register_group(&openll_mgmt_group);
	LOG_INF("openll_mgmt registered (group %d)", OPENLL_MGMT_GROUP_ID);
}

MCUMGR_HANDLER_DEFINE(openll_mgmt, openll_mgmt_register);
