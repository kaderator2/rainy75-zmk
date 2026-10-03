/*
 * Custom mcumgr group for power counters of the open BLE controller.
 *
 * Group ID 66 (MGMT_GROUP_ID_PERUSER + 2), 1 command:
 *   0: stats (read) -> {"rc":0, "up": uptime ms, "idle": idle-thread ms,
 *                       "plan": uint, "listen": uint,
 *                       "skip": uint, "kick": uint, "ev": uint, "miss": uint,
 *                       "wake": uint, "mv": uint (battery, 0 if unavailable)}
 *   All counters are cumulative since boot and uint32 (wrap after 49 days).
 *   plan/listen/skip/kick/ev/miss come from ll_conn_get_stats (ev = events
 *   issued to the radio, miss = events without a valid packet), wake counts
 *   controller-thread passes. With peripheral latency active and idle, skip
 *   grows much faster than listen.
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
#include "ll_conn.h"

LOG_MODULE_REGISTER(openll_mgmt, LOG_LEVEL_INF);

#define OPENLL_MGMT_GROUP_ID  (MGMT_GROUP_ID_PERUSER + 2)
#define OPENLL_MGMT_ID_STATS  0

#if DT_HAS_CHOSEN(zmk_battery)
#define BATT_NODE DT_CHOSEN(zmk_battery)
#endif

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

static int openll_mgmt_stats(struct smp_streamer *ctxt)
{
	zcbor_state_t *zse = ctxt->writer->zs;
	struct ll_conn_stats s;

	ll_conn_get_stats(&s);

	bool ok = zcbor_tstr_put_lit(zse, "rc") && zcbor_int32_put(zse, 0) &&
		  zcbor_tstr_put_lit(zse, "up") &&
		  zcbor_uint32_put(zse, (uint32_t)k_uptime_get()) &&
#ifdef CONFIG_SCHED_THREAD_USAGE_ALL
		  zcbor_tstr_put_lit(zse, "idle") && zcbor_uint32_put(zse, idle_ms()) &&
#endif
		  zcbor_tstr_put_lit(zse, "plan") && zcbor_uint32_put(zse, s.planned) &&
		  zcbor_tstr_put_lit(zse, "listen") && zcbor_uint32_put(zse, s.listened) &&
		  zcbor_tstr_put_lit(zse, "skip") && zcbor_uint32_put(zse, s.skipped) &&
		  zcbor_tstr_put_lit(zse, "kick") && zcbor_uint32_put(zse, s.kicks) &&
		  zcbor_tstr_put_lit(zse, "ev") && zcbor_uint32_put(zse, s.events) &&
		  zcbor_tstr_put_lit(zse, "miss") && zcbor_uint32_put(zse, s.missed) &&
		  zcbor_tstr_put_lit(zse, "wake") &&
		  zcbor_uint32_put(zse, b91_bt_controller_wakeups()) &&
		  zcbor_tstr_put_lit(zse, "mv") && zcbor_uint32_put(zse, battery_mv());
	return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

static const struct mgmt_handler openll_mgmt_handlers[] = {
	[OPENLL_MGMT_ID_STATS] = { .mh_read = openll_mgmt_stats, .mh_write = NULL },
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
