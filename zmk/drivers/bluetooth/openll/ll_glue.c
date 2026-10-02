/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Open link layer behind the b91_bt.h seam: hci_b91.c sends HCI packets in
 * through b91_bt_host_send_packet(); events go out through a message queue
 * drained by the controller thread, so the host always receives events in
 * thread context, as with the blob shim.
 */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include "b91_bt.h"
#include "b91_mac.h"
#include "trng.h"
#include "ll_adv.h"
#include "ll_hci.h"
#include "ll_plat.h"
#include "ll_radio.h"
#include "ll_sched.h"

LOG_MODULE_REGISTER(openll, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);

struct evt_item {
	uint16_t len;
	uint8_t data[LL_HCI_EVT_MAX];
};

K_MSGQ_DEFINE(evt_q, sizeof(struct evt_item), 8, 4);
K_MSGQ_DEFINE(conn_q, sizeof(struct ll_connect_ind), 2, 4);

static K_THREAD_STACK_DEFINE(ctrl_stack, CONFIG_BT_HCI_B91_RX_STACK_SIZE);
static struct k_thread ctrl_thread;
static b91_bt_host_callback_t host_cb;
static uint8_t bd_addr[6];

uint32_t ll_plat_rand32(void)
{
	return trng_rand();
}

unsigned int ll_plat_lock(void)
{
	return irq_lock();
}

void ll_plat_unlock(unsigned int key)
{
	irq_unlock(key);
}

static void rand_bytes(uint8_t *out, uint8_t len)
{
	uint32_t r = 0;

	for (uint8_t i = 0; i < len; i++) {
		if ((i & 3) == 0) {
			r = trng_rand();
		}
		out[i] = (uint8_t)(r >> (8 * (i & 3)));
	}
}

static void mac_rand(int len, unsigned char *data)
{
	rand_bytes(data, (uint8_t)len);
}

static void get_bd_addr(uint8_t addr[6])
{
	memcpy(addr, bd_addr, 6);
}

static void unknown_opcode(uint16_t op)
{
	static uint16_t seen[32];
	static uint8_t n;

	for (uint8_t i = 0; i < n; i++) {
		if (seen[i] == op) {
			return;
		}
	}
	if (n < ARRAY_SIZE(seen)) {
		seen[n++] = op;
		LOG_WRN("unsupported HCI opcode 0x%04x", op);
		return;
	}
	/* table full: still report it, but at DBG so a device that sends many
	 * distinct unsupported opcodes cannot spam the log at WRN forever */
	LOG_DBG("unsupported HCI opcode 0x%04x (opcode table full)", op);
}

static void evt_sink(const uint8_t *h4, uint16_t len)
{
	struct evt_item it;

	if (len > LL_HCI_EVT_MAX) {
		LOG_ERR("event too large (%u bytes), dropped", len);
		return;
	}
	it.len = len;
	memcpy(it.data, h4, len);
	if (k_msgq_put(&evt_q, &it, K_NO_WAIT) != 0) {
		LOG_ERR("event queue full, dropped event 0x%02x", h4[1]);
	}
}

static atomic_t conn_drops;

static void on_connect_ind(const struct ll_connect_ind *ci)
{
	/* ISR context: log later, from the controller thread */
	if (k_msgq_put(&conn_q, ci, K_NO_WAIT) != 0) {
		atomic_inc(&conn_drops);
	}
}

/* A bonded central retries CONNECT_IND many times per second while slice 1
 * has no connection state; dumping every one floods the log (dropped lines,
 * NMP timeouts on the shared CDC ACM port). Dump the first one, then at most
 * one every CONNECT_IND_LOG_MS, with the count received since the last dump. */
#define CONNECT_IND_LOG_MS 10000

static void log_connect_ind(const struct ll_connect_ind *ci)
{
	static bool dumped;
	static int64_t next_dump;
	static uint32_t since_dump;
	int64_t now = k_uptime_get();

	since_dump++;
	if (dumped && now < next_dump) {
		return;
	}
	dumped = true;
	next_dump = now + CONNECT_IND_LOG_MS;

	LOG_INF("CONNECT_IND from %02x:%02x:%02x:%02x:%02x:%02x (%s), %u since last dump",
		ci->init_a[5], ci->init_a[4], ci->init_a[3], ci->init_a[2],
		ci->init_a[1], ci->init_a[0], ci->init_addr_random ? "random" : "public",
		since_dump);
	since_dump = 0;
	LOG_INF("  AA 0x%08x CRCInit 0x%06x WinSize %u WinOffset %u",
		ci->aa, ci->crc_init, ci->win_size, ci->win_offset);
	LOG_INF("  Interval %u Latency %u Timeout %u Hop %u SCA %u ChSel %u",
		ci->interval, ci->latency, ci->timeout, ci->hop, ci->sca, ci->chsel);
	LOG_INF("  ChM %02x %02x %02x %02x %02x",
		ci->chm[0], ci->chm[1], ci->chm[2], ci->chm[3], ci->chm[4]);
}

static const struct ll_hci_ops hci_ops = {
	.get_bd_addr = get_bd_addr,
	.rand = rand_bytes,
	.reset = ll_adv_reset,
	.adv_set_params = ll_adv_set_params,
	.adv_set_data = ll_adv_set_data,
	.adv_set_scan_rsp = ll_adv_set_scan_rsp,
	.adv_enable = ll_adv_enable,
	.unknown = unknown_opcode,
};

/* Periodic radio health log: one line every 2 s while there is anything to
 * report, plus a stall warning if advertising is enabled but tx2rx is not
 * advancing. Helps spot a wedged radio from the log alone on first bring-up. */
static void report_radio_stats(struct ll_radio_stats *last)
{
	struct ll_radio_stats st;

	ll_radio_get_stats(&st);
	if (memcmp(&st, last, sizeof(st)) != 0 || ll_adv_is_enabled()) {
		LOG_INF("radio: tx2rx %u rx_ok %u crc %u timeout %u rsp %u rsp_late %u",
			st.tx2rx, st.rx_ok, st.rx_crc, st.rx_timeout, st.rsp_tx, st.rsp_late);
	}
	if (ll_adv_is_enabled() && st.tx2rx == last->tx2rx) {
		LOG_WRN("radio stalled");
	}
	*last = st;

	uint32_t drops = (uint32_t)atomic_clear(&conn_drops);

	if (drops != 0) {
		LOG_WRN("dropped %u CONNECT_IND event(s) (queue full)", drops);
	}
}

static void ctrl_thread_fn(void *p1, void *p2, void *p3)
{
	struct evt_item it;
	struct ll_connect_ind ci;
	int64_t next_stats = 0;
	struct ll_radio_stats last_stats = {0};

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		if (k_msgq_get(&evt_q, &it, K_MSEC(100)) == 0 && host_cb.host_read_packet) {
			host_cb.host_read_packet(it.data, it.len);
		}
		while (k_msgq_get(&conn_q, &ci, K_NO_WAIT) == 0) {
			log_connect_ind(&ci);
		}

		int64_t now = k_uptime_get();

		if (now >= next_stats) {
			next_stats = now + 2000;
			report_radio_stats(&last_stats);
		}
	}
}

int b91_bt_controller_init(void)
{
	uint8_t mac_random_static[6];

	trng_init();
	b91_mac_init(B91_MAC_FLASH_ADDR, mac_rand, bd_addr, mac_random_static);
	ll_radio_init(ll_adv_radio_evt);
	ll_sched_init();
	ll_adv_init(bd_addr, on_connect_ind);
	ll_hci_init(&hci_ops, evt_sink);

	k_thread_create(&ctrl_thread, ctrl_stack, K_THREAD_STACK_SIZEOF(ctrl_stack),
			ctrl_thread_fn, NULL, NULL, NULL,
			CONFIG_BT_HCI_B91_RX_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&ctrl_thread, "openll");
	LOG_INF("open link layer up, BD_ADDR %02x:%02x:%02x:%02x:%02x:%02x",
		bd_addr[5], bd_addr[4], bd_addr[3], bd_addr[2], bd_addr[1], bd_addr[0]);
	return 0;
}

void b91_bt_host_send_packet(uint8_t type, uint8_t *data, uint16_t len)
{
	if (type == 0x01) {
		ll_hci_cmd(data, len);
	} else {
		LOG_WRN("dropping H4 type 0x%02x (%u bytes): no connections in slice 1",
			type, len);
	}
}

void b91_bt_host_callback_register(const b91_bt_host_callback_t *cb)
{
	host_cb = *cb;
}
