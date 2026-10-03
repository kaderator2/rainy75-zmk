/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Open link layer behind the b91_bt.h seam: hci_b91.c sends HCI packets in
 * through b91_bt_host_send_packet(); everything toward the host (events,
 * ACL data) leaves from the controller thread, as with the blob shim.
 *
 * Execution contexts (spec "Execution contexts"):
 * - ISR (radio + stimer): ll_adv, ll_conn, ll_txq, ll_rxq producer. The
 *   ll_conn event callback and the ll_txq completion callback only record
 *   what happened (pending bits, counters) and wake the controller thread.
 * - Controller thread: consumes ll_rxq (LLCP to ll_llcp, ACL to the host),
 *   sends host ACL through ll_llcp_tx() (the single encrypt+push point),
 *   emits LE Connection Complete / Connection Update Complete / Number Of
 *   Completed Packets / Disconnection Complete, and on a disconnect resets
 *   ll_rxq and ll_llcp in thread context, before advertising can be enabled
 *   again (adv enable is refused while that reset is pending).
 * - HCI thread (host TX): HCI commands (ll_hci) and host ACL, which is only
 *   parsed and queued here, so encryption and the TX packet counter run in
 *   one thread for data (LTK reply / Disconnect queue control PDUs from the
 *   HCI thread under ll_plat_tx_lock(), see ll_llcp.h).
 *
 * The controller thread sleeps until something happens (K_FOREVER): the
 * radio/stimer ISRs, the HCI thread and two timers wake it. The LLCP
 * response timer (llcp_tmr) runs only while an LL control procedure is
 * pending; the stats timer only with CONFIG_BT_HCI_B91_OPENLL_STATS_LOG.
 * Host ACL held back (-ENOMEM: TX backlog full; -EAGAIN: encryption start
 * pauses data) is retried on the wakeup that frees it: an ll_txq ack
 * (txq_done), the end of the procedure (LL_START_ENC_RSP received here,
 * the host's LTK negative reply, which wakes it) or the disconnect.
 *
 * Toward the host, the controller thread delivers directly (after draining
 * the event queue first, so the order of everything it produced is kept);
 * only events produced in other contexts (HCI command completions from the
 * HCI thread) go through evt_q.
 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include "b91_bt.h"
#include "b91_mac.h"
#include "aes.h"
#include "trng.h"
#include "ll_adv.h"
#include "ll_conn.h"
#include "ll_crypt.h"
#include "ll_hci.h"
#include "ll_llcp.h"
#include "ll_plat.h"
#include "ll_radio.h"
#include "ll_rxq.h"
#include "ll_sched.h"
#include "ll_txq.h"

LOG_MODULE_REGISTER(openll, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);

#define STATS_PERIOD_MS  2000  /* CONFIG_BT_HCI_B91_OPENLL_STATS_LOG */
#define RX_BUDGET        16    /* PDUs per wakeup (= ll_rxq ring depth) */
#define RESET_WAIT_MS    200   /* HCI Reset: wait for the connection to end */
/* Consecutive guard-ended events without any CRC-valid packet after which
 * the radio is considered wedged: the connection is ended with a
 * supervision timeout (0x08); the next advertising enable does the baseband
 * reset (ll_radio_adv_restore). A guard that cuts a long central MD burst
 * (packets received; the cap is interval based, see LL_CONN_EVENT_SAFETY_US)
 * resets the streak, so a healthy link is never ended by it. */
#define GUARD_STREAK_MAX 3

struct evt_item {
	uint16_t len;
	uint8_t data[LL_HCI_EVT_MAX];
};

/* An H4 ACL packet fits an item too (not queued today, see file header). */
BUILD_ASSERT(LL_HCI_ACL_MAX <= LL_HCI_EVT_MAX);

/* Only the HCI thread queues here (command completions, one outstanding
 * command at a time with Zephyr), so a small depth suffices. */
K_MSGQ_DEFINE(evt_q, sizeof(struct evt_item), 8, 4);
K_MSGQ_DEFINE(conn_q, sizeof(struct ll_connect_ind), 2, 4);

/* Host ACL waiting for the controller thread. The host holds at most
 * LL_ACL_NUM unacknowledged packets (LE Read Buffer Size), one of which may
 * be held by the thread itself. gen: connection generation at queue time. */
struct acl_item {
	uint32_t gen;
	struct ll_hci_acl_pdu pdu;
};
K_MSGQ_DEFINE(acl_q, sizeof(struct acl_item), LL_ACL_NUM + 1, 4);

static K_SEM_DEFINE(wake, 0, 1);

/* LLCP procedure response timeout (40 s, ll_llcp_tick): armed by the
 * controller thread from ll_llcp_timeout_ticks() only while a procedure is
 * pending; the expiry only wakes the controller thread. */
static void llcp_tmr_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	k_sem_give(&wake);
}
static K_WORK_DELAYABLE_DEFINE(llcp_tmr, llcp_tmr_fn);
static bool llcp_tmr_armed;   /* controller thread only */

/* Periodic stats line (CONFIG_BT_HCI_B91_OPENLL_STATS_LOG only). */
static atomic_t stats_due;
static void stats_tmr_fn(struct k_timer *t)
{
	ARG_UNUSED(t);
	atomic_set(&stats_due, 1);
	k_sem_give(&wake);
}
static K_TIMER_DEFINE(stats_tmr, stats_tmr_fn, NULL);
static K_SEM_DEFINE(reset_done, 0, 1);

static K_THREAD_STACK_DEFINE(ctrl_stack, CONFIG_BT_HCI_B91_RX_STACK_SIZE);
static struct k_thread ctrl_thread;
static b91_bt_host_callback_t host_cb;
static uint8_t bd_addr[6];

/* ---- state shared with ISR context ---- */

enum { PEND_CONNECTED, PEND_UPDATED, PEND_DISCONNECTED };
static atomic_t pend;                    /* PEND_* bits set by the ll_conn callback */
static struct ll_connect_ind pend_ci;    /* written before the bit is set */
static struct ll_conn_params pend_params;
static uint8_t pend_reason;
static atomic_t nocp_pending;            /* acked ACL PDUs not yet reported */
static atomic_t conn_gen;                /* incremented per handled CONNECTED */
static volatile bool conn_up;            /* Connection Complete sent, no Disconnection Complete yet */
static volatile bool silent_end;         /* HCI Reset: end without Disconnection Complete */

/* ---- controller thread state ---- */

static struct acl_item held;             /* host ACL waiting for ll_llcp_tx() */
static bool held_valid;
static bool mic_failed;                  /* MIC failure logged for this connection */

/* ---- counters ---- */

static atomic_t cnt_tx_acked, cnt_acl_in, cnt_acl_out, cnt_acl_drop, cnt_evt_drop, conn_drops;
static atomic_t cnt_guard_escalations;
static uint32_t lock_depth, lock_t0, lock_max_ticks, acl_tx_lock_max_ticks, aes_max_ticks;
static volatile bool in_acl_tx;          /* controller thread is inside ll_llcp_tx() for ACL */
static bool aes_reversed;                /* hal AES needs reversed byte order (self-test) */

/* ---- platform hooks (ll_plat.h) ---- */

uint32_t ll_plat_rand32(void)
{
	return trng_rand();
}

/* irq_lock() nests; the hold time of the outermost lock is measured (stats
 * "lock max"), and separately while the controller thread encrypts and
 * pushes host ACL (the IRQ-lock cost of the ACL TX path: the push and the
 * per-block AES locks; encryption itself runs outside the lock). */
unsigned int ll_plat_lock(void)
{
	unsigned int key = irq_lock();

	if (lock_depth++ == 0) {
		lock_t0 = ll_radio_now();
	}
	return key;
}

void ll_plat_unlock(unsigned int key)
{
	if (lock_depth > 0 && --lock_depth == 0) {
		uint32_t d = ll_radio_now() - lock_t0;

		if (d > lock_max_ticks) {
			lock_max_ticks = d;
		}
		if (in_acl_tx && !k_is_in_isr() && d > acl_tx_lock_max_ticks) {
			acl_tx_lock_max_ticks = d;
		}
	}
	irq_unlock(key);
}

/* TX producer serialization (ll_plat.h): encrypt + push of one PDU, from
 * the controller thread or the HCI thread. k_mutex is recursive and has
 * priority inheritance. */
static K_MUTEX_DEFINE(tx_mutex);

void ll_plat_tx_lock(void)
{
	(void)k_mutex_lock(&tx_mutex, K_FOREVER);
}

void ll_plat_tx_unlock(void)
{
	(void)k_mutex_unlock(&tx_mutex);
}

static void rev16(uint8_t *dst, const uint8_t *src)
{
	for (int i = 0; i < 16; i++) {
		dst[i] = src[15 - i];
	}
}

/* FIPS-197 byte order (ll_plat.h). The hal engine has one register set and
 * a static data buffer: one block runs with interrupts locked, so no other
 * caller (controller thread decrypting, HCI thread encrypting) can
 * interleave. Whether the hal wants the bytes reversed is decided once at
 * boot by aes_selftest(). */
void ll_plat_aes_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
	uint8_t k[16], d[16], r[16];

	if (aes_reversed) {
		rev16(k, key);
		rev16(d, in);
	} else {
		memcpy(k, key, 16);
		memcpy(d, in, 16);
	}
	unsigned int lk = ll_plat_lock();
	uint32_t t0 = ll_radio_now();

	aes_encrypt(k, d, r);
	uint32_t dt = ll_radio_now() - t0;

	if (dt > aes_max_ticks) {
		aes_max_ticks = dt;
	}
	ll_plat_unlock(lk);
	if (aes_reversed) {
		rev16(out, r);
	} else {
		memcpy(out, r, 16);
	}
	/* key, input (SKD, CCM blocks) and output (session key, keystream)
	 * copies must not stay on the stack */
	ll_crypt_wipe(k, sizeof(k));
	ll_crypt_wipe(d, sizeof(d));
	ll_crypt_wipe(r, sizeof(r));
}

/* FIPS-197 Appendix C.1 (AES-128). Logged once at boot. */
static void aes_selftest(void)
{
	static const uint8_t key[16] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	};
	static const uint8_t pt[16] = {
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
		0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
	};
	static const uint8_t ct[16] = {
		0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
		0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a,
	};
	uint8_t out[16];

	aes_reversed = false;
	ll_plat_aes_ecb(key, pt, out);
	if (memcmp(out, ct, 16) == 0) {
		LOG_INF("AES self-test (FIPS-197 C.1): pass");
		return;
	}
	aes_reversed = true;
	ll_plat_aes_ecb(key, pt, out);
	if (memcmp(out, ct, 16) == 0) {
		LOG_INF("AES self-test (FIPS-197 C.1): pass (hal byte order reversed)");
		return;
	}
	aes_reversed = false;
	LOG_ERR("AES self-test (FIPS-197 C.1): FAIL, link encryption will not work");
}

/* ---- toward the host ---- */

/* Controller thread only. */
static void deliver(const uint8_t *h4, uint16_t len)
{
	if (host_cb.host_read_packet) {
		host_cb.host_read_packet((uint8_t *)h4, len);
	}
}

/* Controller thread only. */
static void drain_evt_q(void)
{
	struct evt_item it;

	while (k_msgq_get(&evt_q, &it, K_NO_WAIT) == 0) {
		deliver(it.data, it.len);
	}
}

/* ll_hci sink, any context. */
static void evt_sink(const uint8_t *h4, uint16_t len)
{
	struct evt_item it;

	if (len > LL_HCI_EVT_MAX) {
		LOG_ERR("event too large (%u bytes), dropped", len);
		return;
	}
	if (!k_is_in_isr() && k_current_get() == &ctrl_thread) {
		drain_evt_q();
		deliver(h4, len);
		return;
	}
	it.len = len;
	memcpy(it.data, h4, len);
	if (k_msgq_put(&evt_q, &it, K_NO_WAIT) != 0) {
		atomic_inc(&cnt_evt_drop);
	}
	k_sem_give(&wake);
}

/* ---- callbacks from the link layer ---- */

/* ll_conn event: ISR, or thread with ll_plat_lock() held. Record only. */
static void conn_evt(enum ll_conn_evt what, const void *arg)
{
	switch (what) {
	case LL_CONN_EVT_CONNECTED:
		pend_ci = *(const struct ll_connect_ind *)arg;
		atomic_set_bit(&pend, PEND_CONNECTED);
		break;
	case LL_CONN_EVT_UPDATED:
		pend_params = *(const struct ll_conn_params *)arg;
		atomic_set_bit(&pend, PEND_UPDATED);
		break;
	case LL_CONN_EVT_DISCONNECTED:
		pend_reason = *(const uint8_t *)arg;
		atomic_set_bit(&pend, PEND_DISCONNECTED);
		break;
	}
	k_sem_give(&wake);
}

/* ll_txq completion (ISR), forwarded by ll_conn. */
static void txq_done(enum ll_txq_kind kind, uint8_t ctrl_opcode)
{
	ARG_UNUSED(ctrl_opcode);
	if (kind == LL_TXQ_ACL) {
		atomic_inc(&nocp_pending);
	}
	if (kind != LL_TXQ_EMPTY) {
		atomic_inc(&cnt_tx_acked);
	}
	k_sem_give(&wake);
}

/* One dispatcher for both users of the radio: ll_adv ignores everything
 * while advertising is disabled, ll_conn everything outside a connection
 * event. */
static void radio_evt(enum ll_radio_evt evt, const uint8_t *pdu, uint8_t len, uint32_t tick)
{
	ll_adv_radio_evt(evt, pdu, len, tick);
	ll_conn_radio_evt(evt, pdu, len, tick);
	if (evt != LL_RADIO_CONN_DONE) {
		return;
	}
	if (ll_radio_conn_guard_streak() >= GUARD_STREAK_MAX && ll_conn_active()) {
		/* the event is closed (CONN_DONE handled), so this ends now */
		atomic_inc(&cnt_guard_escalations);
		ll_conn_end(LL_ST_CONN_TIMEOUT);
	}
	if (len != 0) {
		k_sem_give(&wake);   /* ll_rxq has data */
	}
}

static const struct ll_conn_ops conn_ops = {
	.evt = conn_evt,
	.txq_done = txq_done,
	.ctrl_tx = ll_llcp_ctrl_tx,
	.busy = ll_llcp_busy,
};

static const struct ll_llcp_ops llcp_ops = {
	.ltk_req = ll_hci_evt_ltk_req,
	.enc_change = ll_hci_evt_enc_change,
};

/* ---- HCI ops ---- */

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

/* HCI Reset (HCI thread): stop advertising and end a connection silently
 * (Reset semantics: no Disconnection Complete). The controller thread does
 * the LL state reset (ll_rxq, ll_llcp, held ACL) as for any disconnect and
 * signals reset_done; the wait covers an event on air (ll_conn ends at its
 * CONN_DONE, bounded by the radio guard). */
static void hci_reset(void)
{
	ll_adv_reset();
	k_sem_reset(&reset_done);
	if (!ll_conn_active() && !conn_up && !atomic_test_bit(&pend, PEND_DISCONNECTED) &&
	    !atomic_test_bit(&pend, PEND_CONNECTED)) {
		return;
	}
	silent_end = true;
	ll_conn_end(LL_ST_LOCAL_TERM);
	k_sem_give(&wake);
	if (k_sem_take(&reset_done, K_MSEC(RESET_WAIT_MS)) != 0) {
		LOG_ERR("HCI Reset: connection did not end within %u ms", RESET_WAIT_MS);
	}
}

/* Advertising may only start again once the controller thread has reset
 * the RX queue and LLCP state of the last connection (ll_rxq_reset() must
 * not race with a new connection's ISR producer). Checked under the lock so
 * a DISCONNECTED raised by the ISR cannot slip in between. */
static uint8_t adv_enable(bool enable)
{
	unsigned int key = ll_plat_lock();
	uint8_t st;

	if (enable && atomic_test_bit(&pend, PEND_DISCONNECTED)) {
		st = LL_ST_DISALLOWED;
	} else {
		st = ll_adv_enable(enable);
	}
	ll_plat_unlock(key);
	return st;
}

/* HCI thread: the negative reply resumes paused host ACL; wake the
 * controller thread, which holds it (nothing else would). */
static uint8_t ltk_neg_reply(void)
{
	uint8_t st = ll_llcp_ltk_neg_reply();

	k_sem_give(&wake);
	return st;
}

static const struct ll_hci_ops hci_ops = {
	.get_bd_addr = get_bd_addr,
	.rand = rand_bytes,
	.reset = hci_reset,
	.adv_set_params = ll_adv_set_params,
	.adv_set_data = ll_adv_set_data,
	.adv_set_scan_rsp = ll_adv_set_scan_rsp,
	.adv_enable = adv_enable,
	.unknown = unknown_opcode,
	.disconnect = ll_llcp_terminate,
	.ltk_reply = ll_llcp_ltk_reply,
	.ltk_neg_reply = ltk_neg_reply,
};

/* ---- controller thread ---- */

static void on_connect_ind(const struct ll_connect_ind *ci)
{
	/* ISR context: log later, from the controller thread */
	if (k_msgq_put(&conn_q, ci, K_NO_WAIT) != 0) {
		atomic_inc(&conn_drops);
	}
}

/* A bonded central retries CONNECT_IND many times per second when it is
 * not answered; dumping every one floods the log (dropped lines, NMP
 * timeouts on the shared CDC ACM port). Dump the first one, then at most
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

static void handle_connected(void)
{
	struct ll_connect_ind ci;
	unsigned int key = ll_plat_lock();

	ci = pend_ci;
	atomic_clear_bit(&pend, PEND_CONNECTED);
	ll_plat_unlock(key);

	atomic_inc(&conn_gen);
	held_valid = false;
	mic_failed = false;
	/* No credit of an earlier connection may reach this one (credit_back()
	 * and handle_disconnected() already exclude it; this is the backstop). */
	key = ll_plat_lock();
	atomic_clear(&nocp_pending);
	conn_up = true;
	ll_plat_unlock(key);
	LOG_INF("connected: interval %u latency %u timeout %u", ci.interval, ci.latency,
		ci.timeout);
	if (!silent_end) {
		ll_hci_evt_conn_complete(&ci);
	}
}

static void handle_updated(void)
{
	struct ll_conn_params p;
	unsigned int key = ll_plat_lock();

	p = pend_params;
	atomic_clear_bit(&pend, PEND_UPDATED);
	ll_plat_unlock(key);
	LOG_INF("connection update: interval %u latency %u timeout %u", p.interval, p.latency,
		p.timeout);
	if (conn_up && !atomic_test_bit(&pend, PEND_DISCONNECTED)) {
		ll_hci_evt_conn_update(&p);
	}
}

/* Acked ACL PDUs -> Number Of Completed Packets (never after the
 * Disconnection Complete of the connection). */
static void flush_nocp(void)
{
	uint32_t n = (uint32_t)atomic_clear(&nocp_pending);

	if (n != 0 && conn_up) {
		ll_hci_evt_num_completed((uint16_t)n);
	}
}

/* Received PDUs: LLID 3 to ll_llcp, LLID 1/2 to the host. */
static void handle_rx(void)
{
	struct ll_rx_pdu pdu;
	uint8_t h4[LL_HCI_ACL_MAX];

	if (!conn_up) {
		return;
	}
	for (int i = 0; i < RX_BUDGET; i++) {
		if (atomic_test_bit(&pend, PEND_DISCONNECTED)) {
			return;   /* the link is gone; ll_rxq is reset below */
		}
		enum ll_rxq_result r = ll_rxq_get(&pdu);

		if (r == LL_RXQ_EMPTY) {
			return;
		}
		if (r == LL_RXQ_MIC_FAIL) {
			/* sticky in ll_rxq until the reset at the disconnect:
			 * log once, ll_conn_end() is idempotent */
			if (!mic_failed) {
				mic_failed = true;
				LOG_WRN("MIC failure, ending the connection");
			}
			ll_conn_end(LL_ST_MIC_FAILURE);
			return;
		}
		uint8_t llid = pdu.hdr0 & 0x03;

		if (llid == LL_LLID_CTRL) {
			ll_llcp_rx(pdu.data, pdu.len);
			continue;
		}
		uint16_t n = ll_hci_acl_to_host(h4, llid, pdu.data, pdu.len);

		if (n != 0) {
			drain_evt_q();
			deliver(h4, n);
			atomic_inc(&cnt_acl_out);
		}
	}
	k_sem_give(&wake);   /* budget used up: continue on the next pass */
}

/* Host ACL -> ll_llcp_tx(). -EAGAIN (encryption start pauses data) and
 * -ENOMEM (TX backlog full) keep the PDU for the next wakeup; it is never
 * counted as completed before ll_txq reports its ack. */
static void handle_acl_tx(void)
{
	while (conn_up && !atomic_test_bit(&pend, PEND_DISCONNECTED)) {
		if (!held_valid) {
			if (k_msgq_get(&acl_q, &held, K_NO_WAIT) != 0) {
				return;
			}
			if (held.gen != (uint32_t)atomic_get(&conn_gen)) {
				atomic_inc(&cnt_acl_drop);   /* queued for a previous connection */
				continue;
			}
			held_valid = true;
		}
		in_acl_tx = true;
		int r = ll_llcp_tx(LL_TXQ_ACL, held.pdu.llid, held.pdu.data, held.pdu.len);

		in_acl_tx = false;
		if (r == 0) {
			held_valid = false;
			continue;
		}
		if (r == -EAGAIN || r == -ENOMEM) {
			return;
		}
		LOG_WRN("host ACL dropped (%d)", r);
		atomic_inc(&cnt_acl_drop);
		atomic_inc(&nocp_pending);   /* the host's buffer credit comes back */
		held_valid = false;
	}
}

/* Thread-side end of a connection: ll_rxq_reset() and ll_llcp_reset() run
 * here, never in ISR context (the controller thread is the ll_rxq consumer,
 * so it cannot be inside ll_rxq_get() now). Advertising stays refused until
 * PEND_DISCONNECTED is cleared below, so no new connection can start before
 * the reset. Held and queued host ACL is dropped without Number Of
 * Completed Packets (the host frees its buffers on disconnect). */
static void handle_disconnected(void)
{
	uint8_t reason;
	bool silent, was_up;
	unsigned int key;

	/* CONNECTED and DISCONNECTED may both have been raised since the loop
	 * checked CONNECTED (e.g. 0x3E after six events while the thread was
	 * busy): report the connection first, so it is not announced to the
	 * host after its end (or never ended for it). */
	if (atomic_test_bit(&pend, PEND_CONNECTED)) {
		handle_connected();
	}
	flush_nocp();   /* acks that happened before the end */
	key = ll_plat_lock();
	reason = pend_reason;
	ll_plat_unlock(key);

	ll_rxq_reset();
	ll_llcp_reset();
	k_msgq_purge(&acl_q);
	held_valid = false;
	atomic_clear_bit(&pend, PEND_UPDATED);
	/* Under the lock, paired with credit_back(): an HCI-thread credit is
	 * either counted before this clear or sees conn_up false. */
	key = ll_plat_lock();
	atomic_clear(&nocp_pending);
	was_up = conn_up;
	conn_up = false;
	ll_plat_unlock(key);

	key = ll_plat_lock();
	silent = silent_end;
	silent_end = false;
	atomic_clear_bit(&pend, PEND_DISCONNECTED);   /* advertising may start again */
	ll_plat_unlock(key);

	LOG_INF("disconnected, reason 0x%02x%s", reason, silent ? " (HCI Reset, not reported)" : "");
	if (!silent && was_up) {
		ll_hci_evt_disconn_complete(reason);
	}
	k_sem_give(&reset_done);
}

static uint32_t ticks_to_us(uint32_t t)
{
	return t / LL_TICKS_PER_US;
}

/* Periodic health log (CONFIG_BT_HCI_B91_OPENLL_STATS_LOG): one line every
 * 2 s while there is anything to report, plus a stall warning if advertising is enabled but tx2rx is not
 * advancing. Connection counters are logged while connected or changed. */
static void report_stats(struct ll_radio_stats *last, struct ll_conn_stats *last_c)
{
	struct ll_radio_stats st;
	struct ll_conn_stats cs;

	ll_radio_get_stats(&st);
	ll_conn_get_stats(&cs);
	if (st.tx2rx != last->tx2rx || st.rx_ok != last->rx_ok || st.rx_timeout != last->rx_timeout ||
	    st.rsp_tx != last->rsp_tx || st.rsp_late != last->rsp_late || ll_adv_is_enabled()) {
		LOG_INF("radio: tx2rx %u rx_ok %u crc %u timeout %u rsp %u rsp_late %u",
			st.tx2rx, st.rx_ok, st.rx_crc, st.rx_timeout, st.rsp_tx, st.rsp_late);
	}
	if (ll_adv_is_enabled() && st.tx2rx == last->tx2rx) {
		LOG_WRN("radio stalled");
	}
	if (conn_up || memcmp(&cs, last_c, sizeof(cs)) != 0 ||
	    st.conn_events != last->conn_events) {
		unsigned int key = ll_plat_lock();
		uint32_t lock_max = lock_max_ticks, acl_lock = acl_tx_lock_max_ticks;
		uint32_t aes_max = aes_max_ticks;

		ll_plat_unlock(key);
		LOG_INF("conn: ev %u rx_ev %u miss %u late %u rx %u crc %u fto %u guard %u widen %u",
			cs.events, cs.rx_events, cs.missed, cs.late, st.conn_rx, st.rx_crc,
			st.conn_fto, st.conn_guard, cs.widen_max_us);
		LOG_INF("conn: tx %u acked %u tifs<=150 %u 151-152 %u >152 %u rxq_of %u ptr_odd %u",
			st.conn_tx, (uint32_t)atomic_get(&cnt_tx_acked), st.tifs_le150,
			st.tifs_151_152, st.tifs_gt152, ll_rxq_overflow_count(), st.rx_ptr_odd);
		LOG_INF("conn: first_bad %u nodata %u outside %u ptr_skip %u wptr_max %u fst_capped %u guard_esc %u",
			cs.first_bad, cs.first_nodata, cs.first_outside, st.rx_ptr_skip,
			st.rx_wptr_max, st.fst_capped,
			(uint32_t)atomic_get(&cnt_guard_escalations));
		LOG_INF("conn: latency planned %u listened %u skipped %u kicks %u",
			cs.planned, cs.listened, cs.skipped, cs.kicks);
		LOG_INF("conn: acl in %u out %u drop %u evt_drop %u lock max %u us acl_tx %u us aes %u us",
			(uint32_t)atomic_get(&cnt_acl_in), (uint32_t)atomic_get(&cnt_acl_out),
			(uint32_t)atomic_get(&cnt_acl_drop), (uint32_t)atomic_get(&cnt_evt_drop),
			ticks_to_us(lock_max), ticks_to_us(acl_lock), ticks_to_us(aes_max));
	}
	if (st.restores != last->restores) {
		LOG_INF("adv restore #%u: tx fifo rptr/wptr 0x%04x -> 0x%04x", st.restores,
			st.restore_ptrs_before, st.restore_ptrs_after);
	}
	*last = st;
	*last_c = cs;
}

/* Arm llcp_tmr for the LLCP response timeout while a procedure is pending,
 * cancel it otherwise (stimer ticks -> kernel ms, rounded up; firing early
 * only costs a wakeup that re-arms it). */
static void llcp_tmr_update(void)
{
	int32_t left = conn_up ? ll_llcp_timeout_ticks(ll_radio_now()) : -1;

	if (left < 0) {
		if (llcp_tmr_armed) {
			(void)k_work_cancel_delayable(&llcp_tmr);
			llcp_tmr_armed = false;
		}
		return;
	}
	(void)k_work_reschedule(&llcp_tmr,
				K_MSEC((uint32_t)left / (LL_TICKS_PER_US * 1000u) + 1u));
	llcp_tmr_armed = true;
}

static void ctrl_thread_fn(void *p1, void *p2, void *p3)
{
	struct ll_connect_ind ci;
	struct ll_radio_stats last_stats = {0};
	struct ll_conn_stats last_conn = {0};

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		(void)k_sem_take(&wake, K_FOREVER);

		if (atomic_test_bit(&pend, PEND_CONNECTED)) {
			handle_connected();
		}
		handle_rx();
		if (atomic_test_bit(&pend, PEND_UPDATED)) {
			handle_updated();
		}
		handle_acl_tx();
		flush_nocp();
		if (atomic_test_bit(&pend, PEND_DISCONNECTED)) {
			handle_disconnected();
		}
		if (conn_up) {
			ll_llcp_tick(ll_radio_now());
		}
		llcp_tmr_update();
		drain_evt_q();
		while (k_msgq_get(&conn_q, &ci, K_NO_WAIT) == 0) {
			log_connect_ind(&ci);
		}

		uint32_t drops = (uint32_t)atomic_clear(&conn_drops);

		if (drops != 0) {
			LOG_WRN("dropped %u CONNECT_IND event(s) (queue full)", drops);
		}
		if (IS_ENABLED(CONFIG_BT_HCI_B91_OPENLL_STATS_LOG) && atomic_clear(&stats_due)) {
			report_stats(&last_stats, &last_conn);
		}
	}
}

int b91_bt_controller_init(void)
{
	uint8_t mac_random_static[6];

	trng_init();
	b91_mac_init(B91_MAC_FLASH_ADDR, mac_rand, bd_addr, mac_random_static);
	aes_selftest();
	ll_radio_init(radio_evt);
	ll_sched_init();
	ll_conn_init(&conn_ops);
	ll_llcp_init(&llcp_ops);
	ll_rxq_reset();
	ll_llcp_reset();
	ll_adv_init(bd_addr, on_connect_ind);
	ll_hci_init(&hci_ops, evt_sink);

	k_thread_create(&ctrl_thread, ctrl_stack, K_THREAD_STACK_SIZEOF(ctrl_stack),
			ctrl_thread_fn, NULL, NULL, NULL,
			CONFIG_BT_HCI_B91_RX_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&ctrl_thread, "openll");
	if (IS_ENABLED(CONFIG_BT_HCI_B91_OPENLL_STATS_LOG)) {
		k_timer_start(&stats_tmr, K_MSEC(STATS_PERIOD_MS), K_MSEC(STATS_PERIOD_MS));
	}
	LOG_INF("open link layer up, BD_ADDR %02x:%02x:%02x:%02x:%02x:%02x",
		bd_addr[5], bd_addr[4], bd_addr[3], bd_addr[2], bd_addr[1], bd_addr[0]);
	return 0;
}

/* HCI thread: give the host's LE ACL buffer credit back for a packet that
 * will never be sent. The conn_up test and the increment are one step under
 * ll_plat_lock(), so a credit of an ended connection cannot leak into the
 * next one (handle_disconnected() clears the count and conn_up under the
 * same lock). */
static void credit_back(void)
{
	unsigned int key = ll_plat_lock();

	if (conn_up) {
		atomic_inc(&nocp_pending);
	}
	ll_plat_unlock(key);
	k_sem_give(&wake);
}

void b91_bt_host_send_packet(uint8_t type, uint8_t *data, uint16_t len)
{
	struct acl_item it;
	int r;

	switch (type) {
	case 0x01:
		ll_hci_cmd(data, len);
		break;
	case 0x02:
		r = ll_hci_acl_from_host(data, len, &it.pdu);
		if (r != 0) {
			LOG_WRN("host ACL rejected (%d, %u bytes)", r, len);
			atomic_inc(&cnt_acl_drop);
			if (r == -EINVAL || r == -ENOTCONN) {
				/* the host counted it against its LE ACL
				 * buffers (one pool for the controller, the
				 * only connection is ours): give the credit
				 * back (only while connected) */
				credit_back();
			}
			break;
		}
		if (!conn_up) {
			atomic_inc(&cnt_acl_drop);   /* no connection (or it just ended) */
			break;
		}
		it.gen = (uint32_t)atomic_get(&conn_gen);
		if (k_msgq_put(&acl_q, &it, K_NO_WAIT) != 0) {
			LOG_ERR("host ACL queue full (host exceeded LE ACL buffers)");
			atomic_inc(&cnt_acl_drop);
			credit_back();
			break;
		}
		atomic_inc(&cnt_acl_in);
		k_sem_give(&wake);
		break;
	default:
		LOG_WRN("dropping H4 type 0x%02x (%u bytes)", type, len);
		break;
	}
}

void b91_bt_host_callback_register(const b91_bt_host_callback_t *cb)
{
	host_cb = *cb;
}
