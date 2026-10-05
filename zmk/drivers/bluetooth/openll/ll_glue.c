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
 *   sends host ACL through ll_llcp_tx_acl() (the single encrypt+push point),
 *   emits LE Connection Complete / Connection Update Complete / Number Of
 *   Completed Packets / Disconnection Complete, and on a disconnect resets
 *   the link's ll_rxq and ll_llcp in thread context before it releases the
 *   link id (ll_conn keeps it taken until then, so no new connection can
 *   use it earlier).
 * - HCI thread (host TX): HCI commands (ll_hci) and host ACL, which is only
 *   parsed and queued here, so encryption and the TX packet counter run in
 *   one thread for data (LTK reply / Disconnect queue control PDUs from the
 *   HCI thread under ll_plat_tx_lock(), see ll_llcp.h). HCI Disconnect
 *   queues its Command Status after ll_conn_terminate() returns; if the
 *   HCI thread is then starved for a whole air round trip, Disconnection
 *   Complete can reach the host first (not seen; Zephyr tolerates it).
 *
 * Multilink (slice 6a): every per-connection item below is per link (link
 * id == HCI handle): pending bits, Number Of Completed Packets, the host ACL
 * queue and the held PDU, the connection generation. The controller thread
 * serves the links round-robin (the first link served rotates per pass). A
 * link's DISCONNECTED resets that link's ll_rxq and ll_llcp and then calls
 * ll_conn_release(), so its id is reused only after the reset.
 *
 * The controller thread sleeps until something happens (K_FOREVER): the
 * radio/stimer ISRs, the HCI thread and two timers wake it. The LLCP
 * timer (llcp_tmr) runs only while an LL control procedure is pending or
 * an encrypted link's authenticated payload timeout (slice 6d, LE Ping)
 * runs, whose expiry wakes it at most once per timeout; the stats timer
 * only with CONFIG_BT_HCI_B91_OPENLL_STATS_LOG.
 * Host ACL held back (-ENOMEM: TX backlog full; -EAGAIN: encryption start
 * pauses data, or the link owes control PDUs) is retried on the wakeup that
 * frees it: an ll_txq ack (txq_done), the end of the procedure
 * (LL_START_ENC_RSP received here, the host's LTK negative reply, which
 * wakes it, or the 40 s response timeout, which ends the link) or the
 * disconnect. Owed control PDUs (slice 7, ll_llcp.h) are retried before the
 * link's host ACL at every pass (ll_llcp_retry), and llcp_tmr wakes the
 * thread within LL_LLCP_RETRY_MS while one is owed.
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
#include "ll_arb.h"
#include "ll_conn.h"
#include "ll_credit.h"
#include "ll_crypt.h"
#include "ll_flash.h"
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

/* ACL toward the host never goes through evt_q: the controller thread
 * delivers it directly (handle_rx), so an item only holds events. */

/* Only the HCI thread queues here (command completions, one outstanding
 * command at a time with Zephyr), so a small depth suffices. */
K_MSGQ_DEFINE(evt_q, sizeof(struct evt_item), 8, 4);
K_MSGQ_DEFINE(conn_q, sizeof(struct ll_connect_ind), 2, 4);

/* Host ACL waiting for the controller thread, one queue per link (a packet
 * held back on one link never blocks another). The packets themselves
 * (up to LL_ACL_MTU = 251 octets, slice 6b) live in one slab shared by all
 * links: the host holds at most LL_ACL_NUM unacknowledged packets over all
 * links (LE Read Buffer Size is shared), and an item is freed when its
 * fragments are queued (the credit then waits for the ack in ll_credit),
 * so LL_ACL_NUM items suffice and no queue can overflow; a host exceeding
 * its credits gets the packet dropped with the credit back. gen: the
 * link's connection generation at queue time. */
struct acl_item {
	uint32_t gen;
	struct ll_hci_acl_pdu pdu;
};
#define ACL_Q_DEPTH LL_ACL_NUM
K_MEM_SLAB_DEFINE_STATIC(acl_slab, sizeof(struct acl_item), LL_ACL_NUM, 4);
static struct k_msgq acl_q[LL_MAX_CONN];
static char __aligned(4) acl_q_buf[LL_MAX_CONN][ACL_Q_DEPTH * sizeof(struct acl_item *)];

static K_SEM_DEFINE(wake, 0, 1);

/* LLCP procedure response timeout (40 s, ll_llcp_tick), the retry of
 * owed control PDUs (LL_LLCP_RETRY_MS) and the authenticated payload
 * timeout of encrypted links (slice 6d): armed by the controller thread
 * from ll_llcp_timeout_ticks() only while one of them runs; the expiry
 * only wakes the controller thread. */
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

/* All per link (index = link id = HCI handle). */
enum { PEND_CONNECTED, PEND_UPDATED, PEND_DISCONNECTED };
static atomic_t pend[LL_MAX_CONN];                     /* PEND_* bits set by the ll_conn callback */
static struct ll_connect_ind pend_ci[LL_MAX_CONN];     /* written before the bit is set */
static struct ll_conn_params pend_params[LL_MAX_CONN];
static uint8_t pend_reason[LL_MAX_CONN];
/* Up for the host (Connection Complete sent, no Disconnection Complete
 * yet), connection generation and unreported acked ACL PDUs (Number Of
 * Completed Packets): ll_credit.h, per link. */
static volatile bool silent_end[LL_MAX_CONN]; /* HCI Reset: end without Disconnection Complete */

/* ---- controller thread state ---- */

static struct acl_item *held[LL_MAX_CONN];  /* host ACL waiting for ll_llcp_tx_acl() */
/* LE Data Length Change to report (slice 6b Task 4, see llcp_data_len_change) */
static atomic_t dle_pend;                    /* bit per link, set for the controller thread */
static struct ll_llcp_dle dle_val[LL_MAX_CONN];   /* under ll_plat_lock() */
/* connection generation (ll_credit) the value belongs to, under
 * ll_plat_lock(): a change raised for an ended connection is never
 * reported to a new one on the reused id */
static uint32_t dle_gen[LL_MAX_CONN];
/* HCI thread only (set from ll_llcp_set_data_len() inside a command, taken
 * in b91_bt_host_send_packet() after it): no lock needed */
static uint32_t hci_dle_defer;
static bool mic_failed[LL_MAX_CONN];        /* MIC failure logged for this connection */
static uint8_t rr_first;                    /* round-robin: link served first in this pass */

static bool pend_test(uint8_t link, int bit)
{
	return atomic_test_bit(&pend[link], bit);
}

static bool any_conn_up(void)
{
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		if (ll_credit_up(i)) {
			return true;
		}
	}
	return false;
}

/* ---- counters ---- */

static atomic_t cnt_tx_acked, cnt_acl_in, cnt_acl_out, cnt_acl_drop, cnt_evt_drop, conn_drops;
static atomic_t cnt_acl_frag;           /* host ACL packets sent in more than one PDU */
static atomic_t cnt_guard_escalations;
static atomic_t cnt_wakeups;             /* controller thread passes (power counter) */
static atomic_t cnt_apto;                /* authenticated payload timeouts (our LL_PING_REQs) */
static uint32_t lock_depth, lock_t0, lock_max_ticks, acl_tx_lock_max_ticks, aes_max_ticks;
static volatile bool in_acl_tx;          /* controller thread is inside ll_llcp_tx_acl() */
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
static void conn_evt(uint8_t link, enum ll_conn_evt what, const void *arg)
{
	if (link >= LL_MAX_CONN) {
		return;
	}
	switch (what) {
	case LL_CONN_EVT_CONNECTED:
		pend_ci[link] = *(const struct ll_connect_ind *)arg;
		atomic_set_bit(&pend[link], PEND_CONNECTED);
		break;
	case LL_CONN_EVT_UPDATED:
		pend_params[link] = *(const struct ll_conn_params *)arg;
		atomic_set_bit(&pend[link], PEND_UPDATED);
		break;
	case LL_CONN_EVT_DISCONNECTED:
		pend_reason[link] = *(const uint8_t *)arg;
		atomic_set_bit(&pend[link], PEND_DISCONNECTED);
		break;
	}
	k_sem_give(&wake);
}

/* ll_txq completion (ISR), forwarded by ll_conn. A host ACL packet's
 * buffer credit returns with the ack of its last fragment. */
static void txq_done(uint8_t link, enum ll_txq_kind kind, uint8_t ctrl_opcode, bool last)
{
	ARG_UNUSED(ctrl_opcode);
	if (kind == LL_TXQ_ACL && last) {
		ll_credit_acked(link);
	}
	if (kind != LL_TXQ_EMPTY) {
		atomic_inc(&cnt_tx_acked);
	}
	k_sem_give(&wake);
}

/* The arbiter's owners: links 0..LL_MAX_CONN-1 and advertising. */
static void arb_start(uint8_t id, uint32_t cap_us)
{
	if (id == LL_ARB_ADV) {
		ll_adv_arb_start(cap_us);
	} else {
		ll_conn_arb_start(id, cap_us);
	}
}

static void arb_bumped(uint8_t id)
{
	if (id == LL_ARB_ADV) {
		ll_adv_arb_bumped();
	} else {
		ll_conn_arb_bumped(id);
	}
}

static const struct ll_arb_ops arb_ops = {
	.start = arb_start,
	.bumped = arb_bumped,
};

/* One dispatcher for both users of the radio: ll_adv ignores everything
 * while advertising is disabled, ll_conn everything outside a connection
 * event. */
static void radio_evt(enum ll_radio_evt evt, const uint8_t *pdu, uint16_t len, uint32_t tick)
{
	/* advertising PDUs are at most 39 bytes (8-bit length there) */
	ll_adv_radio_evt(evt, pdu, (uint8_t)len, tick);
	ll_conn_radio_evt(evt, pdu, len, tick);
	/* Wake the controller thread before the event ends when the event
	 * owner's RX queue fills up: half full (entries or bytes, ll_rxq), or
	 * one maximum PDU short of the point where ll_conn's RX flow control
	 * stops the event (ll_conn_rx_wake_due; slice 7 Task 2c review: with
	 * maximum PDUs that point, 3 queued, lies below half full, 4). The
	 * thread then runs between the exchanges of a long central burst and
	 * drains ll_rxq; the wake comes before the stop, at the latest with the
	 * stopping packet (random sizes), and if the thread does not get the
	 * CPU in time (cooperative host threads run first) the event is still
	 * stopped, so ll_rxq never overflows (an overflow ends the link with
	 * 0x08). With long PDUs the byte area holds 8 maximum PDUs
	 * (LL_RXQ_POOL_BYTES), and one event at a 50 ms interval can carry 11
	 * exchanges of 4.5 ms; 27-octet PDUs fill the 16 entries in about 12 ms
	 * of MD burst. Only the owner: no other link receives during this
	 * event, and its queue was looked at in its own events. The CONN_RX
	 * callbacks run only once our response has started (ll_radio.c): CPU
	 * work in the RX -> TX turnaround moves the TX later, and a thread woken
	 * there would do so too. */
	if (evt == LL_RADIO_CONN_RX) {
		int owner = ll_conn_event_owner();

		if (owner >= 0 && (ll_rxq_isr_half_full((uint8_t)owner) ||
				   ll_conn_rx_wake_due((uint8_t)owner))) {
			k_sem_give(&wake);
		}
	}
	if (evt != LL_RADIO_CONN_DONE) {
		return;
	}
	if (ll_radio_conn_guard_streak() >= GUARD_STREAK_MAX && ll_conn_count() != 0) {
		/* One radio: the streak is counted over all links' events, and
		 * a wedge ends every active link (ll_conn_end_all, host-tested).
		 * The event is closed (CONN_DONE handled), so these end now. */
		if (ll_conn_end_all(LL_ST_CONN_TIMEOUT) != 0) {
			atomic_inc(&cnt_guard_escalations);
		}
	}
	/* At the end of the event, wake it for a data PDU the RX wake above
	 * has not reported yet (len counts empty PDUs too, which would wake
	 * it at every listened event). Acks of our PDUs wake it via
	 * txq_done, a link end via conn_evt (DISCONNECTED, also after an
	 * ll_rxq overflow). */
	if (ll_rxq_isr_take_queued()) {
		k_sem_give(&wake);
	}
}

static const struct ll_conn_ops conn_ops = {
	.evt = conn_evt,
	.txq_done = txq_done,
	.ctrl_tx = ll_llcp_ctrl_tx,
	.busy = ll_llcp_busy,
};

/* HCI handle == link id */
static bool llcp_ltk_req(uint8_t link, const uint8_t rand[8], uint16_t ediv)
{
	return ll_hci_evt_ltk_req(link, rand, ediv);
}

static void llcp_enc_change(uint8_t link, uint8_t status, bool enabled)
{
	ll_hci_evt_enc_change(link, status, enabled);
}

/* Controller thread (LL_LENGTH_REQ / _RSP) or HCI thread (LE Set Data
 * Length). Slice 6b Task 4 (Task 3 review carry-over c): the event is
 * always sent by the controller thread (flush_dle), which also sends
 * Disconnection Complete, so it never follows that; a change from inside
 * an HCI command is handed over only after the command returned
 * (b91_bt_host_send_packet), so its Command Complete, already in evt_q,
 * goes first. Not for a link the host does not know (yet) or any more. */
static void llcp_data_len_change(uint8_t link, const struct ll_llcp_dle *eff)
{
	unsigned int key;
	uint32_t gen;

	/* never from an ISR: ll_llcp reports changes from the LENGTH PDUs
	 * (controller thread) and LE Set Data Length (HCI thread) only */
	__ASSERT_NO_MSG(!k_is_in_isr());
	LOG_INF("data length (handle %u): tx %u B / %u us, rx %u B / %u us", link,
		eff->max_tx_octets, eff->max_tx_time, eff->max_rx_octets, eff->max_rx_time);
	if (!ll_credit_up_gen(link, &gen)) {
		return;   /* not reported to the host (yet), or ended */
	}
	key = ll_plat_lock();
	dle_val[link] = *eff;
	dle_gen[link] = gen;
	ll_plat_unlock(key);
	if (k_current_get() == &ctrl_thread) {
		atomic_set_bit(&dle_pend, link);
	} else {
		hci_dle_defer |= BIT(link);   /* the HCI thread, see above */
	}
}

/* Controller thread (ll_llcp_tick): the link's authenticated payload
 * timeout passed; ll_llcp has queued LL_PING_REQ. The event goes out only
 * if the host enabled it (Event Mask Page 2 bit 23, ll_hci), and only
 * while the host knows the link. */
static void llcp_apto_expired(uint8_t link)
{
	atomic_inc(&cnt_apto);
	if (ll_credit_up(link) && !pend_test(link, PEND_DISCONNECTED)) {
		ll_hci_evt_apto_expired(link);
	}
}

/* Controller thread: the central's LL_CONNECTION_PARAM_REQ (slice 6d Task
 * 2). LE Remote Connection Parameter Request goes out only while the host
 * knows the link and the event is enabled (LE event mask bit 5, which
 * Zephyr sets because we claim the feature); else ll_llcp accepts the
 * request as the Link Layer (returns false). */
static bool llcp_conn_param_req(uint8_t link, const struct ll_llcp_cpr *r)
{
	bool sent = false;

	/* logged first: the host may answer before this returns */
	LOG_INF("LL_CONNECTION_PARAM_REQ (handle %u): interval %u..%u latency %u timeout %u",
		link, r->interval_min, r->interval_max, r->latency, r->timeout);
	if (ll_credit_up(link) && !pend_test(link, PEND_DISCONNECTED)) {
		sent = ll_hci_evt_conn_param_req(link, r->interval_min, r->interval_max, r->latency,
						 r->timeout);
	}
	if (!sent) {
		LOG_INF("  not indicated to the host: accepted by the LL");
	}
	return sent;
}

static const struct ll_llcp_ops llcp_ops = {
	.ltk_req = llcp_ltk_req,
	.enc_change = llcp_enc_change,
	.data_len_change = llcp_data_len_change,
	.apto_expired = llcp_apto_expired,
	.conn_param_req = llcp_conn_param_req,
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

/* A link the controller thread still has to finish (connected, or an
 * event not handled yet). */
static bool link_busy(uint8_t i)
{
	return ll_conn_active(i) || ll_credit_up(i) || pend_test(i, PEND_DISCONNECTED) ||
	       pend_test(i, PEND_CONNECTED);
}

/* HCI Reset (HCI thread): stop advertising and end every connection
 * silently (Reset semantics: no Disconnection Complete). The controller
 * thread does the LL state reset (ll_rxq, ll_llcp, held ACL) as for any
 * disconnect and signals reset_done per link; the wait covers an event on
 * air (ll_conn ends at its CONN_DONE, bounded by the radio guard). */
static void hci_reset(void)
{
	int64_t deadline;
	bool any = false;

	ll_adv_reset();
	k_sem_reset(&reset_done);
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		if (link_busy(i)) {
			silent_end[i] = true;
			ll_conn_end(i, LL_ST_LOCAL_TERM);
			any = true;
		}
	}
	if (!any) {
		return;
	}
	k_sem_give(&wake);
	deadline = k_uptime_get() + RESET_WAIT_MS;
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		while (link_busy(i)) {
			int64_t left = deadline - k_uptime_get();

			if (left <= 0 || k_sem_take(&reset_done, K_MSEC(left)) != 0) {
				LOG_ERR("HCI Reset: connection did not end within %u ms",
					RESET_WAIT_MS);
				return;
			}
		}
	}
}

/* HCI thread: the negative reply resumes paused host ACL; wake the
 * controller thread, which holds it (nothing else would). */
static uint8_t ltk_neg_reply(uint16_t handle)
{
	uint8_t st = ll_llcp_ltk_neg_reply((uint8_t)handle);

	k_sem_give(&wake);
	return st;
}

/* ll_hci validated the handle (handle_valid), so it is a link id. HCI
 * thread: LL_TERMINATE_IND may be owed (backlog full, slice 7); wake the
 * controller thread so it retries it and arms the retry timer. */
static uint8_t hci_disconnect(uint16_t handle, uint8_t reason)
{
	uint8_t st = ll_llcp_terminate((uint8_t)handle, reason);

	k_sem_give(&wake);
	return st;
}

/* HCI thread: LL_START_ENC_REQ may be owed, and the 40 s timer restarts;
 * wake the controller thread (retry, timer). */
static uint8_t ltk_reply(uint16_t handle, const uint8_t ltk[16])
{
	uint8_t st = ll_llcp_ltk_reply((uint8_t)handle, ltk);

	k_sem_give(&wake);
	return st;
}

/* HCI thread: our LL_LENGTH_REQ may start the 40 s response timer, which
 * the controller thread arms on its next pass: wake it. */
static uint8_t hci_set_data_len(uint16_t handle, uint16_t tx_octets, uint16_t tx_time)
{
	uint8_t st = ll_llcp_set_data_len((uint8_t)handle, tx_octets, tx_time);

	k_sem_give(&wake);
	return st;
}

/* Slice 6d: Read / Write Authenticated Payload Timeout. A write restarts
 * the timer (possibly shorter): wake the controller thread to re-arm it. */
static uint8_t hci_read_apto(uint16_t handle, uint16_t *apto)
{
	return ll_llcp_read_apto((uint8_t)handle, apto);
}

static uint8_t hci_write_apto(uint16_t handle, uint16_t apto)
{
	uint8_t st = ll_llcp_write_apto((uint8_t)handle, apto);

	k_sem_give(&wake);
	return st;
}

/* Slice 6d Task 2: the host's answer to LE Remote Connection Parameter
 * Request. HCI thread: LL_CONNECTION_PARAM_RSP / LL_REJECT_EXT_IND may be
 * owed and the 40 s timer restarts: wake the controller thread (retry,
 * timer). */
static uint8_t hci_conn_param_reply(uint16_t handle, uint16_t interval_min,
				    uint16_t interval_max, uint16_t latency, uint16_t timeout)
{
	const struct ll_llcp_cpr p = {interval_min, interval_max, latency, timeout};
	uint8_t st = ll_llcp_conn_param_reply((uint8_t)handle, &p);

	LOG_INF("conn param reply (handle %u): interval %u..%u latency %u timeout %u, status 0x%02x",
		handle, interval_min, interval_max, latency, timeout, st);
	k_sem_give(&wake);
	return st;
}

static uint8_t hci_conn_param_neg_reply(uint16_t handle, uint8_t reason)
{
	uint8_t st = ll_llcp_conn_param_neg_reply((uint8_t)handle, reason);

	LOG_INF("conn param negative reply (handle %u): reason 0x%02x, status 0x%02x", handle,
		reason, st);
	k_sem_give(&wake);
	return st;
}

/* 1M only (slice 6b): both directions are always LE 1M */
static uint8_t hci_read_phy(uint16_t handle, uint8_t *tx_phy, uint8_t *rx_phy)
{
	ARG_UNUSED(handle);
	*tx_phy = LL_PHY_1M;
	*rx_phy = LL_PHY_1M;
	return LL_ST_SUCCESS;
}

/* ll_hci checked the PHYs (only 1M can pass): nothing to request on air;
 * ll_hci reports LE PHY Update Complete (no change) after the status. */
static uint8_t hci_set_phy(uint16_t handle, uint8_t all_phys, uint8_t tx_phys, uint8_t rx_phys,
			   uint16_t opts)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(all_phys);
	ARG_UNUSED(tx_phys);
	ARG_UNUSED(rx_phys);
	ARG_UNUSED(opts);
	return LL_ST_SUCCESS;
}

static bool handle_valid(uint16_t handle)
{
	return handle < LL_MAX_CONN && ll_conn_active((uint8_t)handle);
}

static const struct ll_hci_ops hci_ops = {
	.get_bd_addr = get_bd_addr,
	.rand = rand_bytes,
	.reset = hci_reset,
	.adv_set_params = ll_adv_set_params,
	.adv_set_data = ll_adv_set_data,
	.adv_set_scan_rsp = ll_adv_set_scan_rsp,
	/* No glue-side refusal while a disconnect is pending (slice 6a Task
	 * 6): ll_conn keeps an ended link taken until handle_disconnected()
	 * has reset its ll_rxq / ll_llcp and released it, so a CONNECT_IND
	 * during advertising can never reuse that id before the reset, and
	 * ll_adv_enable() counts it as taken (0x09 when it was the last). */
	.adv_enable = ll_adv_enable,
	.unknown = unknown_opcode,
	.disconnect = hci_disconnect,
	.ltk_reply = ltk_reply,
	.ltk_neg_reply = ltk_neg_reply,
	.handle_valid = handle_valid,
	.set_data_len = hci_set_data_len,
	.read_phy = hci_read_phy,
	.set_phy = hci_set_phy,
	/* Slice 6c: host-based privacy (Zephyr sets its RPA with 0x2005,
	 * after stopping advertising when it rotates it) */
	.set_random_addr = ll_adv_set_random_addr,
	.read_apto = hci_read_apto,
	.write_apto = hci_write_apto,
	.conn_param_reply = hci_conn_param_reply,
	.conn_param_neg_reply = hci_conn_param_neg_reply,
};

/* ---- controller thread ---- */

static void on_connect_ind(const struct ll_connect_ind *ci)
{
	/* ISR context: log later, from the controller thread */
	if (k_msgq_put(&conn_q, ci, K_NO_WAIT) != 0) {
		atomic_inc(&conn_drops);
	}
	k_sem_give(&wake);   /* no periodic wakeup any more */
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

static void handle_connected(uint8_t link)
{
	struct ll_connect_ind ci;
	unsigned int key = ll_plat_lock();

	ci = pend_ci[link];
	atomic_clear_bit(&pend[link], PEND_CONNECTED);
	ll_plat_unlock(key);

	mic_failed[link] = false;
	atomic_clear_bit(&dle_pend, link);
	/* the lower bound of the authenticated payload timeout (slice 6d) */
	ll_llcp_conn_params(link, ci.interval, ci.latency, ci.timeout);
	/* new generation, up, no credit of an earlier connection (ll_credit) */
	ll_credit_open(link);
	/* ci (also kept by ll_conn for the link) records the local address
	 * the connection was made to: public or the random AdvA (slice 6c).
	 * LE Connection Complete stays the legacy event; Zephyr derives its
	 * local RPA itself. */
	LOG_INF("connected (handle %u): interval %u latency %u timeout %u, local %s "
		"%02x:%02x:%02x:%02x:%02x:%02x", link, ci.interval, ci.latency, ci.timeout,
		ci.adv_addr_random ? "random" : "public", ci.adv_a[5], ci.adv_a[4],
		ci.adv_a[3], ci.adv_a[2], ci.adv_a[1], ci.adv_a[0]);
	if (!silent_end[link]) {
		ll_hci_evt_conn_complete(link, &ci);
		/* ll_conn uses CSA#2 exactly when the CONNECT_IND has ChSel 1
		 * (our ADV_IND always has it) */
		ll_hci_evt_chan_sel_algo(link, ci.chsel ? 0x01 : 0x00);
		/* connInitialMaxTx* from the host's Suggested Default Data
		 * Length (4.5.10 "For a new connection"); ll_llcp starts the
		 * LENGTH procedure only when the central does not know our
		 * values yet (with the host's default of 251 / 2120: at every
		 * connection, slice 6b Task 4). A full
		 * backlog owes LL_LENGTH_REQ (retried, slice 7); only a full
		 * owed queue fails it (Memory Capacity, ignored: the link then
		 * keeps 27 / 328, which stay valid) */
		uint16_t def_oct, def_time;

		ll_hci_default_data_len(&def_oct, &def_time);
		(void)ll_llcp_set_data_len(link, def_oct, def_time);
	}
}

static void handle_updated(uint8_t link)
{
	struct ll_conn_params p;
	unsigned int key = ll_plat_lock();

	p = pend_params[link];
	atomic_clear_bit(&pend[link], PEND_UPDATED);
	ll_plat_unlock(key);
	LOG_INF("connection update (handle %u): interval %u latency %u timeout %u", link,
		p.interval, p.latency, p.timeout);
	ll_llcp_conn_params(link, p.interval, p.latency, p.timeout);
	if (ll_credit_up(link) && !pend_test(link, PEND_DISCONNECTED)) {
		ll_hci_evt_conn_update(link, &p);
	}
}

/* Acked ACL PDUs -> Number Of Completed Packets, one handle per event
 * (never after the Disconnection Complete of the connection). */
static void flush_nocp(uint8_t link)
{
	uint16_t n = ll_credit_take(link);   /* 0 while the link is down */

	if (n != 0) {
		ll_hci_evt_num_completed(link, n);
	}
}

/* The LENGTH, PHY and connection update requests from the central (rare:
 * a few per connection, typically): logged so the exchange is visible on
 * the device. LL_CONNECTION_PARAM_REQ is logged with its outcome
 * (llcp_conn_param_req). */
static void log_llcp_rx(uint8_t link, const uint8_t *d, uint8_t len)
{
	if (len == 12 && d[0] == 0x00) {
		LOG_INF("LL_CONNECTION_UPDATE_IND (handle %u): interval %u latency %u timeout %u "
			"instant %u", link, ll_get_le16(&d[4]), ll_get_le16(&d[6]),
			ll_get_le16(&d[8]), ll_get_le16(&d[10]));
	} else if (len == 3 && d[0] == 0x11) {
		LOG_INF("LL_REJECT_EXT_IND (handle %u): opcode 0x%02x error 0x%02x", link, d[1],
			d[2]);
	} else if (len == 9 && (d[0] == 0x14 || d[0] == 0x15)) {
		LOG_INF("LL_LENGTH_%s (handle %u): rx %u B / %u us, tx %u B / %u us",
			d[0] == 0x14 ? "REQ" : "RSP", link, ll_get_le16(&d[1]), ll_get_le16(&d[3]),
			ll_get_le16(&d[5]), ll_get_le16(&d[7]));
	} else if (len == 3 && d[0] == 0x16) {
		LOG_INF("LL_PHY_REQ (handle %u): tx 0x%02x rx 0x%02x, answering 1M", link, d[1],
			d[2]);
	} else if (len == 5 && d[0] == 0x18) {
		LOG_INF("LL_PHY_UPDATE_IND (handle %u): c_to_p 0x%02x p_to_c 0x%02x", link, d[1],
			d[2]);
	}
}

/* LE Data Length Change of the link, if one is pending (controller
 * thread; see llcp_data_len_change). */
static void flush_dle(uint8_t link)
{
	struct ll_llcp_dle e;
	unsigned int key;
	uint32_t gen, cur;

	if (!atomic_test_and_clear_bit(&dle_pend, link)) {
		return;
	}
	key = ll_plat_lock();
	e = dle_val[link];
	gen = dle_gen[link];
	ll_plat_unlock(key);
	if (ll_credit_up_gen(link, &cur) && cur == gen && !pend_test(link, PEND_DISCONNECTED)) {
		ll_hci_evt_data_len_change(link, e.max_tx_octets, e.max_tx_time, e.max_rx_octets,
					   e.max_rx_time);
	}
}

/* Received PDUs: LLID 3 to ll_llcp, LLID 1/2 to the host. The PDU and the
 * H4 packet (up to 257 + 256 bytes with long PDUs) are static: controller
 * thread only, so they stay off its stack. */
static void handle_rx_pdus(uint8_t link, bool *got)
{
	static struct ll_rx_pdu pdu;
	static uint8_t h4[LL_HCI_ACL_MAX];

	for (int i = 0; i < RX_BUDGET; i++) {
		if (pend_test(link, PEND_DISCONNECTED)) {
			return;   /* the link is gone; its ll_rxq is reset below */
		}
		enum ll_rxq_result r = ll_rxq_get(link, &pdu);

		if (r == LL_RXQ_EMPTY) {
			return;
		}
		if (r == LL_RXQ_MIC_FAIL) {
			/* sticky in ll_rxq until the reset at the disconnect:
			 * log once, ll_conn_end() is idempotent */
			if (!mic_failed[link]) {
				mic_failed[link] = true;
				LOG_WRN("MIC failure (handle %u), ending the connection", link);
			}
			ll_conn_end(link, LL_ST_MIC_FAILURE);
			return;
		}
		/* non-empty and, once the link encrypts, decrypted with a
		 * valid MIC (ll_rxq never delivers a retransmission) */
		*got = true;
		uint8_t llid = pdu.hdr0 & 0x03;

		if (llid == LL_LLID_CTRL) {
			log_llcp_rx(link, pdu.data, pdu.len);
			ll_llcp_rx(link, pdu.data, pdu.len, pdu.event);
			continue;
		}
		uint16_t n = ll_hci_acl_to_host(h4, link, llid, pdu.data, pdu.len);

		if (n != 0) {
			drain_evt_q();
			deliver(h4, n);
			atomic_inc(&cnt_acl_out);
		}
	}
	k_sem_give(&wake);   /* budget used up: continue on the next pass */
}

static void handle_rx(uint8_t link)
{
	bool got = false;

	if (!ll_credit_up(link)) {
		return;
	}
	handle_rx_pdus(link, &got);
	if (got) {
		/* slice 6d: restarts the authenticated payload timer (a no-op
		 * while the link is unencrypted) */
		ll_llcp_rx_auth(link);
	}
}

static void acl_free(struct acl_item *it)
{
	k_mem_slab_free(&acl_slab, (void *)it);
}

/* Drop the link's held and queued host ACL (no Number Of Completed
 * Packets: the host frees its buffers on disconnect). */
static void acl_drop_all(uint8_t link)
{
	struct acl_item *it;

	if (held[link]) {
		acl_free(held[link]);
		held[link] = NULL;
	}
	while (k_msgq_get(&acl_q[link], &it, K_NO_WAIT) == 0) {
		acl_free(it);
	}
}

/* Host ACL -> ll_llcp_tx_acl(): split into PDUs of the link's TX limit
 * (slice 6b Task 4: effective Octets and Time, MIC when encrypted) and
 * queued all or none, under the TX lock so the limit and the encryption
 * state cannot change in between. -EAGAIN (encryption start pauses data,
 * or control PDUs are owed) and -ENOMEM (not all fragments fit the TX
 * queue) keep the packet for the next wakeup; its credit returns only when
 * the last fragment is acked (txq_done). */
static void handle_acl_tx(uint8_t link)
{
	struct ll_acl_frag frags[(LL_ACL_MTU + LL_DLE_MIN_OCTETS - 1) / LL_DLE_MIN_OCTETS];

	while (ll_credit_up(link) && !pend_test(link, PEND_DISCONNECTED)) {
		struct acl_item *h = held[link];
		uint8_t n;
		int r;

		if (!h) {
			if (k_msgq_get(&acl_q[link], &h, K_NO_WAIT) != 0) {
				return;
			}
			if (h->gen != ll_credit_gen(link)) {
				atomic_inc(&cnt_acl_drop);   /* queued for a previous connection */
				acl_free(h);
				continue;
			}
			held[link] = h;
		}
		in_acl_tx = true;
		ll_plat_tx_lock();
		n = ll_hci_acl_fragment(&h->pdu, ll_llcp_tx_limit(link), frags, ARRAY_SIZE(frags));
		r = n ? ll_llcp_tx_acl(link, h->pdu.data, frags, n) : -EINVAL;
		ll_plat_tx_unlock();
		in_acl_tx = false;
		if (r == 0) {
			if (n > 1) {
				atomic_inc(&cnt_acl_frag);
			}
			held[link] = NULL;
			acl_free(h);
			continue;
		}
		if (r == -EAGAIN || r == -ENOMEM) {
			return;
		}
		LOG_WRN("host ACL dropped (handle %u, %d)", link, r);
		atomic_inc(&cnt_acl_drop);
		(void)ll_credit_back(link);   /* the host's buffer credit comes back */
		held[link] = NULL;
		acl_free(h);
	}
}

/* Thread-side end of a connection: ll_rxq_reset() and ll_llcp_reset() of
 * the link run here, never in ISR context (the controller thread is the
 * ll_rxq consumer, so it cannot be inside ll_rxq_get() now), and only then
 * is the link id released (ll_conn_release), so no new connection can use
 * it before the reset. Advertising stays refused until PEND_DISCONNECTED is
 * cleared below. Held and queued host ACL of the link is dropped without
 * Number Of Completed Packets (the host frees its buffers on disconnect). */
static void handle_disconnected(uint8_t link)
{
	uint8_t reason;
	bool silent, was_up;
	unsigned int key;

	/* CONNECTED and DISCONNECTED may both have been raised since the loop
	 * checked CONNECTED (e.g. 0x3E after six events while the thread was
	 * busy): report the connection first, so it is not announced to the
	 * host after its end (or never ended for it). */
	if (pend_test(link, PEND_CONNECTED)) {
		handle_connected(link);
	}
	flush_nocp(link);   /* acks that happened before the end */
	key = ll_plat_lock();
	reason = pend_reason[link];
	ll_plat_unlock(key);

	ll_rxq_reset(link);
	ll_llcp_reset(link);
	acl_drop_all(link);
	atomic_clear_bit(&dle_pend, link);
	atomic_clear_bit(&pend[link], PEND_UPDATED);
	/* credits and "up" end together (ll_credit: an HCI-thread credit is
	 * either counted before this or sees the link down) */
	was_up = ll_credit_close(link);

	/* One lock section: this connection's silent_end and DISCONNECTED are
	 * consumed before the id is released, so a new connection on the
	 * reused id (CONNECT_IND ISR right after the release) can neither
	 * lose its own DISCONNECTED nor inherit silent_end. */
	key = ll_plat_lock();
	silent = silent_end[link];
	silent_end[link] = false;
	atomic_clear_bit(&pend[link], PEND_DISCONNECTED);
	ll_conn_release(link);   /* the id may be reused from now on */
	ll_plat_unlock(key);

	LOG_INF("disconnected (handle %u), reason 0x%02x%s", link, reason,
		silent ? " (HCI Reset, not reported)" : "");
	if (!silent && was_up) {
		ll_hci_evt_disconn_complete(link, reason);
	}
	k_sem_give(&reset_done);
}

static uint32_t ticks_to_us(uint32_t t)
{
	return t / LL_TICKS_PER_US;
}

/* Periodic health log (CONFIG_BT_HCI_B91_OPENLL_STATS_LOG): one line every
 * 2 s while there is anything to report, plus a stall warning if
 * advertising is enabled but tx2rx is not advancing. Connection counters
 * are logged while connected or changed; the ll_conn counters and the
 * ll_rxq overflow count are sums over all links (per-link counters: group
 * 66). */
static void report_stats(struct ll_radio_stats *last, struct ll_conn_stats *last_c)
{
	struct ll_radio_stats st;
	struct ll_conn_stats cs;

	uint32_t rxq_of = 0;
	struct ll_adv_stats as;
	struct ll_flash_stats fs;

	ll_radio_get_stats(&st);
	ll_conn_get_stats_total(&cs);
	ll_adv_get_stats(&as);
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		rxq_of += ll_rxq_overflow_count(i);
	}
	if (st.tx2rx != last->tx2rx || st.rx_ok != last->rx_ok || st.rx_timeout != last->rx_timeout ||
	    st.rsp_tx != last->rsp_tx || st.rsp_late != last->rsp_late || ll_adv_is_enabled()) {
		LOG_INF("radio: tx2rx %u rx_ok %u crc %u timeout %u rsp %u rsp_late %u",
			st.tx2rx, st.rx_ok, st.rx_crc, st.rx_timeout, st.rsp_tx, st.rsp_late);
	}
	if (ll_adv_is_enabled() && st.tx2rx == last->tx2rx) {
		LOG_WRN("radio stalled");
	}
	if (any_conn_up() || memcmp(&cs, last_c, sizeof(cs)) != 0 ||
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
			st.tifs_151_152, st.tifs_gt152, rxq_of, st.rx_ptr_odd);
		LOG_INF("conn: first_bad %u nodata %u outside %u ptr_skip %u wptr_max %u fst_capped %u guard_esc %u hold %u",
			cs.first_bad, cs.first_nodata, cs.first_outside, st.rx_ptr_skip,
			st.rx_wptr_max, st.fst_capped,
			(uint32_t)atomic_get(&cnt_guard_escalations), st.holds);
		LOG_INF("conn: latency planned %u listened %u skipped %u kicks %u coll %u links %u apto %u",
			cs.planned, cs.listened, cs.skipped, cs.kicks, cs.collisions,
			ll_conn_count(), (uint32_t)atomic_get(&cnt_apto));
		LOG_INF("conn: rx flow paused %u (longest run %u) stops %u (radio %u)",
			cs.rx_paused, cs.rx_pause_streak_max, cs.rx_stops, st.conn_stopped);
		ll_flash_get_stats(&fs);
		LOG_INF("flash: windows %u waits %u forced %u wait max %u us hold max %u us, conn paused %u cut %u kicks %u, adv %u, radio aborts %u",
			fs.windows, fs.waits, fs.forced, fs.wait_max_us, fs.hold_max_us,
			cs.flash_paused, cs.flash_cut, cs.flash_kicks, as.flash, st.flash_aborts);
		LOG_INF("adv: events %u slid %u dropped %u cut %u stuck %u adv_guard %u",
			as.events, as.slid, as.dropped, as.cut, as.stuck, st.adv_guard);
		LOG_INF("conn: acl in %u out %u drop %u frag %u evt_drop %u lock max %u us acl_tx %u us aes %u us",
			(uint32_t)atomic_get(&cnt_acl_in), (uint32_t)atomic_get(&cnt_acl_out),
			(uint32_t)atomic_get(&cnt_acl_drop), (uint32_t)atomic_get(&cnt_acl_frag),
			(uint32_t)atomic_get(&cnt_evt_drop),
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
	int32_t left = any_conn_up() ? ll_llcp_timeout_ticks(ll_radio_now()) : -1;

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
		atomic_inc(&cnt_wakeups);

		/* links round-robin: the first one served rotates per pass,
		 * so a link with a long RX burst cannot always go first */
		for (uint8_t k = 0; k < LL_MAX_CONN; k++) {
			uint8_t i = (uint8_t)((rr_first + k) % LL_MAX_CONN);

			if (pend_test(i, PEND_CONNECTED)) {
				handle_connected(i);
			}
			handle_rx(i);
			flush_dle(i);
			if (pend_test(i, PEND_UPDATED)) {
				handle_updated(i);
			}
			/* owed control PDUs before host ACL (which waits for
			 * them with -EAGAIN) */
			ll_llcp_retry(i);
			handle_acl_tx(i);
			flush_nocp(i);
			if (pend_test(i, PEND_DISCONNECTED)) {
				handle_disconnected(i);
			}
		}
		rr_first = (uint8_t)((rr_first + 1) % LL_MAX_CONN);
		if (any_conn_up()) {
			ll_llcp_tick(ll_radio_now());   /* all links */
		}
		/* changes raised later in the pass (another link's step, the
		 * LLCP tick): reported in this pass, not on the next wakeup */
		for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
			flush_dle(i);
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

uint32_t b91_bt_controller_wakeups(void)
{
	return (uint32_t)atomic_get(&cnt_wakeups);
}

/* Called from z_sys_poweroff() (irq locked, scheduler context irrelevant): no
 * radio or stimer IRQ may reach the link layer while the SoC powers down. The
 * SoC cold-boots on wakeup, so nothing is re-enabled. */
void b91_bt_controller_poweroff(void)
{
	ll_sched_quiesce();
	ll_radio_quiesce();
}

int b91_bt_controller_init(void)
{
	uint8_t mac_random_static[6];

	trng_init();
	b91_mac_init(B91_MAC_FLASH_ADDR, mac_rand, bd_addr, mac_random_static);
	aes_selftest();
	ll_radio_init(radio_evt);
	ll_sched_init();
	ll_arb_init(&arb_ops);
	ll_conn_init(&conn_ops);
	ll_llcp_init(&llcp_ops);   /* resets every link's LLCP state */
	ll_credit_init();
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		ll_rxq_reset(i);
		k_msgq_init(&acl_q[i], acl_q_buf[i], sizeof(struct acl_item *), ACL_Q_DEPTH);
	}
	ll_adv_init(bd_addr, on_connect_ind);
	ll_hci_init(&hci_ops, evt_sink);
	ll_flash_reset();
	ll_flash_wrap_enable();   /* flash erases/writes from now on in a window */

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
 * will never be sent, on the handle it was sent to (NOCP is per handle; a
 * handle that is no link cannot get one). ll_credit_back() tests "up" and
 * counts in one lock section, so a credit of an ended connection cannot
 * leak into the next one on that link (ll_credit_close() clears the count
 * and "up" together). */
static void credit_back(uint16_t handle)
{
	if (handle >= LL_MAX_CONN) {
		return;
	}
	(void)ll_credit_back((uint8_t)handle);
	k_sem_give(&wake);
}

void b91_bt_host_send_packet(uint8_t type, uint8_t *data, uint16_t len)
{
	struct acl_item *it;
	int r;

	switch (type) {
	case 0x01:
		ll_hci_cmd(data, len);
		/* LE Data Length Change produced inside the command (LE Set
		 * Data Length): its Command Complete is in evt_q now, so the
		 * controller thread may report the change (after it) */
		if (hci_dle_defer) {
			atomic_or(&dle_pend, (atomic_val_t)hci_dle_defer);
			hci_dle_defer = 0;
			k_sem_give(&wake);
		}
		break;
	case 0x02:
		if (k_mem_slab_alloc(&acl_slab, (void **)&it, K_NO_WAIT) != 0) {
			/* more packets than LE Read Buffer Size allows */
			struct ll_hci_acl_pdu hdr;

			LOG_ERR("host ACL buffers exhausted (host exceeded LE ACL buffers)");
			atomic_inc(&cnt_acl_drop);
			hdr.handle = 0xFFFF;
			if (len >= 2) {
				hdr.handle = ll_get_le16(data) & 0x0FFF;
			}
			credit_back(hdr.handle);
			break;
		}
		it->pdu.handle = 0xFFFF;   /* set by the parser once the header is read */
		r = ll_hci_acl_from_host(data, len, &it->pdu);
		if (r != 0) {
			LOG_WRN("host ACL rejected (%d, handle 0x%04x, %u bytes)", r,
				it->pdu.handle, len);
			atomic_inc(&cnt_acl_drop);
			if (r == -EINVAL || r == -ENOTCONN) {
				/* the host counted it against its LE ACL
				 * buffers (one pool for the controller): give
				 * the credit back on that handle (only while
				 * it is connected) */
				credit_back(it->pdu.handle);
			}
			acl_free(it);
			break;
		}
		/* handle_valid() passed: the handle is a link id. Up and
		 * generation in one lock section (ll_credit_up_gen): a packet
		 * is never tagged with a newer connection's generation after
		 * its own one ended on that id. */
		if (!ll_credit_up_gen((uint8_t)it->pdu.handle, &it->gen)) {
			atomic_inc(&cnt_acl_drop);   /* not reported yet (or it just ended) */
			acl_free(it);
			break;
		}
		if (k_msgq_put(&acl_q[it->pdu.handle], &it, K_NO_WAIT) != 0) {
			LOG_ERR("host ACL queue full (host exceeded LE ACL buffers)");
			atomic_inc(&cnt_acl_drop);
			credit_back(it->pdu.handle);
			acl_free(it);
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
