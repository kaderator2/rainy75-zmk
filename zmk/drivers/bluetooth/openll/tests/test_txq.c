/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_txq against a fake TX FIFO that implements the measured hardware model
 * (md-spike-report "TX FIFO model", points 1-6) and nothing else:
 * 1. empty FIFO (rptr == wptr): the base buffer (empty PDU) is sent;
 * 2. non-empty: ring entry rptr & 3 is sent;
 * 3. MD on air = (wptr - rptr) > 1 at send time;
 * 4. pop (rptr++) on ack: inside a command per the real SN, at the first RX
 *    of a command iff the FIFO is non-empty and central NESN != SN_INIT;
 * 5. a non-acked head is resent with the same SN;
 * 6. the header byte in the buffer is not written back.
 * Multilink (slice 6a, ml-spike-report S1/S2b): there is one FIFO for all
 * links, its rptr cannot be reset, and the hardware reads the head entry
 * from RAM at TX time. Each link has its own Core Spec SN/NESN state and its
 * own central model, which records every data PDU it accepts as new, so
 * loss, duplicates and PDUs of another link show. A "foreign" link event
 * (another link's traffic not driven through ll_txq) fills all four entries
 * with junk, pops some of them (rptr moves) and leaves the rest queued.
 *
 * Built by run_host_tests.sh for LL_MAX_CONN 1, 3 and 5, normal and with
 * -DLL_TXQ_SAFE_MODE. Tests with two real links need LL_MAX_CONN >= 2; the
 * second link is LL_MAX_CONN - 1 (the top of the per-link arrays).
 */
#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include "test.h"
#include "../ll_defs.h"
#include "../ll_radio.h"
#include "../ll_txq.h"

#define HDR_NESN 0x04
#define HDR_SN   0x08
#define HDR_MD   0x10
#define MAX_XCHG 16
#define MAX_AIR  65536
#define MAX_GOT  1024
#define PAYLEN   3
#define JUNK     0xA5     /* data[1] of a foreign entry (ours carry 0xA0 + link) */
#ifdef LL_TXQ_SAFE_MODE
#define SAFE_MODE_ON 1
#else
#define SAFE_MODE_ON 0
#endif

/* ---------------- fake hardware (pipe 0 TX FIFO + BRX SN logic) ---------------- */

struct fake_entry {
	uint8_t hdr0;
	uint8_t len;
	uint8_t data[LL_DATA_PDU_MAX + LL_MIC_LEN];
};

struct air_pkt {          /* one of our packets as sent on air */
	int ring;         /* -1: base buffer, else ring slot 0..3 */
	uint8_t hdr0;     /* as on air: LLID from the buffer, NESN/SN/MD from hw */
	uint8_t len;
	uint8_t data[LL_DATA_PDU_MAX + LL_MIC_LEN];
	int event;
	uint8_t link;
};

/* Task 9 (device): the pipe 0 TX FIFO pointers are 5-bit registers (values
 * read back wrap at 32; a written wptr keeps only its low 5 bits). */
#define HW_PTR_MASK 0x1f

static struct {
	uint8_t rptr, wptr;
	struct fake_entry ring[4];
	uint8_t sn_init;
	uint8_t nesn_init;    /* BRX NESN init: expected central SN at command start */
	uint8_t sn, nesn;     /* live command state: SN of our last TX, our NESN */
	bool first;           /* next RX is the first of the command */
	bool foreign;         /* a foreign link event runs before each of ours */
	int foreign_events;
	int bad_write;        /* write into an entry not yet popped */
	int overfill;         /* wptr - rptr > 4 */
	int sn_init_calls;
	int nesn_init_calls;
	int max_data_in_ring; /* non-empty entries queued at a command start */
	struct air_pkt air[MAX_AIR];
	int n_air;
	int event;
} hw;

/* Per link: the true Core Spec state of our side, the central, completions */
struct link_model {
	uint8_t sn, nesn;     /* SN of our last TX on this link, our NESN */
	bool sent_any;        /* a packet was sent on this link */
	int sn_init_wrong;    /* SN_INIT != SN of our last TX at command start */
	int nesn_init_wrong;  /* NESN_INIT != our NESN at command start */
	int rx_dup;           /* central packets dropped as old (not in the RX FIFO) */
	struct {
		uint8_t sn, nesn;
		uint8_t got[MAX_GOT];  /* tags of data PDUs accepted as new */
		int n_got;
		uint8_t len[MAX_GOT];  /* their lengths */
		int n_len;
		int bad_content;       /* junk, a foreign entry or another link's PDU */
	} c;
	struct {
		int acl, ctrl, empty;
		int not_last;          /* completions with last == false */
		uint8_t op[MAX_GOT];
		int n_op;
	} cpl;
};

static struct link_model lk[LL_MAX_CONN];

void ll_radio_conn_set_sn_init(uint8_t sn)
{
	hw.sn_init = sn & 1;
	hw.sn_init_calls++;
}

/* Task 9 (device): at the first RX of a BRX command the hardware takes the
 * expected central SN from ll_ctrl_1 BRX NESN init, as it takes the ack
 * reference from SN init (md-spike). A central packet whose SN differs is
 * treated as a retransmission: acked, but not written to the RX FIFO. */
void ll_radio_conn_set_nesn_init(uint8_t nesn)
{
	hw.nesn_init = nesn & 1;
	hw.nesn_init_calls++;
}

uint8_t ll_radio_fifo_rptr(void)
{
	return hw.rptr & HW_PTR_MASK;
}

uint8_t ll_radio_fifo_wptr(void)
{
	return hw.wptr & HW_PTR_MASK;
}

void ll_radio_fifo_write(uint8_t idx, uint8_t hdr0, const uint8_t *payload, uint8_t len)
{
	if (((idx - hw.rptr) & HW_PTR_MASK) < ((hw.wptr - hw.rptr) & HW_PTR_MASK)) {
		hw.bad_write++;
	}
	hw.ring[idx & 3].hdr0 = hdr0;
	hw.ring[idx & 3].len = len;
	if (len) {
		memcpy(hw.ring[idx & 3].data, payload, len);
	}
}

void ll_radio_fifo_set_wptr(uint8_t wptr)
{
	wptr &= HW_PTR_MASK;
	if (((wptr - hw.rptr) & HW_PTR_MASK) > 4) {
		hw.overfill++;
	}
	hw.wptr = wptr;
}

/* Another link's event between two of ours, not driven through ll_txq: it
 * fills all four entries with its own PDUs (junk for our centrals), gets k
 * of them acked (rptr moves) and leaves the other 4 - k queued. */
static void foreign_link_event(void)
{
	uint8_t k = (uint8_t)(hw.event % 4);

	for (uint8_t i = 0; i < 4; i++) {
		struct fake_entry *e = &hw.ring[i];

		e->hdr0 = LL_LLID_START;
		e->len = PAYLEN;
		e->data[0] = 0xEF;
		e->data[1] = JUNK;
		e->data[2] = 0x10;
	}
	hw.rptr = (uint8_t)((hw.rptr + k) & HW_PTR_MASK);
	hw.wptr = (uint8_t)((hw.rptr + 4 - k) & HW_PTR_MASK);
	hw.foreign_events++;
}

static void model_link_reset(uint8_t link)
{
	memset(&lk[link], 0, sizeof(lk[link]));   /* reset_sn_nesn, new central */
}

static void fake_power_on(uint8_t ptr)
{
	memset(&hw, 0, sizeof(hw));
	hw.rptr = ptr & HW_PTR_MASK;
	hw.wptr = ptr & HW_PTR_MASK;
	for (uint8_t l = 0; l < LL_MAX_CONN; l++) {
		model_link_reset(l);
	}
}

/* Hardware: the central's packet with header c_hdr was received; decide the
 * ack, pop, and send our response. Returns the index in hw.air; *is_new
 * tells whether the packet went into the RX FIFO (new data). */
static int hw_rx_and_respond(uint8_t link, uint8_t c_hdr, bool *is_new)
{
	uint8_t c_nesn = (c_hdr & HDR_NESN) ? 1 : 0;
	uint8_t c_sn = (c_hdr & HDR_SN) ? 1 : 0;
	uint8_t ref = hw.first ? hw.sn_init : hw.sn;
	bool acked = c_nesn != ref;
	struct air_pkt *p = &hw.air[hw.n_air];
	const struct fake_entry *e;
	static const struct fake_entry base = { .hdr0 = LL_LLID_CONT, .len = 0 };

	if (hw.first) {
		hw.nesn = hw.nesn_init;
	}
	hw.first = false;
	if (acked && hw.rptr != hw.wptr) {
		hw.rptr = (hw.rptr + 1) & HW_PTR_MASK;     /* point 4 */
	}
	hw.sn = acked ? !ref : ref;                         /* point 5 */
	*is_new = c_sn == hw.nesn;
	if (*is_new) {
		hw.nesn ^= 1;
	} else {
		lk[link].rx_dup++;
	}
	if (hw.rptr == hw.wptr) {
		e = &base;                                  /* point 1 */
		p->ring = -1;
	} else {
		e = &hw.ring[hw.rptr & 3];                  /* point 2 */
		p->ring = hw.rptr & 3;
	}
	p->hdr0 = (uint8_t)((e->hdr0 & 0x03) | (hw.nesn ? HDR_NESN : 0) |
			    (hw.sn ? HDR_SN : 0) |
			    (((hw.wptr - hw.rptr) & HW_PTR_MASK) > 1 ? HDR_MD : 0)); /* point 3 */
	p->len = e->len;
	memcpy(p->data, e->data, e->len);
	p->event = hw.event;
	p->link = link;
	return hw.n_air++;
}

/* Central of link (Core Spec): ack handling and new-data acceptance. Our
 * PDUs carry tag, 0xA0 + link, ~tag. */
static void central_rx(uint8_t link, const struct air_pkt *p)
{
	struct link_model *m = &lk[link];
	uint8_t nesn = (p->hdr0 & HDR_NESN) ? 1 : 0;
	uint8_t sn = (p->hdr0 & HDR_SN) ? 1 : 0;

	if (nesn != m->c.sn) {
		m->c.sn ^= 1;
	}
	if (sn == m->c.nesn) {
		m->c.nesn ^= 1;
		if (p->len) {
			bool bad = p->len < PAYLEN || p->data[1] != (uint8_t)(0xA0 + link) ||
				   p->data[2] != (uint8_t)~p->data[0];

			for (int i = PAYLEN; !bad && i < p->len; i++) {
				bad = p->data[i] != (uint8_t)(p->data[0] + i);
			}
			if (bad) {
				m->c.bad_content++;
			}
			if (m->c.n_len < MAX_GOT) {
				m->c.len[m->c.n_len++] = p->len;
			}
			if (m->c.n_got < MAX_GOT) {
				m->c.got[m->c.n_got++] = p->data[0];
			}
		}
	}
}

struct xchg {
	bool rx_lost;   /* the central's packet does not reach us */
	bool tx_lost;   /* our response does not reach the central */
	bool c_md;      /* the central's MD bit */
};

static bool use_txq = true;

/* One connection event (BRX command) of link. Script entries beyond n
 * default to "both packets received, central MD 0". Returns the number of
 * exchanges. */
static int run_event_l(uint8_t link, const struct xchg *x, int n)
{
	struct link_model *m = &lk[link];
	int i, done = 0;
	uint8_t data = 0;

	hw.event++;
	if (hw.foreign) {
		foreign_link_event();
	}
	if (use_txq) {
		ll_txq_event_start(link);
	} else {
		hw.nesn_init = m->nesn;   /* the model test is about TX only */
	}
	if (hw.sn_init != (m->sent_any ? m->sn : 0)) {
		m->sn_init_wrong++;
	}
	if (use_txq && hw.nesn_init != m->nesn) {
		m->nesn_init_wrong++;
	}
	for (uint8_t k = hw.rptr; k != hw.wptr; k = (k + 1) & HW_PTR_MASK) {
		data += hw.ring[k & 3].len ? 1 : 0;
	}
	if (data > hw.max_data_in_ring) {
		hw.max_data_in_ring = data;
	}
	hw.first = true;
	for (i = 0; i < MAX_XCHG; i++) {
		struct xchg e = i < n ? x[i] : (struct xchg){ 0 };
		uint8_t c_hdr = (uint8_t)(LL_LLID_CONT | (m->c.nesn ? HDR_NESN : 0) |
					  (m->c.sn ? HDR_SN : 0) | (e.c_md ? HDR_MD : 0));
		int a;
		bool is_new;

		if (e.rx_lost) {
			break;  /* RX timeout: the command ends */
		}
		a = hw_rx_and_respond(link, c_hdr, &is_new);
		if (use_txq && is_new) {
			ll_txq_rx(link, c_hdr);
		}
		done++;
		if (!e.tx_lost) {
			central_rx(link, &hw.air[a]);
		}
		if (!e.c_md && (e.tx_lost || !(hw.air[a].hdr0 & HDR_MD))) {
			break;  /* neither side has more data */
		}
	}
	if (use_txq) {
		ll_txq_event_end(link);
	}
	if (done) {
		m->sn = hw.sn;
		m->nesn = hw.nesn;
		m->sent_any = true;
	}
	return done;
}

static int run_event(const struct xchg *x, int n)
{
	return run_event_l(0, x, n);
}

static void run_ok_l(uint8_t link, int events)
{
	while (events--) {
		run_event_l(link, NULL, 0);
	}
}

static void run_ok(int events)
{
	run_ok_l(0, events);
}

/* ---------------- completion callback ---------------- */

static int done_bad_link;

static void done_cb(uint8_t link, enum ll_txq_kind kind, uint8_t ctrl_opcode, bool last)
{
	struct link_model *m;

	if (link >= LL_MAX_CONN) {
		done_bad_link++;
		return;
	}
	m = &lk[link];
	if (!last) {
		m->cpl.not_last++;
	}
	switch (kind) {
	case LL_TXQ_ACL:
		m->cpl.acl++;
		break;
	case LL_TXQ_CTRL:
		m->cpl.ctrl++;
		if (m->cpl.n_op < MAX_GOT) {
			m->cpl.op[m->cpl.n_op++] = ctrl_opcode;
		}
		break;
	default:
		m->cpl.empty++;
		break;
	}
}

/* A PDU of len (>= PAYLEN) bytes: tag, 0xA0 + link, ~tag, then tag + i. */
static int push_len_l(uint8_t link, enum ll_txq_kind kind, uint8_t tag, uint8_t len, bool last)
{
	uint8_t p[LL_DATA_PDU_MAX + LL_MIC_LEN] = { tag, (uint8_t)(0xA0 + link), (uint8_t)~tag };

	for (int i = PAYLEN; i < len; i++) {
		p[i] = (uint8_t)(tag + i);
	}
	return ll_txq_push(link, kind, kind == LL_TXQ_CTRL ? LL_LLID_CTRL : LL_LLID_START, p,
			   len, tag, last);
}

static int push_l(uint8_t link, enum ll_txq_kind kind, uint8_t tag)
{
	return push_len_l(link, kind, tag, PAYLEN, true);
}

static int push(enum ll_txq_kind kind, uint8_t tag)
{
	return push_l(0, kind, tag);
}

static void new_conn_l(uint8_t link)
{
	model_link_reset(link);
	ll_txq_reset(link);
}

/* fresh hardware and queue: all links reset */
static void boot(uint8_t ptr)
{
	fake_power_on(ptr);
	ll_txq_init(done_cb);
	done_bad_link = 0;
	for (uint8_t l = 0; l < LL_MAX_CONN; l++) {
		new_conn_l(l);
	}
}

static void new_conn(void)
{
	new_conn_l(0);
}

static void check_link_clean(uint8_t link)
{
	CHECK(lk[link].sn_init_wrong == 0);
	CHECK(lk[link].nesn_init_wrong == 0);
	CHECK(lk[link].c.bad_content == 0);
}

static void check_hw_clean(void)
{
	CHECK(hw.bad_write == 0);
	CHECK(hw.overfill == 0);
	CHECK(done_bad_link == 0);
	for (uint8_t l = 0; l < LL_MAX_CONN; l++) {
		check_link_clean(l);
	}
#ifdef LL_TXQ_SAFE_MODE
	CHECK(hw.max_data_in_ring <= 1);
#endif
}

/* ---------------- tests: single link (slice 3 behaviour) ---------------- */

/* The fake itself, driven without ll_txq: points 1-4 and the hazard that
 * motivates the placeholder rule (a head queued while the base is in flight
 * is popped unsent when the central acks the base). */
static void test_fake_model(void)
{
	use_txq = false;
	fake_power_on(0);
	hw.nesn_init = 0;
	ll_radio_conn_set_sn_init(0);
	run_event(NULL, 0);
	CHECK(hw.n_air == 1 && hw.air[0].ring == -1);           /* base when empty */
	CHECK((hw.air[0].hdr0 & (HDR_SN | HDR_MD)) == 0);

	/* two entries: ring slot rptr & 3 sent, MD 1 then 0, in-command pops */
	ll_radio_fifo_write(0, LL_LLID_START, (const uint8_t[]){ 1, 0xA0, 0xFE }, 3);
	ll_radio_fifo_write(1, LL_LLID_START, (const uint8_t[]){ 2, 0xA0, 0xFD }, 3);
	ll_radio_fifo_set_wptr(2);
	ll_radio_conn_set_sn_init(0);   /* last TX had SN 0 */
	run_event(NULL, 0);
	/* central acked the base: NESN 1 != SN_INIT 0, pops entry 0 unsent */
	CHECK(hw.rptr == 1);
	CHECK(lk[0].c.n_got == 1 && lk[0].c.got[0] == 2);         /* entry 1 only */
	CHECK(hw.air[1].ring == 1 && !(hw.air[1].hdr0 & HDR_MD));

	/* retransmission with the same SN when not acked */
	fake_power_on(0);
	ll_radio_fifo_write(0, LL_LLID_START, (const uint8_t[]){ 7, 0xA0, 0xF8 }, 3);
	ll_radio_fifo_set_wptr(1);
	ll_radio_conn_set_sn_init(0);
	run_event((const struct xchg[]){ { .tx_lost = true } }, 1);
	ll_radio_conn_set_sn_init(0);
	run_event(NULL, 0);
	CHECK(hw.air[0].ring == 0 && hw.air[1].ring == 0);
	CHECK((hw.air[0].hdr0 & HDR_SN) == (hw.air[1].hdr0 & HDR_SN));
	CHECK(lk[0].c.n_got == 1 && lk[0].c.got[0] == 7);
	CHECK(hw.rptr == 0);
	ll_radio_conn_set_sn_init(0);
	run_event(NULL, 0);
	CHECK(hw.rptr == 1 && hw.air[2].ring == -1);
	use_txq = true;
}

static void test_single_acl(void)
{
	boot(0);
	CHECK(ll_txq_backlog(0) == 0);
	CHECK(push(LL_TXQ_ACL, 1) == 0);
	CHECK(ll_txq_backlog(0) == 1);
	run_event(NULL, 0);
	CHECK(lk[0].c.n_got == 1 && lk[0].c.got[0] == 1);
	CHECK(lk[0].cpl.acl == 0);            /* ack arrives in the next event */
	CHECK(ll_txq_backlog(0) == 1);
	run_event(NULL, 0);
	CHECK(lk[0].cpl.acl == 1 && lk[0].cpl.ctrl == 0 && lk[0].cpl.empty == 0);
	CHECK(ll_txq_backlog(0) == 0);
	CHECK(hw.air[hw.n_air - 1].ring == -1);
	run_ok(3);
	CHECK(lk[0].cpl.acl == 1 && lk[0].c.n_got == 1);
	CHECK(hw.sn_init_calls == 5);
	check_hw_clean();
}

/* Task 9 device finding: NESN init is taken per BRX command. Every new
 * central packet must reach the RX FIFO (ll_txq_rx), in every event, and a
 * genuine retransmission (our ack lost) must be dropped without breaking
 * the expected SN of the next command. */
static void test_nesn_init_per_event(void)
{
	boot(0);
	run_ok(6);
	CHECK(hw.nesn_init_calls == 6);
	CHECK(lk[0].rx_dup == 0);
	CHECK(lk[0].nesn_init_wrong == 0);
	/* our response lost: the central resends the same SN next event */
	run_event((const struct xchg[]){ { .tx_lost = true } }, 1);
	run_ok(3);
	CHECK(lk[0].rx_dup == 1);
	CHECK(lk[0].nesn_init_wrong == 0);
	CHECK(lk[0].sn_init_wrong == 0);
}

/* Task 9 device finding: the TX FIFO pointers wrap at 32. Sending one PDU
 * at a time past the wrap must keep working (on the device the queue
 * stalled after about 20 PDUs: no more acks, LL_UNKNOWN_RSP never sent). */
static void test_pointer_wrap(void)
{
	boot(0x1c);
	for (uint8_t t = 0; t < 40; t++) {
		CHECK(push(LL_TXQ_ACL, t) == 0);
		run_ok(3);
	}
	CHECK(lk[0].c.n_got == 40);
	CHECK(lk[0].cpl.acl == 40);
	CHECK(ll_txq_backlog(0) == 0);
	check_hw_clean();
}

static void test_nack_retransmit(void)
{
	boot(0);
	run_ok(2);                       /* base in flight */
	CHECK(push(LL_TXQ_ACL, 9) == 0);
	run_event((const struct xchg[]){ { .tx_lost = true } }, 1);
	CHECK(lk[0].c.n_got == 0 && lk[0].cpl.acl == 0);
	/* the central's next packet nacks: same data, same SN */
	run_event((const struct xchg[]){ { .tx_lost = true } }, 1);
	CHECK(lk[0].c.n_got == 0 && lk[0].cpl.acl == 0);
	run_event(NULL, 0);
	CHECK(lk[0].c.n_got == 1 && lk[0].c.got[0] == 9 && lk[0].cpl.acl == 0);
	run_event((const struct xchg[]){ { .rx_lost = true } }, 1); /* timeout */
	CHECK(lk[0].cpl.acl == 0);
	run_ok(1);
	CHECK(lk[0].cpl.acl == 1 && lk[0].c.n_got == 1);
	run_ok(3);
	CHECK(lk[0].cpl.acl == 1 && lk[0].c.n_got == 1);
	CHECK(ll_txq_backlog(0) == 0);
	check_hw_clean();
}

static void test_backlog_md(void)
{
	int ev = 0;
	const int n = LL_TXQ_ENTRIES;

	boot(0);
	for (uint8_t t = 1; t <= n; t++) {
		CHECK(push(LL_TXQ_CTRL, t) == 0);
	}
	CHECK(push(LL_TXQ_CTRL, 99) == -ENOMEM);   /* LL_TXQ_ENTRIES per link */
	CHECK(ll_txq_backlog(0) == (unsigned int)n);
	run_event(NULL, 0);
	ev++;
#ifndef LL_TXQ_SAFE_MODE
	/* four ring entries in one event, MD 1, 1, 1, 0, chained by our MD */
	CHECK(hw.n_air == 4);
	for (int i = 0; i < 4; i++) {
		CHECK(hw.air[i].ring == i);
		CHECK(!!(hw.air[i].hdr0 & HDR_MD) == (i < 3));
	}
	CHECK(lk[0].c.n_got == 4);
	CHECK(lk[0].cpl.ctrl == 3);            /* the 4th is acked next event */
#else
	CHECK(hw.n_air == 1 && lk[0].c.n_got == 1 && !(hw.air[0].hdr0 & HDR_MD));
#endif
	/* the acks made room again (safe mode: after the next event) */
	if (SAFE_MODE_ON) {
		CHECK(push(LL_TXQ_CTRL, (uint8_t)(n + 1)) == -ENOMEM);
		run_event(NULL, 0);
		ev++;
	}
	CHECK(push(LL_TXQ_CTRL, (uint8_t)(n + 1)) == 0);
	while (ll_txq_backlog(0) && ev < 100) {
		run_event(NULL, 0);
		ev++;
	}
	CHECK(lk[0].c.n_got == n + 1 && lk[0].cpl.ctrl == n + 1);
	for (int i = 0; i < n + 1; i++) {
		CHECK(lk[0].c.got[i] == i + 1);
		CHECK(lk[0].cpl.op[i] == i + 1);
	}
#ifndef LL_TXQ_SAFE_MODE
	CHECK(ev == 7);                  /* 4, then 3 new per event (one unacked) */
#else
	CHECK(ev == 2 * (n + 1));        /* sent in one event, acked in the next */
#endif
	check_hw_clean();
}

/* Ring ran empty (base sent last), then new data: placeholder in front.
 * With foreign, another link's event runs before each of ours (the ring is
 * rebuilt at a moved rptr, over junk). */
static void test_ring_empty_then_data(bool foreign)
{
	/* a) the central acks the base: the placeholder is popped, data sent */
	boot(foreign ? 0x1e : 0);
	hw.foreign = foreign;
	push(LL_TXQ_ACL, 1);
	run_ok(2);                       /* 1 sent, acked; base in flight */
	CHECK(hw.air[hw.n_air - 1].ring == -1);
	push(LL_TXQ_ACL, 2);
	run_event(NULL, 0);
	CHECK(lk[0].c.n_got == 2 && lk[0].c.got[1] == 2);
#ifndef LL_TXQ_SAFE_MODE
	CHECK(((hw.wptr - hw.rptr) & HW_PTR_MASK) == 1);   /* placeholder popped, data in flight */
#endif
	run_ok(1);
	CHECK(lk[0].cpl.acl == 2 && lk[0].cpl.empty == 0);
	check_hw_clean();

	/* b) our base response was lost: the placeholder is resent (as the
	 * base would be, same SN), the central continues on our MD */
	boot(foreign ? 0x1f : 0);
	hw.foreign = foreign;
	push(LL_TXQ_ACL, 1);
	run_event(NULL, 0);
	run_event((const struct xchg[]){ { .tx_lost = true } }, 1); /* base lost */
	push(LL_TXQ_ACL, 2);
	run_event(NULL, 0);
	{
		int n = hw.n_air;

		CHECK(hw.air[n - 2].ring >= 0 && hw.air[n - 2].len == 0); /* placeholder */
		CHECK(hw.air[n - 2].hdr0 & HDR_MD);
		CHECK((hw.air[n - 2].hdr0 & HDR_SN) == (hw.air[n - 3].hdr0 & HDR_SN));
		CHECK(hw.air[n - 1].len == PAYLEN && hw.air[n - 1].data[0] == 2);
	}
	run_ok(1);
	CHECK(lk[0].c.n_got == 2 && lk[0].c.got[0] == 1 && lk[0].c.got[1] == 2);
	CHECK(lk[0].cpl.acl == 2 && ll_txq_backlog(0) == 0);
	check_hw_clean();

	/* c) first RX lost right after queueing, central MD chains, then data */
	boot(foreign ? 0x1d : 0);
	hw.foreign = foreign;
	run_event((const struct xchg[]){ { .c_md = true }, { .c_md = true } }, 2);
	push(LL_TXQ_ACL, 5);
	push(LL_TXQ_ACL, 6);
	run_event((const struct xchg[]){ { .rx_lost = true } }, 1);
	run_event((const struct xchg[]){ { .c_md = true }, { .tx_lost = true } }, 2);
	run_ok(4);
	CHECK(lk[0].c.n_got == 2 && lk[0].c.got[0] == 5 && lk[0].c.got[1] == 6);
	CHECK(lk[0].cpl.acl == 2 && ll_txq_backlog(0) == 0);
	check_hw_clean();
	if (foreign) {
		CHECK(hw.foreign_events > 0);
	}
	hw.foreign = false;
}

static uint32_t rng = 12345;

static uint32_t rnd(void)
{
	rng = rng * 1103515245u + 12345u;
	return rng >> 16;
}

static void random_script(struct xchg *x, int n)
{
	for (int i = 0; i < n; i++) {
		uint32_t r = rnd() % 16;

		x[i].rx_lost = r == 0;
		x[i].tx_lost = r == 1 || r == 2;
		x[i].c_md = (rnd() % 3) == 0;
	}
}

/* Our SN across events, invisible base acks, NACKs, lost packets and MD
 * chains of both sides, 5-bit pointer wrap: SN_INIT always equals the SN of
 * our last TX (checked in run_event), every PDU is delivered exactly once,
 * in order, and completed exactly once, in order. With foreign, every event
 * of ours follows another link's event that moved rptr and left entries. */
static void test_sn_tracking_random(uint8_t start_ptr, uint32_t seed, bool foreign, bool big)
{
	uint8_t next_tag = 0;
	int pushed = 0;

	rng = seed;
	boot(start_ptr);
	hw.foreign = foreign;
	for (int ev = 0; ev < 3000; ev++) {
		struct xchg x[MAX_XCHG];
		int n = (int)(rnd() % 6);

		random_script(x, n);
		if (rnd() % 3 == 0) {
			int k = (int)(rnd() % 4);

			while (k-- && push_len_l(0, LL_TXQ_CTRL, next_tag,
						 big ? (uint8_t)(PAYLEN + rnd() % 253) : PAYLEN,
						 true) == 0) {
				next_tag++;
				pushed++;
			}
		}
		run_event(x, n);
		if (pushed > 900) {
			break;
		}
	}
	for (int ev = 0; ev < 200 && ll_txq_backlog(0); ev++) {
		run_event(NULL, 0);
	}
	run_ok(1);
	CHECK(pushed > 300);
	CHECK(lk[0].c.n_got == pushed);
	CHECK(lk[0].cpl.ctrl == pushed);
	for (int i = 0; i < lk[0].c.n_got && i < pushed; i++) {
		CHECK(lk[0].c.got[i] == (uint8_t)i);
		CHECK(lk[0].cpl.op[i] == (uint8_t)i);
	}
	CHECK(ll_txq_backlog(0) == 0);
	check_hw_clean();
	hw.foreign = false;
}

static void test_reset_per_connection(void)
{
	int air0;

	boot(0xFD);
	for (uint8_t t = 1; t <= 6; t++) {
		push(LL_TXQ_ACL, t);
	}
	run_event((const struct xchg[]){ { .tx_lost = true } }, 1);
	CHECK(hw.wptr != hw.rptr);       /* unacked entries left in the ring */

	/* new connection: old entries dropped, no completions; the ring is
	 * emptied at the next event start, before anything is sent */
	new_conn();
	CHECK(ll_txq_backlog(0) == 0);
	CHECK(lk[0].cpl.acl == 0);
	air0 = hw.n_air;
	push(LL_TXQ_ACL, 42);
	run_ok(3);
	CHECK(lk[0].c.n_got == 1 && lk[0].c.got[0] == 42);
	CHECK(lk[0].cpl.acl == 1);
	for (int i = air0; i < hw.n_air; i++) {
		CHECK(hw.air[i].len == 0 || hw.air[i].data[0] == 42);
	}
	/* first event of the new connection: SN_INIT 0 (reset_sn_nesn) */
	CHECK(hw.air[air0].ring >= 0 && !(hw.air[air0].hdr0 & HDR_SN));
	CHECK(push(LL_TXQ_ACL, 1) == 0);
	check_hw_clean();
}

/* ---------------- tests: long PDUs (slice 6b Task 4) ---------------- */

/* Maximum PDUs (251 + MIC = 255 bytes): the byte area takes
 * LL_TXQ_POOL_BYTES / 256 of them; each is sent intact (content checked
 * byte by byte by the central model), also when the ring is rebuilt after
 * a NACK and after another link moved rptr (foreign). */
static void test_big_pdus(void)
{
	const int cap = LL_TXQ_POOL_BYTES / 256;
	int k;

	for (int f = 0; f < 2; f++) {
		boot(f ? 0x1d : 0);
		hw.foreign = f;
		for (k = 0; k < 64; k++) {
			if (push_len_l(0, LL_TXQ_ACL, (uint8_t)k, 255, true) != 0) {
				break;
			}
		}
		CHECK(k == cap);
		CHECK(push_len_l(0, LL_TXQ_ACL, 99, 255, true) == -ENOMEM);
		CHECK(push_len_l(0, LL_TXQ_ACL, 99, 1 + 252, true) == -ENOMEM);   /* rounded to 256 */
		CHECK(ll_txq_backlog(0) == (unsigned int)cap);
		/* our response lost twice: the same entries are rebuilt and resent */
		run_event((const struct xchg[]){ { .tx_lost = true } }, 1);
		run_event((const struct xchg[]){ { .tx_lost = true } }, 1);
		for (int ev = 0; ev < 40 && ll_txq_backlog(0); ev++) {
			run_event(NULL, 0);
			/* refill as the acks make room: the area wraps */
			if (k < 3 * cap && push_len_l(0, LL_TXQ_ACL, (uint8_t)k, 255, true) == 0) {
				k++;
			}
		}
		run_ok(1);
		CHECK(lk[0].c.n_got == k && lk[0].cpl.acl == k);
		for (int i = 0; i < k && i < lk[0].c.n_got; i++) {
			CHECK(lk[0].c.got[i] == i && lk[0].c.len[i] == 255);
		}
		CHECK(k > cap);
		CHECK(ll_txq_backlog(0) == 0);
		check_hw_clean();
	}
	hw.foreign = false;
}

/* ll_txq_fits: all or nothing for the fragments of one host packet. */
static void test_fits(void)
{
	const int cap = LL_TXQ_POOL_BYTES / 256;

	boot(0);
	CHECK(!ll_txq_fits(LL_MAX_CONN, 1, 27, 27));
	CHECK(!ll_txq_fits(0, 0, 27, 27));
	CHECK(ll_txq_fits(0, 1, 0, 255));
	/* 251 octets in 27-octet fragments, encrypted: 9 x 31 + 1 x 12 */
	CHECK(ll_txq_fits(0, 10, 31, 12));
	CHECK(ll_txq_fits(0, LL_TXQ_ENTRIES, 4, 4));
	CHECK(!ll_txq_fits(0, LL_TXQ_ENTRIES + 1, 4, 4));
	CHECK(ll_txq_fits(0, (uint8_t)cap, 255, 255));
	CHECK(!ll_txq_fits(0, (uint8_t)(cap + 1), 255, 4));
	/* entries in use count */
	for (int i = 0; i < LL_TXQ_ENTRIES - 3; i++) {
		CHECK(push(LL_TXQ_ACL, (uint8_t)i) == 0);
	}
	CHECK(ll_txq_fits(0, 3, 31, 31));
	CHECK(!ll_txq_fits(0, 4, 31, 31));
	/* bytes in use count: one maximum PDU less than the area holds */
	boot(0);
	for (int i = 0; i < cap - 1; i++) {
		CHECK(push_len_l(0, LL_TXQ_ACL, (uint8_t)i, 255, true) == 0);
	}
	CHECK(ll_txq_fits(0, 1, 255, 255));
	CHECK(ll_txq_fits(0, 8, 32, 32));
	CHECK(!ll_txq_fits(0, 9, 32, 32));
	CHECK(!ll_txq_fits(0, 2, 255, 1));
	/* what fits is accepted, one by one */
	for (int i = 0; i < 8; i++) {
		CHECK(push_len_l(0, LL_TXQ_ACL, (uint8_t)(100 + i), 32, i == 7) == 0);
	}
	CHECK(!ll_txq_fits(0, 1, 1, 1));
	CHECK(push_len_l(0, LL_TXQ_ACL, 1, 1, true) == -ENOMEM);
	/* fits only when the area is free at the end or at the start (no
	 * record wraps): after the first acks the start is free again */
	for (int ev = 0; ev < 3; ev++) {
		run_event(NULL, 0);
	}
	CHECK(ll_txq_fits(0, 1, 255, 255));
	check_hw_clean();
}

/* The completion carries the last flag of its push; it never changes the
 * on-air content or order. */
static void test_last_flag(void)
{
	boot(0);
	for (uint8_t t = 0; t < 10; t++) {
		CHECK(push_len_l(0, LL_TXQ_ACL, t, 31, t == 9) == 0);
	}
	CHECK(push_len_l(0, LL_TXQ_CTRL, 50, PAYLEN, true) == 0);
	for (int ev = 0; ev < 60 && ll_txq_backlog(0); ev++) {
		run_event(NULL, 0);
	}
	run_ok(1);
	CHECK(lk[0].cpl.acl == 10 && lk[0].cpl.not_last == 9 && lk[0].cpl.ctrl == 1);
	CHECK(lk[0].c.n_got == 11);
	check_hw_clean();
}

/* ---------------- tests: per-link state (slice 6a) ---------------- */

static void test_link_bounds(void)
{
	boot(0);
	CHECK(push_l(LL_MAX_CONN, LL_TXQ_ACL, 1) == -EINVAL);
	CHECK(ll_txq_backlog(LL_MAX_CONN) == 0);
	/* out-of-range ids are ignored by the ISR entry points */
	ll_txq_event_start(LL_MAX_CONN);
	ll_txq_rx(LL_MAX_CONN, 0);
	ll_txq_event_end(LL_MAX_CONN);
	ll_txq_reset(LL_MAX_CONN);
	for (uint8_t l = 0; l < LL_MAX_CONN; l++) {
		CHECK(push_l(l, LL_TXQ_ACL, l) == 0);
		CHECK(ll_txq_backlog(l) == 1);
	}
	/* a reset of one link leaves the others' queues alone */
	ll_txq_reset(LL_MAX_CONN - 1);
	CHECK(ll_txq_backlog(LL_MAX_CONN - 1) == 0);
	for (uint8_t l = 0; l + 1 < LL_MAX_CONN; l++) {
		CHECK(ll_txq_backlog(l) == 1);
	}
	CHECK(hw.bad_write == 0 && hw.overfill == 0);
}

#if LL_MAX_CONN >= 2
#define LA 0
#define LB (LL_MAX_CONN - 1)

/* Interleaved events of two links with different backlogs: each link's
 * acks complete only its own entries, each central gets only its own PDUs. */
static void test_two_links_interleaved(void)
{
	int a_cpl, b_cpl;

	boot(0x1b);   /* wraps the 5-bit pointers during the test */
	for (uint8_t t = 0; t < 6; t++) {
		CHECK(push_l(LA, LL_TXQ_ACL, t) == 0);
	}
	CHECK(push_l(LB, LL_TXQ_CTRL, 100) == 0);
	CHECK(push_l(LB, LL_TXQ_CTRL, 101) == 0);
	CHECK(ll_txq_backlog(LA) == 6 && ll_txq_backlog(LB) == 2);

	for (int ev = 0; ev < 60; ev++) {
		uint8_t l = (ev & 1) ? LB : LA;
		uint8_t o = (ev & 1) ? LA : LB;

		a_cpl = lk[o].cpl.acl + lk[o].cpl.ctrl;
		/* a lost response now and then, so entries stay unacked across
		 * the other link's event */
		if (ev % 5 == 2) {
			run_event_l(l, (const struct xchg[]){ { .tx_lost = true } }, 1);
		} else {
			run_event_l(l, NULL, 0);
		}
		b_cpl = lk[o].cpl.acl + lk[o].cpl.ctrl;
		CHECK(a_cpl == b_cpl);   /* this event completed nothing of the other link */
	}
	CHECK(lk[LA].c.n_got == 6);
	for (int i = 0; i < 6 && i < lk[LA].c.n_got; i++) {
		CHECK(lk[LA].c.got[i] == i);
	}
	CHECK(lk[LB].c.n_got == 2 && lk[LB].c.got[0] == 100 && lk[LB].c.got[1] == 101);
	CHECK(lk[LA].cpl.acl == 6 && lk[LA].cpl.ctrl == 0);
	CHECK(lk[LB].cpl.ctrl == 2 && lk[LB].cpl.acl == 0);
	CHECK(lk[LB].cpl.op[0] == 100 && lk[LB].cpl.op[1] == 101);
	CHECK(ll_txq_backlog(LA) == 0 && ll_txq_backlog(LB) == 0);
	check_hw_clean();
}

/* The other link's event moves rptr and leaves its own entries queued
 * between ours: the ring is rebuilt at the new rptr from the software
 * copies, including a placeholder in flight (base_last) and the unacked
 * tail, across the 5-bit wrap. */
static void test_rebuild_after_other_link(void)
{
	int base_i, s;

	boot(0x1f);
	/* LA: 1 sent and acked, base in flight (base_last) */
	push_l(LA, LL_TXQ_ACL, 1);
	run_ok_l(LA, 2);
	CHECK(hw.air[hw.n_air - 1].ring == -1 && hw.air[hw.n_air - 1].link == LA);
	/* LB leaves unacked entries in the ring (its response lost) */
	for (uint8_t t = 50; t < 54; t++) {
		push_l(LB, LL_TXQ_CTRL, t);
	}
	run_event_l(LB, (const struct xchg[]){ { .tx_lost = true } }, 1);
	CHECK(((hw.wptr - hw.rptr) & HW_PTR_MASK) == (SAFE_MODE_ON ? 1 : 4));
	/* LA: our base response is lost, then new data: the placeholder in
	 * front must be resent with the base's SN, at LB's rptr */
	run_event_l(LA, (const struct xchg[]){ { .tx_lost = true } }, 1);
	base_i = hw.n_air - 1;
	CHECK(hw.air[base_i].ring == -1);
	push_l(LA, LL_TXQ_ACL, 2);
	push_l(LA, LL_TXQ_ACL, 3);
	run_event_l(LB, (const struct xchg[]){ { .rx_lost = true } }, 1);
	CHECK(hw.wptr != hw.rptr);       /* LB's entries are queued again */
	s = hw.n_air;
	run_event_l(LA, NULL, 0);
	CHECK(hw.n_air >= s + 2);
	CHECK(hw.air[s].link == LA && hw.air[s].ring >= 0 && hw.air[s].len == 0);
	CHECK((hw.air[s].hdr0 & HDR_SN) == (hw.air[base_i].hdr0 & HDR_SN));
	CHECK(hw.air[s].hdr0 & HDR_MD);
	CHECK(hw.air[s + 1].len == PAYLEN && hw.air[s + 1].data[0] == 2);
	CHECK(lk[LA].c.n_got == (SAFE_MODE_ON ? 2 : 3) && lk[LA].c.got[1] == 2);
	/* drain both, alternating */
	for (int ev = 0; ev < 60 && (ll_txq_backlog(LA) || ll_txq_backlog(LB)); ev++) {
		run_event_l((ev & 1) ? LB : LA, NULL, 0);
	}
	run_ok_l(LA, 1);
	run_ok_l(LB, 1);
	CHECK(lk[LA].c.n_got == 3);
	for (int i = 0; i < 3 && i < lk[LA].c.n_got; i++) {
		CHECK(lk[LA].c.got[i] == i + 1);
	}
	CHECK(lk[LB].c.n_got == 4);
	for (int i = 0; i < 4 && i < lk[LB].c.n_got; i++) {
		CHECK(lk[LB].c.got[i] == 50 + i);
	}
	CHECK(lk[LA].cpl.acl == 3 && lk[LB].cpl.ctrl == 4);
	CHECK(lk[LA].cpl.empty == 0 && lk[LB].cpl.empty == 0);
	check_hw_clean();
}

/* Up to 3 links (the first, the middle, the last id), random event order,
 * random losses/NACKs/MD, random pushes, foreign events in between, pointer
 * wrap: per link, SN_INIT/NESN_INIT always right, every PDU delivered once
 * and in order to its own central only, completed once and in order. */
static void test_links_random(uint32_t seed, bool foreign, bool big)
{
	uint8_t ids[3] = { 0, LL_MAX_CONN / 2, LL_MAX_CONN - 1 };
	int nl = LL_MAX_CONN >= 3 ? 3 : 2;
	uint8_t next_tag[3] = { 0 };
	int pushed[3] = { 0 };

	if (nl == 2) {
		ids[1] = LL_MAX_CONN - 1;
	}
	rng = seed;
	boot((uint8_t)(seed & 0x1f));
	for (int ev = 0; ev < 6000; ev++) {
		struct xchg x[MAX_XCHG];
		int n = (int)(rnd() % 6);
		int j = (int)(rnd() % (uint32_t)nl);

		hw.foreign = foreign && (rnd() % 4 == 0);
		random_script(x, n);
		for (int p = 0; p < nl; p++) {
			if (rnd() % 5 == 0) {
				int k = (int)(rnd() % 4);

				while (k-- && pushed[p] < 600 &&
				       push_len_l(ids[p], (p & 1) ? LL_TXQ_ACL : LL_TXQ_CTRL,
						  next_tag[p],
						  big ? (uint8_t)(PAYLEN + rnd() % 253) : PAYLEN,
						  true) == 0) {
					next_tag[p]++;
					pushed[p]++;
				}
			}
		}
		run_event_l(ids[j], x, n);
	}
	hw.foreign = false;
	for (int ev = 0; ev < 600; ev++) {
		bool any = false;

		for (int p = 0; p < nl; p++) {
			any |= ll_txq_backlog(ids[p]) != 0;
		}
		if (!any) {
			break;
		}
		run_event_l(ids[ev % nl], NULL, 0);
	}
	for (int p = 0; p < nl; p++) {
		struct link_model *m = &lk[ids[p]];
		int got_cpl = (p & 1) ? m->cpl.acl : m->cpl.ctrl;

		run_ok_l(ids[p], 1);
		CHECK(pushed[p] > 200);
		CHECK(m->c.n_got == pushed[p]);
		got_cpl = (p & 1) ? m->cpl.acl : m->cpl.ctrl;
		CHECK(got_cpl == pushed[p]);
		for (int i = 0; i < m->c.n_got && i < pushed[p]; i++) {
			CHECK(m->c.got[i] == (uint8_t)i);
		}
		if (!(p & 1)) {
			for (int i = 0; i < m->cpl.n_op && i < pushed[p]; i++) {
				CHECK(m->cpl.op[i] == (uint8_t)i);
			}
		}
		CHECK(ll_txq_backlog(ids[p]) == 0);
	}
	check_hw_clean();
}
#endif /* LL_MAX_CONN >= 2 */

int main(void)
{
	printf("(LL_MAX_CONN %d%s)\n", LL_MAX_CONN,
#ifdef LL_TXQ_SAFE_MODE
	       ", LL_TXQ_SAFE_MODE"
#else
	       ""
#endif
	);
	test_fake_model();
	test_single_acl();
	test_nesn_init_per_event();
	test_pointer_wrap();
	test_nack_retransmit();
	test_backlog_md();
	test_ring_empty_then_data(false);
	test_ring_empty_then_data(true);
	test_sn_tracking_random(0, 12345, false, false);
	test_sn_tracking_random(0xF0, 777, false, false);
	test_sn_tracking_random(0x7F, 31337, false, false);
	test_sn_tracking_random(0x1c, 4242, true, false);
	test_sn_tracking_random(0x03, 9001, true, false);
	/* slice 6b: PDUs of 3..255 bytes through the byte area (wraps) */
	test_sn_tracking_random(0x11, 2468, false, true);
	test_sn_tracking_random(0x1e, 8642, true, true);
	test_big_pdus();
	test_fits();
	test_last_flag();
	test_reset_per_connection();
	test_link_bounds();
#if LL_MAX_CONN >= 2
	test_two_links_interleaved();
	test_rebuild_after_other_link();
	test_links_random(55, false, false);
	test_links_random(1234567, true, false);
	test_links_random(0xC0FFEE, true, false);
	test_links_random(97531, true, true);
	test_links_random(0xBEEF, false, true);
#endif
	DONE();
}
