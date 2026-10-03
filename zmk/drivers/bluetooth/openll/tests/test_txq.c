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
 * Our SN/NESN follow the Core Spec (reset_sn_nesn: both 0). The central is
 * modelled per event as a script of exchanges (its packet lost or not, our
 * response lost or not, its MD bit) with Core Spec SN/NESN handling, and it
 * records every data PDU it accepts as new, so loss and duplicates show.
 *
 * Built twice by run_host_tests.sh: normal and with -DLL_TXQ_SAFE_MODE.
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
#define MAX_AIR  4096
#define MAX_GOT  1024
#define PAYLEN   3

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
};

static struct {
	uint8_t rptr, wptr;
	struct fake_entry ring[4];
	uint8_t sn_init;
	uint8_t sn, nesn;     /* SN of our last TX, our NESN */
	bool sent_any;        /* a packet was sent in this connection */
	bool first;           /* next RX is the first of the command */
	int bad_write;        /* write into an entry not yet popped */
	int overfill;         /* wptr - rptr > 4 */
	int sn_init_calls;
	int sn_init_wrong;    /* SN_INIT != SN of our last TX at command start */
	int max_data_in_ring; /* non-empty entries queued at a command start */
	struct air_pkt air[MAX_AIR];
	int n_air;
	int event;
} hw;

static struct {
	uint8_t sn, nesn;
	uint8_t got[MAX_GOT];  /* tags of data PDUs accepted as new */
	int n_got;
	int bad_content;
} central;

void ll_radio_conn_set_sn_init(uint8_t sn)
{
	hw.sn_init = sn & 1;
	hw.sn_init_calls++;
}

uint8_t ll_radio_fifo_rptr(void)
{
	return hw.rptr;
}

uint8_t ll_radio_fifo_wptr(void)
{
	return hw.wptr;
}

void ll_radio_fifo_write(uint8_t idx, uint8_t hdr0, const uint8_t *payload, uint8_t len)
{
	if ((uint8_t)(idx - hw.rptr) < (uint8_t)(hw.wptr - hw.rptr)) {
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
	if ((uint8_t)(wptr - hw.rptr) > 4) {
		hw.overfill++;
	}
	hw.wptr = wptr;
}

/* Per connection (ll_radio_conn_setup): reset_sn_nesn; rptr cannot be reset
 * and wptr is left as is. */
static void fake_conn_setup(void)
{
	hw.sn = 0;
	hw.nesn = 0;
	hw.sent_any = false;
	central.sn = 0;
	central.nesn = 0;
	central.n_got = 0;
	central.bad_content = 0;
}

static void fake_power_on(uint8_t ptr)
{
	memset(&hw, 0, sizeof(hw));
	hw.rptr = ptr;
	hw.wptr = ptr;
	fake_conn_setup();
}

/* Hardware: the central's packet with header c_hdr was received; decide the
 * ack, pop, and send our response. Returns the index in hw.air. */
static int hw_rx_and_respond(uint8_t c_hdr)
{
	uint8_t c_nesn = (c_hdr & HDR_NESN) ? 1 : 0;
	uint8_t c_sn = (c_hdr & HDR_SN) ? 1 : 0;
	uint8_t ref = hw.first ? hw.sn_init : hw.sn;
	bool acked = c_nesn != ref;
	struct air_pkt *p = &hw.air[hw.n_air];
	const struct fake_entry *e;
	static const struct fake_entry base = { .hdr0 = LL_LLID_CONT, .len = 0 };

	hw.first = false;
	if (acked && hw.rptr != hw.wptr) {
		hw.rptr++;                                  /* point 4 */
	}
	hw.sn = acked ? !ref : ref;                         /* point 5 */
	if (c_sn == hw.nesn) {
		hw.nesn ^= 1;
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
			    ((uint8_t)(hw.wptr - hw.rptr) > 1 ? HDR_MD : 0)); /* point 3 */
	p->len = e->len;
	memcpy(p->data, e->data, e->len);
	p->event = hw.event;
	hw.sent_any = true;
	return hw.n_air++;
}

/* Central (Core Spec): ack handling and new-data acceptance. */
static void central_rx(const struct air_pkt *p)
{
	uint8_t nesn = (p->hdr0 & HDR_NESN) ? 1 : 0;
	uint8_t sn = (p->hdr0 & HDR_SN) ? 1 : 0;

	if (nesn != central.sn) {
		central.sn ^= 1;
	}
	if (sn == central.nesn) {
		central.nesn ^= 1;
		if (p->len) {
			if (p->len != PAYLEN || p->data[1] != 0xA0 ||
			    p->data[2] != (uint8_t)~p->data[0]) {
				central.bad_content++;
			}
			central.got[central.n_got++] = p->data[0];
		}
	}
}

struct xchg {
	bool rx_lost;   /* the central's packet does not reach us */
	bool tx_lost;   /* our response does not reach the central */
	bool c_md;      /* the central's MD bit */
};

static bool use_txq = true;

/* One connection event (BRX command). Script entries beyond n default to
 * "both packets received, central MD 0". Returns the number of exchanges. */
static int run_event(const struct xchg *x, int n)
{
	int i, done = 0;
	uint8_t data = 0;

	hw.event++;
	if (use_txq) {
		ll_txq_event_start();
	}
	if (hw.sn_init != (hw.sent_any ? hw.sn : 0)) {
		hw.sn_init_wrong++;
	}
	for (uint8_t k = hw.rptr; k != hw.wptr; k++) {
		data += hw.ring[k & 3].len ? 1 : 0;
	}
	if (data > hw.max_data_in_ring) {
		hw.max_data_in_ring = data;
	}
	hw.first = true;
	for (i = 0; i < MAX_XCHG; i++) {
		struct xchg e = i < n ? x[i] : (struct xchg){ 0 };
		uint8_t c_hdr = (uint8_t)(LL_LLID_CONT | (central.nesn ? HDR_NESN : 0) |
					  (central.sn ? HDR_SN : 0) | (e.c_md ? HDR_MD : 0));
		int a;

		if (e.rx_lost) {
			break;  /* RX timeout: the command ends */
		}
		a = hw_rx_and_respond(c_hdr);
		if (use_txq) {
			ll_txq_rx(c_hdr);
		}
		done++;
		if (!e.tx_lost) {
			central_rx(&hw.air[a]);
		}
		if (!e.c_md && (e.tx_lost || !(hw.air[a].hdr0 & HDR_MD))) {
			break;  /* neither side has more data */
		}
	}
	if (use_txq) {
		ll_txq_event_end();
	}
	return done;
}

static void run_ok(int events)
{
	while (events--) {
		run_event(NULL, 0);
	}
}

/* ---------------- completion callback ---------------- */

static struct {
	int acl, ctrl, empty;
	uint8_t op[MAX_GOT];
	int n_op;
} cpl;

static void done_cb(enum ll_txq_kind kind, uint8_t ctrl_opcode)
{
	switch (kind) {
	case LL_TXQ_ACL:
		cpl.acl++;
		break;
	case LL_TXQ_CTRL:
		cpl.ctrl++;
		cpl.op[cpl.n_op++] = ctrl_opcode;
		break;
	default:
		cpl.empty++;
		break;
	}
}

static int push(enum ll_txq_kind kind, uint8_t tag)
{
	uint8_t p[PAYLEN] = { tag, 0xA0, (uint8_t)~tag };

	return ll_txq_push(kind, kind == LL_TXQ_CTRL ? LL_LLID_CTRL : LL_LLID_START, p,
			   PAYLEN, tag);
}

static void new_conn(void)
{
	fake_conn_setup();
	memset(&cpl, 0, sizeof(cpl));
	ll_txq_reset(done_cb);
}

static void check_hw_clean(void)
{
	CHECK(hw.bad_write == 0);
	CHECK(hw.overfill == 0);
	CHECK(hw.sn_init_wrong == 0);
	CHECK(central.bad_content == 0);
#ifdef LL_TXQ_SAFE_MODE
	CHECK(hw.max_data_in_ring <= 1);
#endif
}

/* ---------------- tests ---------------- */

/* The fake itself, driven without ll_txq: points 1-4 and the hazard that
 * motivates the placeholder rule (a head queued while the base is in flight
 * is popped unsent when the central acks the base). */
static void test_fake_model(void)
{
	use_txq = false;
	fake_power_on(0);
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
	CHECK(central.n_got == 1 && central.got[0] == 2);         /* entry 1 only */
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
	CHECK(central.n_got == 1 && central.got[0] == 7);
	CHECK(hw.rptr == 0);
	ll_radio_conn_set_sn_init(0);
	run_event(NULL, 0);
	CHECK(hw.rptr == 1 && hw.air[2].ring == -1);
	use_txq = true;
}

static void test_single_acl(void)
{
	fake_power_on(0);
	new_conn();
	CHECK(ll_txq_backlog() == 0);
	CHECK(push(LL_TXQ_ACL, 1) == 0);
	CHECK(ll_txq_backlog() == 1);
	run_event(NULL, 0);
	CHECK(central.n_got == 1 && central.got[0] == 1);
	CHECK(cpl.acl == 0);            /* ack arrives in the next event */
	CHECK(ll_txq_backlog() == 1);
	run_event(NULL, 0);
	CHECK(cpl.acl == 1 && cpl.ctrl == 0 && cpl.empty == 0);
	CHECK(ll_txq_backlog() == 0);
	CHECK(hw.air[hw.n_air - 1].ring == -1);
	run_ok(3);
	CHECK(cpl.acl == 1 && central.n_got == 1);
	CHECK(hw.sn_init_calls == 5);
	check_hw_clean();
}

static void test_nack_retransmit(void)
{
	fake_power_on(0);
	new_conn();
	run_ok(2);                       /* base in flight */
	CHECK(push(LL_TXQ_ACL, 9) == 0);
	run_event((const struct xchg[]){ { .tx_lost = true } }, 1);
	CHECK(central.n_got == 0 && cpl.acl == 0);
	/* the central's next packet nacks: same data, same SN */
	run_event((const struct xchg[]){ { .tx_lost = true } }, 1);
	CHECK(central.n_got == 0 && cpl.acl == 0);
	run_event(NULL, 0);
	CHECK(central.n_got == 1 && central.got[0] == 9 && cpl.acl == 0);
	run_event((const struct xchg[]){ { .rx_lost = true } }, 1); /* timeout */
	CHECK(cpl.acl == 0);
	run_ok(1);
	CHECK(cpl.acl == 1 && central.n_got == 1);
	run_ok(3);
	CHECK(cpl.acl == 1 && central.n_got == 1);
	CHECK(ll_txq_backlog() == 0);
	check_hw_clean();
}

static void test_backlog_md(void)
{
	int ev = 0;

	fake_power_on(0);
	new_conn();
	for (uint8_t t = 1; t <= LL_TXQ_BACKLOG; t++) {
		CHECK(push(LL_TXQ_CTRL, t) == 0);
	}
	CHECK(push(LL_TXQ_CTRL, 99) == -ENOMEM);
	CHECK(ll_txq_backlog() == LL_TXQ_BACKLOG);
	run_event(NULL, 0);
	ev++;
#ifndef LL_TXQ_SAFE_MODE
	/* four ring entries in one event, MD 1, 1, 1, 0, chained by our MD */
	CHECK(hw.n_air == 4);
	for (int i = 0; i < 4; i++) {
		CHECK(hw.air[i].ring == i);
		CHECK(!!(hw.air[i].hdr0 & HDR_MD) == (i < 3));
	}
	CHECK(central.n_got == 4);
	CHECK(cpl.ctrl == 3);            /* the 4th is acked next event */
#else
	CHECK(hw.n_air == 1 && central.n_got == 1 && !(hw.air[0].hdr0 & HDR_MD));
#endif
	/* the software backlog has room again */
	CHECK(push(LL_TXQ_CTRL, 9) == 0);
	run_ok(2);
	ev += 2;
	CHECK(push(LL_TXQ_CTRL, 10) == 0);
	while (ll_txq_backlog() && ev < 100) {
		run_event(NULL, 0);
		ev++;
	}
	CHECK(central.n_got == 10 && cpl.ctrl == 10);
	for (int i = 0; i < 10; i++) {
		CHECK(central.got[i] == i + 1);
		CHECK(cpl.op[i] == i + 1);
	}
#ifndef LL_TXQ_SAFE_MODE
	CHECK(ev == 5);                  /* 10 at event 4: pushed late */
#else
	CHECK(ev == 20);                 /* sent in one event, acked in the next */
#endif
	check_hw_clean();
}

/* Ring ran empty (base sent last), then new data: placeholder in front. */
static void test_ring_empty_then_data(void)
{
	/* a) the central acks the base: the placeholder is popped, data sent */
	fake_power_on(0);
	new_conn();
	push(LL_TXQ_ACL, 1);
	run_ok(2);                       /* 1 sent, acked; base in flight */
	CHECK(hw.air[hw.n_air - 1].ring == -1);
	push(LL_TXQ_ACL, 2);
	run_event(NULL, 0);
	CHECK(central.n_got == 2 && central.got[1] == 2);
#ifndef LL_TXQ_SAFE_MODE
	CHECK(hw.wptr - hw.rptr == 1);   /* placeholder popped, data in flight */
#endif
	run_ok(1);
	CHECK(cpl.acl == 2 && cpl.empty == 0);
	check_hw_clean();

	/* b) our base response was lost: the placeholder is resent (as the
	 * base would be, same SN), the central continues on our MD */
	fake_power_on(0);
	new_conn();
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
	CHECK(central.n_got == 2 && central.got[0] == 1 && central.got[1] == 2);
	CHECK(cpl.acl == 2 && ll_txq_backlog() == 0);
	check_hw_clean();

	/* c) first RX lost right after queueing, central MD chains, then data */
	fake_power_on(0);
	new_conn();
	run_event((const struct xchg[]){ { .c_md = true }, { .c_md = true } }, 2);
	push(LL_TXQ_ACL, 5);
	push(LL_TXQ_ACL, 6);
	run_event((const struct xchg[]){ { .rx_lost = true } }, 1);
	run_event((const struct xchg[]){ { .c_md = true }, { .tx_lost = true } }, 2);
	run_ok(4);
	CHECK(central.n_got == 2 && central.got[0] == 5 && central.got[1] == 6);
	CHECK(cpl.acl == 2 && ll_txq_backlog() == 0);
	check_hw_clean();
}

static uint32_t rng = 12345;

static uint32_t rnd(void)
{
	rng = rng * 1103515245u + 12345u;
	return rng >> 16;
}

/* Our SN across events, invisible base acks, NACKs, lost packets and MD
 * chains of both sides, 8-bit pointer wrap: SN_INIT always equals the SN of
 * our last TX (checked in run_event), every PDU is delivered exactly once,
 * in order, and completed exactly once, in order. */
static void test_sn_tracking_random(uint8_t start_ptr, uint32_t seed)
{
	uint8_t next_tag = 0;
	int pushed = 0;

	rng = seed;
	fake_power_on(start_ptr);
	new_conn();
	for (int ev = 0; ev < 3000; ev++) {
		struct xchg x[MAX_XCHG];
		int n = (int)(rnd() % 6);

		for (int i = 0; i < n; i++) {
			uint32_t r = rnd() % 16;

			x[i].rx_lost = r == 0;
			x[i].tx_lost = r == 1 || r == 2;
			x[i].c_md = (rnd() % 3) == 0;
		}
		if (rnd() % 3 == 0) {
			int k = (int)(rnd() % 4);

			while (k-- && push(LL_TXQ_CTRL, next_tag) == 0) {
				next_tag++;
				pushed++;
			}
		}
		run_event(x, n);
		if (pushed > 900) {
			break;
		}
	}
	for (int ev = 0; ev < 200 && ll_txq_backlog(); ev++) {
		run_event(NULL, 0);
	}
	run_ok(1);
	CHECK(pushed > 300);
	CHECK(central.n_got == pushed);
	CHECK(cpl.ctrl == pushed);
	for (int i = 0; i < central.n_got && i < pushed; i++) {
		CHECK(central.got[i] == (uint8_t)i);
		CHECK(cpl.op[i] == (uint8_t)i);
	}
	CHECK(ll_txq_backlog() == 0);
	check_hw_clean();
}

static void test_reset_per_connection(void)
{
	int air0;

	fake_power_on(0xFD);
	new_conn();
	for (uint8_t t = 1; t <= 6; t++) {
		push(LL_TXQ_ACL, t);
	}
	run_event((const struct xchg[]){ { .tx_lost = true } }, 1);
	CHECK(hw.wptr != hw.rptr);       /* unacked entries left in the ring */

	/* new connection: old entries dropped, no completions, pointers equal */
	new_conn();
	CHECK(hw.wptr == hw.rptr);
	CHECK(ll_txq_backlog() == 0);
	CHECK(cpl.acl == 0);
	air0 = hw.n_air;
	push(LL_TXQ_ACL, 42);
	run_ok(3);
	CHECK(central.n_got == 1 && central.got[0] == 42);
	CHECK(cpl.acl == 1);
	for (int i = air0; i < hw.n_air; i++) {
		CHECK(hw.air[i].len == 0 || hw.air[i].data[0] == 42);
	}
	/* first event of the new connection: SN_INIT 0 (reset_sn_nesn) */
	CHECK(hw.air[air0].ring >= 0 && !(hw.air[air0].hdr0 & HDR_SN));
	CHECK(push(LL_TXQ_ACL, 1) == 0);
	CHECK(ll_txq_push(LL_TXQ_ACL, LL_LLID_START, (const uint8_t[32]){ 0 },
			  LL_DATA_PDU_MAX + LL_MIC_LEN + 1, 0) == -EINVAL);
	check_hw_clean();
}

int main(void)
{
#ifdef LL_TXQ_SAFE_MODE
	printf("(LL_TXQ_SAFE_MODE)\n");
#endif
	test_fake_model();
	test_single_acl();
	test_nack_retransmit();
	test_backlog_md();
	test_ring_empty_then_data();
	test_sn_tracking_random(0, 12345);
	test_sn_tracking_random(0xF0, 777);
	test_sn_tracking_random(0x7F, 31337);
	test_reset_per_connection();
	DONE();
}
