/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_rxq host tests.
 *
 * The decrypt cases reuse the Bluetooth Core Specification v6.0 (vAtlanta
 * r00), Vol 6 Part C "Encryption sample data" vectors already used by
 * tests/test_crypt.c (same source, fetched 2026-10-03): the spec's
 * "Central -> Peripheral" packets are what ll_rxq must decrypt for us (we
 * are the peripheral, directionBit 1).
 */
#include <string.h>
#include "test.h"
#include "../ll_crypt.h"
#include "../ll_defs.h"
#include "../ll_rxq.h"

void aes_ref_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

void ll_plat_aes_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
	aes_ref_encrypt(key, in, out);
}

#define HDR_SN 0x08

/* Link the single-link suite runs on (slice 6a: every link behaves alike). */
static uint8_t L;

/* Build an ISR-delivered PDU buffer (header + payload); returns the put result. */
static bool try_put_l(uint8_t link, uint8_t hdr0, const uint8_t *payload, uint8_t paylen)
{
	uint8_t pdu[2 + LL_DATA_PDU_MAX + LL_MIC_LEN];

	pdu[0] = hdr0;
	pdu[1] = paylen;
	if (paylen) {
		memcpy(&pdu[2], payload, paylen);
	}
	return ll_rxq_isr_put(link, pdu, (uint8_t)(2 + paylen));
}

static void put_l(uint8_t link, uint8_t hdr0, const uint8_t *payload, uint8_t paylen)
{
	CHECK(try_put_l(link, hdr0, payload, paylen));
}

static void put(uint8_t hdr0, const uint8_t *payload, uint8_t paylen)
{
	put_l(L, hdr0, payload, paylen);
}

/* LTK 0x4C68384139F574D836BCF34E9DFB01BF in HCI order (LSB first). */
static const uint8_t ltk[16] = {0xBF, 0x01, 0xFB, 0x9D, 0x4E, 0xF3, 0xBC, 0x36,
				0xD8, 0x74, 0xF5, 0x39, 0x41, 0x38, 0x68, 0x4C};
static const uint8_t skdm[8] = {0x13, 0x02, 0xF1, 0xE0, 0xDF, 0xCE, 0xBD, 0xAC};
static const uint8_t skds[8] = {0x79, 0x68, 0x57, 0x46, 0x35, 0x24, 0x13, 0x02};
static const uint8_t iv[8] = {0x24, 0xAB, 0xDC, 0xBA, 0xBE, 0xBA, 0xAF, 0xDE};

/* LL_START_ENC_RSP1 (Central -> Peripheral, packet 0), header 0x0F: decrypts
 * to a single byte 0x06. */
static const uint8_t rsp1_air[5] = {0x9F, 0xCD, 0xA7, 0xF4, 0x48};
/* LL_DATA1 (Central -> Peripheral, packet 1), header 0x0E. */
static const uint8_t data1_air[31] = {
	0x7A, 0x70, 0xD6, 0x64, 0x15, 0x22, 0x6D, 0xF2, 0x6B, 0x17, 0x83, 0x9A, 0x06, 0x04,
	0x05, 0x59, 0x6B, 0xD6, 0x56, 0x4F, 0x79, 0x6B, 0x5B, 0x9C, 0xE6, 0xFF, 0x32,
	0xF7, 0x5A, 0x6D, 0x33};
static const uint8_t data1_clear[27] = {
	0x17, 0x00, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E,
	0x6F, 0x70, 0x71, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30};

static void crypt_setup(struct ll_crypt *c)
{
	memset(c, 0, sizeof(*c));
	ll_crypt_session_key(ltk, skdm, skds, c->sk);
	memcpy(c->iv, iv, sizeof(iv));
	c->enc_rx = true;
}

/* The single-link suite (slices 2-5) on link L. */
static void single_link_suite(void)
{
	struct ll_rx_pdu out;

	/* Plain (unencrypted) delivery: one non-empty PDU in, same PDU out. */
	{
		static const uint8_t payload[3] = {0x11, 0x22, 0x33};

		ll_rxq_reset(L);
		put(0x02, payload, 3);   /* LLID 2 (ACL start), NESN/SN/MD = 0 */
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_OK);
		CHECK(out.hdr0 == 0x02);
		CHECK(out.len == 3);
		CHECK(memcmp(out.data, payload, 3) == 0);
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_EMPTY);
	}

	/* Empty PDU (LLID 1, length 0): dropped, but ll_rxq_get must have
	 * consumed the ring entry (not left pending). */
	{
		ll_rxq_reset(L);
		put(0x01, NULL, 0);
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_EMPTY);
	}

	/* First packet of a connection is accepted whatever its SN bit. */
	{
		static const uint8_t payload[1] = {0xAA};

		ll_rxq_reset(L);
		put(0x02 | HDR_SN, payload, 1);
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_OK);
		CHECK(out.data[0] == 0xAA);
	}

	/* No SN filter in software (Task 10): the hardware writes only new
	 * packets into the RX FIFO (a retransmission is acked via NESN and
	 * not written), so every PDU put here is new, whatever its SN. With
	 * empty PDUs no longer queued, two data PDUs in a row can carry the
	 * same SN bit (the empty one in between had the other); both must be
	 * delivered. */
	{
		static const uint8_t p1[1] = {0x01};
		static const uint8_t p3[1] = {0x03};

		ll_rxq_reset(L);
		put(0x02, p1, 1);                /* SN 0 */
		put(0x02, p3, 1);                /* SN 0: new (an SN 1 packet in between was
						  * empty, or lost by an RX ring overflow) */

		CHECK(ll_rxq_get(L, &out) == LL_RXQ_OK);
		CHECK(out.data[0] == 0x01);
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_OK);
		CHECK(out.data[0] == 0x03);
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_EMPTY);
	}

	/* Empty PDUs never take ring room (Task 10: the host's LE Connection
	 * Complete processing blocks the controller thread for about 300 ms,
	 * 20 events whose empty PDUs overflowed the 16-entry ring). */
	{
		ll_rxq_reset(L);
		for (int i = 0; i < 40; i++) {
			put((uint8_t)(0x01 | ((i & 1) ? HDR_SN : 0)), NULL, 0);
		}
		for (int i = 0; i < 16; i++) {
			uint8_t payload[1] = {(uint8_t)i};

			put(0x02, payload, 1);
		}
		CHECK(ll_rxq_overflow_count(L) == 0);
		for (int i = 0; i < 16; i++) {
			CHECK(ll_rxq_get(L, &out) == LL_RXQ_OK);
			CHECK(out.data[0] == (uint8_t)i);
		}
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_EMPTY);
	}

	/* Overflow: 16-entry ring, the 17th isr_put is refused and counted;
	 * the 16 already queued are still delivered in order. */
	{
		ll_rxq_reset(L);
		CHECK(ll_rxq_overflow_count(L) == 0);
		for (int i = 0; i < 16; i++) {
			uint8_t payload[1] = {(uint8_t)i};
			uint8_t hdr0 = (uint8_t)(0x02 | ((i & 1) ? HDR_SN : 0));

			put(hdr0, payload, 1);
		}
		{
			uint8_t pdu[3] = {0x02, 1, 0xFF};

			CHECK(!ll_rxq_isr_put(L, pdu, 3));
		}
		CHECK(ll_rxq_overflow_count(L) == 1);
		for (int i = 0; i < 16; i++) {
			CHECK(ll_rxq_get(L, &out) == LL_RXQ_OK);
			CHECK(out.data[0] == (uint8_t)i);
		}
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_EMPTY);
	}

	/* Decrypt via ll_crypt when enc_rx: Core Spec sample packets, two in a
	 * row. The sample headers (0x0F, 0x0E) both happen to carry SN=1 (the
	 * spec picked them to illustrate encryption, not a real SN sequence),
	 * so the second delivery uses 0x06 instead of 0x0E for SN=0 only;
	 * NESN/SN/MD are masked out of the AAD (proven in test_crypt.c), so
	 * the ciphertext and MIC are unaffected and data1_air still decrypts
	 * to data1_clear under 0x06. rx_ctr tracks both decrypts. */
	{
		struct ll_crypt c;

		crypt_setup(&c);
		ll_rxq_reset(L);
		ll_rxq_set_crypt(L, &c);

		put(0x0F, rsp1_air, sizeof(rsp1_air));         /* SN 1 */
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_OK);
		CHECK(out.hdr0 == 0x0F);
		CHECK(out.len == 1);
		CHECK(out.data[0] == 0x06);
		CHECK(c.rx_ctr == 1);

		put(0x06, data1_air, sizeof(data1_air));       /* SN 0: new */
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_OK);
		CHECK(out.len == 27);
		CHECK(memcmp(out.data, data1_clear, 27) == 0);
		CHECK(c.rx_ctr == 2);
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_EMPTY);
	}

	/* MIC failure: reported distinctly (LL_RXQ_MIC_FAIL), counter
	 * unchanged. The caller is expected to terminate the connection on
	 * this result (Core Spec: MIC failure ends the link immediately), so
	 * there is no in-connection "retry"; a later connection starts clean
	 * via ll_rxq_reset(L). */
	{
		struct ll_crypt c;
		uint8_t bad[sizeof(rsp1_air)];

		crypt_setup(&c);
		ll_rxq_reset(L);
		ll_rxq_set_crypt(L, &c);

		memcpy(bad, rsp1_air, sizeof(bad));
		bad[0] ^= 0x01;
		put(0x0F, bad, sizeof(bad));
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_MIC_FAIL);
		CHECK(c.rx_ctr == 0);

		/* Sticky until ll_rxq_reset(L): no later PDU of this link is
		 * delivered, even one that would decrypt (the stream has a hole
		 * and the link is going away), and an empty ring still reports
		 * the failure rather than EMPTY. */
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_MIC_FAIL);
		put(0x0F, rsp1_air, sizeof(rsp1_air));
		out.len = 0xEE;
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_MIC_FAIL);
		CHECK(out.len == 0xEE);
		CHECK(c.rx_ctr == 0);
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_MIC_FAIL);

		/* Next connection: fresh crypt state. */
		ll_rxq_reset(L);
		ll_rxq_set_crypt(L, &c);
		put(0x0F, rsp1_air, sizeof(rsp1_air));
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_OK);
		CHECK(out.data[0] == 0x06);
		CHECK(c.rx_ctr == 1);
	}

	/* ll_rxq_reset(L) clears the crypt context too: after a reset without
	 * ll_rxq_set_crypt(), delivery is unencrypted even though the old
	 * context object still has enc_rx set. */
	{
		struct ll_crypt c;
		static const uint8_t payload[2] = {0x55, 0x66};

		crypt_setup(&c);
		ll_rxq_set_crypt(L, &c);
		ll_rxq_reset(L);
		put(0x02, payload, 2);
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_OK);
		CHECK(memcmp(out.data, payload, 2) == 0);
	}

	/* ---- ll_rxq_isr_take_queued: wake the consumer only for data PDUs ---- */
	ll_rxq_reset(L);
	CHECK(!ll_rxq_isr_take_queued());
	put(0x01, NULL, 0);                       /* empty PDU: nothing queued */
	put(0x09, NULL, 0);
	CHECK(!ll_rxq_isr_take_queued());
	{
		static const uint8_t d[3] = {0x01, 0x02, 0x03};
		struct ll_rx_pdu out;
		uint8_t bad[1] = {0};

		put(0x02, d, 3);
		put(0x01, NULL, 0);
		CHECK(ll_rxq_isr_take_queued());
		CHECK(!ll_rxq_isr_take_queued());     /* cleared by the take */
		CHECK(!ll_rxq_isr_put(L, bad, 1));       /* malformed: dropped, not queued */
		CHECK(!ll_rxq_isr_take_queued());
		/* consuming does not clear a pending flag of a later put */
		put(0x02, d, 3);
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_OK);
		CHECK(ll_rxq_get(L, &out) == LL_RXQ_OK);
		CHECK(ll_rxq_isr_take_queued());
		/* reset clears it */
		put(0x02, d, 3);
		ll_rxq_reset(L);
		CHECK(!ll_rxq_isr_take_queued());
	}
}


static void reset_all(void)
{
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		ll_rxq_reset(i);
	}
	(void)ll_rxq_isr_take_queued();
}

/* Out-of-range link ids are refused and touch no link. */
static void test_link_bounds(void)
{
	struct ll_rx_pdu out;
	static const uint8_t d[2] = {0x12, 0x34};
	struct ll_crypt c;

	reset_all();
	CHECK(!try_put_l(LL_MAX_CONN, 0x02, d, 2));
	CHECK(!try_put_l(0xFF, 0x02, d, 2));
	CHECK(!ll_rxq_isr_take_queued());
	CHECK(ll_rxq_get(LL_MAX_CONN, &out) == LL_RXQ_EMPTY);
	CHECK(ll_rxq_overflow_count(LL_MAX_CONN) == 0);
	crypt_setup(&c);
	ll_rxq_set_crypt(LL_MAX_CONN, &c);   /* ignored */
	ll_rxq_reset(LL_MAX_CONN);           /* ignored */
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		CHECK(ll_rxq_overflow_count(i) == 0);
		put_l(i, 0x02, d, 2);
		CHECK(ll_rxq_get(i, &out) == LL_RXQ_OK && out.len == 2);
		CHECK(memcmp(out.data, d, 2) == 0);   /* plaintext: no crypt on any link */
		CHECK(ll_rxq_get(i, &out) == LL_RXQ_EMPTY);
	}
}

/* Interleaved puts are delivered per link, in order; overflow and reset of
 * one link leave the others' queues intact. */
static void test_links_isolated(void)
{
	struct ll_rx_pdu out;

	reset_all();
	for (int k = 0; k < 10; k++) {
		for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
			uint8_t d[2] = {i, (uint8_t)k};

			put_l(i, 0x02, d, 2);
		}
	}
	/* fill link 0 to overflow: its 16-entry ring has 10 */
	for (int k = 10; k < 16; k++) {
		uint8_t d[2] = {0, (uint8_t)k};

		put_l(0, 0x02, d, 2);
	}
	{
		uint8_t d[2] = {0, 99};

		CHECK(!try_put_l(0, 0x02, d, 2));
	}
	CHECK(ll_rxq_overflow_count(0) == 1);
	for (uint8_t i = 1; i < LL_MAX_CONN; i++) {
		uint8_t d[2] = {i, 10};

		CHECK(ll_rxq_overflow_count(i) == 0);
		put_l(i, 0x02, d, 2);   /* the others still have room */
	}
	/* reset the last link: the others keep their data */
	if (LL_MAX_CONN > 1) {
		ll_rxq_reset(LL_MAX_CONN - 1);
		CHECK(ll_rxq_get(LL_MAX_CONN - 1, &out) == LL_RXQ_EMPTY);
	}
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		int n = i == 0 ? 16 : (i == LL_MAX_CONN - 1 ? 0 : 11);

		for (int k = 0; k < n; k++) {
			CHECK(ll_rxq_get(i, &out) == LL_RXQ_OK);
			CHECK(out.len == 2 && out.data[0] == i && out.data[1] == (uint8_t)k);
		}
		CHECK(ll_rxq_get(i, &out) == LL_RXQ_EMPTY);
	}
}

/* Each link decrypts with its own context: independent counters. A MIC
 * failure on one link is sticky there only; the others keep delivering. */
static void test_crypt_per_link(void)
{
	struct ll_crypt c[LL_MAX_CONN];
	struct ll_rx_pdu out;
	uint8_t bad[sizeof(rsp1_air)];
	const uint8_t fail = (uint8_t)(LL_MAX_CONN / 2);   /* 0, 1, 2 for N 1, 3, 5 */
	static const uint8_t plain[3] = {0xA1, 0xA2, 0xA3};

	reset_all();
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		crypt_setup(&c[i]);
		/* the last link of N > 1 stays unencrypted */
		if (LL_MAX_CONN == 1 || i != LL_MAX_CONN - 1) {
			ll_rxq_set_crypt(i, &c[i]);
		}
	}
	memcpy(bad, rsp1_air, sizeof(bad));
	bad[2] ^= 0x40;
	/* packet 0 on every link (the failing link gets the corrupted one) */
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		if (i == fail) {
			put_l(i, 0x0F, bad, sizeof(bad));
		} else if (LL_MAX_CONN > 1 && i == LL_MAX_CONN - 1) {
			put_l(i, 0x02, plain, 3);
		} else {
			put_l(i, 0x0F, rsp1_air, sizeof(rsp1_air));
		}
	}
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		if (i == fail) {
			CHECK(ll_rxq_get(i, &out) == LL_RXQ_MIC_FAIL);
			CHECK(c[i].rx_ctr == 0);
		} else if (LL_MAX_CONN > 1 && i == LL_MAX_CONN - 1) {
			CHECK(ll_rxq_get(i, &out) == LL_RXQ_OK);
			CHECK(out.len == 3 && memcmp(out.data, plain, 3) == 0);
		} else {
			CHECK(ll_rxq_get(i, &out) == LL_RXQ_OK);
			CHECK(out.len == 1 && out.data[0] == 0x06);
			CHECK(c[i].rx_ctr == 1);
		}
	}
	/* packet 1 everywhere: the failed link stays failed, the others deliver */
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		if (LL_MAX_CONN > 1 && i == LL_MAX_CONN - 1) {
			put_l(i, 0x02, plain, 3);
		} else {
			put_l(i, 0x06, data1_air, sizeof(data1_air));
		}
	}
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		if (i == fail) {
			CHECK(ll_rxq_get(i, &out) == LL_RXQ_MIC_FAIL);
			CHECK(c[i].rx_ctr == 0);
		} else if (LL_MAX_CONN > 1 && i == LL_MAX_CONN - 1) {
			CHECK(ll_rxq_get(i, &out) == LL_RXQ_OK && out.len == 3);
			CHECK(ll_rxq_get(i, &out) == LL_RXQ_EMPTY);
		} else {
			CHECK(ll_rxq_get(i, &out) == LL_RXQ_OK);
			CHECK(out.len == 27 && memcmp(out.data, data1_clear, 27) == 0);
			CHECK(c[i].rx_ctr == 2);
			CHECK(ll_rxq_get(i, &out) == LL_RXQ_EMPTY);
		}
	}
	/* a reset of the failed link clears only its failure */
	ll_rxq_reset(fail);
	CHECK(ll_rxq_get(fail, &out) == LL_RXQ_EMPTY);
	for (uint8_t i = 0; i < LL_MAX_CONN; i++) {
		CHECK(ll_rxq_get(i, &out) == LL_RXQ_EMPTY);
	}
}

/* The wake flag covers every link; a reset of one link keeps the flag of
 * another link's data. */
static void test_take_queued_any_link(void)
{
	static const uint8_t d[1] = {0x42};
	struct ll_rx_pdu out;
	const uint8_t last = (uint8_t)(LL_MAX_CONN - 1);

	reset_all();
	CHECK(!ll_rxq_isr_take_queued());
	put_l(last, 0x02, d, 1);
	CHECK(ll_rxq_isr_take_queued());
	CHECK(!ll_rxq_isr_take_queued());
	if (LL_MAX_CONN > 1) {
		put_l(last, 0x02, d, 1);
		ll_rxq_reset(0);
		CHECK(ll_rxq_isr_take_queued());
		put_l(0, 0x02, d, 1);
		put_l(last, 0x02, d, 1);
		ll_rxq_reset(last);
		CHECK(ll_rxq_isr_take_queued());   /* link 0 still has data */
		CHECK(ll_rxq_get(0, &out) == LL_RXQ_OK);
	}
	reset_all();
}

int main(void)
{
	L = 0;
	single_link_suite();
	L = (uint8_t)(LL_MAX_CONN - 1);
	single_link_suite();
	test_link_bounds();
	test_links_isolated();
	test_crypt_per_link();
	test_take_queued_any_link();
	DONE();
}
