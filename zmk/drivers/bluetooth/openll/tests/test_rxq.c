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

/* Build an ISR-delivered PDU buffer (header + payload) and feed it in. */
static void put(uint8_t hdr0, const uint8_t *payload, uint8_t paylen)
{
	uint8_t pdu[2 + LL_DATA_PDU_MAX + LL_MIC_LEN];

	pdu[0] = hdr0;
	pdu[1] = paylen;
	if (paylen) {
		memcpy(&pdu[2], payload, paylen);
	}
	CHECK(ll_rxq_isr_put(pdu, (uint8_t)(2 + paylen)));
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

int main(void)
{
	struct ll_rx_pdu out;

	/* Plain (unencrypted) delivery: one non-empty PDU in, same PDU out. */
	{
		static const uint8_t payload[3] = {0x11, 0x22, 0x33};

		ll_rxq_reset();
		put(0x02, payload, 3);   /* LLID 2 (ACL start), NESN/SN/MD = 0 */
		CHECK(ll_rxq_get(&out) == LL_RXQ_OK);
		CHECK(out.hdr0 == 0x02);
		CHECK(out.len == 3);
		CHECK(memcmp(out.data, payload, 3) == 0);
		CHECK(ll_rxq_get(&out) == LL_RXQ_EMPTY);
	}

	/* Empty PDU (LLID 1, length 0): dropped, but ll_rxq_get must have
	 * consumed the ring entry (not left pending). */
	{
		ll_rxq_reset();
		put(0x01, NULL, 0);
		CHECK(ll_rxq_get(&out) == LL_RXQ_EMPTY);
	}

	/* First packet of a connection is accepted whatever its SN bit. */
	{
		static const uint8_t payload[1] = {0xAA};

		ll_rxq_reset();
		put(0x02 | HDR_SN, payload, 1);
		CHECK(ll_rxq_get(&out) == LL_RXQ_OK);
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

		ll_rxq_reset();
		put(0x02, p1, 1);                /* SN 0 */
		put(0x02, p3, 1);                /* SN 0: new (an SN 1 packet in between was
						  * empty, or lost by an RX ring overflow) */

		CHECK(ll_rxq_get(&out) == LL_RXQ_OK);
		CHECK(out.data[0] == 0x01);
		CHECK(ll_rxq_get(&out) == LL_RXQ_OK);
		CHECK(out.data[0] == 0x03);
		CHECK(ll_rxq_get(&out) == LL_RXQ_EMPTY);
	}

	/* Empty PDUs never take ring room (Task 10: the host's LE Connection
	 * Complete processing blocks the controller thread for about 300 ms,
	 * 20 events whose empty PDUs overflowed the 16-entry ring). */
	{
		ll_rxq_reset();
		for (int i = 0; i < 40; i++) {
			put((uint8_t)(0x01 | ((i & 1) ? HDR_SN : 0)), NULL, 0);
		}
		for (int i = 0; i < 16; i++) {
			uint8_t payload[1] = {(uint8_t)i};

			put(0x02, payload, 1);
		}
		CHECK(ll_rxq_overflow_count() == 0);
		for (int i = 0; i < 16; i++) {
			CHECK(ll_rxq_get(&out) == LL_RXQ_OK);
			CHECK(out.data[0] == (uint8_t)i);
		}
		CHECK(ll_rxq_get(&out) == LL_RXQ_EMPTY);
	}

	/* Overflow: 16-entry ring, the 17th isr_put is refused and counted;
	 * the 16 already queued are still delivered in order. */
	{
		ll_rxq_reset();
		CHECK(ll_rxq_overflow_count() == 0);
		for (int i = 0; i < 16; i++) {
			uint8_t payload[1] = {(uint8_t)i};
			uint8_t hdr0 = (uint8_t)(0x02 | ((i & 1) ? HDR_SN : 0));

			put(hdr0, payload, 1);
		}
		{
			uint8_t pdu[3] = {0x02, 1, 0xFF};

			CHECK(!ll_rxq_isr_put(pdu, 3));
		}
		CHECK(ll_rxq_overflow_count() == 1);
		for (int i = 0; i < 16; i++) {
			CHECK(ll_rxq_get(&out) == LL_RXQ_OK);
			CHECK(out.data[0] == (uint8_t)i);
		}
		CHECK(ll_rxq_get(&out) == LL_RXQ_EMPTY);
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
		ll_rxq_reset();
		ll_rxq_set_crypt(&c);

		put(0x0F, rsp1_air, sizeof(rsp1_air));         /* SN 1 */
		CHECK(ll_rxq_get(&out) == LL_RXQ_OK);
		CHECK(out.hdr0 == 0x0F);
		CHECK(out.len == 1);
		CHECK(out.data[0] == 0x06);
		CHECK(c.rx_ctr == 1);

		put(0x06, data1_air, sizeof(data1_air));       /* SN 0: new */
		CHECK(ll_rxq_get(&out) == LL_RXQ_OK);
		CHECK(out.len == 27);
		CHECK(memcmp(out.data, data1_clear, 27) == 0);
		CHECK(c.rx_ctr == 2);
		CHECK(ll_rxq_get(&out) == LL_RXQ_EMPTY);
	}

	/* MIC failure: reported distinctly (LL_RXQ_MIC_FAIL), counter
	 * unchanged. The caller is expected to terminate the connection on
	 * this result (Core Spec: MIC failure ends the link immediately), so
	 * there is no in-connection "retry"; a later connection starts clean
	 * via ll_rxq_reset(). */
	{
		struct ll_crypt c;
		uint8_t bad[sizeof(rsp1_air)];

		crypt_setup(&c);
		ll_rxq_reset();
		ll_rxq_set_crypt(&c);

		memcpy(bad, rsp1_air, sizeof(bad));
		bad[0] ^= 0x01;
		put(0x0F, bad, sizeof(bad));
		CHECK(ll_rxq_get(&out) == LL_RXQ_MIC_FAIL);
		CHECK(c.rx_ctr == 0);

		/* Next connection: fresh crypt state. */
		ll_rxq_reset();
		ll_rxq_set_crypt(&c);
		put(0x0F, rsp1_air, sizeof(rsp1_air));
		CHECK(ll_rxq_get(&out) == LL_RXQ_OK);
		CHECK(out.data[0] == 0x06);
		CHECK(c.rx_ctr == 1);
	}

	/* ll_rxq_reset() clears the crypt context too: after a reset without
	 * ll_rxq_set_crypt(), delivery is unencrypted even though the old
	 * context object still has enc_rx set. */
	{
		struct ll_crypt c;
		static const uint8_t payload[2] = {0x55, 0x66};

		crypt_setup(&c);
		ll_rxq_set_crypt(&c);
		ll_rxq_reset();
		put(0x02, payload, 2);
		CHECK(ll_rxq_get(&out) == LL_RXQ_OK);
		CHECK(memcmp(out.data, payload, 2) == 0);
	}

	DONE();
}
