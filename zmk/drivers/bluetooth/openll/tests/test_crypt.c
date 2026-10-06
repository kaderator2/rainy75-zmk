/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_crypt host tests.
 *
 * Sample data: Bluetooth Core Specification v6.0 (vAtlanta r00), Vol 6
 * Part C, section 1 "Encryption sample data" with 1.1 "Encrypt Command" and
 * 1.2 "Derivation of the MIC and encrypted data", public HTML edition:
 * https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/low-energy-controller/sample-data.html
 * (fetched 2026-10-03). The spec's central is the remote device; we are the
 * peripheral, so its "Peripheral -> Central" packets are our encrypt
 * outputs (directionBit 0) and its "Central -> Peripheral" packets are our
 * decrypt inputs (directionBit 1).
 */
#include <string.h>
#include "test.h"
#include "../ll_crypt.h"
#include "../ll_defs.h"
#include "../ll_plat.h"

void aes_ref_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

static int aes_calls;

void ll_plat_aes_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
	aes_calls++;
	aes_ref_encrypt(key, in, out);
}

/* LTK 0x4C68384139F574D836BCF34E9DFB01BF in HCI order (LSB first). */
static const uint8_t ltk[16] = {0xBF, 0x01, 0xFB, 0x9D, 0x4E, 0xF3, 0xBC, 0x36,
				0xD8, 0x74, 0xF5, 0x39, 0x41, 0x38, 0x68, 0x4C};
/* SKD_C, SKD_P and IV_C || IV_P as they appear in LL_ENC_REQ / LL_ENC_RSP. */
static const uint8_t skdm[8] = {0x13, 0x02, 0xF1, 0xE0, 0xDF, 0xCE, 0xBD, 0xAC};
static const uint8_t skds[8] = {0x79, 0x68, 0x57, 0x46, 0x35, 0x24, 0x13, 0x02};
static const uint8_t iv[8] = {0x24, 0xAB, 0xDC, 0xBA, 0xBE, 0xBA, 0xAF, 0xDE};
/* SK = 0x99AD1B5226A37E3E058E3B8E27C2C666 (MSB first, FIPS-197 order). */
static const uint8_t sk_msb[16] = {0x99, 0xAD, 0x1B, 0x52, 0x26, 0xA3, 0x7E, 0x3E,
				   0x05, 0x8E, 0x3B, 0x8E, 0x27, 0xC2, 0xC6, 0x66};

/* LL_START_ENC_RSP1 (Central -> Peripheral, packet 0): 0f 05 9f cd a7 f4 48 */
static const uint8_t rsp1_air[5] = {0x9F, 0xCD, 0xA7, 0xF4, 0x48};
/* LL_START_ENC_RSP2 (Peripheral -> Central, packet 0): 07 05 a3 4c 13 a4 15 */
static const uint8_t rsp2_air[5] = {0xA3, 0x4C, 0x13, 0xA4, 0x15};

/* LL_DATA1 (Central -> Peripheral, packet 1), header 0x0E */
static const uint8_t data1_clear[27] = {
	0x17, 0x00, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E,
	0x6F, 0x70, 0x71, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30};
static const uint8_t data1_air[31] = {
	0x7A, 0x70, 0xD6, 0x64, 0x15, 0x22, 0x6D, 0xF2, 0x6B, 0x17, 0x83, 0x9A, 0x06, 0x04,
	0x05, 0x59, 0x6B, 0xD6, 0x56, 0x4F, 0x79, 0x6B, 0x5B, 0x9C, 0xE6, 0xFF, 0x32,
	0xF7, 0x5A, 0x6D, 0x33};
/* LL_DATA2 (Peripheral -> Central, packet 1), header 0x06 */
static const uint8_t data2_clear[27] = {
	0x17, 0x00, 0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x30, 0x41, 0x42, 0x43, 0x44,
	0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51};
static const uint8_t data2_air[31] = {
	0xF3, 0x88, 0x81, 0xE7, 0xBD, 0x94, 0xC9, 0xC3, 0x69, 0xB9, 0xA6, 0x68, 0x46, 0xDD,
	0x47, 0x86, 0xAA, 0x8C, 0x39, 0xCE, 0x54, 0x0D, 0x0D, 0xAE, 0x3A, 0xDC, 0xDF,
	0x89, 0xB9, 0x60, 0x88};

#include "crypt_max_vec.h"

static void crypt_setup(struct ll_crypt *c)
{
	memset(c, 0, sizeof(*c));
	ll_crypt_session_key(ltk, skdm, skds, c->sk);
	memcpy(c->iv, iv, sizeof(iv));
}

int main(void)
{
	struct ll_crypt c;
	uint8_t buf[255];

	/* aes_ref against FIPS-197 Appendix C.1 (AES-128). */
	{
		static const uint8_t key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
						0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
		static const uint8_t pt[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
					       0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
		static const uint8_t ct[16] = {0x69, 0xC4, 0xE0, 0xD8, 0x6A, 0x7B, 0x04, 0x30,
					       0xD8, 0xCD, 0xB7, 0x80, 0x70, 0xB4, 0xC5, 0x5A};
		uint8_t out[16];

		aes_ref_encrypt(key, pt, out);
		CHECK(memcmp(out, ct, 16) == 0);
	}

	/* Session key: SK = e(LTK, SKD_P || SKD_C), spec 1.1 HCI_LE_Encrypt. */
	crypt_setup(&c);
	CHECK(memcmp(c.sk, sk_msb, 16) == 0);

	/* TX (directionBit 0): LL_START_ENC_RSP2 then LL_DATA2, counter 0, 1. */
	buf[0] = 0x06;
	CHECK(ll_crypt_encrypt(&c, 0x07, buf, 1) == 5);
	CHECK(memcmp(buf, rsp2_air, 5) == 0);
	CHECK(c.tx_ctr == 1);
	CHECK(c.rx_ctr == 0);
	memcpy(buf, data2_clear, 27);
	CHECK(ll_crypt_encrypt(&c, 0x06, buf, 27) == 31);
	CHECK(memcmp(buf, data2_air, 31) == 0);
	CHECK(c.tx_ctr == 2);

	/* RX (directionBit 1): LL_START_ENC_RSP1 then LL_DATA1, counter 0, 1. */
	memcpy(buf, rsp1_air, 5);
	CHECK(ll_crypt_decrypt(&c, 0x0F, buf, 5) == 1);
	CHECK(buf[0] == 0x06);
	CHECK(c.rx_ctr == 1);
	memcpy(buf, data1_air, 31);
	CHECK(ll_crypt_decrypt(&c, 0x0E, buf, 31) == 27);
	CHECK(memcmp(buf, data1_clear, 27) == 0);
	CHECK(c.rx_ctr == 2);
	CHECK(c.tx_ctr == 2);

	/* Header AAD masking: NESN (bit 2), SN (bit 3) and MD (bit 4) do not
	 * change the output; LLID and the RFU/CP bits do. */
	{
		static const uint8_t hdrs[] = {0x03, 0x07, 0x0B, 0x0F, 0x13, 0x1F};

		for (unsigned int i = 0; i < sizeof(hdrs); i++) {
			crypt_setup(&c);
			buf[0] = 0x06;
			CHECK(ll_crypt_encrypt(&c, hdrs[i], buf, 1) == 5);
			CHECK(memcmp(buf, rsp2_air, 5) == 0);
		}
		crypt_setup(&c);
		buf[0] = 0x06;
		ll_crypt_encrypt(&c, 0x02, buf, 1);  /* LLID 2 instead of 3 */
		CHECK(buf[0] == rsp2_air[0]);        /* keystream unchanged */
		CHECK(memcmp(&buf[1], &rsp2_air[1], 4) != 0); /* MIC differs */
		crypt_setup(&c);
		buf[0] = 0x06;
		ll_crypt_encrypt(&c, 0x27, buf, 1);  /* RFU bit 5 is authenticated */
		CHECK(memcmp(&buf[1], &rsp2_air[1], 4) != 0);
		/* masking on the RX side too */
		crypt_setup(&c);
		memcpy(buf, rsp1_air, 5);
		CHECK(ll_crypt_decrypt(&c, 0x13, buf, 5) == 1);
		CHECK(buf[0] == 0x06);
	}

	/* MIC failure: -1, counter unchanged, next good packet still decrypts. */
	{
		uint8_t bad[5];

		crypt_setup(&c);
		for (unsigned int i = 0; i < 5; i++) {
			memcpy(bad, rsp1_air, 5);
			bad[i] ^= 0x01;
			CHECK(ll_crypt_decrypt(&c, 0x0F, bad, 5) == -1);
			CHECK(c.rx_ctr == 0);
		}
		memcpy(bad, rsp1_air, 5);  /* wrong header (LLID 2) */
		CHECK(ll_crypt_decrypt(&c, 0x0E, bad, 5) == -1);
		CHECK(c.rx_ctr == 0);
		/* packet 1 decrypted with counter 0 fails */
		memcpy(buf, data1_air, 31);
		CHECK(ll_crypt_decrypt(&c, 0x0E, buf, 31) == -1);
		CHECK(c.rx_ctr == 0);
		memcpy(buf, rsp1_air, 5);
		CHECK(ll_crypt_decrypt(&c, 0x0F, buf, 5) == 1);
		CHECK(buf[0] == 0x06);
		CHECK(c.rx_ctr == 1);
	}

	/* Too short: len <= MIC is rejected without touching the counter or
	 * calling AES; encrypt of an empty payload is rejected too. */
	crypt_setup(&c);
	aes_calls = 0;
	CHECK(ll_crypt_decrypt(&c, 0x0F, buf, LL_MIC_LEN) == -1);
	CHECK(ll_crypt_decrypt(&c, 0x0F, buf, 0) == -1);
	CHECK(ll_crypt_encrypt(&c, 0x07, buf, 0) == -1);
	CHECK(aes_calls == 0);
	CHECK(c.rx_ctr == 0 && c.tx_ctr == 0);

	/* Max length payload at the top of the 39-bit counter, both
	 * directions: encrypt gives the directionBit 0 vector, decrypting the
	 * directionBit 1 vector gives the plaintext back (round trip of the
	 * same payload through both roles). The counter's bit 38 sits in
	 * nonce byte 4 next to the directionBit. */
	crypt_setup(&c);
	c.tx_ctr = 0x7FFFFFFFFFull;
	c.rx_ctr = 0x7FFFFFFFFFull;
	for (int i = 0; i < 251; i++) {
		buf[i] = (uint8_t)i;
	}
	CHECK(ll_crypt_encrypt(&c, 0x1E, buf, 251) == 255);
	CHECK(memcmp(buf, max_dir0, 255) == 0);
	CHECK(c.tx_ctr == 0x8000000000ull);
	memcpy(buf, max_dir1, 255);
	CHECK(ll_crypt_decrypt(&c, 0x1E, buf, 255) == 251);
	for (int i = 0; i < 251; i++) {
		if (buf[i] != (uint8_t)i) {
			CHECK(buf[i] == (uint8_t)i);
			break;
		}
	}
	CHECK(c.rx_ctr == 0x8000000000ull);
	/* the directionBit matters: the dir0 vector does not authenticate on RX */
	c.rx_ctr = 0x7FFFFFFFFFull;
	memcpy(buf, max_dir0, 255);
	CHECK(ll_crypt_decrypt(&c, 0x1E, buf, 255) == -1);

	/* ll_crypt_wipe zeroes exactly n bytes (key copies on the stack) */
	{
		uint8_t k[18];

		memset(k, 0xA5, sizeof(k));
		ll_crypt_wipe(&k[1], 16);
		CHECK(k[0] == 0xA5 && k[17] == 0xA5);
		for (int i = 1; i <= 16; i++) {
			CHECK(k[i] == 0);
		}
		ll_crypt_wipe(k, 0);
		CHECK(k[0] == 0xA5);
	}

	DONE();
}
