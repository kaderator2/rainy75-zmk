/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Test-only software AES-128 encryption (FIPS-197), written from the
 * standard: the S-box is derived at first use from the GF(2^8)
 * multiplicative inverse plus the affine transform (FIPS-197 5.1.1).
 * Byte order as in FIPS-197: key[0], in[0], out[0] are the first bytes.
 * Not constant time; host tests only.
 */
#include <stdint.h>
#include <string.h>

static uint8_t sbox[256];

static uint8_t xtime(uint8_t a)
{
	return (uint8_t)((a << 1) ^ ((a & 0x80) ? 0x1B : 0x00));
}

static uint8_t gmul(uint8_t a, uint8_t b)
{
	uint8_t r = 0;

	while (b) {
		if (b & 1) {
			r ^= a;
		}
		a = xtime(a);
		b >>= 1;
	}
	return r;
}

static uint8_t rotl8(uint8_t x, int s)
{
	return (uint8_t)((x << s) | (x >> (8 - s)));
}

static void sbox_init(void)
{
	for (int i = 0; i < 256; i++) {
		uint8_t inv = 0;

		if (i) {
			for (int j = 1; j < 256; j++) {
				if (gmul((uint8_t)i, (uint8_t)j) == 1) {
					inv = (uint8_t)j;
					break;
				}
			}
		}
		sbox[i] = (uint8_t)(inv ^ rotl8(inv, 1) ^ rotl8(inv, 2) ^ rotl8(inv, 3) ^
				    rotl8(inv, 4) ^ 0x63);
	}
}

/* State as 16 bytes in input order: s[r + 4c] is row r, column c. */
static void sub_shift(uint8_t s[16])
{
	uint8_t t[16];

	for (int c = 0; c < 4; c++) {
		for (int r = 0; r < 4; r++) {
			t[r + 4 * c] = sbox[s[r + 4 * ((c + r) % 4)]];
		}
	}
	memcpy(s, t, 16);
}

static void mix_columns(uint8_t s[16])
{
	for (int c = 0; c < 4; c++) {
		uint8_t *p = &s[4 * c];
		uint8_t a0 = p[0], a1 = p[1], a2 = p[2], a3 = p[3];

		p[0] = (uint8_t)(gmul(a0, 2) ^ gmul(a1, 3) ^ a2 ^ a3);
		p[1] = (uint8_t)(a0 ^ gmul(a1, 2) ^ gmul(a2, 3) ^ a3);
		p[2] = (uint8_t)(a0 ^ a1 ^ gmul(a2, 2) ^ gmul(a3, 3));
		p[3] = (uint8_t)(gmul(a0, 3) ^ a1 ^ a2 ^ gmul(a3, 2));
	}
}

void aes_ref_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
	uint8_t w[176], s[16], rcon = 1;

	if (sbox[0] != 0x63) {
		sbox_init();
	}
	/* Key expansion, FIPS-197 5.2 (Nk = 4, Nr = 10). */
	memcpy(w, key, 16);
	for (int i = 16; i < 176; i += 4) {
		uint8_t t[4] = {w[i - 4], w[i - 3], w[i - 2], w[i - 1]};

		if (i % 16 == 0) {
			uint8_t t0 = t[0];

			t[0] = (uint8_t)(sbox[t[1]] ^ rcon);
			t[1] = sbox[t[2]];
			t[2] = sbox[t[3]];
			t[3] = sbox[t0];
			rcon = xtime(rcon);
		}
		for (int k = 0; k < 4; k++) {
			w[i + k] = (uint8_t)(w[i - 16 + k] ^ t[k]);
		}
	}
	for (int i = 0; i < 16; i++) {
		s[i] = (uint8_t)(in[i] ^ w[i]);
	}
	for (int round = 1; round <= 10; round++) {
		sub_shift(s);
		if (round != 10) {
			mix_columns(s);
		}
		for (int i = 0; i < 16; i++) {
			s[i] ^= w[16 * round + i];
		}
	}
	memcpy(out, s, 16);
}
