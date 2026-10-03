/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * BLE link layer encryption, Core Spec Vol 6 Part E: AES-CCM (RFC 3610)
 * with a 4 octet MIC (M = 4), a 2 octet length field (L = 2) and a
 * 13 octet nonce:
 *   nonce[0..4]  = 39-bit packet counter, LSB first, directionBit in
 *                  bit 7 of nonce[4] (1 = central -> peripheral)
 *   nonce[5..12] = IV (IVm || IVs, LSB first as on air)
 * The additional authenticated data is the single octet data PDU header
 * byte 0 with NESN, SN and MD set to zero. Blocks are in FIPS-197 byte
 * order (block[0] first), so SK is kept MSB first as well.
 */
#include <string.h>

#include "ll_crypt.h"
#include "ll_defs.h"
#include "ll_plat.h"

#define CCM_FLAGS_B0   0x49 /* Adata, M' = (4 - 2) / 2 = 1, L' = 2 - 1 = 1 */
#define CCM_FLAGS_A    0x01 /* L' = 1 */
#define HDR_AAD_MASK   0xE3 /* clear NESN (bit 2), SN (bit 3), MD (bit 4) */
#define DIR_C_TO_P     1
#define DIR_P_TO_C     0

static void reverse(uint8_t *dst, const uint8_t *src, unsigned int n)
{
	for (unsigned int i = 0; i < n; i++) {
		dst[i] = src[n - 1 - i];
	}
}

void ll_crypt_session_key(const uint8_t ltk[16], const uint8_t skdm[8], const uint8_t skds[8],
			  uint8_t sk[16])
{
	uint8_t key[16], skd[16];

	/* SKD = SKDs || SKDm with SKDs the most significant half. */
	reverse(key, ltk, 16);
	reverse(&skd[0], skds, 8);
	reverse(&skd[8], skdm, 8);
	ll_plat_aes_ecb(key, skd, sk);
}

/* First 14 octets of B0 and of the A_i blocks: flags || nonce. */
static void ccm_block(uint8_t blk[16], uint8_t flags, const struct ll_crypt *c, uint64_t ctr,
		      uint8_t dir)
{
	blk[0] = flags;
	for (int i = 0; i < 5; i++) {
		blk[1 + i] = (uint8_t)(ctr >> (8 * i));
	}
	blk[5] = (uint8_t)((blk[5] & 0x7F) | (dir << 7));
	memcpy(&blk[6], c->iv, 8);
}

/* CBC-MAC over B0, B1 (AAD) and the plaintext: returns the raw tag T in
 * mic[0..3]. */
static void ccm_mac(const struct ll_crypt *c, uint64_t ctr, uint8_t dir, uint8_t hdr0,
		    const uint8_t *pt, uint8_t len, uint8_t mic[LL_MIC_LEN])
{
	uint8_t x[16], b[16];

	ccm_block(b, CCM_FLAGS_B0, c, ctr, dir);
	b[14] = 0;
	b[15] = len;
	ll_plat_aes_ecb(c->sk, b, x);

	/* B1: l(a) = 1 as a 2 octet length, then the masked header octet. */
	memset(b, 0, sizeof(b));
	b[1] = 1;
	b[2] = hdr0 & HDR_AAD_MASK;
	for (int i = 0; i < 16; i++) {
		b[i] ^= x[i];
	}
	ll_plat_aes_ecb(c->sk, b, x);

	for (unsigned int off = 0; off < len; off += 16) {
		unsigned int n = (len - off < 16) ? len - off : 16;

		memcpy(b, x, 16);
		for (unsigned int i = 0; i < n; i++) {
			b[i] ^= pt[off + i];
		}
		ll_plat_aes_ecb(c->sk, b, x);
	}
	memcpy(mic, x, LL_MIC_LEN);
}

/* CTR mode: XOR mic with S_0 and data with S_1, S_2, ... */
static void ccm_ctr(const struct ll_crypt *c, uint64_t ctr, uint8_t dir, uint8_t *data,
		    uint8_t len, uint8_t mic[LL_MIC_LEN])
{
	uint8_t a[16], s[16];

	ccm_block(a, CCM_FLAGS_A, c, ctr, dir);
	a[14] = 0;
	a[15] = 0;
	ll_plat_aes_ecb(c->sk, a, s);
	for (int k = 0; k < LL_MIC_LEN; k++) {
		mic[k] ^= s[k];
	}
	/* len <= 251 needs at most 16 blocks, so a[14] stays 0. */
	for (unsigned int off = 0; off < len; off += 16) {
		unsigned int n = (len - off < 16) ? len - off : 16;

		a[15]++;
		ll_plat_aes_ecb(c->sk, a, s);
		for (unsigned int k = 0; k < n; k++) {
			data[off + k] ^= s[k];
		}
	}
}

int ll_crypt_encrypt(struct ll_crypt *c, uint8_t hdr0, uint8_t *payload, uint8_t len)
{
	uint8_t mic[LL_MIC_LEN];

	if (len == 0 || len > 255 - LL_MIC_LEN) {
		return -1;
	}
	ccm_mac(c, c->tx_ctr, DIR_P_TO_C, hdr0, payload, len, mic);
	ccm_ctr(c, c->tx_ctr, DIR_P_TO_C, payload, len, mic);
	memcpy(&payload[len], mic, LL_MIC_LEN);
	c->tx_ctr++;
	return len + LL_MIC_LEN;
}

int ll_crypt_decrypt(struct ll_crypt *c, uint8_t hdr0, uint8_t *payload, uint8_t len)
{
	uint8_t mic[LL_MIC_LEN], tag[LL_MIC_LEN];
	uint8_t n, diff = 0;

	if (len <= LL_MIC_LEN) {
		return -1;
	}
	n = len - LL_MIC_LEN;
	memcpy(mic, &payload[n], LL_MIC_LEN);
	ccm_ctr(c, c->rx_ctr, DIR_C_TO_P, payload, n, mic);
	ccm_mac(c, c->rx_ctr, DIR_C_TO_P, hdr0, payload, n, tag);
	for (int i = 0; i < LL_MIC_LEN; i++) {
		diff |= mic[i] ^ tag[i];
	}
	if (diff) {
		return -1;
	}
	c->rx_ctr++;
	return n;
}
