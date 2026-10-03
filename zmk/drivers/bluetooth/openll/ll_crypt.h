/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * BLE link layer encryption (AES-CCM, Core Spec Vol 6 Part E) for the
 * peripheral role. AES-ECB comes from ll_plat_aes_ecb().
 */
#ifndef LL_CRYPT_H_
#define LL_CRYPT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct ll_crypt {
	uint8_t sk[16];   /* session key, MSB first (FIPS-197 order, as ll_plat_aes_ecb takes it) */
	uint8_t iv[8];    /* IV = IVm || IVs, LSB first as on air (IVm in iv[0..3]) */
	uint64_t tx_ctr;  /* 39-bit packet counter, peripheral -> central */
	uint64_t rx_ctr;  /* 39-bit packet counter, central -> peripheral */
	bool enc_tx;
	bool enc_rx;
};

/* SK = e(LTK, SKD), SKD = SKDm || SKDs, Core Spec Vol 6 Part B 5.1.3.1.
 * ltk is in HCI order (LSB first); skdm/skds are as received/sent on air
 * (LSB first). */
void ll_crypt_session_key(const uint8_t ltk[16], const uint8_t skdm[8], const uint8_t skds[8],
			  uint8_t sk[16]);
/* Encrypt one data PDU in place (we are the peripheral: directionBit 0,
 * tx_ctr, incremented per call). hdr0 = data PDU header byte 0 (NESN, SN and
 * MD are masked for the AAD), len = plaintext payload length (> 0). payload
 * must have room for len + LL_MIC_LEN. Returns len + LL_MIC_LEN, or -1
 * (counter unchanged) if len is 0 or len + LL_MIC_LEN exceeds 255. */
int ll_crypt_encrypt(struct ll_crypt *c, uint8_t hdr0, uint8_t *payload, uint8_t len);
/* Decrypt one data PDU from the central in place (directionBit 1, rx_ctr,
 * incremented only on success). len includes the MIC. Returns
 * len - LL_MIC_LEN, or -1 on MIC failure or len <= LL_MIC_LEN. After a
 * MIC failure the payload content is unspecified (the link is terminated
 * with LL_ST_MIC_FAILURE anyway). */
int ll_crypt_decrypt(struct ll_crypt *c, uint8_t hdr0, uint8_t *payload, uint8_t len);
/* Zero n bytes of key material in a way the compiler cannot elide: a memset
 * of a local that is dead afterwards may legally be removed (dead store
 * elimination), stores through a volatile pointer may not. Used for every
 * stack copy of the LTK, SKD, session key and AES blocks. */
void ll_crypt_wipe(void *p, size_t n);

#endif /* LL_CRYPT_H_ */
