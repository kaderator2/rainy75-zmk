/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform hooks for the link layer (implemented in ll_glue.c).
 */
#ifndef LL_PLAT_H_
#define LL_PLAT_H_

#include <stdint.h>

uint32_t ll_plat_rand32(void);
unsigned int ll_plat_lock(void);
void ll_plat_unlock(unsigned int key);

/* AES-128 block encryption (ECB, one block), standard FIPS-197 byte order:
 * key[0], in[0] and out[0] are the first (most significant) bytes, as in the
 * Core Spec security function e (Vol 3 Part H 2.2.1) written MSB first.
 * Values that arrive little-endian (HCI LTK, SKD, the spec's LSB-first
 * sample data) are byte-reversed by the caller (ll_crypt). The B91 glue
 * implements this with hal aes_encrypt() and must do any byte reversal the
 * hardware needs itself (checked on device against FIPS-197 Appendix C.1).
 * Faked in host tests with a software AES. Thread context only. */
void ll_plat_aes_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

#endif /* LL_PLAT_H_ */
