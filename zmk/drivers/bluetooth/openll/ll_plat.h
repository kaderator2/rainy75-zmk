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

/* Thread-level serialization of the LL TX producers (ll_llcp: controller
 * thread and HCI thread). Encryption and the push of one data PDU run under
 * it, so the TX packet counter order equals the queue order, while
 * ll_plat_lock() (interrupts off) is held only for the push itself and per
 * AES block. Recursive; thread context only (B91: a k_mutex). */
void ll_plat_tx_lock(void);
void ll_plat_tx_unlock(void);

/* AES-128 block encryption (ECB, one block), standard FIPS-197 byte order:
 * key[0], in[0] and out[0] are the first (most significant) bytes, as in the
 * Core Spec security function e (Vol 3 Part H 2.2.1) written MSB first.
 * Values that arrive little-endian (HCI LTK, SKD, the spec's LSB-first
 * sample data) are byte-reversed by the caller (ll_crypt). The B91 glue
 * implements this with hal aes_encrypt() and must do any byte reversal the
 * hardware needs itself (checked on device against FIPS-197 Appendix C.1).
 * Faked in host tests with a software AES. Thread context only.
 *
 * Atomicity: one call is one complete block operation that cannot
 * interleave with another call. The B91 engine (hal aes_encrypt) has a
 * single key/data register set and a shared static data buffer, while
 * ll_rxq decrypts in the controller thread without any lock and ll_llcp
 * encrypts from the HCI thread (LTK reply, Disconnect) or the controller
 * thread, so the implementation must lock out every other caller for the
 * block (the B91 glue holds ll_plat_lock() per block; the lock nests). */
void ll_plat_aes_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

#endif /* LL_PLAT_H_ */
