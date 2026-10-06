/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Per-link host bookkeeping of the glue (slice 6a Task 6, extracted so it
 * is host-tested): whether the link is "up" for the host (LE Connection
 * Complete sent, Disconnection Complete not yet), its connection
 * generation, and the host's LE ACL buffer credits not reported yet
 * (Number Of Completed Packets, one handle per event).
 *
 * Rules:
 * - Credits of a link reach the host only while it is up, and never cross
 *   from one connection into the next one on the same link id: open() and
 *   close() clear the count, and the "is it up" test and the increment of
 *   back() are one step under ll_plat_lock(), as are close()'s clear and
 *   its "down".
 * - Acks (acked(), ISR) count unconditionally; take() reports them only
 *   while the link is up (an ack racing the end is dropped with the
 *   connection, the host frees its buffers on disconnect).
 * - up_gen() reads "up" and the generation in one lock section, so a host
 *   ACL packet is never tagged with the generation of a connection that
 *   has already ended while it is queued for the next one.
 * Any context; every call takes ll_plat_lock() (nesting ok).
 */
#ifndef LL_CREDIT_H_
#define LL_CREDIT_H_

#include <stdbool.h>
#include <stdint.h>

#include "ll_defs.h"

/* Boot: every link down, no credits, generation 0. */
void ll_credit_init(void);
/* The link's connection is reported to the host: count 0, up, generation + 1. */
void ll_credit_open(uint8_t link);
/* The link's disconnect is handled: count 0, down. Returns whether it was up. */
bool ll_credit_close(uint8_t link);
bool ll_credit_up(uint8_t link);
/* up (return) and generation (*gen) of the link, read together. */
bool ll_credit_up_gen(uint8_t link, uint32_t *gen);
uint32_t ll_credit_gen(uint8_t link);
/* One ACL PDU of the link was acked by the central (ISR). */
void ll_credit_acked(uint8_t link);
/* A host ACL packet for the link will never be sent: its credit goes back
 * to the host, only while the link is up. Returns whether it was counted. */
bool ll_credit_back(uint8_t link);
/* The count to report now (cleared); 0 while the link is down (cleared
 * too). */
uint16_t ll_credit_take(uint8_t link);

#endif /* LL_CREDIT_H_ */
