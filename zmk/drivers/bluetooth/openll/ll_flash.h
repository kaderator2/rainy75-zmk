/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Flash window: no radio activity while a flash erase or write runs with
 * interrupts off.
 *
 * The hal flash functions (hal_telink flash.c) run every erase and write
 * with interrupts off (13..29 ms per 4 KB sector erase, about 2.4 ms per
 * 256-byte page write). A connection event that is open then keeps going
 * in hardware: the baseband receives and acks the central's packets into
 * the 4-entry RX DMA ring (LL_RADIO_RX_RING_N), but neither the RF ISR
 * (which drains the ring) nor the guard alarm (which ends the event) can
 * run. A central burst of more than 4 packets overwrites acked entries
 * (ll_radio_stats.rx_ptr_skip); the lost PDU makes the next one fail its
 * MIC and the link ends with 0x3D. Measured on the device (flash-window
 * report): the RF ISR ran 0.2 ms before a 12.4 ms sector erase, the
 * hardware received 5..6 packets during it.
 *
 * So the controller keeps the radio idle for the duration of each flash
 * operation (one sector erase, one page write):
 * - ll_flash_open() ends whatever is on air (ll_radio_flash_abort(): an
 *   open or armed connection event ends with the packets already received,
 *   an advertising channel ends like an RX timeout) and sets the window;
 * - while the window is set, ll_conn does not issue connection events
 *   (stats.flash_paused, also missed; the central resends, as after any
 *   missed event) and ll_adv sends no advertising channel (the event ends,
 *   stats.flash);
 * - ll_flash_close() ends it; the next planned events run as usual.
 * Supervision: an operation starts only while every link can afford it
 * (ll_conn_flash_ready: established, and the time since its last received
 * packet plus LL_FLASH_OP_MAX_US stays within half its supervision
 * timeout). Otherwise the links' next events are pulled in
 * (ll_conn_flash_kick) and the caller waits with interrupts on and asks
 * again; a chain of back-to-back erases therefore lets an event through
 * whenever it is needed. After LL_FLASH_WAIT_MAX_US of waiting the window
 * opens anyway (counted in forced), so a link that cannot receive (central
 * gone, consumer stalled) never blocks the flash user for long.
 *
 * Context: ll_flash_open / ll_flash_close run in thread context with
 * ll_plat_lock() held (the B91 glue wraps the hal flash calls); never
 * from an ISR, and the window is never held across a sleep (the caller
 * waits before opening it). ll_flash_active() is read in ISR context.
 */
#ifndef LL_FLASH_H_
#define LL_FLASH_H_

#include <stdbool.h>
#include <stdint.h>

/* Longest flash operation with interrupts off that one window covers: a
 * 4 KB sector erase (measured 13..29 ms; block erases are split into
 * sectors by the glue). */
#define LL_FLASH_OP_MAX_US   30000u
/* Longest wait for the links before a window opens anyway. */
#define LL_FLASH_WAIT_MAX_US 100000u

/* ISR: a flash operation is running or about to run; issue no radio
 * activity. */
bool ll_flash_active(void);
/* Thread, ll_plat_lock() held. waited_us: how long this operation has
 * waited so far. Returns true when the window is open (ready links, or
 * waited_us >= LL_FLASH_WAIT_MAX_US); the radio is then idle until
 * ll_flash_close(). Returns false after pulling the links' next events in:
 * the caller waits (interrupts on, about 1 ms) and calls again. */
bool ll_flash_open(uint32_t now, uint32_t waited_us);
/* Thread, ll_plat_lock() held. No-op without an open window. */
void ll_flash_close(void);
/* Drops the window (controller init). Stats stay cumulative. */
void ll_flash_reset(void);

struct ll_flash_stats {
	uint32_t windows;     /* windows opened (flash operations covered) */
	uint32_t waits;       /* ll_flash_open() calls that asked the caller to wait */
	uint32_t forced;      /* windows opened after LL_FLASH_WAIT_MAX_US with a link not ready */
	uint32_t wait_max_us; /* longest wait before a window opened */
};
void ll_flash_get_stats(struct ll_flash_stats *s);

/* B91 glue (ll_flash_wrap.c, device only): start gating the hal flash
 * calls; called once the controller is initialized. */
void ll_flash_wrap_enable(void);

#endif /* LL_FLASH_H_ */
