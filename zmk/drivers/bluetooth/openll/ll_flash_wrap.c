/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * B91 glue of the flash window (ll_flash.h): every flash erase and write
 * of the hal (hal_telink tlsr9/drivers/B91/flash.c, which turns interrupts
 * off for the whole operation) runs inside a window, one operation at a
 * time. The hal functions are wrapped at link time (-Wl,--wrap, see
 * CMakeLists.txt), so every caller is covered without a Zephyr or hal
 * patch: the Zephyr flash driver (NVS / settings, mcumgr img_mgmt, the
 * flash_mgmt group 64 erase/write) and b91_mac.
 *
 * - Sector and page erases: one window each. 32 KB / 64 KB block erases
 *   (the Zephyr driver uses them for aligned ranges) are split into sector
 *   erases, so no single interrupt-off section exceeds LL_FLASH_OP_MAX_US
 *   and the links get events between them.
 * - Writes: one window per 256-byte page (the hal enables interrupts
 *   between pages anyway).
 * - Reads: no window. Measured with interrupts off: at most 66 us (NVS
 *   and settings reads are small), against at least 2.8 ms of blocked RF
 *   ISR for 4 short central packets to fill the RX ring. Split into
 *   256-byte reads so a long read never holds interrupts off for long.
 *   (Interrupt-off times measured in the same run: sector erase up to
 *   12.4 ms, here and 13..29 ms in slice 7 Task 2c; page write up to
 *   1.9 ms.)
 *
 * Waiting: ll_flash_open() refuses while a link needs an event first; the
 * caller then sleeps 1 ms (k_msleep, or k_busy_wait where it cannot
 * yield; interrupts stay on either way, the radio runs from ISRs) and asks
 * again. The window itself is only held across the hal call, never across
 * a sleep. In ISR context and before the controller is up (b91_mac reads
 * the MAC during init, MCUboot confirmation) the operation runs ungated.
 */
#include <zephyr/kernel.h>

#include "ll_defs.h"
#include "ll_flash.h"
#include "ll_plat.h"
#include "ll_radio.h"

#define FLASH_PAGE   256u
#define FLASH_SECTOR 0x1000u

void __real_flash_erase_sector(unsigned long addr);
void __real_flash_erase_page(unsigned int addr);
void __real_flash_write_page(unsigned long addr, unsigned long len, unsigned char *buf);
void __real_flash_read_page(unsigned long addr, unsigned long len, unsigned char *buf);

void __wrap_flash_erase_sector(unsigned long addr);
void __wrap_flash_erase_page(unsigned int addr);
void __wrap_flash_erase_32kblock(unsigned int addr);
void __wrap_flash_erase_64kblock(unsigned int addr);
void __wrap_flash_write_page(unsigned long addr, unsigned long len, unsigned char *buf);
void __wrap_flash_read_page(unsigned long addr, unsigned long len, unsigned char *buf);

static bool gate_on;

void ll_flash_wrap_enable(void)
{
	gate_on = true;
}

/* true: a window is open and must be closed with win_exit() */
static bool win_enter(void)
{
	uint32_t t0;

	if (!gate_on || k_is_in_isr()) {
		return false;
	}
	t0 = ll_radio_now();
	for (;;) {
		uint32_t waited = (ll_radio_now() - t0) / LL_TICKS_PER_US;
		unsigned int key = ll_plat_lock();
		bool ok = ll_flash_open(ll_radio_now(), waited);

		ll_plat_unlock(key);
		if (ok) {
			return true;
		}
		if (k_can_yield()) {
			k_msleep(1);
		} else {
			k_busy_wait(1000);
		}
	}
}

static void win_exit(bool open)
{
	if (open) {
		unsigned int key = ll_plat_lock();

		ll_flash_close();
		ll_plat_unlock(key);
	}
}

void __wrap_flash_erase_sector(unsigned long addr)
{
	bool w = win_enter();

	__real_flash_erase_sector(addr);
	win_exit(w);
}

void __wrap_flash_erase_page(unsigned int addr)
{
	bool w = win_enter();

	__real_flash_erase_page(addr);
	win_exit(w);
}

void __wrap_flash_erase_32kblock(unsigned int addr)
{
	for (unsigned int i = 0; i < 0x8000u / FLASH_SECTOR; i++) {
		__wrap_flash_erase_sector(addr + i * FLASH_SECTOR);
	}
}

void __wrap_flash_erase_64kblock(unsigned int addr)
{
	for (unsigned int i = 0; i < 0x10000u / FLASH_SECTOR; i++) {
		__wrap_flash_erase_sector(addr + i * FLASH_SECTOR);
	}
}

void __wrap_flash_write_page(unsigned long addr, unsigned long len, unsigned char *buf)
{
	while (len > 0) {
		unsigned long n = FLASH_PAGE - (addr & (FLASH_PAGE - 1));
		bool w;

		if (n > len) {
			n = len;
		}
		w = win_enter();
		__real_flash_write_page(addr, n, buf);
		win_exit(w);
		addr += n;
		buf += n;
		len -= n;
	}
}

void __wrap_flash_read_page(unsigned long addr, unsigned long len, unsigned char *buf)
{
	while (len > 0) {
		unsigned long n = len > FLASH_PAGE ? FLASH_PAGE : len;

		__real_flash_read_page(addr, n, buf);
		addr += n;
		buf += n;
		len -= n;
	}
}
