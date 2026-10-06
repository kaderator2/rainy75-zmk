/*
 * Copyright (c) 2022 Telink Semiconductor (Shanghai) Co., Ltd.
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * MAC address init, ported from hal_telink b91_bt_init.c. Shared by the blob
 * shim and the open link layer so both report the same BD_ADDR.
 */
#include <string.h>
#include "b91_mac.h"

/* hal_telink flash.c */
extern void flash_read_page(unsigned long addr, unsigned long len, unsigned char *buf);
extern void flash_write_page(unsigned long addr, unsigned long len, unsigned char *buf);

void b91_mac_init(uint32_t flash_addr, void (*rand_fn)(int len, unsigned char *data),
		  uint8_t mac_public[6], uint8_t mac_random_static[6])
{
	static const uint8_t ff_six_byte[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	uint8_t mac_read[8];
	uint8_t value_rand[5];

	if (flash_addr == 0) {
		return;
	}

	flash_read_page(flash_addr, 8, mac_read);
	rand_fn(5, value_rand);

	if (memcmp(mac_read, ff_six_byte, 6)) {
		memcpy(mac_public, mac_read, 6);
	} else {
		mac_public[0] = value_rand[0];
		mac_public[1] = value_rand[1];
		mac_public[2] = value_rand[2];
		mac_public[3] = 0x38; /* company id: 0xA4C138 */
		mac_public[4] = 0xC1;
		mac_public[5] = 0xA4;

		flash_write_page(flash_addr, 6, mac_public);
	}

	mac_random_static[0] = mac_public[0];
	mac_random_static[1] = mac_public[1];
	mac_random_static[2] = mac_public[2];
	mac_random_static[5] = 0xC0; /* random static marker */

	uint16_t high_2_byte = (uint16_t)(mac_read[6] | mac_read[7] << 8);

	if (high_2_byte != 0xFFFF) {
		memcpy(&mac_random_static[3], &mac_read[6], 2);
	} else {
		mac_random_static[3] = value_rand[3];
		mac_random_static[4] = value_rand[4];

		flash_write_page(flash_addr + 6, 2, &mac_random_static[3]);
	}
}
