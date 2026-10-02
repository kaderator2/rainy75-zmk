/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * BLE public / random static address from the flash MAC sector.
 */
#ifndef B91_MAC_H_
#define B91_MAC_H_

#include <stdint.h>

/* MAC address flash offset: 1 MB flash uses 0xFF000 */
#define B91_MAC_FLASH_ADDR 0xFF000

void b91_mac_init(uint32_t flash_addr, void (*rand_fn)(int len, unsigned char *data),
		  uint8_t mac_public[6], uint8_t mac_random_static[6]);

#endif /* B91_MAC_H_ */
