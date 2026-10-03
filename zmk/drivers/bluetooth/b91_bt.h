/*
 * Copyright (c) 2022 Telink Semiconductor (Shanghai) Co., Ltd.
 * Copyright (c) 2025 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * BLE controller shim for Telink B91 (TLSR951x).
 * API-compatible with hal_telink's b91_bt.h, but compiled against
 * standard Zephyr headers to avoid SDK conflicts.
 */

#ifndef B91_BT_H_
#define B91_BT_H_

#include <stdint.h>

typedef struct b91_bt_host_callback {
	void (*host_send_available)(void);
	void (*host_read_packet)(uint8_t *data, uint16_t len);
} b91_bt_host_callback_t;

void b91_bt_host_callback_register(const b91_bt_host_callback_t *callback);
void b91_bt_host_send_packet(uint8_t type, uint8_t *data, uint16_t len);
int b91_bt_controller_init(void);

/* Called from z_sys_poweroff() (IRQs locked) before the SoC enters deep sleep.
 * Open controller: stop the radio, drop both stimer alarm slots, mask the RF
 * and stimer PLIC IRQs. Blob: no-op, its own sleep path is not used for deep
 * sleep and behaviour stays as before. */
void b91_bt_controller_poweroff(void);

/* Open controller only (BT_HCI_B91_CTLR_OPEN): controller-thread wakeups since
 * boot, a power counter read by openll_mgmt. */
uint32_t b91_bt_controller_wakeups(void);

#endif /* B91_BT_H_ */
