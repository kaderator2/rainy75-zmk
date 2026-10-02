/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * HCI command handling for the open B91 link layer. Pure logic: all
 * controller actions go through struct ll_hci_ops, all events leave through
 * the sink as H4 packets (first byte 0x04).
 */
#ifndef LL_HCI_H_
#define LL_HCI_H_

#include <stdbool.h>
#include <stdint.h>
#include "ll_adv.h"

/* Largest event: Command Complete for Read Local Supported Commands
 * = H4(1) + evt hdr(2) + ncmd(1) + opcode(2) + status(1) + 64 */
#define LL_HCI_EVT_MAX 71

struct ll_hci_ops {
	void (*get_bd_addr)(uint8_t addr[6]);
	void (*rand)(uint8_t *out, uint8_t len);
	void (*reset)(void);
	uint8_t (*adv_set_params)(const struct ll_adv_params *p);
	uint8_t (*adv_set_data)(const uint8_t *data, uint8_t len);
	uint8_t (*adv_set_scan_rsp)(const uint8_t *data, uint8_t len);
	uint8_t (*adv_enable)(bool enable);
	void (*unknown)(uint16_t opcode); /* may be NULL */
};

typedef void (*ll_hci_sink_t)(const uint8_t *h4, uint16_t len);

void ll_hci_init(const struct ll_hci_ops *ops, ll_hci_sink_t sink);

/* cmd = HCI command packet without the H4 type byte:
 * opcode (LE16), parameter length, parameters */
void ll_hci_cmd(const uint8_t *cmd, uint16_t len);

#endif /* LL_HCI_H_ */
