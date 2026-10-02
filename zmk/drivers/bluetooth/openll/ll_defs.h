/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Open BLE link layer for Telink B91: shared constants and byte helpers.
 */
#ifndef LL_DEFS_H_
#define LL_DEFS_H_

#include <stdint.h>

/* HCI status codes (Core Spec Vol 1 Part F) */
#define LL_ST_SUCCESS        0x00
#define LL_ST_UNKNOWN_CMD    0x01
#define LL_ST_DISALLOWED     0x0C
#define LL_ST_UNSUPPORTED    0x11
#define LL_ST_INVALID_PARAM  0x12

/* Advertising physical channel PDU types (Vol 6 Part B 2.3) */
#define LL_PDU_ADV_IND          0x0
#define LL_PDU_ADV_DIRECT_IND   0x1
#define LL_PDU_ADV_NONCONN_IND  0x2
#define LL_PDU_SCAN_REQ         0x3
#define LL_PDU_SCAN_RSP         0x4
#define LL_PDU_CONNECT_IND      0x5
#define LL_PDU_ADV_SCAN_IND     0x6

#define LL_ADV_DATA_MAX   31
#define LL_ADV_PDU_MAX    (2 + 6 + LL_ADV_DATA_MAX)

/* Controller identity reported via Read Local Version Information */
#define LL_HCI_VERSION    0x09   /* Bluetooth Core 5.0 */
#define LL_COMPANY_ID     0xFFFF /* reserved for internal use / testing */
#define LL_SUBVERSION     0x0001

/* ACL buffers reported via LE Read Buffer Size (used from slice 3 on) */
#define LL_ACL_MTU        27
#define LL_ACL_NUM        3

/* Timing: B91 system timer runs at 16 MHz */
#define LL_TICKS_PER_US   16
#define LL_T_IFS_US       150

static inline uint16_t ll_get_le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

static inline uint32_t ll_get_le24(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

static inline uint32_t ll_get_le32(const uint8_t *p)
{
	return ll_get_le24(p) | ((uint32_t)p[3] << 24);
}

static inline void ll_put_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

#endif /* LL_DEFS_H_ */
