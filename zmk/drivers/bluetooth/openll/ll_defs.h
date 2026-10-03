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
#define LL_ST_UNKNOWN_CONN_ID 0x02
#define LL_ST_DISALLOWED     0x0C
#define LL_ST_UNSUPPORTED    0x11
#define LL_ST_INVALID_PARAM  0x12
#define LL_ST_PIN_KEY_MISSING 0x06
#define LL_ST_CONN_TIMEOUT    0x08
#define LL_ST_REMOTE_TERM     0x13
#define LL_ST_LOCAL_TERM      0x16
#define LL_ST_UNSUPP_REMOTE   0x1A
#define LL_ST_LMP_TIMEOUT     0x22
#define LL_ST_LMP_PDU_NOT_ALLOWED 0x24
#define LL_ST_INVALID_LL_PARAM 0x1E
#define LL_ST_UNSPECIFIED     0x1F
#define LL_ST_INSTANT_PASSED  0x28
#define LL_ST_MIC_FAILURE     0x3D
#define LL_ST_CONN_FAIL_EST   0x3E

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

/* Data channel PDUs (Vol 6 Part B 2.4), single connection, no DLE */
#define LL_CONN_HANDLE    0x0000
#define LL_DATA_PDU_MAX   27     /* payload bytes */
#define LL_MIC_LEN        4
#define LL_LLID_CONT      0x1    /* ACL continuation fragment or empty PDU */
#define LL_LLID_START     0x2    /* ACL start fragment or complete message */
#define LL_LLID_CTRL      0x3    /* LL control PDU */
#define LL_OWN_SCA_PPM    50     /* own sleep clock accuracy (crystal) */

/* LL feature set (Vol 6 Part B 4.6), byte 0; bytes 1..7 are 0. Used by
 * LL_FEATURE_RSP and HCI LE Read Local Supported Features. */
#define LL_FEAT_LE_ENC        0x01   /* bit 0: LE Encryption */
#define LL_FEAT_EXT_REJ_IND   0x04   /* bit 2: Extended Reject Indication */
#define LL_FEATURES_LOW       (LL_FEAT_LE_ENC | LL_FEAT_EXT_REJ_IND)

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
