/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Open BLE link layer for Telink B91: shared constants and byte helpers.
 */
#ifndef LL_DEFS_H_
#define LL_DEFS_H_

#include <stdbool.h>
#include <stdint.h>

/* Number of peripheral links (slice 6a multilink). Link ids are
 * 0..LL_MAX_CONN-1. Device builds take it from Kconfig; host tests pass
 * -DLL_MAX_CONN=N. */
#ifdef CONFIG_BT_HCI_B91_OPENLL_MAX_CONN
#define LL_MAX_CONN CONFIG_BT_HCI_B91_OPENLL_MAX_CONN
#else
#ifndef LL_MAX_CONN
#define LL_MAX_CONN 1
#endif
#endif

_Static_assert(LL_MAX_CONN >= 1 && LL_MAX_CONN <= 5, "LL_MAX_CONN must be 1..5");

/* HCI status codes (Core Spec Vol 1 Part F) */
#define LL_ST_SUCCESS        0x00
#define LL_ST_UNKNOWN_CMD    0x01
#define LL_ST_UNKNOWN_CONN_ID 0x02
#define LL_ST_MEM_CAPACITY   0x07   /* Memory Capacity Exceeded */
#define LL_ST_DISALLOWED     0x0C
#define LL_ST_UNSUPPORTED    0x11
#define LL_ST_INVALID_PARAM  0x12
#define LL_ST_PIN_KEY_MISSING 0x06
#define LL_ST_CONN_LIMIT      0x09   /* Connection Limit Exceeded */
#define LL_ST_CONN_TIMEOUT    0x08
#define LL_ST_REMOTE_TERM     0x13
#define LL_ST_LOCAL_TERM      0x16
#define LL_ST_UNSUPP_REMOTE   0x1A
#define LL_ST_LMP_TIMEOUT     0x22
#define LL_ST_LMP_PDU_NOT_ALLOWED 0x24
#define LL_ST_INVALID_LL_PARAM 0x1E
#define LL_ST_UNSPECIFIED     0x1F
#define LL_ST_INSTANT_PASSED  0x28
#define LL_ST_LL_PROC_COLLISION 0x23   /* LMP Error Transaction Collision / LL Procedure Collision */
#define LL_ST_DIFF_TRANS_COLLISION 0x2A /* Different Transaction Collision */
#define LL_ST_UNACCEPT_CONN_PARAM 0x3B  /* Unacceptable Connection Parameters */
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

/* Data channel PDUs (Vol 6 Part B 2.4). The data path carries at most
 * LL_DATA_PDU_MAX payload octets (slice 6b Task 4: the Core Spec maximum;
 * 251 + MIC = 255 bytes on air, the 8-bit header Length); the Data Length
 * Update procedure (ll_llcp) negotiates the per-link limit, with
 * supportedMax following this (LL_DLE_SUPP_OCTETS). */
#define LL_DATA_PDU_MAX   251    /* payload bytes */
#define LL_MIC_LEN        4
#define LL_LLID_CONT      0x1    /* ACL continuation fragment or empty PDU */
#define LL_LLID_START     0x2    /* ACL start fragment or complete message */
#define LL_LLID_CTRL      0x3    /* LL control PDU */
#define LL_OWN_SCA_PPM    50     /* own sleep clock accuracy (crystal) */

/* LL feature set (Vol 6 Part B 4.6), bytes 0 and 1; bytes 2..7 are 0.
 * Used by LL_FEATURE_RSP and HCI LE Read Local Supported Features. */
#define LL_FEAT_LE_ENC        0x01   /* bit 0: LE Encryption */
#define LL_FEAT_CONN_PARAM_REQ 0x02  /* bit 1: Connection Parameters Request procedure (slice 6d Task 2) */
#define LL_FEAT_EXT_REJ_IND   0x04   /* bit 2: Extended Reject Indication */
#define LL_FEAT_LE_PING       0x10   /* bit 4: LE Ping (slice 6d; "O" to the peer, Table 4.7) */
#define LL_FEAT_DLE           0x20   /* bit 5: LE Data Packet Length Extension */
#define LL_FEATURES_LOW       (LL_FEAT_LE_ENC | LL_FEAT_CONN_PARAM_REQ | LL_FEAT_EXT_REJ_IND | \
			       LL_FEAT_LE_PING | LL_FEAT_DLE)
/* byte 1 of the feature set (LL_FEATURE_RSP sends our own byte 1, HCI LE
 * Read Local Supported Features reports it) */
#define LL_FEAT1_CSA2         0x40   /* bit 14: Channel Selection Algorithm #2 */
#define LL_FEATURES_BYTE1     (LL_FEAT1_CSA2)

/* Data Length Extension (Vol 6 Part B 4.5.10, Table 4.6, 1M only).
 * "Octets" is the Payload length WITHOUT the MIC (2.4: the header Length
 * covers Payload and MIC, the Payload is at most 251), so the plaintext
 * limit of an encrypted PDU is connEffectiveMaxTxOctets itself. "Time" is
 * the whole packet incl. the MIC: preamble 1 + AA 4 + header 2 + payload +
 * MIC 4 + CRC 3 octets at 8 us: LL_DLE_TIME_1M(27) = 328,
 * LL_DLE_TIME_1M(251) = 2120. */
#define LL_DLE_MIN_OCTETS     27
#define LL_DLE_MIN_TIME       328
#define LL_DLE_MAX_OCTETS     251
#define LL_DLE_MAX_TIME_1M    2120
#define LL_DLE_MAX_TIME_ANY   17040  /* Table 4.6 upper bound (Coded); remote values are capped to it */
#define LL_DLE_TIME_1M(octets) (((octets) + 14) * 8)
/* Our supportedMax{Tx,Rx}Octets: what the data path can carry. Follows
 * LL_DATA_PDU_MAX (251); host tests may override it (27: the LENGTH rules
 * of a controller without long PDUs). supportedMax{Tx,Rx}Time is the 1M
 * time for it. */
#ifndef LL_DLE_SUPP_OCTETS
#define LL_DLE_SUPP_OCTETS    (LL_DATA_PDU_MAX < LL_DLE_MAX_OCTETS ? LL_DATA_PDU_MAX : LL_DLE_MAX_OCTETS)
#endif
/* Overridable for device tests (e.g. -DLL_DLE_SUPP_TIME=415: every link then
 * runs at 251 B / 415 us both ways, as with a central that limits Time) */
#ifndef LL_DLE_SUPP_TIME
#define LL_DLE_SUPP_TIME      LL_DLE_TIME_1M(LL_DLE_SUPP_OCTETS)
#endif
_Static_assert(LL_DLE_SUPP_OCTETS >= LL_DLE_MIN_OCTETS && LL_DLE_SUPP_OCTETS <= LL_DLE_MAX_OCTETS,
	       "LL_DLE_SUPP_OCTETS must be 27..251");

/* Largest plaintext payload the link may send in one PDU (slice 6b Task
 * 4): connEffectiveMaxTxOctets, and no PDU may take longer than
 * connEffectiveMaxTxTime on air (4.5.10), whose 1M packet time counts
 * preamble 1 + AA 4 + header 2 + CRC 3 octets and the MIC when encrypted:
 * min(octets, time / 8 - 14) encrypted, min(octets, time / 8 - 10) plain.
 * Kept within 27..LL_DATA_PDU_MAX (the effective values never go below
 * 27 / 328). */
static inline uint8_t ll_dle_tx_limit(uint16_t max_tx_octets, uint16_t max_tx_time, bool enc)
{
	uint32_t by_time = max_tx_time / 8u;
	uint32_t over = enc ? 14u : 10u;
	uint32_t lim = max_tx_octets;

	by_time = by_time > over ? by_time - over : 0;
	if (by_time < lim) {
		lim = by_time;
	}
	if (lim > LL_DATA_PDU_MAX) {
		lim = LL_DATA_PDU_MAX;
	}
	return (uint8_t)(lim < LL_DLE_MIN_OCTETS ? LL_DLE_MIN_OCTETS : lim);
}

/* One PDU of a fragmented host ACL packet (ll_hci_acl_fragment): payload
 * bytes off .. off + len - 1 of the packet, with LLID llid. */
struct ll_acl_frag {
	uint8_t off;
	uint8_t len;
	uint8_t llid;
};

/* PHY bits (Vol 6 Part B 2.4.2.22, HCI TX_PHYs/RX_PHYs) and HCI PHY values */
#define LL_PHY_1M             0x01

/* Controller identity reported via Read Local Version Information */
#define LL_HCI_VERSION    0x09   /* Bluetooth Core 5.0 */
#define LL_COMPANY_ID     0xFFFF /* reserved for internal use / testing */
#define LL_SUBVERSION     0x0001

/* ACL buffers reported via LE Read Buffer Size (used from slice 3 on).
 * Slice 6b Task 4: a host ACL packet of up to LL_ACL_MTU octets is split
 * into PDUs of the link's TX limit (ll_hci_acl_fragment, ll_dle_tx_limit). */
#define LL_ACL_MTU        251
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
