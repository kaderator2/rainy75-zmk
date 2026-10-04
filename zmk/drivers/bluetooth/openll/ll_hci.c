/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 */
#include <assert.h>
#include <errno.h>
#include <string.h>
#include "ll_hci.h"
#include "ll_defs.h"

#define OP(ogf, ocf) ((uint16_t)(((ogf) << 10) | (ocf)))

#define OP_DISCONNECT            OP(0x01, 0x0006)
#define OP_SET_EVENT_MASK        OP(0x03, 0x0001)
#define OP_RESET                 OP(0x03, 0x0003)
#define OP_HOST_NUM_COMPLETED    OP(0x03, 0x0035)
#define OP_SET_EVENT_MASK_PAGE_2 OP(0x03, 0x0063)
#define OP_READ_AUTH_PAYLOAD_TO  OP(0x03, 0x007B)
#define OP_WRITE_AUTH_PAYLOAD_TO OP(0x03, 0x007C)
#define OP_READ_LOCAL_VERSION    OP(0x04, 0x0001)
#define OP_READ_LOCAL_CMDS       OP(0x04, 0x0002)
#define OP_READ_LOCAL_FEATURES   OP(0x04, 0x0003)
#define OP_READ_BD_ADDR          OP(0x04, 0x0009)
#define OP_LE_SET_EVENT_MASK     OP(0x08, 0x0001)
#define OP_LE_READ_BUFFER_SIZE   OP(0x08, 0x0002)
#define OP_LE_READ_FEATURES      OP(0x08, 0x0003)
#define OP_LE_SET_RANDOM_ADDR    OP(0x08, 0x0005)
#define OP_LE_SET_ADV_PARAMS     OP(0x08, 0x0006)
#define OP_LE_SET_ADV_DATA       OP(0x08, 0x0008)
#define OP_LE_SET_SCAN_RSP_DATA  OP(0x08, 0x0009)
#define OP_LE_SET_ADV_ENABLE     OP(0x08, 0x000A)
#define OP_LE_RAND               OP(0x08, 0x0018)
#define OP_LE_LTK_REPLY          OP(0x08, 0x001A)
#define OP_LE_LTK_NEG_REPLY      OP(0x08, 0x001B)
#define OP_LE_CPR_REPLY          OP(0x08, 0x0020)
#define OP_LE_CPR_NEG_REPLY      OP(0x08, 0x0021)
#define OP_LE_SET_DATA_LEN       OP(0x08, 0x0022)
#define OP_LE_READ_DEF_DATA_LEN  OP(0x08, 0x0023)
#define OP_LE_WRITE_DEF_DATA_LEN OP(0x08, 0x0024)
#define OP_LE_READ_MAX_DATA_LEN  OP(0x08, 0x002F)
#define OP_LE_READ_PHY           OP(0x08, 0x0030)
#define OP_LE_SET_DEFAULT_PHY    OP(0x08, 0x0031)
#define OP_LE_SET_PHY            OP(0x08, 0x0032)

/* Events (Vol 4 Part E 7.7) */
#define EVT_DISCONN_COMPLETE     0x05
#define EVT_ENC_CHANGE           0x08
#define EVT_CMD_COMPLETE         0x0E
#define EVT_CMD_STATUS           0x0F
#define EVT_NUM_COMPLETED        0x13
#define EVT_LE_META              0x3E
#define EVT_AUTH_PAYLOAD_TO_EXP  0x57
#define SUBEVT_CONN_COMPLETE     0x01
#define SUBEVT_CONN_UPDATE       0x03
#define SUBEVT_LTK_REQ           0x05
#define SUBEVT_CONN_PARAM_REQ    0x06
#define SUBEVT_DATA_LEN_CHANGE   0x07
#define SUBEVT_PHY_UPDATE        0x0C
#define SUBEVT_CHAN_SEL_ALGO     0x14

/* Event mask bits (7.3.1) and defaults (7.3.1, 7.8.1) */
#define MASK_DISCONN_COMPLETE    (1ULL << 4)
#define MASK_ENC_CHANGE          (1ULL << 7)
#define MASK_LE_META             (1ULL << 61)
#define EVENT_MASK_DEFAULT       0x00001FFFFFFFFFFFULL
#define LE_EVENT_MASK_DEFAULT    0x000000000000001FULL
/* Event Mask Page 2 (7.3.69): all 0 by default; bit 23 Authenticated
 * Payload Timeout Expired */
#define MASK2_AUTH_PAYLOAD_TO    (1ULL << 23)

#define HCI_ROLE_PERIPHERAL      0x01
#define ACL_PB_FIRST_NONFLUSH    0x0
#define ACL_PB_CONT              0x1
#define ACL_PB_FIRST_FLUSH       0x2

/* Supported Commands bitmap entries: {octet, bit} */
static const uint8_t supported_cmds[][2] = {
	{0, 5},  /* Disconnect */
	{5, 6},  /* Set Event Mask */
	{5, 7},  /* Reset */
	{14, 3}, /* Read Local Version Information */
	{14, 5}, /* Read Local Supported Features */
	{15, 1}, /* Read BD_ADDR */
	{22, 2}, /* Set Event Mask Page 2 (slice 6d) */
	{25, 0}, /* LE Set Event Mask */
	{25, 1}, /* LE Read Buffer Size */
	{25, 2}, /* LE Read Local Supported Features */
	{25, 4}, /* LE Set Random Address (slice 6c) */
	{25, 5}, /* LE Set Advertising Parameters */
	{25, 7}, /* LE Set Advertising Data */
	{26, 0}, /* LE Set Scan Response Data */
	{26, 1}, /* LE Set Advertising Enable */
	{27, 7}, /* LE Rand */
	{28, 1}, /* LE Long Term Key Request Reply */
	{28, 2}, /* LE Long Term Key Request Negative Reply */
	{32, 4}, /* Read Authenticated Payload Timeout (slice 6d) */
	{32, 5}, /* Write Authenticated Payload Timeout (slice 6d) */
	{33, 4}, /* LE Remote Connection Parameter Request Reply (slice 6d) */
	{33, 5}, /* LE Remote Connection Parameter Request Negative Reply (slice 6d) */
	{33, 6}, /* LE Set Data Length */
	{33, 7}, /* LE Read Suggested Default Data Length */
	{34, 0}, /* LE Write Suggested Default Data Length */
	{35, 3}, /* LE Read Maximum Data Length */
	{35, 4}, /* LE Read PHY */
	{35, 5}, /* LE Set Default PHY */
	{35, 6}, /* LE Set PHY */
};

/* HCI parameter ranges of TX_Octets / TX_Time (Vol 4 Part E 7.8.33-35) */
#define HCI_DLE_OCTETS_MIN       0x001B
#define HCI_DLE_OCTETS_MAX       0x00FB
#define HCI_DLE_TIME_MIN         0x0148
#define HCI_DLE_TIME_MAX         0x4290

static const struct ll_hci_ops *hci_ops;
static ll_hci_sink_t hci_sink;
static uint64_t event_mask = EVENT_MASK_DEFAULT;
static uint64_t le_event_mask = LE_EVENT_MASK_DEFAULT;
static uint64_t event_mask2;
/* Host suggestions for new connections (7.8.35) and default PHYs (7.8.48);
 * the PHY preference is stored only (1M is the one PHY we have). */
static uint16_t def_tx_octets = LL_DLE_MIN_OCTETS, def_tx_time = LL_DLE_MIN_TIME;
static uint8_t def_phy[3] = {0x00, LL_PHY_1M, LL_PHY_1M};   /* All_PHYs, TX, RX */

static void masks_default(void)
{
	event_mask = EVENT_MASK_DEFAULT;
	le_event_mask = LE_EVENT_MASK_DEFAULT;
	event_mask2 = 0;
	def_tx_octets = LL_DLE_MIN_OCTETS;
	def_tx_time = LL_DLE_MIN_TIME;
	def_phy[0] = 0x00;
	def_phy[1] = LL_PHY_1M;
	def_phy[2] = LL_PHY_1M;
}

void ll_hci_default_data_len(uint16_t *tx_octets, uint16_t *tx_time)
{
	*tx_octets = def_tx_octets;
	*tx_time = def_tx_time;
}

void ll_hci_init(const struct ll_hci_ops *ops, ll_hci_sink_t sink)
{
	/* required by the connection commands and host ACL: fail loudly at
	 * init instead of a NULL call on the first Disconnect / ACL packet */
	assert(ops != NULL && ops->handle_valid != NULL);
	assert(ops->set_data_len != NULL && ops->read_phy != NULL && ops->set_phy != NULL);
	assert(ops->set_random_addr != NULL);
	assert(ops->read_apto != NULL && ops->write_apto != NULL);
	assert(ops->conn_param_reply != NULL && ops->conn_param_neg_reply != NULL);
	hci_ops = ops;
	hci_sink = sink;
	masks_default();
}

static uint64_t get_le64(const uint8_t *p)
{
	return (uint64_t)ll_get_le32(p) | ((uint64_t)ll_get_le32(&p[4]) << 32);
}

static void cmd_complete(uint16_t op, const uint8_t *ret, uint8_t ret_len)
{
	uint8_t evt[LL_HCI_EVT_MAX];

	evt[0] = 0x04;               /* H4: event */
	evt[1] = EVT_CMD_COMPLETE;
	evt[2] = 3 + ret_len;
	evt[3] = 1;                  /* Num_HCI_Command_Packets */
	ll_put_le16(&evt[4], op);
	memcpy(&evt[6], ret, ret_len);
	hci_sink(evt, 6 + ret_len);
}

static void status_only(uint16_t op, uint8_t status)
{
	cmd_complete(op, &status, 1);
}

static void cmd_status(uint16_t op, uint8_t status)
{
	uint8_t evt[7];

	evt[0] = 0x04;
	evt[1] = EVT_CMD_STATUS;
	evt[2] = 4;
	evt[3] = status;
	evt[4] = 1;                  /* Num_HCI_Command_Packets */
	ll_put_le16(&evt[5], op);
	hci_sink(evt, sizeof(evt));
}

/* Disconnect reasons allowed by Vol 4 Part E 7.1.6 */
static bool disconnect_reason_ok(uint8_t r)
{
	switch (r) {
	case 0x05: case 0x13: case 0x14: case 0x15: case 0x1A: case 0x29: case 0x3B:
		return true;
	default:
		return false;
	}
}

static void disconnect(uint16_t op, const uint8_t *p, uint8_t plen)
{
	if (plen != 3 || !disconnect_reason_ok(p[2])) {
		cmd_status(op, LL_ST_INVALID_PARAM);
	} else if (!hci_ops->handle_valid(ll_get_le16(p))) {
		cmd_status(op, LL_ST_UNKNOWN_CONN_ID);
	} else {
		cmd_status(op, hci_ops->disconnect(ll_get_le16(p), p[2]));
	}
}

/* LE LTK Request Reply / Negative Reply: Command Complete with status and
 * the connection handle from the command */
static void ltk_reply(uint16_t op, const uint8_t *p, uint8_t plen)
{
	uint8_t ret[3];
	uint8_t want = op == OP_LE_LTK_REPLY ? 18 : 2;

	if (plen != want) {
		status_only(op, LL_ST_INVALID_PARAM);
		return;
	}
	memcpy(&ret[1], p, 2);
	if (!hci_ops->handle_valid(ll_get_le16(p))) {
		ret[0] = LL_ST_UNKNOWN_CONN_ID;
	} else if (op == OP_LE_LTK_REPLY) {
		ret[0] = hci_ops->ltk_reply(ll_get_le16(p), &p[2]);
	} else {
		ret[0] = hci_ops->ltk_neg_reply(ll_get_le16(p));
	}
	cmd_complete(op, ret, 3);
}

static bool dle_range_ok(uint16_t octets, uint16_t time)
{
	return octets >= HCI_DLE_OCTETS_MIN && octets <= HCI_DLE_OCTETS_MAX &&
	       time >= HCI_DLE_TIME_MIN && time <= HCI_DLE_TIME_MAX;
}

/* LE Set Data Length: Command Complete (status, handle) */
static void set_data_len(uint16_t op, const uint8_t *p, uint8_t plen)
{
	uint8_t ret[3];
	uint16_t h;

	if (plen != 6) {
		status_only(op, LL_ST_INVALID_PARAM);
		return;
	}
	h = ll_get_le16(p);
	memcpy(&ret[1], p, 2);
	if (!dle_range_ok(ll_get_le16(&p[2]), ll_get_le16(&p[4]))) {
		ret[0] = LL_ST_INVALID_PARAM;
	} else if (!hci_ops->handle_valid(h)) {
		ret[0] = LL_ST_UNKNOWN_CONN_ID;
	} else {
		ret[0] = hci_ops->set_data_len(h, ll_get_le16(&p[2]), ll_get_le16(&p[4]));
	}
	cmd_complete(op, ret, 3);
}

/* Read Authenticated Payload Timeout (7.3.93): Command Complete (status,
 * handle, timeout); Write (7.3.94): (status, handle), 0x0001..0xFFFF, the
 * connInterval x (1 + latency) rule is the op's (ll_llcp_write_apto). */
static void auth_payload_to(uint16_t op, const uint8_t *p, uint8_t plen)
{
	uint8_t ret[5] = {0};
	uint16_t h, v = 0;
	bool wr = op == OP_WRITE_AUTH_PAYLOAD_TO;

	if (plen != (wr ? 4 : 2)) {
		status_only(op, LL_ST_INVALID_PARAM);
		return;
	}
	h = ll_get_le16(p);
	memcpy(&ret[1], p, 2);
	if (wr && ll_get_le16(&p[2]) == 0) {
		ret[0] = LL_ST_INVALID_PARAM;
	} else if (!hci_ops->handle_valid(h)) {
		ret[0] = LL_ST_UNKNOWN_CONN_ID;
	} else if (wr) {
		ret[0] = hci_ops->write_apto(h, ll_get_le16(&p[2]));
	} else {
		ret[0] = hci_ops->read_apto(h, &v);
	}
	if (wr) {
		cmd_complete(op, ret, 3);
	} else {
		ll_put_le16(&ret[3], ret[0] == LL_ST_SUCCESS ? v : 0);
		cmd_complete(op, ret, 5);
	}
}

/* Connection parameter ranges of the Reply (7.8.31) */
#define HCI_CONN_INTERVAL_MIN    0x0006
#define HCI_CONN_INTERVAL_MAX    0x0C80
#define HCI_CONN_LATENCY_MAX     0x01F3
#define HCI_SUP_TIMEOUT_MIN      0x000A
#define HCI_SUP_TIMEOUT_MAX      0x0C80

/* LE Remote Connection Parameter Request Reply (7.8.31) / Negative Reply
 * (7.8.32): Command Complete (status, handle). The values are checked
 * before the handle, as in LE Set Data Length. Timeout x 10 ms > (1 +
 * Max_Latency) x Interval_Max x 1.25 ms x 2 (no subrating) is Timeout x 4 >
 * (1 + Max_Latency) x Interval_Max. The Negative Reply's Reason is passed
 * on as the LL_REJECT_EXT_IND ErrorCode: 7.8.32 lists 0x3B only, Zephyr
 * sends 0x1E; 0x00 (no error) is refused. */
static void conn_param_reply(uint16_t op, const uint8_t *p, uint8_t plen)
{
	uint8_t ret[3];
	uint16_t h;

	if (plen != (op == OP_LE_CPR_REPLY ? 14 : 3)) {
		status_only(op, LL_ST_INVALID_PARAM);
		return;
	}
	h = ll_get_le16(p);
	memcpy(&ret[1], p, 2);
	if (op == OP_LE_CPR_REPLY) {
		uint16_t imin = ll_get_le16(&p[2]), imax = ll_get_le16(&p[4]);
		uint16_t lat = ll_get_le16(&p[6]), to = ll_get_le16(&p[8]);

		if (imin < HCI_CONN_INTERVAL_MIN || imax > HCI_CONN_INTERVAL_MAX || imin > imax ||
		    lat > HCI_CONN_LATENCY_MAX || to < HCI_SUP_TIMEOUT_MIN ||
		    to > HCI_SUP_TIMEOUT_MAX || (uint32_t)to * 4u <= (1u + lat) * (uint32_t)imax ||
		    ll_get_le16(&p[10]) > ll_get_le16(&p[12])) {
			ret[0] = LL_ST_INVALID_PARAM;
		} else if (!hci_ops->handle_valid(h)) {
			ret[0] = LL_ST_UNKNOWN_CONN_ID;
		} else {
			ret[0] = hci_ops->conn_param_reply(h, imin, imax, lat, to);
		}
	} else if (p[2] == 0) {
		ret[0] = LL_ST_INVALID_PARAM;
	} else if (!hci_ops->handle_valid(h)) {
		ret[0] = LL_ST_UNKNOWN_CONN_ID;
	} else {
		ret[0] = hci_ops->conn_param_neg_reply(h, p[2]);
	}
	cmd_complete(op, ret, 3);
}

/* LE Read PHY: Command Complete (status, handle, TX_PHY, RX_PHY) */
static void read_phy(uint16_t op, const uint8_t *p, uint8_t plen)
{
	uint8_t ret[5] = {0};
	uint16_t h;

	if (plen != 2) {
		status_only(op, LL_ST_INVALID_PARAM);
		return;
	}
	h = ll_get_le16(p);
	memcpy(&ret[1], p, 2);
	if (!hci_ops->handle_valid(h)) {
		ret[0] = LL_ST_UNKNOWN_CONN_ID;
	} else {
		ret[0] = hci_ops->read_phy(h, &ret[3], &ret[4]);
	}
	cmd_complete(op, ret, 5);
}

/* All_PHYs / TX_PHYs / RX_PHYs of LE Set Default PHY and LE Set PHY
 * (7.8.48, 7.8.49): a direction with a preference needs at least one bit
 * (else Invalid HCI Command Parameters); a bit for a PHY we lack, RFU
 * included, is Unsupported Feature or Parameter Value. Only 1M here, so no
 * asymmetric combination can pass. */
static uint8_t phys_check(uint8_t all, uint8_t tx, uint8_t rx)
{
	if (!(all & 0x01)) {
		if (tx == 0) {
			return LL_ST_INVALID_PARAM;
		}
		if (tx & ~LL_PHY_1M) {
			return LL_ST_UNSUPPORTED;
		}
	}
	if (!(all & 0x02)) {
		if (rx == 0) {
			return LL_ST_INVALID_PARAM;
		}
		if (rx & ~LL_PHY_1M) {
			return LL_ST_UNSUPPORTED;
		}
	}
	return LL_ST_SUCCESS;
}

/* LE Set PHY: Command Status; on success LE PHY Update Complete follows
 * right away from here (1M only: "neither PHY will change immediately",
 * 7.8.49), through the same sink, so it cannot overtake the status. */
static void set_phy(uint16_t op, const uint8_t *p, uint8_t plen)
{
	uint8_t st, tx = LL_PHY_1M, rx = LL_PHY_1M;
	uint16_t h;

	if (plen != 7) {
		cmd_status(op, LL_ST_INVALID_PARAM);
		return;
	}
	h = ll_get_le16(p);
	if (!hci_ops->handle_valid(h)) {
		cmd_status(op, LL_ST_UNKNOWN_CONN_ID);
		return;
	}
	st = phys_check(p[2], p[3], p[4]);
	if (st == LL_ST_SUCCESS) {
		st = hci_ops->set_phy(h, p[2], p[3], p[4], ll_get_le16(&p[5]));
	}
	cmd_status(op, st);
	if (st == LL_ST_SUCCESS) {
		(void)hci_ops->read_phy(h, &tx, &rx);
		ll_hci_evt_phy_update(h, LL_ST_SUCCESS, tx, rx);
	}
}

static void set_adv_params(uint16_t op, const uint8_t *p, uint8_t plen)
{
	struct ll_adv_params prm;

	if (plen != 15) {
		status_only(op, LL_ST_INVALID_PARAM);
		return;
	}
	prm.interval_min = ll_get_le16(&p[0]);
	prm.interval_max = ll_get_le16(&p[2]);
	prm.type = p[4];
	prm.own_addr_type = p[5];
	prm.peer_addr_type = p[6];
	memcpy(prm.peer_addr, &p[7], 6);
	prm.chan_map = p[13];
	prm.filter_policy = p[14];
	status_only(op, hci_ops->adv_set_params(&prm));
}

static void set_data(uint16_t op, const uint8_t *p, uint8_t plen,
		     uint8_t (*fn)(const uint8_t *, uint8_t))
{
	if (plen != 32 || p[0] > LL_ADV_DATA_MAX) {
		status_only(op, LL_ST_INVALID_PARAM);
		return;
	}
	status_only(op, fn(&p[1], p[0]));
}

void ll_hci_cmd(const uint8_t *cmd, uint16_t len)
{
	uint8_t ret[1 + 64];
	uint16_t op;
	uint8_t plen;
	const uint8_t *p;

	if (len < 3) {
		return;
	}
	op = ll_get_le16(cmd);
	plen = cmd[2];
	p = &cmd[3];
	if (op == OP_HOST_NUM_COMPLETED) {
		/* Host Number Of Completed Packets (7.3.40): no event in normal
		 * operation. Host flow control is not supported (Read Local
		 * Supported Commands has no Set Controller To Host Flow Control,
		 * octet 10 bit 5, so Zephyr never enables it), but Zephyr with
		 * CONFIG_BT_HCI_ACL_FLOW_CONTROL still sends one per received ACL
		 * packet and logs any Command Complete for it as unexpected. A
		 * silent no-op, whatever the parameters. */
		return;
	}
	if (plen != len - 3) {
		status_only(op, LL_ST_INVALID_PARAM);
		return;
	}

	ret[0] = LL_ST_SUCCESS;
	switch (op) {
	case OP_RESET:
		hci_ops->reset();
		masks_default();
		status_only(op, LL_ST_SUCCESS);
		break;
	case OP_SET_EVENT_MASK:
	case OP_LE_SET_EVENT_MASK:
	case OP_SET_EVENT_MASK_PAGE_2:
		if (plen != 8) {
			status_only(op, LL_ST_INVALID_PARAM);
			break;
		}
		if (op == OP_SET_EVENT_MASK) {
			event_mask = get_le64(p);
		} else if (op == OP_LE_SET_EVENT_MASK) {
			le_event_mask = get_le64(p);
		} else {
			/* bits of events we do not send act as 0 (7.3.69) */
			event_mask2 = get_le64(p);
		}
		status_only(op, LL_ST_SUCCESS);
		break;
	case OP_DISCONNECT:
		disconnect(op, p, plen);
		break;
	case OP_READ_AUTH_PAYLOAD_TO:
	case OP_WRITE_AUTH_PAYLOAD_TO:
		auth_payload_to(op, p, plen);
		break;
	case OP_LE_LTK_REPLY:
	case OP_LE_LTK_NEG_REPLY:
		ltk_reply(op, p, plen);
		break;
	case OP_LE_CPR_REPLY:
	case OP_LE_CPR_NEG_REPLY:
		conn_param_reply(op, p, plen);
		break;
	case OP_READ_LOCAL_VERSION:
		ret[1] = LL_HCI_VERSION;
		ll_put_le16(&ret[2], 0);
		ret[4] = LL_HCI_VERSION;
		ll_put_le16(&ret[5], LL_COMPANY_ID);
		ll_put_le16(&ret[7], LL_SUBVERSION);
		cmd_complete(op, ret, 9);
		break;
	case OP_READ_LOCAL_CMDS:
		memset(&ret[1], 0, 64);
		for (size_t i = 0; i < sizeof(supported_cmds) / sizeof(supported_cmds[0]); i++) {
			ret[1 + supported_cmds[i][0]] |= (uint8_t)(1 << supported_cmds[i][1]);
		}
		cmd_complete(op, ret, 65);
		break;
	case OP_READ_LOCAL_FEATURES:
		memset(&ret[1], 0, 8);
		ret[1 + 4] = 0x60; /* BR/EDR Not Supported | LE Supported (Controller) */
		cmd_complete(op, ret, 9);
		break;
	case OP_READ_BD_ADDR:
		hci_ops->get_bd_addr(&ret[1]);
		cmd_complete(op, ret, 7);
		break;
	case OP_LE_READ_BUFFER_SIZE:
		ll_put_le16(&ret[1], LL_ACL_MTU);
		ret[3] = LL_ACL_NUM;
		cmd_complete(op, ret, 4);
		break;
	case OP_LE_READ_FEATURES:
		memset(&ret[1], 0, 8);
		ret[1] = LL_FEATURES_LOW;
		ret[2] = LL_FEATURES_BYTE1;
		cmd_complete(op, ret, 9);
		break;
	case OP_LE_RAND:
		hci_ops->rand(&ret[1], 8);
		cmd_complete(op, ret, 9);
		break;
	case OP_LE_SET_RANDOM_ADDR:
		/* Vol 4 Part E 7.8.4: Random_Address (6 octets); the 0x0C rule
		 * (advertising with own type random) is ll_adv's */
		if (plen != 6) {
			status_only(op, LL_ST_INVALID_PARAM);
		} else {
			status_only(op, hci_ops->set_random_addr(p));
		}
		break;
	case OP_LE_SET_ADV_PARAMS:
		set_adv_params(op, p, plen);
		break;
	case OP_LE_SET_ADV_DATA:
		set_data(op, p, plen, hci_ops->adv_set_data);
		break;
	case OP_LE_SET_SCAN_RSP_DATA:
		set_data(op, p, plen, hci_ops->adv_set_scan_rsp);
		break;
	case OP_LE_SET_DATA_LEN:
		set_data_len(op, p, plen);
		break;
	case OP_LE_READ_DEF_DATA_LEN:
		if (plen != 0) {
			status_only(op, LL_ST_INVALID_PARAM);
			break;
		}
		ll_put_le16(&ret[1], def_tx_octets);
		ll_put_le16(&ret[3], def_tx_time);
		cmd_complete(op, ret, 5);
		break;
	case OP_LE_WRITE_DEF_DATA_LEN:
		if (plen != 4 || !dle_range_ok(ll_get_le16(p), ll_get_le16(&p[2]))) {
			status_only(op, LL_ST_INVALID_PARAM);
			break;
		}
		/* stored as written (7.8.34 reads them back); clamped to what
		 * we support per connection (ll_llcp_set_data_len) */
		def_tx_octets = ll_get_le16(p);
		def_tx_time = ll_get_le16(&p[2]);
		status_only(op, LL_ST_SUCCESS);
		break;
	case OP_LE_READ_MAX_DATA_LEN:
		if (plen != 0) {
			status_only(op, LL_ST_INVALID_PARAM);
			break;
		}
		ll_put_le16(&ret[1], LL_DLE_SUPP_OCTETS);
		ll_put_le16(&ret[3], LL_DLE_SUPP_TIME);
		ll_put_le16(&ret[5], LL_DLE_SUPP_OCTETS);
		ll_put_le16(&ret[7], LL_DLE_SUPP_TIME);
		cmd_complete(op, ret, 9);
		break;
	case OP_LE_READ_PHY:
		read_phy(op, p, plen);
		break;
	case OP_LE_SET_DEFAULT_PHY:
		if (plen != 3) {
			status_only(op, LL_ST_INVALID_PARAM);
			break;
		}
		ret[0] = phys_check(p[0], p[1], p[2]);
		if (ret[0] == LL_ST_SUCCESS) {
			memcpy(def_phy, p, 3);
		}
		status_only(op, ret[0]);
		break;
	case OP_LE_SET_PHY:
		set_phy(op, p, plen);
		break;
	case OP_LE_SET_ADV_ENABLE:
		if (plen != 1 || p[0] > 1) {
			status_only(op, LL_ST_INVALID_PARAM);
		} else {
			status_only(op, hci_ops->adv_enable(p[0] == 1));
		}
		break;
	default:
		if (hci_ops->unknown) {
			hci_ops->unknown(op);
		}
		status_only(op, LL_ST_UNKNOWN_CMD);
		break;
	}
}

/* ---- events ---- */

static void send_evt(uint8_t code, const uint8_t *params, uint8_t plen)
{
	uint8_t evt[3 + 32];

	evt[0] = 0x04;
	evt[1] = code;
	evt[2] = plen;
	memcpy(&evt[3], params, plen);
	hci_sink(evt, (uint16_t)(3 + plen));
}

/* params[0] = subevent code; false when masked */
static bool send_le_evt(const uint8_t *params, uint8_t plen)
{
	if (!(event_mask & MASK_LE_META) || !(le_event_mask & (1ULL << (params[0] - 1)))) {
		return false;
	}
	send_evt(EVT_LE_META, params, plen);
	return true;
}

void ll_hci_evt_conn_complete(uint16_t handle, const struct ll_connect_ind *ci)
{
	uint8_t p[19];

	p[0] = SUBEVT_CONN_COMPLETE;
	p[1] = LL_ST_SUCCESS;
	ll_put_le16(&p[2], handle);
	p[4] = HCI_ROLE_PERIPHERAL;
	p[5] = ci->init_addr_random ? 0x01 : 0x00;
	memcpy(&p[6], ci->init_a, 6);
	ll_put_le16(&p[12], ci->interval);
	ll_put_le16(&p[14], ci->latency);
	ll_put_le16(&p[16], ci->timeout);
	p[18] = ci->sca & 0x07;      /* HCI Central_Clock_Accuracy = LL SCA encoding */
	send_le_evt(p, sizeof(p));
}

void ll_hci_evt_disconn_complete(uint16_t handle, uint8_t reason)
{
	uint8_t p[4];

	if (!(event_mask & MASK_DISCONN_COMPLETE)) {
		return;
	}
	p[0] = LL_ST_SUCCESS;
	ll_put_le16(&p[1], handle);
	p[3] = reason;
	send_evt(EVT_DISCONN_COMPLETE, p, sizeof(p));
}

void ll_hci_evt_num_completed(uint16_t handle, uint16_t count)
{
	uint8_t p[5];

	if (count == 0) {
		return;
	}
	p[0] = 1;                    /* Num_Handles */
	ll_put_le16(&p[1], handle);
	ll_put_le16(&p[3], count);
	send_evt(EVT_NUM_COMPLETED, p, sizeof(p));
}

void ll_hci_evt_ltk_req(uint16_t handle, const uint8_t rand[8], uint16_t ediv)
{
	uint8_t p[13];

	p[0] = SUBEVT_LTK_REQ;
	ll_put_le16(&p[1], handle);
	memcpy(&p[3], rand, 8);
	ll_put_le16(&p[11], ediv);
	send_le_evt(p, sizeof(p));
}

bool ll_hci_evt_conn_param_req(uint16_t handle, uint16_t interval_min, uint16_t interval_max,
			       uint16_t latency, uint16_t timeout)
{
	uint8_t p[11];

	p[0] = SUBEVT_CONN_PARAM_REQ;
	ll_put_le16(&p[1], handle);
	ll_put_le16(&p[3], interval_min);
	ll_put_le16(&p[5], interval_max);
	ll_put_le16(&p[7], latency);
	ll_put_le16(&p[9], timeout);
	return send_le_evt(p, sizeof(p));   /* LE event mask bit 5 */
}

void ll_hci_evt_enc_change(uint16_t handle, uint8_t status, bool enabled)
{
	uint8_t p[4];

	if (!(event_mask & MASK_ENC_CHANGE)) {
		return;
	}
	p[0] = status;
	ll_put_le16(&p[1], handle);
	p[3] = enabled ? 0x01 : 0x00; /* 0x01: on, AES-CCM for LE */
	send_evt(EVT_ENC_CHANGE, p, sizeof(p));
}

void ll_hci_evt_apto_expired(uint16_t handle)
{
	uint8_t p[2];

	if (!(event_mask2 & MASK2_AUTH_PAYLOAD_TO)) {
		return;
	}
	ll_put_le16(p, handle);
	send_evt(EVT_AUTH_PAYLOAD_TO_EXP, p, sizeof(p));
}

void ll_hci_evt_conn_update(uint16_t handle, const struct ll_conn_params *prm)
{
	uint8_t p[10];

	p[0] = SUBEVT_CONN_UPDATE;
	p[1] = LL_ST_SUCCESS;
	ll_put_le16(&p[2], handle);
	ll_put_le16(&p[4], prm->interval);
	ll_put_le16(&p[6], prm->latency);
	ll_put_le16(&p[8], prm->timeout);
	send_le_evt(p, sizeof(p));
}

void ll_hci_evt_chan_sel_algo(uint16_t handle, uint8_t algo)
{
	uint8_t p[4];

	p[0] = SUBEVT_CHAN_SEL_ALGO;
	ll_put_le16(&p[1], handle);
	p[3] = algo;
	send_le_evt(p, sizeof(p));   /* LE event mask bit 19 */
}

void ll_hci_evt_data_len_change(uint16_t handle, uint16_t max_tx_octets, uint16_t max_tx_time,
				uint16_t max_rx_octets, uint16_t max_rx_time)
{
	uint8_t p[11];

	p[0] = SUBEVT_DATA_LEN_CHANGE;
	ll_put_le16(&p[1], handle);
	ll_put_le16(&p[3], max_tx_octets);
	ll_put_le16(&p[5], max_tx_time);
	ll_put_le16(&p[7], max_rx_octets);
	ll_put_le16(&p[9], max_rx_time);
	send_le_evt(p, sizeof(p));   /* LE event mask bit 6 */
}

void ll_hci_evt_phy_update(uint16_t handle, uint8_t status, uint8_t tx_phy, uint8_t rx_phy)
{
	uint8_t p[6];

	p[0] = SUBEVT_PHY_UPDATE;
	p[1] = status;
	ll_put_le16(&p[2], handle);
	p[4] = tx_phy;
	p[5] = rx_phy;
	send_le_evt(p, sizeof(p));   /* LE event mask bit 11 */
}

/* ---- ACL framing ---- */

int ll_hci_acl_from_host(const uint8_t *acl, uint16_t len, struct ll_hci_acl_pdu *out)
{
	uint16_t hf, dlen;
	uint8_t pb, bc;

	if (len < 4) {
		return -EINVAL;
	}
	hf = ll_get_le16(acl);
	dlen = ll_get_le16(&acl[2]);
	out->handle = hf & 0x0FFF;
	pb = (hf >> 12) & 0x3;
	bc = (hf >> 14) & 0x3;
	if (dlen != len - 4 || dlen == 0 || dlen > LL_ACL_MTU || bc != 0 ||
	    (pb != ACL_PB_FIRST_NONFLUSH && pb != ACL_PB_CONT && pb != ACL_PB_FIRST_FLUSH)) {
		return -EINVAL;
	}
	if (!hci_ops->handle_valid(out->handle)) {
		return -ENOTCONN;
	}
	out->llid = pb == ACL_PB_CONT ? LL_LLID_CONT : LL_LLID_START;
	out->len = (uint8_t)dlen;
	memcpy(out->data, &acl[4], dlen);
	return 0;
}

uint16_t ll_hci_acl_to_host(uint8_t *out, uint16_t handle, uint8_t llid, const uint8_t *payload,
			    uint8_t len)
{
	uint8_t pb;

	if (len == 0 || len > LL_DATA_PDU_MAX) {
		return 0;
	}
	if (llid == LL_LLID_START) {
		pb = ACL_PB_FIRST_FLUSH;
	} else if (llid == LL_LLID_CONT) {
		pb = ACL_PB_CONT;
	} else {
		return 0;
	}
	out[0] = 0x02;               /* H4: ACL data */
	ll_put_le16(&out[1], (uint16_t)((handle & 0x0FFF) | (pb << 12)));
	ll_put_le16(&out[3], len);
	memcpy(&out[5], payload, len);
	return (uint16_t)(5 + len);
}

uint8_t ll_hci_acl_fragment(const struct ll_hci_acl_pdu *in, uint8_t frag_max,
			    struct ll_acl_frag *out, uint8_t n_out)
{
	uint8_t n = 0;
	uint16_t off = 0;

	if (frag_max == 0 || in->len == 0 || in->len > LL_ACL_MTU ||
	    (in->llid != LL_LLID_START && in->llid != LL_LLID_CONT) ||
	    (in->len + frag_max - 1) / frag_max > n_out) {
		return 0;
	}
	while (off < in->len) {
		uint16_t left = (uint16_t)(in->len - off);

		out[n].off = (uint8_t)off;
		out[n].len = (uint8_t)(left < frag_max ? left : frag_max);
		out[n].llid = n == 0 ? in->llid : LL_LLID_CONT;
		off = (uint16_t)(off + out[n].len);
		n++;
	}
	return n;
}

