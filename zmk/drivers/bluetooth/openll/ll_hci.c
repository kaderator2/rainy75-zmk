/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "ll_hci.h"
#include "ll_defs.h"

#define OP(ogf, ocf) ((uint16_t)(((ogf) << 10) | (ocf)))

#define OP_SET_EVENT_MASK        OP(0x03, 0x0001)
#define OP_RESET                 OP(0x03, 0x0003)
#define OP_READ_LOCAL_VERSION    OP(0x04, 0x0001)
#define OP_READ_LOCAL_CMDS       OP(0x04, 0x0002)
#define OP_READ_LOCAL_FEATURES   OP(0x04, 0x0003)
#define OP_READ_BD_ADDR          OP(0x04, 0x0009)
#define OP_LE_SET_EVENT_MASK     OP(0x08, 0x0001)
#define OP_LE_READ_BUFFER_SIZE   OP(0x08, 0x0002)
#define OP_LE_READ_FEATURES      OP(0x08, 0x0003)
#define OP_LE_SET_ADV_PARAMS     OP(0x08, 0x0006)
#define OP_LE_SET_ADV_DATA       OP(0x08, 0x0008)
#define OP_LE_SET_SCAN_RSP_DATA  OP(0x08, 0x0009)
#define OP_LE_SET_ADV_ENABLE     OP(0x08, 0x000A)
#define OP_LE_RAND               OP(0x08, 0x0018)

/* Supported Commands bitmap entries: {octet, bit} */
static const uint8_t supported_cmds[][2] = {
	{5, 6},  /* Set Event Mask */
	{5, 7},  /* Reset */
	{14, 3}, /* Read Local Version Information */
	{14, 5}, /* Read Local Supported Features */
	{15, 1}, /* Read BD_ADDR */
	{25, 0}, /* LE Set Event Mask */
	{25, 1}, /* LE Read Buffer Size */
	{25, 2}, /* LE Read Local Supported Features */
	{25, 5}, /* LE Set Advertising Parameters */
	{25, 7}, /* LE Set Advertising Data */
	{26, 0}, /* LE Set Scan Response Data */
	{26, 1}, /* LE Set Advertising Enable */
	{27, 7}, /* LE Rand */
};

static const struct ll_hci_ops *hci_ops;
static ll_hci_sink_t hci_sink;

void ll_hci_init(const struct ll_hci_ops *ops, ll_hci_sink_t sink)
{
	hci_ops = ops;
	hci_sink = sink;
}

static void cmd_complete(uint16_t op, const uint8_t *ret, uint8_t ret_len)
{
	uint8_t evt[LL_HCI_EVT_MAX];

	evt[0] = 0x04;               /* H4: event */
	evt[1] = 0x0E;               /* Command Complete */
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
	if (plen != len - 3) {
		status_only(op, LL_ST_INVALID_PARAM);
		return;
	}

	ret[0] = LL_ST_SUCCESS;
	switch (op) {
	case OP_RESET:
		hci_ops->reset();
		status_only(op, LL_ST_SUCCESS);
		break;
	case OP_SET_EVENT_MASK:
	case OP_LE_SET_EVENT_MASK:
		status_only(op, plen == 8 ? LL_ST_SUCCESS : LL_ST_INVALID_PARAM);
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
		memset(&ret[1], 0, 8); /* slice 1: no LE features (encryption comes in slice 4) */
		cmd_complete(op, ret, 9);
		break;
	case OP_LE_RAND:
		hci_ops->rand(&ret[1], 8);
		cmd_complete(op, ret, 9);
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
