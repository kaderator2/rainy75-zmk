/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * HCI command handling for the open B91 link layer. Pure logic: all
 * controller actions go through struct ll_hci_ops, all events leave through
 * the sink as H4 packets (first byte 0x04).
 *
 * Connection handle == link id (0..LL_MAX_CONN-1, slice 6a); commands act
 * on the handle they carry, events and ACL carry the link's handle, and a
 * handle that is not an active link (ops.handle_valid) is refused with
 * Unknown Connection Identifier (0x02) / -ENOTCONN. Event masks (Set Event Mask, LE
 * Set Event Mask) are honoured for the events below; Number Of Completed
 * Packets cannot be masked. Defaults after init/Reset are the Core Spec
 * ones (Vol 4 Part E 7.3.1, 7.8.1): LE Meta events stay off until the host
 * sets bit 61, as Zephyr does during init.
 */
#ifndef LL_HCI_H_
#define LL_HCI_H_

#include <stdbool.h>
#include <stdint.h>
#include "ll_adv.h"
#include "ll_conn.h"
#include "ll_defs.h"

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
	/* Connection commands; each returns the HCI status of the command.
	 * The handle (handle_valid) and the Disconnect reason are validated
	 * before the call. The glue maps them to the link (handle == link). */
	uint8_t (*disconnect)(uint16_t handle, uint8_t reason);       /* ll_llcp_terminate */
	uint8_t (*ltk_reply)(uint16_t handle, const uint8_t ltk[16]); /* ll_llcp_ltk_reply */
	uint8_t (*ltk_neg_reply)(uint16_t handle);                    /* ll_llcp_ltk_neg_reply */
	/* true while handle is an active link (ll_conn_active(handle)); used
	 * by the commands above and by ll_hci_acl_from_host(). Required. */
	bool (*handle_valid)(uint16_t handle);
};

typedef void (*ll_hci_sink_t)(const uint8_t *h4, uint16_t len);

void ll_hci_init(const struct ll_hci_ops *ops, ll_hci_sink_t sink);

/* cmd = HCI command packet without the H4 type byte:
 * opcode (LE16), parameter length, parameters */
void ll_hci_cmd(const uint8_t *cmd, uint16_t len);

/* ---- Events toward the host. Each builds one H4 event and passes it to
 * the sink unless the event mask suppresses it. Callable from any context
 * (also ISR: ll_conn and ll_txq callbacks), so the sink must be ISR-safe. */
/* LE Connection Complete (0x3E/0x01): status 0, handle, role peripheral,
 * peer address (type) from the CONNECT_IND, interval, latency, supervision
 * timeout, Central_Clock_Accuracy = ci->sca. */
void ll_hci_evt_conn_complete(uint16_t handle, const struct ll_connect_ind *ci);
/* Disconnection Complete (0x05): status 0, handle, reason. */
void ll_hci_evt_disconn_complete(uint16_t handle, uint8_t reason);
/* Number Of Completed Packets (0x13), one handle per event; count 0:
 * nothing is sent. One per acked ACL PDU (ll_txq completion of kind
 * LL_TXQ_ACL), or batched by the caller. Never after the Disconnection
 * Complete of the connection (the host frees its buffers on disconnect
 * itself). */
void ll_hci_evt_num_completed(uint16_t handle, uint16_t count);
/* LE Long Term Key Request (0x3E/0x05): rand as on air (LSB first), EDIV. */
void ll_hci_evt_ltk_req(uint16_t handle, const uint8_t rand[8], uint16_t ediv);
/* Encryption Change (0x08): status, handle, Encryption_Enabled 0/1. */
void ll_hci_evt_enc_change(uint16_t handle, uint8_t status, bool enabled);
/* LE Connection Update Complete (0x3E/0x03): status 0, new parameters. */
void ll_hci_evt_conn_update(uint16_t handle, const struct ll_conn_params *p);
/* LE Channel Selection Algorithm (0x3E/0x14, Vol 4 Part E 7.7.65.20), if LE
 * event mask bit 19 is set: handle, algo 0x00 = CSA#1, 0x01 = CSA#2. Sent by
 * the glue right after LE Connection Complete. */
void ll_hci_evt_chan_sel_algo(uint16_t handle, uint8_t algo);

/* ---- ACL data framing (no H4 type byte on input, H4 type 0x02 on
 * output). LE ACL is never fragmented here: LE Read Buffer Size reports
 * LL_ACL_MTU = LL_DATA_PDU_MAX, so one host ACL packet is one data PDU. */
struct ll_hci_acl_pdu {
	uint16_t handle;                   /* connection handle (== link id) */
	uint8_t llid;                      /* LL_LLID_START or LL_LLID_CONT */
	uint8_t len;                       /* 1..LL_DATA_PDU_MAX */
	uint8_t data[LL_DATA_PDU_MAX];
};
/* Parse one host ACL packet (handle + PB/BC flags LE16, length LE16,
 * data) into a self-contained PDU the glue can hold until ll_llcp_tx()
 * accepts it (-EAGAIN while encryption start pauses data, -ENOMEM while
 * the TX backlog is full). PB 0x00/0x02 (first) -> LLID 2, 0x01
 * (continuation) -> LLID 1. Returns 0, -EINVAL (PB 0x03, broadcast flags,
 * length 0 or > LL_DATA_PDU_MAX, length field not matching len) or
 * -ENOTCONN (ops.handle_valid false). out->handle is set whenever the
 * header could be read (len >= 4), also on an error, so the caller can
 * give the buffer credit back to that handle. Needs ll_hci_init(). */
int ll_hci_acl_from_host(const uint8_t *acl, uint16_t len, struct ll_hci_acl_pdu *out);

/* H4 type + ACL header + one data PDU payload */
#define LL_HCI_ACL_MAX (1 + 4 + LL_DATA_PDU_MAX)
/* Build the H4 ACL packet (type 0x02, handle, PB 0x02 for LLID 2 / 0x01
 * for LLID 1, BC 0) for one received data PDU into out (LL_HCI_ACL_MAX
 * bytes). Returns the packet length, or 0 when the PDU does not go to the
 * host (LLID 3 / 0, empty, len > LL_DATA_PDU_MAX). Pure. */
uint16_t ll_hci_acl_to_host(uint8_t *out, uint16_t handle, uint8_t llid, const uint8_t *payload,
			    uint8_t len);

#endif /* LL_HCI_H_ */
