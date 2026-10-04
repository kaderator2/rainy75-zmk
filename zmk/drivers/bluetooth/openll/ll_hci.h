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
 * Unknown Connection Identifier (0x02) / -ENOTCONN. Event masks (Set Event
 * Mask, LE Set Event Mask) are honoured for the events below; Number Of
 * Completed Packets cannot be masked. Defaults after init/Reset are the
 * Core Spec ones (Vol 4 Part E 7.3.1, 7.8.1): LE Meta events stay off until
 * the host sets bit 61, as Zephyr does during init.
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
	/* Slice 6b Task 3, handle validated before the call, parameters
	 * range-checked (Vol 4 Part E 7.8.33 / 7.8.49). */
	/* ll_llcp_set_data_len */
	uint8_t (*set_data_len)(uint16_t handle, uint16_t tx_octets, uint16_t tx_time);
	/* always 1M / 1M */
	uint8_t (*read_phy)(uint16_t handle, uint8_t *tx_phy, uint8_t *rx_phy);
	/* LE Set PHY: on LL_ST_SUCCESS ll_hci sends the Command Status and right
	 * after it LE PHY Update Complete (status 0, read_phy values), so the
	 * order is fixed whatever thread delivers events. */
	uint8_t (*set_phy)(uint16_t handle, uint8_t all_phys, uint8_t tx_phys, uint8_t rx_phys,
			   uint16_t opts);
	/* Slice 6c: LE Set Random Address (0x2005), the 6-octet address
	 * (LSB first) after the length check; returns the HCI status
	 * (ll_adv_set_random_addr). Required. */
	uint8_t (*set_random_addr)(const uint8_t addr[6]);
	/* Slice 6d: HCI Read / Write Authenticated Payload Timeout (0x0C7B /
	 * 0x0C7C, Vol 4 Part E 7.3.93 / 7.3.94), handle validated before the
	 * call, the Write value (10 ms units) checked for 0x0001..0xFFFF;
	 * ll_llcp_read_apto / ll_llcp_write_apto (the latter checks it against
	 * connInterval x (1 + connPeripheralLatency)). Required. */
	uint8_t (*read_apto)(uint16_t handle, uint16_t *apto);
	uint8_t (*write_apto)(uint16_t handle, uint16_t apto);
	/* Slice 6d Task 2: LE Remote Connection Parameter Request Reply /
	 * Negative Reply (0x2020 / 0x2021, Vol 4 Part E 7.8.31 / 7.8.32),
	 * handle validated before the call; the Reply's values checked against
	 * the 7.8.31 ranges, Interval_Min <= Interval_Max, the timeout rule
	 * (Timeout x 10 ms > (1 + Max_Latency) x Interval_Max x 1.25 ms x 2) and
	 * Min_CE_Length <= Max_CE_Length; the Negative Reply's reason nonzero.
	 * ll_llcp_conn_param_reply / _neg_reply. Required. */
	uint8_t (*conn_param_reply)(uint16_t handle, uint16_t interval_min, uint16_t interval_max,
				    uint16_t latency, uint16_t timeout);
	uint8_t (*conn_param_neg_reply)(uint16_t handle, uint8_t reason);
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

/* LE Data Length Change (0x3E/0x07, Vol 4 Part E 7.7.65.7), LE event mask
 * bit 6: the link's effective values. */
void ll_hci_evt_data_len_change(uint16_t handle, uint16_t max_tx_octets, uint16_t max_tx_time,
				uint16_t max_rx_octets, uint16_t max_rx_time);
/* LE PHY Update Complete (0x3E/0x0C, 7.7.65.12), LE event mask bit 11. */
void ll_hci_evt_phy_update(uint16_t handle, uint8_t status, uint8_t tx_phy, uint8_t rx_phy);
/* Authenticated Payload Timeout Expired (0x57, Vol 4 Part E 7.7.75):
 * handle. Masked by Set Event Mask Page 2 (0x0C63) bit 23, which is 0 after
 * init and Reset (7.3.69), so it is sent only once the host enabled it. */
void ll_hci_evt_apto_expired(uint16_t handle);
/* LE Remote Connection Parameter Request (0x3E/0x06, 7.7.65.6), LE event
 * mask bit 5: handle, Interval_Min, Interval_Max, Max_Latency, Timeout.
 * Returns true when it was sent, false when masked (the LE Meta bit 61 of
 * the event mask or LE bit 5; LE bit 5 is 0 by default, 7.8.1). */
bool ll_hci_evt_conn_param_req(uint16_t handle, uint16_t interval_min, uint16_t interval_max,
			       uint16_t latency, uint16_t timeout);
/* Host's Suggested Default Data Length (LE Write Suggested Default Data
 * Length, 27 / 328 after init and Reset): connInitialMaxTx{Octets,Time} for
 * new connections; the glue hands them to ll_llcp_set_data_len() on connect. */
void ll_hci_default_data_len(uint16_t *tx_octets, uint16_t *tx_time);

/* ---- ACL data framing (no H4 type byte on input, H4 type 0x02 on
 * output). LE Read Buffer Size reports LL_ACL_MTU (251); one host ACL
 * packet is split into PDUs of the link's TX limit (ll_hci_acl_fragment,
 * slice 6b Task 4), received PDUs go to the host one by one (no
 * recombination: the host reassembles L2CAP). */
struct ll_hci_acl_pdu {
	uint16_t handle;                   /* connection handle (== link id) */
	uint8_t llid;                      /* LL_LLID_START or LL_LLID_CONT */
	uint8_t len;                       /* 1..LL_ACL_MTU */
	uint8_t data[LL_ACL_MTU];
};
/* Parse one host ACL packet (handle + PB/BC flags LE16, length LE16,
 * data) into a self-contained PDU the glue can hold until ll_llcp_tx()
 * accepts it (-EAGAIN while encryption start pauses data or the link owes
 * control PDUs, -ENOMEM while the TX backlog is full). PB 0x00/0x02 (first) -> LLID 2, 0x01
 * (continuation) -> LLID 1. Returns 0, -EINVAL (PB 0x03, broadcast flags,
 * length 0 or > LL_ACL_MTU, length field not matching len) or
 * -ENOTCONN (ops.handle_valid false). out->handle is set whenever the
 * header could be read (len >= 4), also on an error, so the caller can
 * give the buffer credit back to that handle. Needs ll_hci_init(). */
int ll_hci_acl_from_host(const uint8_t *acl, uint16_t len, struct ll_hci_acl_pdu *out);
/* Split one host ACL packet (in->len 1..LL_ACL_MTU) into PDUs of at most
 * frag_max octets (the link's TX limit, ll_dle_tx_limit): the first with
 * in->llid (2, or 1 for a host continuation packet), the others LLID 1,
 * all full but the last. Returns the number of fragments written to out[]
 * (at most n_out), 0 on error (frag_max 0, a bad length or LLID, or more
 * than n_out fragments needed). Pure. */
uint8_t ll_hci_acl_fragment(const struct ll_hci_acl_pdu *in, uint8_t frag_max,
			    struct ll_acl_frag *out, uint8_t n_out);

/* H4 type + ACL header + one data PDU payload */
#define LL_HCI_ACL_MAX (1 + 4 + LL_DATA_PDU_MAX)
/* Build the H4 ACL packet (type 0x02, handle, PB 0x02 for LLID 2 / 0x01
 * for LLID 1, BC 0) for one received data PDU into out (LL_HCI_ACL_MAX
 * bytes). Returns the packet length, or 0 when the PDU does not go to the
 * host (LLID 3 / 0, empty, len > LL_DATA_PDU_MAX). Pure. */
uint16_t ll_hci_acl_to_host(uint8_t *out, uint16_t handle, uint8_t llid, const uint8_t *payload,
			    uint8_t len);

#endif /* LL_HCI_H_ */
