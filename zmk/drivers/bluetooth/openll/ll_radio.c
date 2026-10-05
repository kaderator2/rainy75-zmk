/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * B91 radio for the open link layer, built on hal_telink rf.c (Apache-2.0).
 * Advertising uses the BLE TX/RX state machine: stx2rx sends a packet and
 * turns the radio around into RX in hardware; stx sends a single packet at a
 * tick. Connections use BRX (RX, then TX after the hardware turnaround,
 * chained while MD), as measured in the BRX and MD spikes
 * (.superpowers/sdd/brx-spike-report.md, md-spike-report.md).
 *
 * DMA buffer layout (TX): [0..3] DMA length word, [4] PDU header, [5] PDU
 * length, [6..] payload. RX adds a trailer after the payload: CRC (3),
 * timestamp (4), freq offset (2), RSSI (1), status (see ext_rf.h).
 *
 * Connection mode, register recipe from the spikes:
 * - Once (ll_radio_conn_init, called at every connection start, a no-op
 *   until the next ll_radio_adv_restore): save the advertising values of
 *   ll_ctrl_1 and rxtcrcpkt, base entry = an empty PDU, reset_sn_nesn(); the
 *   RX DMA ring (4 x 272 bytes) is set up at boot.
 * - Per event (ll_radio_conn_select, ml-spike-report S2/S3, register writes
 *   only): access address (byte-swapped into 0x80140808) and CRC init (as
 *   parsed) of the link that owns the event, ll_ctrl_1 = connection value
 *   (first-RX timeout enable, SN/NESN init bits as ll_txq set them), TX
 *   timestamps on (T_IFS monitor), TX DMA source = ring base (stx2rx moves
 *   it), RX maxlen, IRQ mask. An advertising event in between needs
 *   ll_radio_adv_enter() (empty TX FIFO first, S3).
 * - TX FIFO (pipe 0): the base is sent while rptr == wptr, else entry
 *   rptr & 3 at base + 272 * (1 + (rptr & 3)); wptr is written by software,
 *   rptr advanced by hardware on the central's ack (never resettable).
 * - Long PDUs (slice 6b, dle-spike-report S1/S2): TX DMA entries and RX DMA
 *   entries of 272 bytes (TX: 4 DMA length word + 2 header + 251 + 4 MIC =
 *   261, in the 16-byte unit of the TX size register; RX: the DMA writes up
 *   to p[3 + 4 * ceil((len + 13) / 4)], p[271] for len 255, and a central
 *   retransmission can leave a full packet's tail in an entry it never
 *   delivers, so every entry holds a full 255-byte packet). RX maxlen 255
 *   (251 + MIC) in connections. The geometry is set at boot only.
 * - Per event: channel, AA, CRC, TX settle 86 us, first-RX timeout, command
 *   schedule tick = RX open - 80 us RX settle, ll_cmd = BRX (0x82). The SN
 *   init bit is programmed by ll_txq before every BRX.
 * - The event ends with CMD_DONE, FIRST_TIMEOUT (nothing received) or
 *   RX_TIMEOUT; a guard alarm on the stimer ends it if none of them comes.
 *
 * SCAN_RSP (slice 7 Task 2 item 1, ll_scanrsp.h): the RX ISR answers a
 * SCAN_REQ for us itself, first thing in the ISR, with a scheduled STX
 * (TX settle CONFIG_BT_HCI_B91_OPENLL_SCANRSP_SETTLE_US, default 63, trigger
 * 150 - settle - 59 us after the request; first bit on air 150 us after
 * the request); the link layer callback only learns the decision
 * (ll_radio_tx_rsp_at). Through the callback chain the decision came 60..90
 * us after the request, too late for that (the former trigger, 72 us after
 * the request with settle 78, put the response on air at 209 us).
 */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/arch/riscv/csr.h>
#include <zephyr/devicetree.h>
#include "types.h"            /* u8/u16 used by ext_rf.h */
#include "rf.h"
#include "dma.h"
#include "sys.h"
#include "stimer.h"
#include "ext_driver/ext_rf.h"
#include "ll_defs.h"
#include "ll_fifo.h"
#include "ll_radio.h"
#include "ll_radio_mode.h"
#include "ll_sched.h"
#include "ll_scanrsp.h"

LOG_MODULE_DECLARE(openll, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);

#define RF_IRQ                 (IRQ_TO_L2(15) | 11)
#define DMA_BUF_SIZE           64   /* adv TX buffers; multiple of 16; 4 + 2 + 37 + 16 trailer = 59 */
/* Slack behind the RX ring: every entry holds a full 255-byte packet
 * (RX_ENTRY_SIZE), so even a packet with the largest length field (noise,
 * maxlen enforcement unverified) stays inside its entry; the DLE spike's
 * canary behind entry 3 was never touched. Kept as a small margin. */
#define RX_BUF_SIZE            32
#define ADV_RX_MAXLEN          37
/* Settle time from ext_rf.h (LL_TX_STL_ADV_1M); the SCAN_RSP settle and
 * trigger lead are in ll_scanrsp.h. */
#define TX_SETTLE_ADV_US       84
#define POWER_INDEX_0DBM       11   /* same index the blob shim uses, about 0 dBm */

/* Connection mode */
#define RING_N                 LL_RADIO_RX_RING_N   /* TX ring entries (tx_chn_dep 2) and RX DMA entries */
BUILD_ASSERT(RING_N == 4, "the boot-time DMA geometry (tx_chn_dep 2) has 4 entries");
#define RX_ENTRY_SIZE          272  /* DMA writes up to p[271] for a 255-byte packet (spike S2) */
#define TX_ENTRY_SIZE          272  /* 4 + 2 + 251 + 4 MIC = 261, 16-byte unit (spike S1) */
/* The hardware rx wptr (0x1004f4) is a 5-bit counter (Task 9: maximum seen
 * 31, then 0); RING_N divides 32, so entry = wptr & (RING_N - 1) holds
 * across its wrap. */
#define RX_WPTR_MASK           0x1f
#define CONN_RX_MAXLEN         (LL_DATA_PDU_MAX + LL_MIC_LEN)   /* 255 */
/* One RX DMA ring of RING_N entries of RX_ENTRY_SIZE for both modes (an
 * advertising PDU of up to 37 bytes needs 4 + 2 + 37 + 16 = 59). The
 * RX_BUF_SIZE behind the ring is slack (see above).
 * The RX and TX DMA are configured once, in ll_radio_init(), and never
 * again: Task 9 found that calling rf_set_rx_dma() while the radio is in
 * use (connection setup from the CONNECT_IND RX ISR, and again in the
 * return to advertising) freezes the SoC within microseconds, before any
 * fault handler runs (watchdog reset, MCUboot revert). */
#define RX_AREA_SIZE           (RING_N * RX_ENTRY_SIZE + RX_BUF_SIZE)
/* md-spike Q3: 86 us gives an on-air T_IFS of 148/149 us (blob: same). */
#define TX_SETTLE_CONN_US      86
/* RX settle of rf_set_ble_1M_mode() (0x80140a0c = 0x50); the first-RX
 * timeout counts from the command trigger including it (brx spike). */
#define RX_SETTLE_US           80
/* Guard: an event that has produced no end IRQ by open + max_event_us
 * (from ll_conn: interval minus the next event's alarm lead and a safety
 * margin, at least the first RX window plus one exchange) is ended by the
 * guard alarm. A chained (MD) event takes about 0.7 ms per exchange of
 * 27-octet PDUs, 4.5 ms of 251-octet ones (ll_conn_exchange_us), so a long
 * central burst may legitimately reach the cap; only a
 * guard-ended event without any CRC-valid packet counts as a wedge sign
 * (guard_streak). */
/* TX timestamp register (with FLD_RF_EN_TS_TX): md-spike round 2, on-air
 * T_IFS = (tx_ts - rx_ts) / 16 us + 13.5 us - 8 us per central payload
 * byte (the RX timestamp marks the end of the access address). */
#define REG_TX_TIMESTAMP       0x80140850
#define TIFS_CORR_TICKS        (27 * LL_TICKS_PER_US / 2)   /* 13.5 us */
/* A response follows the central packet after T_IFS (150 us, the hardware
 * turnaround); a larger estimate belongs to a later TX of the event. */
#define TIFS_MAX_US            200
#define CONN_IRQ_MASK          (FLD_RF_IRQ_RX | FLD_RF_IRQ_TX | FLD_RF_IRQ_RX_TIMEOUT | \
				FLD_RF_IRQ_FIRST_TIMEOUT | FLD_RF_IRQ_CMD_DONE | \
				FLD_RF_IRQ_FSM_TIMEOUT)
#define CONN_END_IRQS          (FLD_RF_IRQ_CMD_DONE | FLD_RF_IRQ_FIRST_TIMEOUT | \
				FLD_RF_IRQ_RX_TIMEOUT | FLD_RF_IRQ_FSM_TIMEOUT)
#define ADV_IRQ_MASK           (FLD_RF_IRQ_RX | FLD_RF_IRQ_TX | FLD_RF_IRQ_RX_TIMEOUT | \
				FLD_RF_IRQ_FIRST_TIMEOUT)

/* The RX timestamp is taken to mark the end of the access address; the
 * packet then has header (2) + payload + CRC (3) bytes at 8 us/byte.
 * Measured (indirectly, by sniffer): consistent with the SCAN_RSP timing,
 * whose T_IFS of about 209 us is explained by a fixed TX path delay. */
#define RX_TS_TO_END_TICKS(plen) (((plen) + 2 + 3) * 8 * LL_TICKS_PER_US)

static uint8_t tx_buf[DMA_BUF_SIZE] __aligned(4);
static uint8_t rsp_buf[DMA_BUF_SIZE] __aligned(4);
static uint8_t rx_buf[RX_AREA_SIZE] __aligned(4);
/* Connection TX: base entry (empty PDU) + RING_N ring entries */
static uint8_t conn_tx_buf[(1 + RING_N) * TX_ENTRY_SIZE] __aligned(4);
BUILD_ASSERT(TX_ENTRY_SIZE % 16 == 0 && TX_ENTRY_SIZE >= 4 + 2 + CONN_RX_MAXLEN,
	     "TX entry: 16-byte unit, a full encrypted PDU");
BUILD_ASSERT(RX_ENTRY_SIZE % 16 == 0 && RX_ENTRY_SIZE >= 4 + 4 * ((CONN_RX_MAXLEN + 13 + 3) / 4),
	     "RX entry: the DMA write extent of a full packet (dle-spike-report S2)");
static ll_radio_cb_t radio_cb;
static volatile bool rsp_in_flight;
/* Prepared SCAN_RSP PDU length in rsp_buf (0: none, nothing is answered) */
static uint8_t rsp_len;
/* The RX ISR's SCAN_RSP decision for the packet being reported, read once
 * by ll_radio_tx_rsp_at() from the callback */
static bool rsp_started;
/* An advertising TX/RX (tx_then_rx, or a SCAN_RSP) whose end IRQ is
 * pending; the adv guard recovers if it never comes. */
static volatile bool adv_open;
/* ll_radio_flash_abort() runs the pending ISR work: a SCAN_REQ found then
 * is not answered (the radio is about to be stopped) */
static bool rsp_blocked;
/* Adv guard: an stx2rx ends within about 0.8 ms (TX + 300 us RX window),
 * a SCAN_RSP within 0.5 ms of its trigger. ml-spike S3 saw one stall in
 * 3791 adv events between connection events (FSM idle, TX seen, no end
 * IRQ); without recovery advertising would stay "in event" forever, and the
 * arbiter would keep its request running and starve every link. */
#define ADV_GUARD_US           2000
#define ADV_RSP_GUARD_US       1500
static atomic_t cnt_adv_guard;

static enum { MODE_ADV, MODE_CONN } mode;

static struct {
	uint32_t aa_reg;        /* byte-swapped AA for 0x80140808 */
	uint32_t crc_init;
	uint8_t rx_sw;          /* RX DMA ring: next entry to read (follows the hw wptr), both modes */
	bool evt_open;          /* BRX issued, LL_RADIO_CONN_DONE not yet reported */
	uint8_t n_valid;        /* CRC-valid packets in this event */
	/* RX DMA entries (valid or not) in this event, plus the NODATA
	 * pseudo-entry: an RX IRQ that finds no new entry before any entry of
	 * the event counts as one (the acked retransmission of the anchor
	 * packet, conn_rx). Should an RX IRQ ever run before the DMA has
	 * advanced the wptr for a new packet, that packet is taken for a
	 * retransmission, reported as NODATA and then delivered as a later
	 * (non-first) packet: benign, ll_conn only skips one re-anchor and
	 * the T_IFS estimate skips that event (n_any != 1). */
	uint8_t n_any;
	bool nodata;            /* NODATA noted by the RX IRQ, reported by the next drain */
	bool first_valid;       /* the event's first packet had a valid CRC */
	uint32_t first_ts;
	uint8_t first_len;
	bool tx_seen;           /* first TX IRQ of the event handled */
	bool stop_req;          /* ll_radio_conn_stop(): end the event after this ISR's callbacks */
} cn;
/* Adv snapshot of ll_ctrl_1 / rxtcrcpkt and "connection setup done"
 * (ll_radio_mode.h, host-tested). */
static struct ll_radio_mode rm;

static atomic_t cnt_tx2rx, cnt_rx_ok, cnt_rx_crc, cnt_rx_timeout, cnt_rsp_tx, cnt_rsp_late;
static atomic_t cnt_conn_events, cnt_conn_rx, cnt_conn_tx, cnt_conn_fto, cnt_conn_guard;
static atomic_t cnt_rx_ptr_odd, cnt_tifs_le150, cnt_tifs_151_152, cnt_tifs_gt152, cnt_restores;
static atomic_t cnt_rx_ptr_skip, cnt_fst_capped, cnt_holds, cnt_conn_stopped, cnt_flash_aborts;
static uint8_t rx_wptr_max;      /* largest raw hardware rx wptr seen */
static uint8_t guard_streak;     /* consecutive guard-ended events without a valid packet */
static uint16_t restore_ptrs_before, restore_ptrs_after;

static void load(uint8_t *dma, const uint8_t *pdu, uint8_t len)
{
	uint32_t dlen = rf_tx_packet_dma_len(len);

	dma[0] = dlen & 0xFF;
	dma[1] = (dlen >> 8) & 0xFF;
	dma[2] = (dlen >> 16) & 0xFF;
	dma[3] = (dlen >> 24) & 0xFF;
	memcpy(&dma[4], pdu, len);
}

static uint16_t tx_ptrs(void)
{
	return (uint16_t)(rf_get_tx_rptr(0) << 8) | rf_get_tx_wptr(0);
}

/* CPU hold: busy wait on the CPU cycle counter (a CSR, no bus access). */
#define CPU_HZ        DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)
#define CYC_PER_TICK  (CPU_HZ / (LL_TICKS_PER_US * 1000000))
BUILD_ASSERT(CYC_PER_TICK >= 1 && CPU_HZ % (LL_TICKS_PER_US * 1000000) == 0,
	     "CPU clock: a whole number of cycles per stimer tick");

/* SCAN_RSP hold: until this long after the response's first bit is due on
 * air. hold_until() never waits longer than HOLD_UNTIL_MAX_US (a bogus
 * tick holds nothing). */
#define RSP_HOLD_PAST_US       10
#define HOLD_UNTIL_MAX_US      200

_attribute_ram_code_sec_noinline_ static void hold_until(uint32_t tick)
{
	int32_t left = (int32_t)(tick - stimer_get_tick());
	uint32_t c0, n;

	if (left <= 0 || left > HOLD_UNTIL_MAX_US * LL_TICKS_PER_US) {
		return;
	}
	c0 = csr_read(mcycle);
	n = (uint32_t)left * CYC_PER_TICK;
	while (csr_read(mcycle) - c0 < n) {
	}
}

static void adv_guard(void);

/* ---- advertising mode ISR ---- */

/* Inline into the RAM ISR: no flash fetch before the SCAN_RSP trigger. */
static inline __attribute__((always_inline)) uint8_t *rx_entry(uint8_t idx)
{
	return &rx_buf[(idx & (RING_N - 1)) * RX_ENTRY_SIZE];
}

/* SCAN_REQ -> SCAN_RSP, first thing in the RF ISR (file header,
 * ll_scanrsp.h): a CRC-valid SCAN_REQ for the AdvA of the prepared SCAN_RSP
 * gets the STX at once, when the trigger can still be met; anything else,
 * and a late decision, gets nothing. A SCAN_REQ for us that arrives while
 * no adv RX window is open or a response is in flight (can_answer false)
 * is only counted in rsp_late. The CPU then holds on the cycle counter
 * until the response is on air: CPU work (flash fetches of the callback
 * chain) while the TX starts delays it by a few us, as in the connection
 * turnaround (slice 6b Task 4 review). Returns true if the response was
 * started; the adv guard covers it like any adv TX. */
_attribute_ram_code_sec_noinline_ static bool adv_rsp_isr(bool can_answer)
{
	uint8_t hw = rf_get_rx_wptr() & RX_WPTR_MASK;
	uint8_t *p = rx_entry((uint8_t)(hw - 1));
	uint8_t plen = p[DMA_RFRX_OFFSET_RFLEN];
	uint32_t trigger, ts;

	if (rsp_len == 0 || hw == cn.rx_sw || !RF_BLE_PACKET_VALIDITY_CHECK(p) ||
	    !ll_scanrsp_for_us(&rsp_buf[4], rsp_len, &p[DMA_RFRX_OFFSET_HEADER],
			       (uint8_t)(plen + 2))) {
		return false;
	}
	if (!can_answer) {
		/* a SCAN_REQ for us outside an open adv RX window (or with a
		 * response already in flight): not answered, counted */
		atomic_inc(&cnt_rsp_late);
		return false;
	}
	ts = ll_get_le32(&p[DMA_RFRX_OFFSET_TIME_STAMP(p)]);
	if (!ll_scanrsp_trigger(ts + RX_TS_TO_END_TICKS(plen), stimer_get_tick(), &trigger)) {
		atomic_inc(&cnt_rsp_late);
		return false;
	}
	rsp_in_flight = true;
	adv_open = true;
	rf_tx_settle_us(LL_SCANRSP_SETTLE_US);
	rf_start_stx(rsp_buf, trigger);
	ll_sched_guard_at(trigger + ADV_RSP_GUARD_US * LL_TICKS_PER_US, adv_guard);
	atomic_inc(&cnt_rsp_tx);
	hold_until(trigger + (LL_SCANRSP_SETTLE_US + LL_SCANRSP_TX_PATH_US + RSP_HOLD_PAST_US) *
			     LL_TICKS_PER_US);
	return true;
}

_attribute_ram_code_sec_noinline_ static void adv_isr(uint16_t st)
{
	bool rsp = false;

	if (st & FLD_RF_IRQ_RX) {
		rsp = adv_rsp_isr(adv_open && !rsp_in_flight && !rsp_blocked);
	}
	rsp_started = rsp;
	if (!rsp && adv_open &&
	    ((st & (FLD_RF_IRQ_RX | FLD_RF_IRQ_RX_TIMEOUT | FLD_RF_IRQ_FIRST_TIMEOUT)) ||
	     ((st & FLD_RF_IRQ_TX) && rsp_in_flight))) {
		/* ended (a SCAN_RSP started above keeps it open with its own
		 * guard): drop the adv guard, so no stimer IRQ per adv
		 * channel. In adv mode the guard slot can only hold the adv
		 * guard (a connection event arms its own guard when it is
		 * issued). */
		adv_open = false;
		if (!cn.evt_open) {
			ll_sched_guard_cancel();
		}
	}
	if (st & FLD_RF_IRQ_RX) {
		/* The DMA wrote entry (wptr & 3) and advanced the wptr; the
		 * advertising RX window ends with this packet, so the newest
		 * entry is the one to read. */
		uint8_t hw = rf_get_rx_wptr() & RX_WPTR_MASK;
		uint8_t *p = rx_entry((uint8_t)(hw - 1));

		if (hw == cn.rx_sw) {
			atomic_inc(&cnt_rx_ptr_odd);   /* RX IRQ without a new entry */
		}
		cn.rx_sw = hw;

		if (!RF_BLE_PACKET_VALIDITY_CHECK(p)) {
			atomic_inc(&cnt_rx_crc);
			radio_cb(LL_RADIO_RX_CRC_ERR, NULL, 0, 0);
			return;
		}
		uint32_t ts = ll_get_le32(&p[DMA_RFRX_OFFSET_TIME_STAMP(p)]);

		atomic_inc(&cnt_rx_ok);
		radio_cb(LL_RADIO_RX_OK, &p[DMA_RFRX_OFFSET_HEADER], p[DMA_RFRX_OFFSET_RFLEN] + 2,
			 ts + RX_TS_TO_END_TICKS(p[DMA_RFRX_OFFSET_RFLEN]));
	} else if (st & (FLD_RF_IRQ_RX_TIMEOUT | FLD_RF_IRQ_FIRST_TIMEOUT)) {
		atomic_inc(&cnt_rx_timeout);
		radio_cb(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	} else if ((st & FLD_RF_IRQ_TX) && rsp_in_flight) {
		rsp_in_flight = false;
		radio_cb(LL_RADIO_TX_DONE, NULL, 0, 0);
	}
	/* TX IRQ of a tx2rx command: the FSM continues into RX by itself. */
}

/* ---- connection mode ISR ---- */

/* CPU activity during the hardware RX -> TX turnaround moves our TX later
 * (slice 6b Task 4 review). First-exchange T_IFS above 152 us on the
 * device (TX timestamp monitor), central 251 B / 2120 us, fread + large
 * SMP echoes, 7.5 ms interval:
 * - RX processing in the RX IRQ (ll_conn, the copy of up to 257 bytes into
 *   ll_rxq, the RX wake), as up to 0e2ac5d: 4..6 %;
 * - the same processing after our TX had started (busy wait first): 0.4 %;
 *   reading and clearing the IRQ status and reading the wptr in the
 *   turnaround were harmless;
 * - processing moved to the TX IRQ: 2.6..3.9 %, the rest from threads that
 *   run on through the turnaround (decryption, copies, the host stack):
 *   with the RX IRQ interrupting the idle thread every first exchange was
 *   at <= 150 us, with a thread interrupted 4..9 % were above 152 us;
 * - both (below): 0.2..0.3 % (0.5 % with 251 B / 415 us both ways).
 * So the RX IRQ of the event's first packet holds the CPU until our
 * response has started (hold_turnaround) and only then delivers it, and
 * the RX IRQ of a later packet does no processing: those RX DMA ring
 * entries are delivered from the TX IRQ of our response (or the end IRQs),
 * long before the DMA wraps around to the entry again (4 packets later,
 * >= 4 * 230 us). Moving the DMA buffers from DLM to ILM
 * did not help (3.1 % with the processing at the TX IRQ).
 *
 * The DMA writes entry (rx wptr & 3) and advances the hardware wptr, so all
 * entries between our read index and the hardware wptr are new (normally
 * exactly one; more if the ISR was late). The hardware writes only new
 * packets: a retransmission of the central (its SN is not our NESN init) is
 * acked but not written, so its RX IRQ finds no new entry (counted in
 * rx_ptr_odd; Task 10 device + sniffer). When that is the event's first
 * packet it was the anchor packet, and it is reported (at the next
 * drain) as LL_RADIO_CONN_RX_NODATA so a chained packet does not
 * re-anchor. The rx rptr is not written: advertising (one entry, mask 0)
 * never touched it either and kept receiving (Task 9: no FIFO-full rule). */

/* The CPU hold: a busy wait on the CPU cycle counter (a CSR, no bus
 * access) until HOLD_TO_US after the end of the central's packet, i.e.
 * until our response (T_IFS 150 us) has started, at most HOLD_MAX_US. Only
 * for the event's first packet (the chained exchanges are not held); it
 * costs up to about 160 us of CPU per event with a received packet (about
 * 2 % at a 7.5 ms interval under load, nothing in skipped events). */
#define HOLD_TO_US    170
#define HOLD_MAX_US   200

static bool hold_turnaround(const uint8_t *p)
{
	uint8_t plen = p[DMA_RFRX_OFFSET_RFLEN];
	uint32_t ts = ll_get_le32(&p[DMA_RFRX_OFFSET_TIME_STAMP(p)]);
	uint32_t until = ts + RX_TS_TO_END_TICKS(plen) + HOLD_TO_US * LL_TICKS_PER_US;
	int32_t left = (int32_t)(until - ll_radio_now());

	if (left > HOLD_MAX_US * LL_TICKS_PER_US) {
		return false;   /* a bogus timestamp */
	}
	if (left <= 0) {
		return true;    /* our TX has started already (late IRQ) */
	}
	atomic_inc(&cnt_holds);
	hold_until(until);
	return true;
}

static void conn_rx_drain(void);

/* RX IRQ: the anchor check above (one register read) and, for the event's
 * first packet (CRC-valid, the only new entry), the CPU hold, after which
 * the packet is delivered at once (our TX has started; waiting for the TX
 * IRQ would delay it by up to a 2120 us response). Otherwise no
 * processing. */
static void conn_rx_irq(void)
{
	uint8_t hw = rf_get_rx_wptr() & RX_WPTR_MASK;

	if (hw != cn.rx_sw) {
		if (cn.n_any == 0 && hw == ((cn.rx_sw + 1) & RX_WPTR_MASK)) {
			uint8_t *p = rx_entry(cn.rx_sw);

			if (RF_BLE_PACKET_VALIDITY_CHECK(p) && hold_turnaround(p)) {
				conn_rx_drain();
			}
		}
		return;   /* entries waiting for the drain */
	}
	atomic_inc(&cnt_rx_ptr_odd);   /* RX IRQ without a new entry */
	if (cn.n_any == 0) {
		cn.n_any++;   /* the anchor packet, reported by the next drain */
		cn.nodata = true;
	}
}

/* TX IRQ, end IRQs and the guard: deliver the anchor retransmission noted
 * by conn_rx_irq(), then every new RX DMA ring entry, in packet order. Each
 * packet is handed to the callback (ll_conn copies it into ll_rxq) before
 * the ISR returns. */
static void conn_rx_drain(void)
{
	uint8_t raw = rf_get_rx_wptr();
	uint8_t hw = raw & RX_WPTR_MASK;
	uint8_t n = (uint8_t)(hw - cn.rx_sw) & RX_WPTR_MASK;

	if (raw > rx_wptr_max) {
		rx_wptr_max = raw;   /* counter width check (31 = 5 bits) */
	}
	if (cn.nodata) {
		cn.nodata = false;
		radio_cb(LL_RADIO_CONN_RX_NODATA, NULL, 0, ll_radio_now());
	}
	if (n == 0) {
		return;
	}
	if (n > RING_N) {
		/* An overrun: the entries cannot be told apart from stale ones
		 * (old timestamps, old NESN), so deliver none of them. */
		atomic_inc(&cnt_rx_ptr_odd);
		atomic_inc(&cnt_rx_ptr_skip);
		cn.rx_sw = hw;
		return;
	}
	while (n--) {
		uint8_t *p = rx_entry(cn.rx_sw);
		bool first = cn.n_any == 0;

		cn.rx_sw = (cn.rx_sw + 1) & RX_WPTR_MASK;
		cn.n_any++;
		if (!RF_BLE_PACKET_VALIDITY_CHECK(p)) {
			/* every length field (<= 255) keeps the timestamp
			 * inside the 272-byte entry */
			uint32_t bts = ll_get_le32(&p[DMA_RFRX_OFFSET_TIME_STAMP(p)]);

			atomic_inc(&cnt_rx_crc);
			radio_cb(LL_RADIO_CONN_RX_CRC_ERR, NULL, 0, bts);
			continue;
		}
		uint8_t plen = p[DMA_RFRX_OFFSET_RFLEN];
		uint32_t ts = ll_get_le32(&p[DMA_RFRX_OFFSET_TIME_STAMP(p)]);

		if (first) {
			cn.first_valid = true;
			cn.first_ts = ts;
			cn.first_len = plen;
		}
		cn.n_valid++;
		atomic_inc(&cnt_conn_rx);
		/* 16-bit: plen + 2 is 257 for a 251-octet encrypted PDU (an
		 * 8-bit length wrapped and the acked PDU was lost, DLE spike) */
		radio_cb(LL_RADIO_CONN_RX, &p[DMA_RFRX_OFFSET_HEADER], (uint16_t)(plen + 2), ts);
	}
}

/* TX IRQ: the first TX of the event answered the event's first packet; its
 * timestamp gives the first-exchange T_IFS (md-spike: the later, chained
 * exchanges are tight, the tail belongs to the first one). */
static void conn_tx(void)
{
	atomic_inc(&cnt_conn_tx);
	if (cn.tx_seen) {
		return;
	}
	cn.tx_seen = true;
	if (!cn.first_valid || cn.n_any != 1) {
		return;   /* first packet CRC-bad, or the ISR was late (register overwritten) */
	}
	int32_t tifs = (int32_t)(REG_ADDR32(REG_TX_TIMESTAMP) - cn.first_ts) -
		       (int32_t)cn.first_len * 8 * LL_TICKS_PER_US + TIFS_CORR_TICKS;
	int32_t us = (tifs + LL_TICKS_PER_US / 2) / LL_TICKS_PER_US;

	if (us > TIFS_MAX_US) {
		/* not the response to the first packet: the TX IRQ of a long
		 * response handled after the next TX latched its timestamp
		 * (about 2.5 ms, seen once the drain moved to the TX IRQ) */
		return;
	}
	if (us <= 150) {
		atomic_inc(&cnt_tifs_le150);
	} else if (us <= 152) {
		atomic_inc(&cnt_tifs_151_152);
	} else {
		atomic_inc(&cnt_tifs_gt152);
	}
}

/* End of the event: exactly one LL_RADIO_CONN_DONE per BRX. */
static void conn_done(void)
{
	if (!cn.evt_open) {
		return;
	}
	cn.evt_open = false;
	rf_set_tx_rx_off_auto_mode();
	radio_cb(LL_RADIO_CONN_DONE, NULL, cn.n_valid, ll_radio_now());
}

/* Guard alarm (stimer ISR, same priority as the RF ISR, so never nested
 * with it): the BRX produced no end IRQ. Stop the FSM and end the event as
 * the hardware would have, with the packets seen so far. An event that
 * received CRC-valid packets was a healthy MD burst cut at the cap: it
 * resets the wedge streak instead of counting toward it. */
static void conn_guard(void)
{
	if (!cn.evt_open) {
		return;
	}
	atomic_inc(&cnt_conn_guard);
	rf_set_tx_rx_off_auto_mode();
	rf_clr_irq_status(FLD_RF_IRQ_ALL);
	conn_rx_drain();
	if (cn.n_valid != 0) {
		guard_streak = 0;
	} else if (guard_streak < UINT8_MAX) {
		guard_streak++;
	}
	conn_done();
}

void ll_radio_conn_stop(void)
{
	if (cn.evt_open) {
		cn.stop_req = true;
	}
}

/* ll_radio_conn_stop() (RX flow control, slice 7 Task 2c): turn the FSM off
 * as the guard does, then deliver what the RX ring already holds (at most
 * RING_N packets, which ll_conn kept room for) and end the event. The
 * central's packets after the stop get no ack and are resent later; our
 * unacked TX entries stay in the ring (rptr is advanced only by acks), as
 * after a guard-cut MD burst. Packets were received, so no wedge sign. */
static void conn_stop_now(void)
{
	ll_sched_guard_cancel();
	rf_set_tx_rx_off_auto_mode();
	rf_clr_irq_status(FLD_RF_IRQ_ALL);
	conn_rx_drain();
	guard_streak = 0;
	atomic_inc(&cnt_conn_stopped);
	conn_done();
}

static void conn_isr(uint16_t st)
{
	if (!cn.evt_open) {
		return;   /* stale IRQ after the event was ended (guard or earlier IRQ) */
	}
	if (st & FLD_RF_IRQ_RX) {
		conn_rx_irq();
	}
	if (st & FLD_RF_IRQ_TX) {
		/* our response is on air: now the packet(s) it answered */
		conn_rx_drain();
		conn_tx();
	}
	if (st & CONN_END_IRQS) {
		if ((st & FLD_RF_IRQ_FIRST_TIMEOUT) && cn.n_any == 0) {
			atomic_inc(&cnt_conn_fto);
		}
		ll_sched_guard_cancel();
		guard_streak = 0;
		conn_rx_drain();
		conn_done();
	}
	if (cn.evt_open && cn.stop_req) {
		conn_stop_now();
	}
}

_attribute_ram_code_sec_noinline_ static void rf_isr(const void *arg)
{
	uint16_t st = reg_rf_irq_status;

	ARG_UNUSED(arg);
	rf_clr_irq_status(FLD_RF_IRQ_ALL);

	if (mode == MODE_CONN) {
		conn_isr(st);
	} else {
		adv_isr(st);
	}
}

void ll_radio_flash_abort(void)
{
	/* Advertising: first what the RF ISR would do with the status already
	 * pending (a CONNECT_IND received just before is not lost). A
	 * connection event needs only the NODATA note below: the stop drains
	 * every packet the hardware has acked. conn_isr() is not called from here,
	 * so it stays inlined into the RAM rf_isr with its helpers (a second
	 * call site moved them to flash, slower in the T_IFS-critical path). */
	if (mode == MODE_ADV) {
		uint16_t st = reg_rf_irq_status;

		if (st != 0) {
			rf_clr_irq_status(FLD_RF_IRQ_ALL);
			rsp_blocked = true;
			adv_isr(st);
			rsp_blocked = false;
		}
	}
	if (mode == MODE_CONN && cn.evt_open) {
		/* A pending RX IRQ without a new ring entry before any entry of
		 * the event is the central's retransmission of the anchor packet
		 * (conn_rx_irq): note it as conn_rx_irq would, so the drain
		 * reports NODATA first and a chained packet does not re-anchor.
		 * (conn_rx_irq's other work, the turnaround hold, does not apply
		 * to an event that is being stopped.) */
		if ((reg_rf_irq_status & FLD_RF_IRQ_RX) && cn.n_any == 0 &&
		    (rf_get_rx_wptr() & RX_WPTR_MASK) == cn.rx_sw) {
			atomic_inc(&cnt_rx_ptr_odd);
			cn.n_any++;
			cn.nodata = true;
		}
		/* as conn_stop_now(), but the guard streak stays as it is (an
		 * armed event that never started is no wedge sign either) */
		atomic_inc(&cnt_flash_aborts);
		ll_sched_guard_cancel();
		rf_set_tx_rx_off_auto_mode();
		rf_clr_irq_status(FLD_RF_IRQ_ALL);
		conn_rx_drain();
		conn_done();
	} else if (mode == MODE_ADV && adv_open) {
		atomic_inc(&cnt_flash_aborts);
		rf_set_tx_rx_off_auto_mode();
		rf_clr_irq_status(FLD_RF_IRQ_ALL);
		adv_open = false;
		rsp_in_flight = false;
		if (!cn.evt_open) {
			ll_sched_guard_cancel();
		}
		radio_cb(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
	}
}

/* Baseband setup for advertising (also the state after a restore). The DMA
 * geometry is set only at boot (dma true), see RX_AREA_SIZE. */
static void hw_init_adv(bool dma)
{
	rf_mode_init();
	rf_set_ble_1M_mode();
	rf_set_power_level_index((rf_power_level_index_e)POWER_INDEX_0DBM);
	if (dma) {
		rf_set_tx_dma(2, TX_ENTRY_SIZE);
		rf_set_rx_dma(rx_buf, RING_N - 1, RX_ENTRY_SIZE);
	}
	rf_set_rx_maxlen(ADV_RX_MAXLEN);
	rf_set_ble_access_code_adv();
	rf_set_ble_crc_adv();
	rf_clr_irq_status(FLD_RF_IRQ_ALL);
	reg_rf_irq_mask = ADV_IRQ_MASK;
}

int ll_radio_init(ll_radio_cb_t cb)
{
	radio_cb = cb;
	mode = MODE_ADV;
	ll_radio_mode_reset(&rm);
	hw_init_adv(true);
	cn.rx_sw = rf_get_rx_wptr() & RX_WPTR_MASK;
	IRQ_CONNECT(RF_IRQ, 1, rf_isr, NULL, 0);
	irq_enable(RF_IRQ);
	return 0;
}

uint32_t ll_radio_now(void)
{
	return stimer_get_tick();
}

void ll_radio_set_adv_channel(uint8_t ch)
{
	rf_set_tx_rx_off_auto_mode();
	rf_set_ble_chn(ch);          /* also sets the whitening seed */
	rf_set_ble_access_code_adv();
	rf_set_ble_crc_adv();
}

static void baseband_restore(bool keep_conn);

/* Adv guard (stimer ISR): the advertising TX/RX produced no end IRQ.
 * Recover as ml-spike S3 did: baseband reset + adv init. Live links keep
 * their connection setup (keep_conn: no new snapshot, ll_radio_conn_init
 * stays a no-op; each connection event re-selects its registers), and the
 * adv snapshot is written back. Then the channel ends for ll_adv like an
 * RX timeout. */
static void adv_guard(void)
{
	if (!adv_open || mode != MODE_ADV) {
		return;
	}
	adv_open = false;
	atomic_inc(&cnt_adv_guard);
	baseband_restore(true);
	radio_cb(LL_RADIO_RX_TIMEOUT, NULL, 0, 0);
}

void ll_radio_tx_then_rx(const uint8_t *pdu, uint8_t len, uint32_t start_tick,
			 uint32_t rx_window_us)
{
	load(tx_buf, pdu, len);
	rsp_in_flight = false;
	rf_tx_settle_us(TX_SETTLE_ADV_US);
	rf_ble_set_rx_timeout(rx_window_us);
	atomic_inc(&cnt_tx2rx);
	adv_open = true;
	rf_start_stx2rx(tx_buf, start_tick);
	ll_sched_guard_at(start_tick + ADV_GUARD_US * LL_TICKS_PER_US, adv_guard);
}

void ll_radio_prepare_rsp(const uint8_t *pdu, uint8_t len)
{
	unsigned int key = irq_lock();

	/* the RX ISR matches SCAN_REQs against rsp_buf: no half-written PDU */
	if (pdu == NULL || len < 2 || len > DMA_BUF_SIZE - 4) {
		rsp_len = 0;
	} else {
		load(rsp_buf, pdu, len);
		rsp_len = len;
	}
	irq_unlock(key);
}

bool ll_radio_tx_rsp_at(uint32_t tick)
{
	/* The RX ISR decided before this callback (adv_rsp_isr), aiming at
	 * the same tick (T_IFS after the request's end). */
	bool started = rsp_started;

	ARG_UNUSED(tick);
	rsp_started = false;
	return started;
}

void ll_radio_get_stats(struct ll_radio_stats *s)
{
	s->tx2rx = (uint32_t)atomic_get(&cnt_tx2rx);
	s->rx_ok = (uint32_t)atomic_get(&cnt_rx_ok);
	s->rx_crc = (uint32_t)atomic_get(&cnt_rx_crc);
	s->rx_timeout = (uint32_t)atomic_get(&cnt_rx_timeout);
	s->rsp_tx = (uint32_t)atomic_get(&cnt_rsp_tx);
	s->rsp_late = (uint32_t)atomic_get(&cnt_rsp_late);
	s->conn_events = (uint32_t)atomic_get(&cnt_conn_events);
	s->conn_rx = (uint32_t)atomic_get(&cnt_conn_rx);
	s->conn_tx = (uint32_t)atomic_get(&cnt_conn_tx);
	s->conn_fto = (uint32_t)atomic_get(&cnt_conn_fto);
	s->conn_guard = (uint32_t)atomic_get(&cnt_conn_guard);
	s->rx_ptr_odd = (uint32_t)atomic_get(&cnt_rx_ptr_odd);
	s->tifs_le150 = (uint32_t)atomic_get(&cnt_tifs_le150);
	s->tifs_151_152 = (uint32_t)atomic_get(&cnt_tifs_151_152);
	s->tifs_gt152 = (uint32_t)atomic_get(&cnt_tifs_gt152);
	s->restores = (uint32_t)atomic_get(&cnt_restores);
	s->adv_guard = (uint32_t)atomic_get(&cnt_adv_guard);
	s->rx_ptr_skip = (uint32_t)atomic_get(&cnt_rx_ptr_skip);
	s->fst_capped = (uint32_t)atomic_get(&cnt_fst_capped);
	s->holds = (uint32_t)atomic_get(&cnt_holds);
	s->conn_stopped = (uint32_t)atomic_get(&cnt_conn_stopped);
	s->flash_aborts = (uint32_t)atomic_get(&cnt_flash_aborts);
	s->rx_wptr_max = rx_wptr_max;
	s->restore_ptrs_before = restore_ptrs_before;
	s->restore_ptrs_after = restore_ptrs_after;
}

void ll_radio_stop(void)
{
	rsp_in_flight = false;
	adv_open = false;
	rf_set_tx_rx_off_auto_mode();
}

void ll_radio_quiesce(void)
{
	ll_radio_stop();
	rf_clr_irq_mask(FLD_RF_IRQ_ALL);
	rf_clr_irq_status(FLD_RF_IRQ_ALL);
	irq_disable(RF_IRQ);
}

/* ---- connection mode ---- */

static uint8_t *ring_entry(uint8_t idx)
{
	return &conn_tx_buf[(1u + (idx & (RING_N - 1))) * TX_ENTRY_SIZE];
}

/* The connection values of the registers that an advertising event (or
 * ll_radio_adv_restore) changes. The SN/NESN init bits of ll_ctrl_1 are
 * kept: ll_txq programs them per event for the link (before or after this). */
static void conn_regs(void)
{
	reg_rf_rxtcrcpkt = rm.adv_rxtcrc | FLD_RF_EN_TS_TX;   /* T_IFS monitor */
	reg_rf_ll_ctrl_1 = (rm.adv_ctrl1 & ~(FLD_RF_BRX_SN_INIT | FLD_RF_BRX_NESN_INIT)) |
			   (reg_rf_ll_ctrl_1 & (FLD_RF_BRX_SN_INIT | FLD_RF_BRX_NESN_INIT)) |
			   FLD_RF_RX_FIRST_TIMEOUT_EN;
	/* TX: base = empty PDU, sent while the FIFO is empty (md-spike round
	 * 5); an advertising stx2rx points DMA0 at its own buffer. */
	dma_set_src_address(DMA0, convert_ram_addr_cpu2bus(conn_tx_buf));
	/* RX: the 4-entry DMA ring set up at boot (not reconfigured here) */
	rf_set_rx_maxlen(CONN_RX_MAXLEN);
	reg_rf_irq_mask = CONN_IRQ_MASK;
	mode = MODE_CONN;
}

void ll_radio_conn_init(void)
{
	static const uint8_t empty_pdu[2] = {LL_LLID_CONT, 0};
	unsigned int key = irq_lock();

	/* Done since the last ll_radio_adv_restore() (an adv-guard restore
	 * keeps it): other links may be live, possibly with an event on air,
	 * so nothing here may touch the FSM, the SN/NESN state, the guard
	 * streak or the RX ring position. Per event, ll_radio_conn_select()
	 * and ll_txq_event_start() set everything a link needs. With one link
	 * every connection follows an adv restore, so the full setup below
	 * runs at every connection start, as in slice 5. The adv snapshot is
	 * taken at the first setup only (ll_radio_mode.h). */
	if (!ll_radio_mode_conn_init(&rm, reg_rf_ll_ctrl_1, reg_rf_rxtcrcpkt)) {
		irq_unlock(key);
		return;
	}
	rf_set_tx_rx_off_auto_mode();
	rsp_in_flight = false;
	cn.evt_open = false;
	/* BRX SN/NESN start values 0, then load them into the live SN/NESN
	 * state. Per event the SN/NESN init bits come from ll_txq. */
	reg_rf_ll_ctrl_1 = (rm.adv_ctrl1 & ~(FLD_RF_BRX_SN_INIT | FLD_RF_BRX_NESN_INIT)) |
			   FLD_RF_RX_FIRST_TIMEOUT_EN;
	reset_sn_nesn();
	load(conn_tx_buf, empty_pdu, sizeof(empty_pdu));
	conn_regs();
	cn.rx_sw = rf_get_rx_wptr() & RX_WPTR_MASK;
	rf_clr_irq_status(FLD_RF_IRQ_ALL);
	guard_streak = 0;
	irq_unlock(key);
}

void ll_radio_conn_select(uint32_t aa, uint32_t crc_init)
{
	rf_set_tx_rx_off_auto_mode();
	adv_open = false;
	/* brx spike round 1: AA byte-swapped, CRC init as parsed; written to
	 * the hardware by ll_radio_conn_event() */
	cn.aa_reg = __builtin_bswap32(aa);
	cn.crc_init = crc_init;
	conn_regs();
}

void ll_radio_adv_enter(void)
{
	unsigned int key = irq_lock();

	rf_set_tx_rx_off_auto_mode();
	cn.evt_open = false;
	rsp_in_flight = false;
	adv_open = false;
	/* S3 round 1: an stx2rx with a non-empty TX FIFO wedges the FSM in
	 * 0x03 and the following BRX commands never end. The next connection
	 * event rebuilds its link's ring. */
	rf_set_tx_wptr(0, rf_get_tx_rptr(0));
	{
		uint8_t c1, rx;

		/* advertising value: no first-RX timeout (it would bound the
		 * stx2rx RX by a connection's window), SN/NESN init 0; the TX
		 * timestamp bit of rxtcrcpkt may stay on. Without a snapshot no
		 * connection ever set up, the register holds the adv value. */
		if (ll_radio_mode_adv_regs(&rm, &c1, &rx)) {
			reg_rf_ll_ctrl_1 = c1;
		}
	}
	rf_set_ble_access_code_adv();
	rf_set_ble_crc_adv();
	rf_set_rx_maxlen(ADV_RX_MAXLEN);
	rf_clr_irq_status(FLD_RF_IRQ_ALL);
	reg_rf_irq_mask = ADV_IRQ_MASK;
	cn.rx_sw = rf_get_rx_wptr() & RX_WPTR_MASK;
	mode = MODE_ADV;
	irq_unlock(key);
}

void ll_radio_conn_event(uint8_t ch, uint32_t open_tick, uint32_t first_timeout_us,
			 uint32_t max_event_us)
{
	uint32_t fst = first_timeout_us + RX_SETTLE_US;
	uint32_t trigger = open_tick - RX_SETTLE_US * LL_TICKS_PER_US;

	rf_set_tx_rx_off_auto_mode();
	rf_set_ble_chn((signed char)ch);
	rf_set_ble_access_code_value(cn.aa_reg);
	rf_set_ble_crc_value(cn.crc_init);
	rf_tx_settle_us(TX_SETTLE_CONN_US);
	/* rf_start_brx() would write 0x0fffffff to the first timeout: the
	 * register sequence is done here with a bounded window instead.
	 * reg_rf_rx_timeout is 12 bits (max 4095 us). As in both spikes it gets
	 * the same window, capped; the first RX of the event is bounded by the
	 * 32-bit first timeout, so the cap only matters if the hardware also
	 * applies rx_timeout to that first RX (transmit-window events and long
	 * widening exceed 4095 us). Counted in fst_capped for Task 9. */
	if (fst > 0xfff) {
		atomic_inc(&cnt_fst_capped);
	}
	rf_ble_set_rx_timeout(fst > 0xfff ? 0xfff : (u16)fst);
	reg_rf_ll_rx_fst_timeout = fst;
	rf_clr_irq_status(FLD_RF_IRQ_ALL);
	/* Skip RX entries written after the previous event was closed (e.g. a
	 * packet completing after a guard stop): they belong to no event. */
	cn.rx_sw = rf_get_rx_wptr() & RX_WPTR_MASK;
	cn.evt_open = true;
	cn.n_valid = 0;
	cn.n_any = 0;
	cn.nodata = false;
	cn.first_valid = false;
	cn.tx_seen = false;
	cn.stop_req = false;
	atomic_inc(&cnt_conn_events);
	reg_rf_ll_cmd_schedule = trigger;
	reg_rf_ll_ctrl3 |= FLD_RF_R_CMD_SCHDULE_EN;
	reg_rf_ll_cmd = FSM_BRX;
	ll_sched_guard_at(open_tick + max_event_us * LL_TICKS_PER_US, conn_guard);
}

uint8_t ll_radio_conn_guard_streak(void)
{
	return guard_streak;
}

void ll_radio_conn_set_sn_init(uint8_t sn)
{
	reg_rf_ll_ctrl_1 = (reg_rf_ll_ctrl_1 & ~FLD_RF_BRX_SN_INIT) | (sn ? FLD_RF_BRX_SN_INIT : 0);
}

void ll_radio_conn_set_nesn_init(uint8_t nesn)
{
	reg_rf_ll_ctrl_1 = (reg_rf_ll_ctrl_1 & ~FLD_RF_BRX_NESN_INIT) |
			   (nesn ? FLD_RF_BRX_NESN_INIT : 0);
}

uint8_t ll_radio_fifo_rptr(void)
{
	return rf_get_tx_rptr(0) & LL_RADIO_FIFO_PTR_MASK;
}

uint8_t ll_radio_fifo_wptr(void)
{
	return rf_get_tx_wptr(0) & LL_RADIO_FIFO_PTR_MASK;
}

void ll_radio_fifo_write(uint8_t idx, uint8_t hdr0, const uint8_t *payload, uint8_t len)
{
	uint8_t *e = ring_entry(idx);
	uint32_t dlen;

	/* len <= 255 = CONN_RX_MAXLEN by its type: always fits the entry */
	dlen = rf_tx_packet_dma_len((uint32_t)len + 2u);
	e[0] = dlen & 0xFF;
	e[1] = (dlen >> 8) & 0xFF;
	e[2] = (dlen >> 16) & 0xFF;
	e[3] = (dlen >> 24) & 0xFF;
	e[4] = hdr0;    /* NESN/SN/MD are set by hardware */
	e[5] = len;
	if (len) {
		/* ll_txq keeps its records 2 bytes past a word boundary, like
		 * e[6]: word copies (the per-event ring rebuild, spike: up to
		 * 4 x 255 bytes in the stimer ISR) */
		ll_fifo_copy(&e[6], payload, len);
	}
}

void ll_radio_fifo_set_wptr(uint8_t wptr)
{
	rf_set_tx_wptr(0, wptr & LL_RADIO_FIFO_PTR_MASK);
}

/* Return to advertising after a connection (spec "Return to advertising",
 * first choice): baseband reset, then the advertising init without touching
 * the DMA geometry. Task 9: the reset does not bring the TX rptr back to 0
 * (the "adv restore" log shows it unchanged), yet advertising and the next
 * connection work; the advertising hang of the MD spike did not reproduce
 * once the DMA is no longer reconfigured at runtime. Thread context, called
 * by ll_adv_enable() under ll_plat_lock(). */
void ll_radio_adv_restore(void)
{
	baseband_restore(false);
}

/* keep_conn: stall recovery while links may be live (the connection setup
 * stays done); false: the return to advertising after the last link (the
 * next connection sets up again). */
static void baseband_restore(bool keep_conn)
{
	unsigned int key = irq_lock();
	uint8_t c1, rx;

	ll_sched_guard_cancel();
	cn.evt_open = false;
	rf_set_tx_rx_off_auto_mode();
	restore_ptrs_before = tx_ptrs();
	rf_baseband_reset();
	if (ll_radio_mode_restore(&rm, keep_conn, &c1, &rx)) {
		reg_rf_ll_ctrl_1 = c1;
		reg_rf_rxtcrcpkt = rx;
	}
	hw_init_adv(false);
	cn.rx_sw = rf_get_rx_wptr() & RX_WPTR_MASK;
	/* Empty FIFO again, whatever survived the reset */
	rf_set_tx_wptr(0, rf_get_tx_rptr(0));
	restore_ptrs_after = tx_ptrs();
	rsp_in_flight = false;
	adv_open = false;
	mode = MODE_ADV;
	atomic_inc(&cnt_restores);
	irq_unlock(key);
}
