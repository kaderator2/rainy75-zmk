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
 * - Per connection: access address (byte-swapped into 0x80140808), CRC init
 *   as parsed, ll_ctrl_1 BRX SN/NESN init 0 + first-RX timeout enable,
 *   reset_sn_nesn(), TX timestamps on (T_IFS monitor), TX DMA source = ring
 *   base (an empty PDU), RX DMA as a 4-entry ring of 64 bytes.
 * - TX FIFO (pipe 0): the base is sent while rptr == wptr, else entry
 *   rptr & 3 at base + 64 * (1 + (rptr & 3)); wptr is written by software,
 *   rptr advanced by hardware on the central's ack (never resettable).
 * - Per event: channel, AA, CRC, TX settle 86 us, first-RX timeout, command
 *   schedule tick = RX open - 80 us RX settle, ll_cmd = BRX (0x82). The SN
 *   init bit is programmed by ll_txq before every BRX.
 * - The event ends with CMD_DONE, FIRST_TIMEOUT (nothing received) or
 *   RX_TIMEOUT; a guard alarm on the stimer ends it if none of them comes.
 */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include "types.h"            /* u8/u16 used by ext_rf.h */
#include "rf.h"
#include "dma.h"
#include "sys.h"
#include "stimer.h"
#include "ext_driver/ext_rf.h"
#include "ll_defs.h"
#include "ll_radio.h"
#include "ll_sched.h"

LOG_MODULE_DECLARE(openll, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);

#define RF_IRQ                 (IRQ_TO_L2(15) | 11)
#define DMA_BUF_SIZE           64   /* multiple of 16; 4 + 2 + 37 + 16 trailer = 59 */
/* Worst case the DMA could write if the baseband does not enforce
 * rf_set_rx_maxlen (noise packet with a full 255-byte length field; maxlen
 * enforcement is unverified on this hardware): 4 DMA length word + 4 header/
 * len + 2 + 255 payload + 16 trailer, rounded up to a multiple of 16. */
#define RX_BUF_SIZE            288  /* (4 + 4 + 2 + 255 + 16) rounded up to 16 */
#define ADV_RX_MAXLEN          37
/* Settle times from ext_rf.h (LL_TX_STL_ADV_1M, LL_SCANRSP_TX_SETTLE). */
#define TX_SETTLE_ADV_US       84
#define TX_SETTLE_RSP_US       78
/* Refuse a SCAN_RSP whose TX trigger tick is closer than this to now: a
 * trigger in the past would leave the FSM waiting and stall advertising. */
#define RSP_MIN_LEAD_TICKS     (10 * LL_TICKS_PER_US)
#define POWER_INDEX_0DBM       11   /* same index the blob shim uses, about 0 dBm */

/* Connection mode */
#define RING_N                 4    /* TX ring entries (tx_chn_dep 2) and RX DMA entries */
#define RX_ENTRY_SIZE          64   /* 4 + 2 + 31 + 16 trailer = 53 */
#define CONN_RX_MAXLEN         (LL_DATA_PDU_MAX + LL_MIC_LEN)
/* One RX area for both modes: advertising uses its first RX_BUF_SIZE bytes
 * as a single entry, a connection RING_N entries of RX_ENTRY_SIZE. The
 * RX_BUF_SIZE behind the ring is slack for an oversize write into the last
 * entry (maxlen enforcement unverified, see above). */
#define RX_AREA_SIZE           (RING_N * RX_ENTRY_SIZE + RX_BUF_SIZE)
/* md-spike Q3: 86 us gives an on-air T_IFS of 148/149 us (blob: same). */
#define TX_SETTLE_CONN_US      86
/* RX settle of rf_set_ble_1M_mode() (0x80140a0c = 0x50); the first-RX
 * timeout counts from the command trigger including it (brx spike). */
#define RX_SETTLE_US           80
/* Guard: an event that has produced no end IRQ by open + first timeout +
 * this is ended by the guard alarm. A chained (MD) event of 27-byte PDUs
 * takes about 0.7 ms per exchange; 6 ms bounds it below the 7.5 ms minimum
 * interval minus the next event's preparation. */
#define CONN_EVENT_MAX_US      6000
/* TX timestamp register (with FLD_RF_EN_TS_TX): md-spike round 2, on-air
 * T_IFS = (tx_ts - rx_ts) / 16 us + 13.5 us - 8 us per central payload
 * byte (the RX timestamp marks the end of the access address). */
#define REG_TX_TIMESTAMP       0x80140850
#define TIFS_CORR_TICKS        (27 * LL_TICKS_PER_US / 2)   /* 13.5 us */
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
static uint8_t conn_tx_buf[(1 + RING_N) * DMA_BUF_SIZE] __aligned(4);
static ll_radio_cb_t radio_cb;
static volatile bool rsp_in_flight;

static enum { MODE_ADV, MODE_CONN } mode;

static struct {
	uint32_t aa_reg;        /* byte-swapped AA for 0x80140808 */
	uint32_t crc_init;
	uint8_t rx_sw;          /* RX DMA ring: next entry to read (follows the hw wptr) */
	bool evt_open;          /* BRX issued, LL_RADIO_CONN_DONE not yet reported */
	uint8_t n_valid;        /* CRC-valid packets in this event */
	uint8_t n_any;          /* RX DMA entries (valid or not) in this event */
	bool first_valid;       /* the event's first packet had a valid CRC */
	uint32_t first_ts;
	uint8_t first_len;
	bool tx_seen;           /* first TX IRQ of the event handled */
	bool saved;             /* adv-mode register values below are saved */
	uint8_t saved_ctrl1;
	uint8_t saved_rxtcrc;
} cn;

static atomic_t cnt_tx2rx, cnt_rx_ok, cnt_rx_crc, cnt_rx_timeout, cnt_rsp_tx, cnt_rsp_late;
static atomic_t cnt_conn_events, cnt_conn_rx, cnt_conn_tx, cnt_conn_fto, cnt_conn_guard;
static atomic_t cnt_rx_ptr_odd, cnt_tifs_le150, cnt_tifs_151_152, cnt_tifs_gt152, cnt_restores;
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

/* ---- advertising mode ISR ---- */

static void adv_isr(uint16_t st)
{
	if (st & FLD_RF_IRQ_RX) {
		uint8_t *p = rx_buf;

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

/* RX IRQ: deliver every new RX DMA ring entry. The DMA writes entry
 * (rx wptr & 3) and advances the hardware wptr, so all entries between our
 * read index and the hardware wptr are new (normally exactly one; more if
 * the ISR was late). Each packet is handed to the callback (ll_conn copies
 * it into ll_rxq) before the ISR returns, i.e. long before the DMA wraps
 * around to its entry again (4 packets later, >= 4 * 230 us).
 * Unverified on hardware (Task 9): that the hardware rx wptr counts every
 * received packet. The rx rptr is not written: advertising (one entry,
 * mask 0) never touched it either and kept receiving, and writing it has
 * clear semantics for bit 7 (rf_clr_rx_rptr). If RX stops after 4 packets
 * of a connection, a FIFO-full condition on wptr - rptr is the suspect.
 * Anomalies are counted in rx_ptr_odd. Also called (rx_irq false) when the
 * event ends, in case the wptr advanced after an RX IRQ was handled. */
static void conn_rx(bool rx_irq)
{
	uint8_t hw = rf_get_rx_wptr();
	uint8_t n = (uint8_t)(hw - cn.rx_sw);

	if (n == 0) {
		if (rx_irq) {
			atomic_inc(&cnt_rx_ptr_odd);   /* RX IRQ without a new entry */
		}
		return;
	}
	if (n > RING_N) {
		atomic_inc(&cnt_rx_ptr_odd);
		cn.rx_sw = (uint8_t)(hw - RING_N);   /* overrun: older entries are gone */
		n = RING_N;
	}
	while (n--) {
		uint8_t *p = &rx_buf[(cn.rx_sw & (RING_N - 1)) * RX_ENTRY_SIZE];
		bool first = cn.n_any == 0;

		cn.rx_sw++;
		cn.n_any++;
		if (!RF_BLE_PACKET_VALIDITY_CHECK(p)) {
			atomic_inc(&cnt_rx_crc);
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
		radio_cb(LL_RADIO_CONN_RX, &p[DMA_RFRX_OFFSET_HEADER], (uint8_t)(plen + 2), ts);
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
 * the hardware would have, with the packets seen so far. */
static void conn_guard(void)
{
	if (!cn.evt_open) {
		return;
	}
	atomic_inc(&cnt_conn_guard);
	rf_set_tx_rx_off_auto_mode();
	rf_clr_irq_status(FLD_RF_IRQ_ALL);
	conn_rx(false);
	conn_done();
}

static void conn_isr(uint16_t st)
{
	if (!cn.evt_open) {
		return;   /* stale IRQ after the event was ended (guard or earlier IRQ) */
	}
	if (st & FLD_RF_IRQ_RX) {
		conn_rx(true);
	}
	if (st & FLD_RF_IRQ_TX) {
		conn_tx();
	}
	if (st & CONN_END_IRQS) {
		if ((st & FLD_RF_IRQ_FIRST_TIMEOUT) && cn.n_any == 0) {
			atomic_inc(&cnt_conn_fto);
		}
		ll_sched_guard_cancel();
		conn_rx(false);
		conn_done();
	}
}

static void rf_isr(const void *arg)
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

/* Baseband setup for advertising (also the state after a restore). */
static void hw_init_adv(void)
{
	rf_mode_init();
	rf_set_ble_1M_mode();
	rf_set_power_level_index((rf_power_level_index_e)POWER_INDEX_0DBM);
	rf_set_tx_dma(2, DMA_BUF_SIZE);
	rf_set_rx_dma(rx_buf, 0, RX_BUF_SIZE);   /* single RX FIFO entry */
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
	hw_init_adv();
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

void ll_radio_tx_then_rx(const uint8_t *pdu, uint8_t len, uint32_t start_tick,
			 uint32_t rx_window_us)
{
	load(tx_buf, pdu, len);
	rsp_in_flight = false;
	rf_tx_settle_us(TX_SETTLE_ADV_US);
	rf_ble_set_rx_timeout(rx_window_us);
	atomic_inc(&cnt_tx2rx);
	rf_start_stx2rx(tx_buf, start_tick);
}

void ll_radio_prepare_rsp(const uint8_t *pdu, uint8_t len)
{
	load(rsp_buf, pdu, len);
}

/* Measured: the SCAN_RSP reaches the air about 209 us after the SCAN_REQ
 * ends (59 us late), from a fixed delay in the TX path. */
bool ll_radio_tx_rsp_at(uint32_t tick)
{
	uint32_t trigger_tick = tick - TX_SETTLE_RSP_US * LL_TICKS_PER_US;

	if ((int32_t)(trigger_tick - ll_radio_now()) < RSP_MIN_LEAD_TICKS) {
		atomic_inc(&cnt_rsp_late);
		return false;
	}
	rsp_in_flight = true;
	rf_tx_settle_us(TX_SETTLE_RSP_US);
	atomic_inc(&cnt_rsp_tx);
	rf_start_stx(rsp_buf, trigger_tick);
	return true;
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
	s->restore_ptrs_before = restore_ptrs_before;
	s->restore_ptrs_after = restore_ptrs_after;
}

void ll_radio_stop(void)
{
	rsp_in_flight = false;
	rf_set_tx_rx_off_auto_mode();
}

/* ---- connection mode ---- */

static uint8_t *ring_entry(uint8_t idx)
{
	return &conn_tx_buf[(1u + (idx & (RING_N - 1))) * DMA_BUF_SIZE];
}

void ll_radio_conn_setup(uint32_t aa, uint32_t crc_init)
{
	static const uint8_t empty_pdu[2] = {LL_LLID_CONT, 0};
	unsigned int key = irq_lock();

	rf_set_tx_rx_off_auto_mode();
	rsp_in_flight = false;
	cn.evt_open = false;
	if (!cn.saved) {
		cn.saved_ctrl1 = reg_rf_ll_ctrl_1;
		cn.saved_rxtcrc = reg_rf_rxtcrcpkt;
		cn.saved = true;
	}
	/* brx spike round 1: AA byte-swapped, CRC init as parsed */
	cn.aa_reg = __builtin_bswap32(aa);
	cn.crc_init = crc_init;
	reg_rf_rxtcrcpkt = cn.saved_rxtcrc | FLD_RF_EN_TS_TX;   /* T_IFS monitor */
	/* BRX SN/NESN start values 0, first-RX timeout on, then load them into
	 * the live SN/NESN state (once per connection). */
	reg_rf_ll_ctrl_1 = (cn.saved_ctrl1 & ~(FLD_RF_BRX_SN_INIT | FLD_RF_BRX_NESN_INIT)) |
			   FLD_RF_RX_FIRST_TIMEOUT_EN;
	reset_sn_nesn();
	/* TX: base = empty PDU, sent while the FIFO is empty; the DMA source
	 * stays at the base for the whole connection (md-spike round 5). */
	load(conn_tx_buf, empty_pdu, sizeof(empty_pdu));
	dma_set_src_address(DMA0, convert_ram_addr_cpu2bus(conn_tx_buf));
	/* RX: 4-entry DMA ring */
	rf_set_rx_dma(rx_buf, RING_N - 1, RX_ENTRY_SIZE);
	rf_set_rx_maxlen(CONN_RX_MAXLEN);
	cn.rx_sw = rf_get_rx_wptr();
	rf_clr_irq_status(FLD_RF_IRQ_ALL);
	reg_rf_irq_mask = CONN_IRQ_MASK;
	mode = MODE_CONN;
	irq_unlock(key);
}

void ll_radio_conn_event(uint8_t ch, uint32_t open_tick, uint32_t first_timeout_us)
{
	uint32_t fst = first_timeout_us + RX_SETTLE_US;
	uint32_t trigger = open_tick - RX_SETTLE_US * LL_TICKS_PER_US;

	rf_set_tx_rx_off_auto_mode();
	rf_set_ble_chn((signed char)ch);
	rf_set_ble_access_code_value(cn.aa_reg);
	rf_set_ble_crc_value(cn.crc_init);
	rf_tx_settle_us(TX_SETTLE_CONN_US);
	/* rf_start_brx() would write 0x0fffffff to the first timeout: the
	 * register sequence is done here with a bounded window instead. */
	rf_ble_set_rx_timeout(fst > 0xfff ? 0xfff : (u16)fst);
	reg_rf_ll_rx_fst_timeout = fst;
	rf_clr_irq_status(FLD_RF_IRQ_ALL);
	/* Skip RX entries written after the previous event was closed (e.g. a
	 * packet completing after a guard stop): they belong to no event. */
	cn.rx_sw = rf_get_rx_wptr();
	cn.evt_open = true;
	cn.n_valid = 0;
	cn.n_any = 0;
	cn.first_valid = false;
	cn.tx_seen = false;
	atomic_inc(&cnt_conn_events);
	reg_rf_ll_cmd_schedule = trigger;
	reg_rf_ll_ctrl3 |= FLD_RF_R_CMD_SCHDULE_EN;
	reg_rf_ll_cmd = FSM_BRX;
	ll_sched_guard_at(open_tick + (first_timeout_us + CONN_EVENT_MAX_US) * LL_TICKS_PER_US,
			  conn_guard);
}

void ll_radio_conn_set_sn_init(uint8_t sn)
{
	reg_rf_ll_ctrl_1 = (reg_rf_ll_ctrl_1 & ~FLD_RF_BRX_SN_INIT) | (sn ? FLD_RF_BRX_SN_INIT : 0);
}

uint8_t ll_radio_fifo_rptr(void)
{
	return rf_get_tx_rptr(0);
}

uint8_t ll_radio_fifo_wptr(void)
{
	return rf_get_tx_wptr(0);
}

void ll_radio_fifo_write(uint8_t idx, uint8_t hdr0, const uint8_t *payload, uint8_t len)
{
	uint8_t *e = ring_entry(idx);
	uint32_t dlen;

	if (len > CONN_RX_MAXLEN) {
		len = CONN_RX_MAXLEN;   /* ll_txq never pushes more (PDU_MAX) */
	}
	dlen = rf_tx_packet_dma_len((uint32_t)len + 2u);
	e[0] = dlen & 0xFF;
	e[1] = (dlen >> 8) & 0xFF;
	e[2] = (dlen >> 16) & 0xFF;
	e[3] = (dlen >> 24) & 0xFF;
	e[4] = hdr0;    /* NESN/SN/MD are set by hardware */
	e[5] = len;
	if (len) {
		memcpy(&e[6], payload, len);
	}
}

void ll_radio_fifo_set_wptr(uint8_t wptr)
{
	rf_set_tx_wptr(0, wptr);
}

/* Return to advertising after a connection (spec "Return to advertising",
 * first choice): baseband reset, then the full advertising init. The MD
 * spike found that STX/STX2RX hang once rptr has moved; whether this reset
 * brings rptr back to 0 is verified on air in Task 9 (pointers before and
 * after are kept in the stats). Thread context, called by ll_adv_enable()
 * under ll_plat_lock(). */
void ll_radio_adv_restore(void)
{
	unsigned int key = irq_lock();

	ll_sched_guard_cancel();
	cn.evt_open = false;
	rf_set_tx_rx_off_auto_mode();
	restore_ptrs_before = tx_ptrs();
	rf_baseband_reset();
	if (cn.saved) {
		reg_rf_ll_ctrl_1 = cn.saved_ctrl1;
		reg_rf_rxtcrcpkt = cn.saved_rxtcrc;
		cn.saved = false;
	}
	hw_init_adv();
	/* Empty FIFO again, whatever survived the reset */
	rf_set_tx_wptr(0, rf_get_tx_rptr(0));
	restore_ptrs_after = tx_ptrs();
	rsp_in_flight = false;
	mode = MODE_ADV;
	atomic_inc(&cnt_restores);
	irq_unlock(key);
}
