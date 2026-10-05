# Open BLE Controller (openll)

Our own BLE link layer for the Telink B91. It replaces the proprietary
controller blob `liblt_9518_zephyr.a`. Tracking issue:
[#13](https://github.com/scholzri/rainy75-zmk/issues/13).

**Status:** slices 1 to 7 are done (5b is open, see
[Known limitations and open items](#known-limitations-and-open-items)). The open
controller is the **default build**; the blob is opt-in with `./build.sh --blob`.
A keyboard running it advertises, keeps up to 3 peripheral links at once
(one per ZMK profile), advertises while connected, encrypts with the existing
bond and works as a BLE HID keyboard. It supports CSA #2, Data Length Extension
up to 251 octets, LE Ping, the Connection Parameters Request procedure (responder),
peripheral latency and deep sleep, and opt-in privacy (`--privacy`). No blob is
linked. Only one central has been tested so far (Intel controller with
Linux/BlueZ), and only with one link at a time on the device.

## Why

The blob is a prebuilt controller library from Telink. It is proprietary (see
[NOTICE](../NOTICE)), so it is not committed to this repository and no prebuilt
firmware image with it can be published. With the blob every user has to build
locally, and `fetch_ble_blob.sh` downloads it at build time.

The link map shows the blob as the only prebuilt binary in the image. Zephyr,
the BT host, mbedTLS, picolibc, hal_telink (including `rf.c`, `aes.c`, `trng.c`
and `stimer.c`) and our drivers all build from source. The per-chip RF
calibration at flash `0xFE000` is factory data, not code. With the open
controller the firmware can be built and shipped without any binary blob.

## Using it

### Build

```bash
./build.sh -p --iso                   # or --ansi; the open controller (default)
./build.sh -p --iso --privacy         # open controller with a resolvable private address
./build.sh -p --iso --blob            # opt-in: the Telink blob (fetched on demand)
grep -c liblt build/zephyr/zmk.map    # 0 for the open controller, 48 for the blob
```

The default build appends `conf/openll.conf` to `conf/app.conf`
(`CONFIG_BT_HCI_B91_CTLR_OPEN=y`, power counters, 251-octet host buffers, CCC
load at boot) and never runs `fetch_ble_blob.sh`. `--blob` appends
`conf/blob.conf` instead and is the only path that fetches the blob.
`--privacy` appends `conf/privacy.conf` and is refused with `--blob`.
`--openll` is still accepted as a no-op alias (it prints a note). Deep sleep
(`CONFIG_ZMK_SLEEP=y`, 15 minute idle timeout) works with both controllers.

Image sizes from the "Memory region" summary (ISO, normal sleep, HEAD of
slice 7):

| Variant | ROM | RAM | RAM_ILM |
|---|---|---|---|
| Open (default, `./build.sh -p --iso`) | 303474 B | 105764 B | 7726 B |
| Blob (`./build.sh -p --iso --blob`) | 328120 B | 85436 B | 40288 B |

The open build's RAM is mostly the 251-octet queues: about 5 KB per link
(`CONFIG_BT_HCI_B91_OPENLL_MAX_CONN`, default 3, range 1 to 5) plus the larger
Zephyr host ACL, L2CAP and ATT buffers.

### Flash over USB (mcumgr)

The same as any other app update (see [zmk-firmware.md](zmk-firmware.md)):

```bash
M="$HOME/go/bin/mcumgr --conntype serial --connstring dev=/dev/ttyACM0,baud=115200"
$M image upload build/zephyr/zmk.signed.bin
$M image list                 # note the hash of the new image in slot 1
$M image test <slot-1 hash>
$M reset
# after the reboot, once BLE typing works:
$M image confirm
```

Nothing else may hold the CDC ACM port while mcumgr runs (no `cat`, no serial
logger): a second reader takes SMP responses away from mcumgr, which then
times out. See [zmk-firmware.md](zmk-firmware.md#upstream-patches) for the two
CDC fixes (zephyr patches 0010 and 0011).

A bonded host reconnects by itself and keeps its bond: the identity address
(MAC from flash) and the host's bond storage are the same for both controllers.
The boot log shows `open link layer up`, `AES self-test (FIPS-197 C.1): pass`
and, once the host connects, `connected (handle 0): interval ...`.

### Switching back to the blob

MCUboot keeps the previous image in slot 1. Either `image test <slot-1 hash>`
plus `reset`, or build the blob (`./build.sh -p --iso --blob`) and upload it as
above. The bond survives both ways (unless `--privacy` was used, see
[Privacy](#privacy-opt-in)).

## Architecture

The open controller sits behind the same seam as the blob shim. `hci_b91.c`
(the Zephyr HCI driver) is unchanged and calls three functions declared in
`b91_bt.h`: `b91_bt_controller_init`, `b91_bt_host_send_packet` and
`b91_bt_host_callback_register`. With the blob these are implemented by
`b91_bt.c`. With the open controller they are implemented by
`openll/ll_glue.c`.

### Build switch

A Kconfig choice under `BT_HCI_B91` (`zmk/drivers/bluetooth/Kconfig`) selects
the implementation:

| Option | Meaning |
|---|---|
| `BT_HCI_B91_CTLR_OPEN` (default) | Open link layer from `openll/`. The blob is not linked. |
| `BT_HCI_B91_CTLR_BLOB` | Telink blob (`conf/blob.conf`, `./build.sh --blob`). |

Further options of the open controller:

| Option | Default | Meaning |
|---|---|---|
| `BT_HCI_B91_OPENLL_MAX_CONN` | 3 | Simultaneous peripheral links (1 to 5) |
| `BT_HCI_B91_OPENLL_SCANRSP_SETTLE_US` | 63 | TX settle of the SCAN_RSP (30 to 87), see [SCAN_RSP](#scan_rsp-at-t_ifs) |
| `BT_HCI_B91_OPENLL_STATS_LOG` | n | Radio and connection counters in the log every 2 s |

Wiring:

- `zmk/CMakeLists.txt` links `liblt_9518_zephyr.a` only under `BT_HCI_B91_CTLR_BLOB`.
- `zmk/drivers/bluetooth/CMakeLists.txt` builds `hci_b91.c` and `b91_mac.c` for
  both, `b91_bt.c` for the blob, and `openll/*.c` plus hal_telink's `rf.c`,
  `aes.c` (and `trng.c` unless the Zephyr TRNG driver already provides it) for
  the open controller. It also wraps the hal flash functions at link time
  (`-Wl,--wrap`, see [Flash window](#flash-window)).
- `b91_mac.c` holds the MAC-from-flash logic (read at `0xFF000`, random static
  fallback), shared by both controllers. Both derive the same public address.
- `patches/hal_telink/0002-...` builds hal_telink's `sys.c` unless the blob is
  selected. The blob ships its own `sys_init`, the open controller needs the
  HAL one.
- `zmk/src/openll_mgmt.c` serves the counters over mcumgr group 66
  (`CONFIG_OPENLL_MGMT`, set in `conf/openll.conf`).

### Files (`zmk/drivers/bluetooth/openll/`)

Everything except `ll_glue.c`, `ll_radio.c`, `ll_sched.c` and `ll_flash_wrap.c`
is pure C without Zephyr or hardware dependencies and is tested on the host
with gcc.

| File | Responsibility |
|---|---|
| `ll_glue.c` | Implements `b91_bt.h`. Init (TRNG, MAC, radio, scheduler, arbiter, AES self-test), controller thread (links served round-robin), ACL in and out (fragmentation, shared host ACL slab), HCI flow control, connection and disconnection handling per link, platform hooks (`ll_plat.h`), optional stats log |
| `ll_hci.c` / `.h` | HCI command parser and dispatcher, Command Complete/Status builders, all events toward the host, ACL framing and fragmentation of host packets |
| `ll_pdu.c` / `.h` | Advertising PDUs (ChSel, TxAdd) and SCAN_RSP, SCAN_REQ match, CONNECT_IND parsing |
| `ll_scanrsp.h` | SCAN_REQ -> SCAN_RSP decision and STX trigger tick, inline for the RF ISR in RAM |
| `ll_adv.c` / `.h` | Advertising state machine: public or random AdvA, advertising while connected, slicing into gaps between connection events; hands a CONNECT_IND for us to `ll_conn_start()` |
| `ll_arb.c` / `.h` | Event arbiter: the only user of the main stimer alarm; keeps the events of all links and advertising free of overlaps by priority |
| `ll_conn.c` / `.h` | Per-link connection state machine: transmit window, window widening, anchor re-sync, event counter, CSA #1 or #2, instants (also late ones), latency skips, RX flow control, flash-window pauses, supervision timeout, termination |
| `ll_csa1.c` / `.h`, `ll_csa2.c` / `.h` | Channel Selection Algorithms #1 and #2 |
| `ll_txq.c` / `.h` | Per link: software model of the 4-entry hardware TX FIFO: backlog, per-event ring rebuild, placeholder rule, SN/NESN init per event, ack detection by read pointer, completion callbacks |
| `ll_fifo.h` | Record placement in a per-link byte area (TX and RX queues of variable-length PDUs), word copies |
| `ll_rxq.c` / `.h` | Per link: RX queue filled by the ISR, decryption in the consumer, the reception event per PDU, room checks for RX flow control |
| `ll_llcp.c` / `.h` | Per link: responder LL control procedures, encryption start, LENGTH, PHY (1M only), LE Ping and the authenticated payload timeout, Connection Parameters Request, owed control PDUs, the single encrypt-and-push point for all outgoing data PDUs |
| `ll_crypt.c` / `.h` | BLE AES-CCM (encrypt and decrypt one PDU, nonce from packet counter, direction and IV), session key derivation |
| `ll_credit.c` / `.h` | Per link: "up" for the host, connection generation, Number Of Completed Packets counts |
| `ll_radio.c` / `.h` | The only RF-touching file. Wraps hal_telink `rf.c`: advertising (STX2RX, STX), connection events (BRX), per-event switch between links and to advertising, TX FIFO pointers, RX DMA ring, CPU hold in the turnaround, SCAN_RSP from the RF ISR, timestamps, T_IFS monitor, return to advertising, counters |
| `ll_radio_mode.c` / `.h` | Advertising register snapshot and "connection setup done" flag across baseband restores |
| `ll_sched.c` / `.h` | One-shot main alarm (owned by `ll_arb`) and a guard alarm (owned by `ll_radio`) at absolute system timer (stimer, 16 MHz) ticks, callbacks in ISR context |
| `ll_flash.c` / `.h` | Flash window: no radio activity while a flash erase or write runs with interrupts off |
| `ll_flash_wrap.c` | Device glue of the flash window: wraps the hal flash erase/write/read functions at link time |
| `ll_defs.h` | Shared constants (HCI status codes, PDU types, LLIDs, features, version, ACL buffer sizes, DLE limits, ticks per us, `LL_MAX_CONN`) |
| `ll_plat.h` | Platform hooks used by the pure code: random number, IRQ lock, TX mutex, AES-128 block |
| `tests/` | Host tests (see [Host tests](#host-tests)) |

### Execution contexts

- **ISR (RF and stimer interrupts):** `ll_arb` (alarm), `ll_adv`, `ll_conn`,
  `ll_txq` (ring rebuild at event start, ack processing at event end) and the
  producer side of `ll_rxq`. The RX ISR copies every received data PDU out of
  the RX DMA ring before the DMA can reuse the entry. Callbacks toward the glue
  only record what happened (pending bits, counters) and wake the controller
  thread.
- **Controller thread** (`CONFIG_BT_HCI_B91_RX_PRIO`, preemptible): serves the
  links round-robin (the first link of a pass rotates). Per link it consumes
  `ll_rxq` (decrypt, LLCP to `ll_llcp`, ACL to the host), retries owed LLCP
  PDUs, sends host ACL through `ll_llcp_tx_acl()`, and emits LE Connection
  Complete, Connection Update Complete, Data Length Change, Number Of Completed
  Packets and Disconnection Complete. After a disconnect it resets that link's
  `ll_rxq` and `ll_llcp` and only then releases the link id.
- **HCI thread** (the host's TX path): HCI commands. Host ACL is only parsed and
  queued here. The LTK reply, HCI Disconnect and the connection parameter
  replies queue control PDUs from this thread.
- **Locks:** `ll_plat_lock()` (interrupts off) protects state shared with the
  ISRs and is held only briefly. `ll_plat_tx_lock()` is a recursive `k_mutex`
  with priority inheritance that serializes the two TX producers (controller
  thread and HCI thread). Encryption and push of one PDU run under it, so the
  CCM packet counter order equals the FIFO order. AES never runs with
  interrupts off. The mutex may block, so it is never taken from an ISR or with
  the IRQ lock held. The per-event ring rebuild is never IRQ-locked either: a
  test that locked it (50 to 230 us per event) stalled the USB CDC console.

### Links (multilink)

`LL_MAX_CONN` links (Kconfig `BT_HCI_B91_OPENLL_MAX_CONN`, default 3) share
the one radio. Link id = HCI connection handle (0 .. N-1). Everything that
belongs to a connection is per link: `ll_conn` state (free, active, or ended
and waiting for release), TX queue and ring copies, RX queue, LLCP state and
timers, crypto context and packet counters, host ACL queue, credits and
pending events. A CONNECT_IND takes the lowest free id; an ended id stays
taken until the controller thread has reset that link's queues and released
it, so it is never reused early.

Per event the radio is switched to the event's owner:

- `ll_radio_conn_select(aa, crc_init)`: register writes only (about 8 us):
  connection-mode registers, AA, CRC init, DMA source of the TX ring.
- `ll_txq_event_start(link)`: the TX FIFO is rebuilt for the link at the
  current read pointer (`wptr = rptr`, then the link's unacked entries from
  their software copies, then its backlog). The hardware reads the head entry
  from RAM at TX time and keeps nothing across commands, so another link's
  traffic in between does no harm (verified on the device with a foreign BRX
  and BTX between every pair of events, 0 pointer changes and 0 lost PDUs).
- `ll_radio_adv_enter()` before an advertising channel while links exist:
  FSM off, **empty TX FIFO (`wptr = rptr`)**, advertising registers (about
  60 us). A non-empty TX FIFO wedges the next STX2RX (the FSM stays in RX wait
  and later BRX commands never end), which was the cause of the "advertising
  hang after a connection" seen in slice 2. The full baseband reset
  (`ll_radio_adv_restore()`, about 140 us) runs only when the last link has
  ended and as the recovery of the advertising guard.

A guard streak of 3 events without any valid packet means the shared radio is
wedged; all links end with 0x08.

### Event arbiter (`ll_arb`)

Every link and advertising keep at most one request: their next event, with a
span `[alarm, RX open + min_len]` (min_len: the first RX window plus one
exchange at the link's effective data length times, plus a clipping reserve;
advertising: the whole adv event or one channel). Accepted requests never
overlap. The arbiter arms the main alarm for the earliest one and calls the
owner when it fires; the event's cap is clipped so it ends 800 us
(`LL_CONN_EVENT_SAFETY_US` + `LL_CONN_ARM_LEAD_US`) before the next accepted
request opens.

Priorities (higher wins):

| Priority | When |
|---|---|
| MUST | transmit-window event (new connection or connection update) or an instant event |
| SUPERVISION | the event opens later than last RX + timeout - 2 intervals |
| ACTIVE | TX backlog or an LLCP procedure waiting |
| IDLE | anything else |
| ADV | advertising (starving advertising asks at ACTIVE) |

Rules:

- A request that overlaps a running event or one of higher priority is
  refused. On a tie the requester wins only if it yielded at its last
  collision and the other did not (round-robin). Otherwise the overlapped
  requests are displaced (bumped).
- A refused or bumped link first tries other events of its latency window,
  latest first (dodge). If none is accepted it yields: the event goes by
  unheard (counter and CSA advance as for a skip, `coll` in the stats, not a
  miss for the latency rule). Instants on yielded events are still applied.
- A running event cannot be displaced. If an alarm fires while another event
  overran its cap, the due request is bumped instead.
- Advertising slides into the next gap (`ll_arb_gap()`). A whole adv event
  reserves 6 ms (2 ms per channel incl. a SCAN_RSP); if it does not fit, the
  channels go into separate gaps, each PDU within 10 ms of the previous one
  (Vol 6 Part B 4.4.2.3), else the event is cut. After 2 intervals without a
  complete event, or 2 drops or cuts in a row, advertising asks at ACTIVE and
  may displace idle or active link events (never SUPERVISION or MUST ones).

Host scenarios with the real `ll_conn`/`ll_adv` (`test_arb`): two links on the
same interval drifting through each other (gap at most 2 events, no
disconnect), an idle link overlapping an always-busy one (supervision priority
keeps it alive), 3 always-busy links at 7.5 ms with advertising (adv gap at
most 250 ms; before the starvation rule advertising never got out).

### Advertising while connected

`LE Set Advertising Enable` is accepted while a link is free. Connectable
advertising with every link taken returns 0x09 (Connection Limit Exceeded):
Vol 4 Part E 7.8.9 names no code for this case, Vol 1 Part F 2.9 fits. With
`MAX_CONN=1` this means 0x09 (not 0x0C) while the one link exists.
Non-connectable and scannable advertising are always allowed. A CONNECT_IND
while connected creates the next link and stops advertising, as the spec
requires; ZMK re-enables advertising whenever the active profile is open or
not connected.

An advertising guard (2 ms after an STX2RX, 1.5 ms after a SCAN_RSP trigger)
ends an advertising channel that gets no end IRQ and restores the baseband
while keeping the connection setup (`ll_radio_mode`).

### Connection event flow

1. **Alarm** (`ll_arb`), `LL_CONN_ARM_LEAD_US` = 500 us before the RX window
   opens. If the alarm runs less than 120 us before the window, the event is
   skipped (counted as `late`). If the cap left by the arbiter is below the
   first RX window plus one exchange, the event is yielded.
2. **Prepare** (`ll_conn`): channel from CSA #1 or #2, RX window = anchor -
   widening - margin, first-RX timeout = 2 x (widening + margin) + 40 us (the
   transmit window size is added for window events). Not issued while the RX
   queue lacks room ([RX flow control](#rx-flow-control)) or a flash window is
   open. `ll_radio_conn_select()` and `ll_txq_event_start()` switch the radio
   to the link.
3. **BRX** (`ll_radio`): one BRX command (`0x82`) with TX settle 86 us,
   triggered 80 us (RX settle) before the window opens. The hardware receives
   the central's packet, answers after its own turnaround and keeps chaining
   RX/TX exchanges while either side has MD set.
4. **RX IRQ:** for the event's first packet the ISR does only the anchor check,
   holds the CPU until our response has started (see
   [CPU hold](#cpu-hold-in-the-turnaround)) and then delivers the packet.
   Later packets are delivered from the TX IRQ of our response or the end
   IRQ. Each PDU goes to `ll_txq_rx()` (SN tracking) and `ll_rxq_isr_put()`
   together with the event counter. Only the event's first packet re-anchors
   and refreshes the supervision timer (see "Anchor rule").
5. **Event end:** CMD_DONE, a first-RX timeout, an RX flow-control stop or the
   guard. `ll_txq_event_end()` reads the read pointer and completes acked
   entries, the event counter advances, supervision and termination are
   checked, instants are applied and the next event is planned and requested.
   The guard alarm ends an event without an end IRQ at RX open + the cap
   (interval minus the widening growth, the 500 us alarm lead and a 300 us
   safety, at least the first RX window + one exchange: 1000 us at 27 octets,
   4540 us at 251 octets / 2120 us), clipped by the arbiter. A long central MD
   burst may use almost the whole interval but never overruns the next event.
   Three guard-ended events in a row that received no valid packet end all
   links with 0x08 (radio wedged); a guard that cuts a burst with packets
   resets that count.

**Anchor rule:** the anchor is the start of the event's first packet. If that
packet had a bad CRC, or was a retransmission the hardware acked without an RX
entry (`LL_RADIO_CONN_RX_NODATA`), or the first reported packet starts after the
RX window, a later packet of the same event is a chained one and must not move
the anchor. It only refreshes the supervision timer.

**Timing:** all link layer timing uses the stimer (16 MHz from the 24 MHz
crystal). Kernel time runs on the 32 kHz RC and is not used. Window widening is
`(central SCA ppm + 50 ppm) x time since the last received anchor + 16 us`,
rounded up, clamped at `interval / 2 - 150 us` (the supervision timeout then
ends the link). RX margin is 60 us for synced events, 200 us for transmit
window events.

### TX path

The B91 baseband has a 4-entry TX FIFO for pipe 0 with a read pointer
(`0x80100501`) that the hardware advances when the central acks the head, and a
write pointer (`0x80100500`) written by software:

- Empty FIFO (`rptr == wptr`): the hardware sends the DMA base buffer, which
  holds an empty PDU.
- Otherwise it sends entry `rptr & 3` at base + 272 x (1 + entry). MD on air is
  set by the hardware when more entries wait behind the one being sent. The MD,
  SN and NESN bits in the buffer are ignored.
- An entry that is not acked is resent with the same SN.

Per link, `ll_txq` keeps up to 16 PDUs (backlog plus unacked ring entries) in
a 1280-byte area and writes the ring only at event start, never while an
event is on air. An entry is complete when the read pointer has passed it: the
last fragment of a host ACL packet counts toward Number Of Completed Packets,
control PDUs notify `ll_llcp` and `ll_conn` (LL_TERMINATE_IND ack).

**Placeholder rule.** At the first RX of a BRX command the hardware judges the
central's ack against the programmed SN init, but it cannot know whether our
last packet was the base buffer or a ring entry. If our last packet was the base
empty PDU, `ll_txq` therefore writes an empty placeholder entry in front of new
data. If the central acked the base, the first pop removes the placeholder
unsent, which is correct. If not, the placeholder is resent with the base's SN
and the same empty content, a correct retransmission. Verified on the device
under forced NACKs (TX settle raised to 102 us, about 18 % of our responses not
received by the central): 2686 of 2686 encrypted SMP echoes matched, and again
with 251-octet PDUs and the per-event ring rebuild (5320 of 5320). On an
encrypted link a duplicate or a lost PDU would have shifted the CCM packet
counter and ended the link with 0x3D. A fallback (`LL_TXQ_SAFE_MODE`, one data
PDU per event) exists in the code and is host-tested, but is not needed.

### RX path

The RX DMA is a 4 x 272 byte ring, configured once at boot. The hardware writes
only new packets into it: a retransmission of the central (SN not the expected
one) is acked through NESN but not written as a new entry, and its RX IRQ finds
no new entry. Retransmissions are therefore filtered in hardware; there is no
software duplicate check (10 595 central retransmissions in one forced-NACK
run, none reached software).

The RX ISR copies each CRC-valid data PDU into the link's `ll_rxq` (16 entries,
2048 bytes: 8 maximum PDUs or 16 short ones), data PDUs only. The controller
thread drains it and decrypts in RX order. A MIC failure ends the link with
0x3D. An overflow cannot be recovered (the hardware has already acked the PDU,
so the central never resends it); `ll_conn` then ends the link with 0x08
instead of continuing with a hole in the L2CAP stream. RX flow control (below)
keeps that from happening.

#### RX flow control

The controller thread is preemptible, but the Zephyr host's BT RX work queue
and the system work queue are cooperative and run inside it
(`bt_hci_recv()` -> `k_sched_unlock()`). Under a burst of 27-octet PDUs
(one 420-byte SMP request is about 20 PDUs, one every 0.7 ms) the thread
drained about one PDU per 3 ms, so the 17th PDU overflowed `ll_rxq` and the
link ended (4 of 4 uploads at the first or second request). The only lossless
lever is to not receive:

- An event is not issued while `ll_rxq` lacks room for 5 (ring size + 1)
  maximum-size PDUs (`rx_paused`, also counted as missed; the central resends,
  as after any missed event).
- A received packet that leaves less room stops the open event once
  (`rx_stops`, `ll_radio_conn_stop()`: FSM off at the end of the RF ISR, ring
  drained). The at most 4 packets still in the radio fit.
- The controller thread is woken one maximum PDU before the stop point, or
  when the queue is half full, whichever comes first.

Result: 27-octet links upload a 297 KB image in 50.7 s with 0 disconnects
(before: lost at once); on 251-octet links flow control is nearly idle (one
run of 12 paused events per three uploads) and costs no throughput.

#### CPU hold in the turnaround

CPU activity during the hardware RX -> TX turnaround delays our TX by 1 to
4 us. With the PDU copy, `ll_conn` work and the thread wake in the RX IRQ,
4 to 6 % of the responses had a T_IFS above 152 us (14 % when the thread was
woken per PDU). The RX IRQ therefore does only the anchor check (one write
pointer read) for the event's first packet, busy-waits on the `mcycle` CSR (no
bus access) until 170 us after the packet's end (at most 200 us; skipped when
the IRQ is late), and then delivers the packet. Later packets are delivered
from the TX IRQ of our response, so no processing runs in any turnaround.

Result on air (sniffer, our response to the event's first central packet):
above 152 us dropped from 2.4 to 2.9 % to 0.00 to 0.03 %. Cost: up to about
160 us of CPU per listened event with a received packet (about 2 % at a
7.5 ms interval under load, nothing in skipped events); under a 120 s upload
plus echo load the echo rate was 4 to 9 % lower, a BLE image upload 27.1 s
instead of 25.0 s. The stats log counts the holds (`hold`).

### Data Length Extension (251 octets)

The LENGTH procedure runs as responder and initiator (Vol 6 Part B 4.5.10,
5.1.9). We support 251 octets / 2120 us both ways; the host asks for them on
every connection (`CONFIG_BT_AUTO_DATA_LEN_UPDATE=y`: LE Write Suggested
Default Data Length 251 / 2120, then the procedure). BlueZ sends
LL_LENGTH_REQ 251 / 2120 by itself; the log shows
`data length (handle 0): tx 251 B / 2120 us, rx 251 B / 2120 us` about 0.5 s
after connect.

- **DMA entries:** TX ring slot and RX DMA entry 272 bytes (TX: 4 DMA length
  word + 2 header + 251 + 4 MIC = 261, rounded to the 16-byte unit; RX: the DMA
  writes up to byte 271 for a 255-byte packet). RX maxlen 255. Central
  retransmissions are DMA-written into the next entry without advancing the
  write pointer, so every entry must hold a full packet.
- **Octets do not count the MIC; times do.** connMaxTxOctets limits the
  plaintext payload (an encrypted 251-octet PDU is 255 bytes on air);
  connMaxTxTime includes the MIC: 8 us x (1 + 4 + 2 + payload + 4 + 3), so 27
  octets = 328 us and 251 = 2120 us.
- **TX limit by octets and time:** `ll_dle_tx_limit()` takes the smaller of
  the effective octets and what fits in the effective time (minus the MIC when
  encrypted). Example: a central that offered 251 B / 415 us both ways gave a
  limit of 37 octets encrypted, and every host packet was fragmented.
- **Fragmentation:** host ACL packets of up to 251 octets (LE Read Buffer Size
  reports 251 x 3) are split into PDUs of the link's TX limit, all fragments
  or none under the TX lock; Number Of Completed Packets is reported when the
  last fragment is acked. PDUs queued before the TX length shrank stay valid
  (4.5.10) and are sent as they are; the event span follows the longest queued
  PDU until it is acked.
- **Event span:** the guard floor and the arbiter's `min_len` cover one
  exchange at the effective times: max(1000, rx + 150 + tx + 150) us, so
  1000 us at 27 octets and 4540 us at 251 / 2120.
- **Host side** (`conf/openll.conf`): `BT_BUF_ACL_TX_SIZE=251`,
  `BT_BUF_ACL_RX_SIZE=251`, `BT_L2CAP_TX_MTU=247` (ATT MTU 247: 244-byte writes
  and notifications in one PDU, used by SMP and ZMK Studio).

Measured on the keyboard with the BlueZ PC (central at 251 / 2120 unless
noted):

| | 27 octets, ATT MTU 65 (before) | 251 octets, ATT MTU 247 |
|---|---|---|
| SMP image upload over BLE, 293 KB, 128 B per request | 122.0 s | 58.5 s |
| SMP image upload, 420 B per request | link lost at the first request (fixed later by RX flow control: 50.7 s on a 27-octet link) | 25.0 s (4.9x); 27.1 to 27.4 s with the CPU hold |
| flash_mgmt read 64 KiB (notifications) | 24.7 s | 8.3 s (3.0x) |
| ZMK Studio RPC over GATT | 10.3 per s | 22.6 per s (2.2x) |

### Channel Selection Algorithm #2

ADV_IND and ADV_DIRECT_IND carry ChSel = 1, as the blob does. A CONNECT_IND
with ChSel 1 selects CSA #2 for that link (`ll_csa2.c`, Vol 6 Part B
4.5.8.3), otherwise CSA #1. The channel map in force (also after a map
instant) comes from the CSA #1 state, which is stepped for both algorithms;
CSA #2 is a function of the event counter, so latency skips, kicks, yields and
instants need no extra code. Feature bit 14 is reported in LL_FEATURE_RSP and
LE Read Local Supported Features. The LE Channel Selection Algorithm event is
sent only when the host enables it (Zephyr does not).

Verified: Core Spec sample data (Vol 6 Part C 3.1 and 3.2) in the host tests,
2143 of 2143 (AA, counter, channel) triples from blob-era sniffer captures, and
on the device: BlueZ sends ChSel 1, a sniffer follow matched CSA #2 in 179 of
179 events (CSA #1: 5 of 179).

### PHY: 1M only

LL_PHY_REQ is answered with LL_PHY_RSP {1M, 1M}; every LL_PHY_UPDATE_IND keeps
1M (Vol 6 Part B 5.1.10: the peripheral shall not change the PHY in a direction
with an invalid or unsupported choice). HCI LE Read PHY reports 1M, LE Set PHY
completes at once with LE PHY Update Complete (1M, 1M).

2M is not possible without a register source: hal_telink's open `rf.c` only
contains the 1M mode (`rf_set_ble_2M_mode()` is declared, the body is not
public), and Telink's other public SDKs ship RF only as binary libraries. A
clean-room 2M implementation would need reverse engineering of the blob, which
this project excludes. (The blob's own 2M caused LL Response Timeout 0x22.)

### LLCP (responder) and encryption

| Received | Action |
|---|---|
| LL_FEATURE_REQ | LL_FEATURE_RSP. Byte 0 `0x37` (LE Encryption, Connection Parameters Request, Extended Reject Indication, LE Ping, Data Length Extension), ANDed with the central's; byte 1 `0x40` (CSA #2) |
| LL_VERSION_IND | LL_VERSION_IND (version 0x09, company 0xFFFF, subversion 1), once per connection |
| LL_CONNECTION_UPDATE_IND | Applied at the instant with the new transmit window; LE Connection Update Complete if interval, latency or timeout changed |
| LL_CHANNEL_MAP_IND | Applied at the instant. Fewer than 2 used channels end the link with 0x1E |
| LL_TERMINATE_IND | Disconnection Complete with the reason from the PDU |
| LL_ENC_REQ | LL_ENC_RSP (SKDs, IVs) right away, then LE LTK Request to the host. On an already encrypted link (or during the procedure): rejected with 0x24 |
| LL_START_ENC_RSP | TX encryption on, encrypted LL_START_ENC_RSP, Encryption Change to the host |
| LL_PAUSE_ENC_REQ | Rejected (0x1A), key refresh not supported |
| LL_LENGTH_REQ | LL_LENGTH_RSP with our values, effective values updated, LE Data Length Change if they changed |
| LL_PHY_REQ / LL_PHY_UPDATE_IND | LL_PHY_RSP {1M, 1M}; 1M kept |
| LL_PING_REQ | LL_PING_RSP (dropped inside the encryption start) |
| LL_CONNECTION_PARAM_REQ | See [Connection Parameters Request](#connection-parameters-request-responder) |
| Unsupported requests (e.g. PERIPHERAL_FEATURE_REQ, MIN_USED_CHANNELS) and known requests with a wrong length | LL_UNKNOWN_RSP |
| Response opcodes outside their procedure and LL_START_ENC_RSP outside the encryption start | Dropped silently |
| own: HCI Disconnect | LL_TERMINATE_IND, Disconnection Complete (0x16) after the ack or the supervision timeout |

Encryption start (Core Spec Vol 6 Part B 5.1.3.1), peripheral side: on the
host's LTK reply the session key `SK = e(LTK, SKDs || SKDm)` is derived, RX
decryption is switched on and LL_START_ENC_REQ is queued in plaintext. The
central's LL_START_ENC_RSP arrives encrypted and turns TX encryption on. A
negative reply sends LL_REJECT_EXT_IND (or LL_REJECT_IND if the central lacks
Extended Reject) with 0x06. No other procedure's PDU is sent inside the
encryption start (a LENGTH request is deferred until our LL_START_ENC_RSP).

Every procedure has its own 40 s response timer per link (encryption, LENGTH,
PHY, Ping, Connection Parameters Request); any of them ending ends the link
with 0x22.

**Owed PDUs.** Every control PDU we send goes through one path. When the TX
queue is full, it is owed (up to 4 per link) and retried in order before the
link's next host ACL (host ACL gets -EAGAIN while control PDUs are owed); the
controller thread wakes every 10 ms while something is owed. Owing never ends
a link by itself; the procedure's 40 s timer does.

`ll_crypt` implements AES-CCM in software on top of the hardware AES-128 block
of the B91 (hal `aes_encrypt()`). It reproduces the Core Spec sample data
(Vol 6 Part C) in the host tests, and the glue runs a FIPS-197 C.1 self-test at
every boot. One AES block takes 31 to 47 us on the device; a 251-octet PDU is
about 32 blocks, encrypted in the thread outside the IRQ lock.

### LE Ping

LE Ping is optional (Vol 6 Part B Table 4.7: "O" toward the peer, also with LE
Encryption). It is implemented: we answer LL_PING_REQ, and the authenticated
payload timer (default 30 s, Vol 6 Part B 5.4) runs while the link is
encrypted and restarts on every PDU with a valid MIC. On expiry we send an
encrypted LL_PING_REQ (at most one per timeout) and the Authenticated Payload
Timeout Expired event if the host enabled it on event mask page 2 (Zephyr
does not). We ping only at expiry, not ahead of it as Zephyr's ll_sw does
(about timeout - (latency + 6) x interval), so with a central that sends no
MIC'd PDU for a whole timeout the event fires once per idle timeout, not
only on a real loss of authenticated traffic; on ZMK the event is masked,
so this costs nothing. HCI Read/Write Authenticated Payload Timeout and Set Event Mask
Page 2 are implemented, since claiming LE Ping makes them mandatory. A value
below the connection's interval x (1 + latency) is raised at connection
updates.

On air: since we claim the feature, the PC's controller sends LL_PING_REQ
about every 15 s (before, it got LL_UNKNOWN_RSP and stopped). Its MIC'd
request restarts our timer, so we never ping it. Cost: one kicked listen per
15 s, within the noise of the idle measurement (90.3 % skipped at latency 10,
before 90.2 %).

### Connection Parameters Request (responder)

Feature bit 1 is set. An LL_CONNECTION_PARAM_REQ (exactly 24 octets) is:

- dropped inside the encryption start;
- rejected with LL_REJECT_EXT_IND 0x23 while an earlier request is open or an
  update instant is ahead, 0x2A while a map instant is ahead or our PHY
  response waits (Vol 6 Part B 5.3);
- rejected with 0x1E for invalid fields;
- answered at once with LL_CONNECTION_PARAM_RSP if it only moves the anchor
  points (interval, latency and timeout unchanged; not indicated to the host,
  5.1.7.2);
- otherwise indicated to the host (LE Remote Connection Parameter Request).
  The host answers with Reply (RSP with the host's values) or Negative Reply
  (LL_REJECT_EXT_IND, always 0x3B on air). A Reply with invalid values is
  refused with 0x12 and the request is rejected with 0x3B.
- If the event is masked, the LL accepts with the central's values: 5.1.7.2
  says the LL "may" indicate the request if the event is not masked, and
  otherwise proceeds as if the host had accepted. The masked 0x1A reject
  applies only when the LL decides to indicate and the event is masked, a path
  we never take. Zephyr enables the event, so the device always indicates.

The procedure ends with the central's LL_CONNECTION_UPDATE_IND (or a reject
naming it, or its 40 s timer). On the device every ZMK request (still sent as
an L2CAP Connection Parameter Update Request, see below) now makes the PC send
LL_CONNECTION_PARAM_REQ, which Zephyr replies to; the resulting parameters are
the same as before (12 / 10 / 400 after connect, 6 / 0 / 42 during SMP,
12 / 30 / 400 after it).

**Initiator: not implemented.** Zephyr uses HCI LE Connection Update (0x2013)
as a peripheral only after it learned the central's features through LE Read
Remote Features, which a peripheral host sends only when the controller claims
feature bit 3 (Peripheral-initiated Feature Exchange). We claim neither bit 3
nor implement 0x2016, so every peripheral request goes over L2CAP. Known gap:
HCI Table 3.1 C.62 makes 0x2013 mandatory for a peripheral that claims bit 1;
we answer it with 0x01 (unknown command). **0x2013 must be implemented before
bit 3 (and 0x2016) may be claimed.**

### HCI subset

Commands: Reset, Set Event Mask, Set Event Mask Page 2, Read Local Version
Information, Read Local Supported Commands, Read Local Supported Features, Read
BD_ADDR, Disconnect, Read/Write Authenticated Payload Timeout, LE Set Event
Mask, LE Read Buffer Size (251 bytes, 3 packets), LE Read Local Supported
Features, LE Rand (hardware TRNG), LE Set Random Address, LE Set Advertising
Parameters, LE Set Advertising Data, LE Set Scan Response Data, LE Set
Advertising Enable, LE Long Term Key Request Reply and Negative Reply, LE
Remote Connection Parameter Request Reply and Negative Reply, LE Set Data
Length, LE Read/Write Suggested Default Data Length, LE Read Maximum Data
Length, LE Read PHY, LE Set Default PHY, LE Set PHY.

Events: Disconnection Complete, Encryption Change, Number Of Completed
Packets, Authenticated Payload Timeout Expired, LE Connection Complete, LE
Connection Update Complete, LE Long Term Key Request, LE Remote Connection
Parameter Request, LE Data Length Change, LE PHY Update Complete, LE Channel
Selection Algorithm. ACL to the host carries the link's handle.

Invalid parameters return 0x12, unknown opcodes 0x01 (logged once per opcode).
The supported-commands bitmap was cross-checked against Zephyr's `ll_sw`
`hci.c`. The feature bits advertise no 2M or Coded PHY, no LL privacy
(resolving list), no extended advertising and no Peripheral-initiated Feature
Exchange, so the host does not try them.

Host Number Of Completed Packets (0x0C35) is accepted silently. Zephyr sends
it for every received ACL packet because `CONFIG_BT_HCI_ACL_FLOW_CONTROL` is
on for a host-only build, even though it never enabled controller-to-host flow
control: we do not claim Set Controller To Host Flow Control (commands octet
10 bits 5 to 7). So the host logs one warning per boot, "Controller to host
flow control not supported". Turning the Kconfig off would remove it but
changes the host's RX buffer handling; not done.

### Privacy (opt-in)

`./build.sh --privacy` adds `conf/privacy.conf` (`CONFIG_BT_PRIVACY=y`, RPA
timeout at the Zephyr default of 900 s). The host then generates an IRK and
advertises with a resolvable private address; the controller implements LE Set
Random Address and own address type random (TxAdd 1 in the advertising PDUs,
SCAN_REQ and CONNECT_IND matched against the address in use). Address
resolution stays in the host (no resolving list).

- **Re-pair every host.** A bond made without privacy has no IRK of ours, so
  the host cannot resolve the RPA and does not reconnect (seen on the device).
  Whether a re-paired host reconnects to the RPA is not tested yet.
- **ZMK patch `zmk-src/0005` is required.** Without it the first boot with
  privacy deadlocked: the host queued storing the new IRK on the system work
  queue while ZMK started advertising inside the settings commit (settings
  lock held); LE Set Random Address waited for the work queue, the HCI command
  timed out and asserted, and MCUboot reverted the test image. With the patch
  ZMK starts advertising from its work queue after the settings load. Verified:
  the RPA rotated every 60 s (test image with `BT_RPA_TIMEOUT=60`), the public
  address never on air.
- After going back to a build without privacy, the stored IRK stays in NVS and
  the boot log shows `settings: set-value failure. key: bt/irk error(-2)`
  (harmless; it goes with a bond reset or a privacy image).

### SCAN_RSP at T_IFS

The RX interrupt answers a SCAN_REQ itself, before the link layer callback:
a CRC-valid SCAN_REQ for the AdvA and TxAdd of the prepared SCAN_RSP gets a
single scheduled TX (`rf_start_stx()`, TX settle
`CONFIG_BT_HCI_B91_OPENLL_SCANRSP_SETTLE_US`, default 63 us) triggered
150 - settle - 59 us (default 28 us) after the request's end, so the first bit
is on air 150 us after the request (trigger + settle + a fixed 59 us TX path
delay, measured on one board). The decision and the trigger run from RAM
(`.ram_code`, `ll_scanrsp.h` inline): with flash-resident helpers the decision
came 50 to 90 us after the request, too late. The CPU then holds on the cycle
counter until the response is on air, which removes a 2 to 5 us jitter of the
TX start. A decision later than 3 us before the trigger is not answered
(`rsp_late`, also a SCAN_REQ for us outside an open RX window). Nothing else is
ever answered.

Why 63 and not the hal's SCAN_RSP settle of 78: 63 is the smallest TX settle
hal_telink publishes for BLE 1M (`ext_rf.h` `LL_SCAN_TX_SETTLE`,
`LL_TX_STL_TIFS_1M`). With 78 the trigger would lie 13 us after the request's
end, while the RX interrupt decides 10 to 30 us after it, so almost every
response would be late.

Measured with settle 63: sniffer T_IFS 149 us 2, 150 us 119, 151 us 8 (not
connected) and 149 us 1, 150 us 27, 151 us 1 (advertising during a connection);
the TX-timestamp estimate gave 150 us in 392 of 392. `rsp_late` 14 of 406
(3.4 %) while not connected (15 min of active scanning), 27 of 77 (35 %) while
advertising during a connection with SMP echo load. The tested Intel scanner
takes the 150 us responses; it never took the old ones.

Why not the hardware turnaround: BRX and RX2TX transmit after whatever they
receive (a CONNECT_IND, another advertiser's SCAN_REQ). Only a CPU veto in the
RX interrupt could stop that, and the interrupt can be held off by
interrupt-locked sections (a flash erase takes 13 to 26 ms), so the controller
would sometimes answer a packet it must not answer. The scheduled TX fails
safe: no answer unless the CPU decided in time.

Before slice 7 the decision went through the link layer callbacks 60 to 90 us
after the request and the trigger used the hal's 78 us: on air at about
209 us, outside the scanner's window.

### Flash window

Every hal flash erase and write (hal_telink `flash.c`) runs with interrupts
off: a 4 KB sector erase 12 to 29 ms, a 256-byte page write up to about 2.5 ms.
A connection event that is open then keeps running in hardware: the baseband
receives and acks the central's packets into the 4-entry RX DMA ring, but
neither the RF interrupt (which empties the ring) nor the guard alarm (which
ends the event) can run. A central burst of more than 4 packets overwrites
acked entries (`ptr_skip` in the stats), the lost PDU makes the next one fail
its MIC, and the link ends with 0x3D. Seen during USB image uploads with bursty
BLE traffic (10 of 10 uploads; 5 or 6 packets arrived during one 12.4 ms sector
erase), and possible whenever ZMK writes settings while the PC sends data.

`ll_flash_wrap.c` wraps the hal functions at link time (`-Wl,--wrap`, set in
`zmk/drivers/bluetooth/CMakeLists.txt`; no Zephyr or hal patch), so every
caller is covered: NVS / settings, mcumgr image upload, flash_mgmt and
`b91_mac.c`. Each sector erase and each 256-byte page write runs in a window
(`ll_flash.h`):

- opening the window stops what is on air (`ll_radio_flash_abort()`: an open
  connection event ends with the packets already in the ring delivered, an
  advertising channel ends like an RX timeout);
- while it is set, `ll_conn` issues no connection event (`flash_paused`, also
  counted as missed; the central resends as after any missed event) and
  `ll_adv` sends no advertising channel;
- 32 and 64 KB block erases are split into sector erases; reads get no window
  (at most 66 us with interrupts off) but are split into 256-byte reads.

Supervision: a window opens only while every link is established and, after
one operation (30 ms budget), still has half of its supervision timeout left
since its last received packet. Otherwise the links' next events are pulled
in (`ll_conn_flash_kick()`) and the flash caller sleeps 1 ms and asks again (the
window itself is never held across a sleep). A chain of back-to-back erases
therefore lets an event through whenever a link needs one; after 100 ms of
waiting the window opens anyway (`forced`); a caller that holds interrupts
off does not wait at all. The scheduler is locked from the open to the close
(the hal call never sleeps), so no other thread can stretch a window past its
operation (longest window `hmax` 13.9 ms on the device, one sector erase). The
30 ms budget is the measured sector erase, not the safety margin: a ready link
starts an operation with half its supervision timeout left, so it survives any
single operation shorter than about timeout / 2 + 30 ms (80 ms at the spec
minimum of 100 ms, 240 ms at mcumgr's 420 ms); NOR datasheets allow longer
worst-case erases than that.

Costs:

- During a long chain of erases a link receives only about every half
  supervision timeout, so BLE throughput and key latency drop while the flash
  is busy (a USB image upload under BLE echo load took 90 s instead of 80 s).
- Advertising sends nothing during back-to-back windows: while a slot is erased
  sector by sector (a 456 KB slot is about 114 erases, over a second) no
  advertising event gets out.
- The Zephyr flash driver holds its write lock during the wait for the links
  (up to 100 ms per operation) and takes it with `K_NO_WAIT`, so a concurrent
  flash erase or write from another thread fails with -EACCES more often. NVS,
  settings and img_mgmt each write from one thread, flash_mgmt reports the
  error to the host. Seen once as `bt_gatt: Failed to store CCCs (err -13)`
  during a BLE upload (harmless).

Measured (default NOSLEEP image): before the fix, 10 of 10 USB image uploads
lost the link (MIC failure 0x3D, `ptr_skip` 1 or 2) under a bursty BLE echo
load (150 to 240 characters in 20-byte writes without response). With it, 10
USB uploads (mcumgr CLI, 90.1 to 90.7 s each; 79.6 s without BLE load) under
one 1000 s echo client: 13 252 of 13 252 echoes, no disconnect, `ptr_skip` 0,
12 451 windows, 0 waits; after the scheduler-lock fix 5 more uploads with
7831 of 7831 echoes, `hmax` 13.9 ms. A 10 minute NVS-like load over BLE
(flash_mgmt: 4 KB erase, then 64 writes of 32 bytes) with the echo load in
parallel: 0 disconnects in two runs (a third run, right after a burst of ZMK
Studio CCC writes, lost the link after 235 s with `ptr_skip` 0; its reason was
not captured). Idle cadence, typing path and BLE image upload time are the
same as without the window.

Why not other fixes: the hal's busy-wait loop (`flash_wait_done()`) calls a
BLE hook only for hal_telink's own `CONFIG_BT_B91` controller path, and running
our RX path from inside it would need all of it in RAM while XIP is stopped. A
deeper RX ring does not exist in hardware (the DMA geometry is fixed at 4
entries), and the baseband acks every new packet by itself.

### Return to advertising

When the last link has ended, `ll_radio_adv_restore()` resets the baseband and
re-initializes advertising. The TX read pointer survives the reset (it cannot be
written either), but advertising works anyway once the DMA is no longer
reconfigured at runtime: up to 11 disconnects and restores in one boot, no
reboot, and the host reconnects within seconds. While other links remain,
advertising uses `ll_radio_adv_enter()` instead (no reset).

## Power management

The SoC is not suspended between connection events (that is slice 5b). The link
layer avoids all work it does not need:

### Peripheral latency rules

After an event the next one is planned up to connPeripheralLatency events
ahead (latency from CONNECT_IND or the last applied update; counter and CSA
advance over the skipped events, window widening keeps using real time since
the last sync) when all of these hold, else the next event is listened to:

- nothing queued or unacknowledged in the link's TX queue;
- no LLCP procedure waiting (`ll_llcp_busy()`, including owed PDUs and the
  running procedure timers) and no local termination;
- no **connection update** instant in the candidate window: then nothing is
  skipped until it was listened to (it changes the timing);
- a **channel map** instant in the window ends the skip at the instant event,
  which is listened to with the new map; the events before it are skipped
  with the old map (the map only changes the channel). A re-plan to an earlier
  event of the window (kick, dodge, bump) makes the map instant pending again,
  so it is applied only at its own event. Before this rule, the PC's adaptive
  frequency hopping (a channel map every 2 to 3 s in a noisy environment) made
  the link listen to every event of each window before an instant: 93 to 95 %
  skipped instead of 96.4 %;
- the event just closed re-anchored: never skip after a miss, a late alarm or
  a first packet that was not the anchor;
- **holdoff:** no event is skipped whose anchor lies less than 1 s
  (`LL_CONN_LATENCY_HOLDOFF_MS`) after the connection start, per link, so the
  central's early requests (features, encryption, connection update) are
  answered at once. The holdoff latches once per link; an update to latency
  > 0 later skips at once;
- the listened event stays within last RX + timeout - 2 intervals (never
  limiting with spec-valid parameters).

**Kick.** Every TX push (ACL, LLCP response, encryption PDU) calls
`ll_conn_kick()`. If the planned listen is a skipped-ahead one, it is re-planned
to the first event whose alarm is still in the future (if the arbiter accepts
it), so queued data leaves within one connection interval. An instant inside a
skip window re-plans to the first reachable event as well, and every event up
to the instant is then listened to.

**Instants are judged against the reception event**, not the time of
handling (see [Late instants](#late-instants)). An instant for a skipped event
whose anchor has passed is applied late instead of ending the link.

### Quiet controller thread and deep sleep

The controller thread blocks with `K_FOREVER` and is woken only by events
(queued data PDUs, TX acknowledgements, host ACL and HCI traffic, link
establishment, parameter updates and link ends, LLCP timers, owed PDUs, the
optional stats timer). There is no polling.

`z_sys_poweroff()` calls `b91_bt_controller_poweroff()` first. The open
controller clears both scheduler slots, masks the system timer compare, stops
the radio, clears the RF interrupt state and disables both PLIC sources. Wake
is a cold boot through MCUboot. The blob variant of this hook is a no-op.

### Power counters (mcumgr group 66)

Read only, command 0, read with `reverse/tools/openll_stats.py`:

```bash
reverse/tools/openll_stats.py                 # one read over USB serial
reverse/tools/openll_stats.py -n 6 -i 10      # six reads, 10 s apart, with deltas
reverse/tools/openll_stats.py --ble           # over BLE (needs bleak)
```

| Key | Meaning |
|---|---|
| `up` | uptime ms |
| `idle` | CPU idle ms (`CONFIG_THREAD_RUNTIME_STATS`) |
| `plan`, `listen`, `skip`, `kick`, `ev`, `miss` | sums over the links: listen alarms armed, events listened to, events skipped by latency, TX kicks that re-planned, events issued, events without a CRC-valid packet (plus late alarms, RX flow pauses and flash pauses) |
| `wake` | controller thread wakeups |
| `mv` | battery millivolts (0 if unavailable) |
| `links` | links up |
| `link` | list, one map per link id: `up`, `listen`, `skip`, `coll` (events yielded to the arbiter), `miss` |
| `adv` | `ev`, `slid` (moved into a gap), `drop`, `cut` (10 ms PDU rule), `stuck` |
| `flash` | `win` (windows), `wait`, `force`, `wmax` / `hmax` (longest wait and window, us, maxima), `pause`, `cut`, `fkick`, `abort`, `pskip` (RX DMA ring overruns, must stay 0) |

The tool prints deltas (not for the maxima), idle percentage and the share of
skipped events. `ev`, `miss` and `skip` are counted when planned or closed, so
`skip` may overstate by up to the latency when a link ends. Over BLE the read
itself is traffic (the host raises the link to 7.5 ms, latency 0 for a few
seconds), so use USB for idle measurements, with nothing else on the port.

More counters exist in `ll_conn_stats` (late, collisions, RX flow pauses and
stops with the longest run, flash pauses, late instants and the catch-up
bound) and `ll_radio_stats` (T_IFS buckets, holds, guards, ring pointers). The
periodic stats log line (every 2 s) is opt-in with
`CONFIG_BT_HCI_B91_OPENLL_STATS_LOG=y`; it shares the CDC port with mcumgr and
slows SMP down.

## Measured results

All on the keyboard with a bonded Linux/BlueZ PC (Intel controller), one link
unless noted. Numbers come from the device counters and the nRF sniffer.

### T_IFS

On air (sniffer follow, our response to the event's first central packet,
251 / 2120 load, final code with the CPU hold):

| run | n | <= 150 us | 151 to 152 us | > 152 us | histogram |
|---|---|---|---|---|---|
| 1 | 2844 | 99.6 % | 0.4 % | 0.00 % | 148:1191 149:1633 150:10 151:4 152:6 |
| 2 | 2875 | 99.7 % | 0.3 % | 0.03 % | 148:1257 149:1605 150:4 151:5 152:3 153:1 |

The same code before the CPU hold: 48.6 / 49.0 / 2.36 % and 50.9 / 46.2 /
2.85 %. The device's own TX-timestamp monitor over a 10 minute soak: 99.6 /
0.4 / 0.07 % (n 75 446). The central accepted every response.

### Soaks and typing path

| Run | Result |
|---|---|
| 33 min encrypted, SMP echo every 5 s, 15 / 7.5 ms intervals (slice 2) | 378 / 378 echoes, 0 disconnects, 756 connection updates |
| 15 min, 251-octet echo every 0.5 s (slice 6b) | 1712 / 1712, 0 disconnects |
| 10 min echo soak (slice 6d) | 1150 / 1150, 0 disconnects |
| SMP echo latency, 300 echoes 50 ms apart (slice 7 HEAD) | median 24.5 ms, p95 25.3 ms, max 444 ms (first echo under latency 30, expected: see known limitations) |
| 140 GATT client connects with the switch to 6 / 0 / 42 (slice 7) | 0 x 0x28, one 0x08 (see open items) |

### Idle cadence

| Link parameters | Skipped | Notes |
|---|---|---|
| 12 / 30 / 400 | 96 % (96.3 to 96.4), wake 0.1 per s | ideal 96.8 %; slice 5 baseline 96 to 97 % |
| 12 / 10 / 400 | 90.3 % | ideal 90.9 %; ZMK Studio asks for latency 10 a few seconds after every connect, so this is the usual idle state now; includes the PC's pings every 15 s |

### Interrupt lock times

| | Before (encrypt under the IRQ lock) | After (TX mutex) |
|---|---|---|
| ACL encrypt + push, IRQ lock | 360 to 398 us | 42 to 64 us |
| Longest IRQ lock while connected (slice 2 soak) | 360 to 398 us | 68 to 86 us |

Later runs saw 103 to 113 us under 251-octet load and once 299 us. The longest
locks overall are at connection setup (355 to 383 us, before any traffic) and
at the baseband reset of the return to advertising (206 to 223 us).

### Connection updates

At connect the PC uses 6 / 0 / 42 or 12 / 30 / 400. A few seconds later ZMK
Studio's request moves the link to 12 / 10 / 400; mcumgr's parameter control
moves it to 6 / 0 / 42 during SMP traffic and to 12 / 30 / 400 5 s after it.
The PC also sends LL_CHANNEL_MAP_IND every few seconds (adaptive frequency
hopping). All are applied at their instant.

### Late instants

The PC sends LL_CHANNEL_MAP_IND also right after the switch to 6 / 0 / 42,
received two events after the update instant, with its own instant 6 events
(45 ms) later. LLCP PDUs are handled by the controller thread after
decryption, and cooperative host work held that thread off for 70 to 220 ms at
exactly that moment (a GATT client connect triggers settings stores whose NVS
reads run in the system work queue about 1 s later, when the update instant
comes). Judged against the event counter at handling time, the instant had
passed and the link ended with 0x28 (Instant Passed) in 11 of 28 client
connects.

The spec judges an instant against the event the PDU was received in (Vol 6
Part B 5.5.1: passed when (Instant - connEventCount) mod 65536 >= 32767).
`ll_rxq` keeps that event with every PDU (`ll_rx_pdu.event`), `ll_llcp` hands
it to `ll_conn_update_at()` / `ll_conn_chmap_at()`, and 0x28 is reported only
for an instant before that event. An instant whose event has gone by since the
reception (also one equal to the reception event, which was issued with the
old values) is applied late (`catch_up()` in `ll_conn.c`): the map from the
instant event on, the new timing from the old anchor of the instant event
plus WinOffset (the transmit window repeats every new interval), and the
first event that can still be prepared is listened to, with the instant
event's MUST priority. The catch-up jumps arithmetically (CSA #1 advanced by
`ll_csa1_skip()`, the target from the elapsed time), so its work with the lock
held is at most 4 `open_of()` evaluations however long the stall
(`catch_up_steps_max`). Stepped-over events count as missed, except those
that were latency skips of the planned window (`late_instants` counts the
cases). After the fix: 0 of 140 client connects ended with 0x28 (the late path
was taken in 7 of 20 logged ones, handled 14 to 25 events after the
reception), and none in 5 minutes of CCC toggles with SMP echo load.

### Connect latency (CCC load at boot)

With Zephyr's default `CONFIG_BT_SETTINGS_CCC_LAZY_LOADING=y` the host loads
the peer's CCC values at every connection of a bonded peer
(`settings_load_subtree_direct()`), which scans the whole NVS storage with
interrupt-locked flash reads inside the cooperative BT RX work queue: for
about 1 s per reconnect no lower-priority thread ran, neither our controller
thread nor ZMK's key processing. `conf/openll.conf` sets it to `n` (+312 B
RAM; the values load at boot instead).

| CONNECT_IND to ... (host-side reconnect, n 4 each) | lazy loading (before) | CCC at boot (now) |
|---|---|---|
| LE Connection Complete returned by the host | 1002 ms | 4.5 to 6.4 ms |
| LE LTK Request | 1053 to 1067 ms | 65 to 77 ms |
| Encryption Change | 1098 to 1112 ms | 117 to 125 ms |

The controller's own part is 1.3 ms (CONNECT_IND to the event handed to the
host). Not verified: whether frequent settings writes fill NVS and lengthen
boot-time settings loading.

### Multilink and advertising while connected

`MAX_CONN=3` image, one real central: echo 725 / 725, reconnect after a host
disconnect in 0.8 s. With a test module that starts connectable advertising
5 s after the connection (as ZMK does for a free profile): 736 ADV_IND in 32 s
on the sniffer while connected, a 10 minute echo run with advertising 5463 /
5463, 0 disconnects.

### Late events and stack usage

Under traffic the stimer alarm often starts 150 to 270 us late, mostly because
the USB ISR (higher interrupt priority) is still running; with the 500 us lead
none were skipped in a 10-minute run at 7.5 ms (slice 2), and 2 to 7 per run
under 251-octet echo load. Stack (slice 2 soak, used / size):
openll controller 888 / 2048, BT RX WQ 1056 / 2208, BT LW WQ 1080 / 1408,
logging 824 / 1024, ISR 608 / 2048. `conf/app.conf` raises the log thread
stack to 1024 bytes for both controllers.

### Power (slice 5)

Link at 12 / 30 / 400: we answer every 31st event on air; kick to RX open of
the listen 1.2 to 13.7 ms (below one interval); 30 min soak 60 / 60 echoes,
0 disconnects; deep sleep stops advertising and detaches USB; wake by keypress
is a cold boot, and the bonded host reconnected by itself about 13 s after
boot (measured before the CCC change; a new measurement is pending).

## Hardware findings

These are measured on this board and do not appear in the Telink
documentation. They may help anyone writing a B91 link layer.

- **BRX turnaround.** The BRX command (`0x80140a00 = 0x82`, register sequence of
  hal `rf_start_brx()`) receives and answers in hardware. TX settle 86 us gives
  an on-air T_IFS of 148/149 us, like the blob, as long as the CPU stays off
  the bus during the turnaround. BRX (and RX2TX) answer whatever they receive,
  so they are not used for SCAN_RSP.
- **Chaining and MD.** One BRX command chains RX/TX exchanges while either side
  has MD. Our MD bit comes from the TX FIFO occupancy, not from the buffer.
- **SN/NESN in hardware.** The baseband sets SN and NESN on air. Before every
  BRX it reads SN init and NESN init from `ll_ctrl_1` (`0x80140a03` bits 4 and
  5) for the first exchange of the command. Both must be programmed per
  command: SN init = SN of our last packet, NESN init = SN of the central's last
  new packet XOR 1. With NESN init left at 0, every central packet with SN 1 was
  acked but dropped. `reset_sn_nesn()` (`0x80140a01 = 1`) is not needed per link.
- **5-bit pointers.** The TX FIFO read/write pointers and the RX DMA write
  pointer are 5-bit counters (wrap at 32). 8-bit arithmetic stalled the TX queue
  after the first wrap.
- **Read pointer not resettable.** Writing `0x80` or `0x00` to the TX read
  pointer has no effect. A baseband reset does not reset it either, yet
  advertising works after a connection as long as the DMA geometry is not
  changed at runtime.
- **TX FIFO is read at TX time.** The hardware takes the head entry from RAM
  when it transmits and keeps nothing across commands: the ring can be rebuilt
  for another link between events.
- **STX2RX with a non-empty TX FIFO wedges the FSM** (stays in RX wait after the
  TX; later BRX commands never end; only a baseband reset recovers). Empty the
  FIFO (`wptr = rptr`) before any advertising TX while links exist.
- **`rf_set_rx_dma()` on a live radio freezes the SoC** within microseconds,
  before any fault handler runs (watchdog reset). Configure the RX and TX DMA
  once at boot.
- **DMA entry sizes.** TX ring entries and the RX DMA entry need 272 bytes for
  255-byte packets (16-byte units; the RX DMA writes up to
  `3 + 4 x ceil((len + 13) / 4)`). A central retransmission is written into the
  next RX entry without advancing the write pointer.
- **Timestamps.** The RX DMA trailer timestamp marks the end of the access
  address. With `0x80140830 |= 0x08` the register `0x80140850` holds the TX
  start time, which gives an on-device T_IFS monitor. A scheduled STX starts
  on air trigger + settle + 59 us (TX timestamp at + 5.4 us).
- **Flash operations block everything.** Every hal flash erase/write runs with
  interrupts off and XIP stopped, while the baseband keeps receiving and
  acking into the 4-entry ring.
- **Kernel clock.** The board ran kernel time 6.67 % fast:
  `SYS_CLOCK_TICKS_PER_SEC` 10000 against a 32 kHz mtime gave 3 instead of 3.2
  cycles per tick. Fixed by a tick rate of 32000 (affects every ZMK build). The
  link layer uses only the stimer anyway.

## Known limitations and open items

- **No 2M PHY.** No open register source exists (see
  [PHY](#phy-1m-only)); needs Telink to publish `rf_set_ble_2M_mode()` or give
  permission.
- **Multi-host device test pending** (slice 6a Task 7). Multilink is
  host-tested with up to 3 simulated centrals and device-tested with one real
  central plus advertising while connected. Profile switching with several
  real hosts (Fn+F1..F3), a second central (nRF dongle) and a phone are not
  tested yet.
- **Reconnect after wake not re-measured** with the CCC load at boot (slice 7
  Task 2 item 4; 13 s was measured in slice 5 with lazy loading).
- **Slice 5b: battery and suspend.** The CPU idles but the SoC is not suspended
  between events. An overnight battery comparison with the blob is not done;
  SoC suspend (Zephyr PM states, timer recovery, GPIO wake, RGB DMA, USB) is the
  remaining step if the open build turns out worse.
- **One unexplained 0x08 in 140 client connects:** 5.4 s without any log output
  after a normal connection update, then the supervision timeout. Not seen again
  in 60 runs with a state sampler.
- **Central-to-peripheral data waits for the next listen (expected).** With
  peripheral latency the peripheral listens only every latency + 1 events, so data
  from the central can wait up to (latency + 1) x interval: 465 ms at 12 / 30 / 400
  (15 ms, latency 30). That is the echo maximum of about 444 ms (median 24.5 ms,
  p95 25.3 ms): it is the first echo, sent while latency 30 is in force; mcumgr's
  parameter request then moves the link to 6 / 0 / 42. It is not a host stall.
  The same bound applies to other central-to-peripheral data, for example a Caps
  Lock LED update from the host. Keypresses (peripheral to central) are not
  affected: a TX push kicks the link to the next connection event.
- **Zephyr host warning** "Controller to host flow control not supported" once
  per boot (see [HCI subset](#hci-subset)).
- **HCI LE Connection Update (0x2013) not implemented** although feature bit 1
  formally requires it for a peripheral (C.62); must be done before claiming
  Peripheral-initiated Feature Exchange.
- **Misses.** About 8 % of the listens are misses where the central did not
  transmit; each costs one extra listen (no skip after a miss). In a noisy RF
  environment misses dominate the residual listens.
- **Interrupt latency from outside the link layer.** USB interrupts can still
  delay single connection events. Flash operations skip the events they cover.
- **Cooperative host work** (NVS settings, BT RX work queue) can hold the
  controller thread off for 70 to 220 ms; late instants and RX flow control make
  that safe, but LLCP answers and the first data after it are late.
- **SCAN_RSP:** a SCAN_REQ whose RX interrupt comes too late is not answered
  (`rsp_late` 3.4 % idle, 35 % while connected under load; the scanner retries,
  and the name is in ADV_IND anyway). Settle values below 63 us (50: 1.5 % /
  12 % late) had valid CRCs at the sniffer and were taken by the scanner but are
  not spectrally verified. Each answered SCAN_REQ holds the CPU in the RF
  interrupt for up to about 150 us; with advertising during a loaded connection
  the connection alarm-late counter rose from 17 to 51 in about 75k events, no
  link was lost.
- **CPU hold cost:** up to about 160 us per listened event with a received
  packet (see [CPU hold](#cpu-hold-in-the-turnaround)).
- **One central tested.** Only an Intel controller with Linux/BlueZ. Windows,
  macOS, Android and iOS are untested.
- **Host-tested only:** fresh pairing without an existing bond, the LTK negative
  reply path, HCI Reset during a connection, the masked Connection Parameters
  Request path, privacy reconnect after re-pairing.
- **LL_ENC_REQ on an encrypted link is rejected (0x24)**: encryption pause and
  key refresh are not supported.
- **Long central MD bursts** (SMP image upload) are cut by the guard at the
  interval-based cap or by RX flow control; the central resends the rest in the
  next event.
- **A plaintext LL_TERMINATE_IND between our LTK reply and the central's
  receipt of LL_START_ENC_REQ** fails the MIC (RX decryption is already on), so
  the host sees 0x3D instead of the central's reason. The link ends either way.

## Measuring with the sniffer

### Hardware and setup

- nRF52840 dongle with Nordic's nRF Sniffer for Bluetooth LE firmware.
- `nrfutil` 8.x from Nordic, plus its plugins:
  `nrfutil install device ble-sniffer`. The sniffer plugin brings the dongle
  firmware under `~/.nrfutil/share/nrfutil-ble-sniffer/firmware/`.
- udev rules from `nrfutil device --help-install-udev-rules` must be installed
  on the host (not inside a distrobox): `71-nrf.rules` and
  `99-mm-nrf-blacklist.rules` into `/etc/udev/rules.d/`, then
  `udevadm control --reload`. They apply on ADD events only, so replug the
  dongle afterwards.
- Flash the dongle while it is in the Open DFU bootloader (press the reset
  button, `1915:521f`):

  ```bash
  nrfutil device program --traits nordicDfu \
    --firmware ~/.nrfutil/share/nrfutil-ble-sniffer/firmware/sniffer_nrf52840dongle_nrf52840_4.1.1.zip
  ```

  Afterwards it enumerates as `1915:522a nRF Sniffer for Bluetooth LE`.
- `tshark` / `wireshark` (4.7.3 used) for decoding. `nrfutil ble-sniffer bootstrap`
  installs the Wireshark extcap shim for live GUI capture.

### Capturing advertising

Use `nrfutil` directly. `tshark -i` needs `dumpcap` capabilities via the
`wireshark` group, which did not work inside the distrobox. The `/dev/serial/by-id`
symlink is rejected, so resolve it:

```bash
P=$(readlink -f /dev/serial/by-id/*nRF_Sniffer*)
timeout 30 nrfutil ble-sniffer sniff --port "$P" --output-pcap-file adv.pcapng
```

`nrfutil` always ends with "Subprocess ... failed with unexpected exit code
None" when `timeout` stops it. The file is still fine.

Read the device log at the same time over the CDC ACM console, but never while
mcumgr uses the port:

```bash
timeout 40 cat "$(readlink -f /dev/serial/by-id/*Rainy_75*)"
```

### Following a connection

Following one advertiser is needed to see CONNECT_INDs addressed to it and the
connection after it. `nrfutil ble-sniffer sniff --follow` is unreliable (about once
in nine tries): it only sends the follow request if the device is advertising at
the moment it starts.

What works headlessly is Nordic's Python SnifferAPI, driven by
`reverse/tools/ble_follow_capture.py`. Install the AUR package `nrf-sniffer-ble`
(4.1.1, matches the dongle firmware) in the arch box, for example with
`git clone https://aur.archlinux.org/nrf-sniffer-ble.git && cd nrf-sniffer-ble && makepkg -si`,
then:

```bash
python3 reverse/tools/ble_follow_capture.py --follow XX:XX:XX:XX:XX:XX \
    --seconds 40 --out reverse/captures/conn.pcapng
```

Trigger a reconnect while it runs. `bluetoothctl disconnect` on the bonded host
does not always lead to an automatic reconnect; powering the host adapter off
and on (`bluetoothctl power off`, `power on`) does, within 2 to 6 s.

Notes:

- The dongle buffers packets while no capture runs and delivers them at the
  start of the next capture. Match connections by access address with the
  device log, and drop rows whose timestamps restart.
- The sniffer cannot follow the hop sequence after an encrypted
  LL_CONNECTION_UPDATE_IND, since it cannot decrypt it. For long follows, build
  a test image with `CONFIG_BT_GAP_AUTO_UPDATE_CONN_PARAMS=n` and
  `CONFIG_MCUMGR_TRANSPORT_BT_CONN_PARAM_CONTROL=n`.
- Encrypted control PDUs can only be identified by LLID, length, direction and
  timing (the sniffer has no LTK).
- Live capture with tshark or the Wireshark GUI also works from the arch box via
  `newgrp wireshark` (Arch has no `sg`).

### Sniffer display quirks

- The sniffer rewrites some fields in what it reports: the ChSel bit in ADV_IND
  and CONNECT_IND headers, and ChM, Hop and SCA in CONNECT_IND. The displayed
  bytes then fail the CRC. Reconstruct the real values by searching for the
  combination that makes the CRC match (CRC24, polynomial `0x65B`, init
  `0x555555` on advertising channels).
- For timing between packets use `nordic_ble.delta_time`, which is end to
  start (T_IFS). `nordic_ble.delta_time_ss` is start to start.

### Producing SCAN_REQs

While a bonded host keeps trying to connect, the kernel pauses discovery and
`bluetoothctl scan on` sends no SCAN_REQs at all. What works:
`bluetoothctl block <addr>`, then restart `bluetoothctl --timeout 5 scan on` in
a loop during the capture, then `bluetoothctl unblock <addr>`. To catch the
SCAN_RSP after a captured SCAN_REQ, follow advertisements only on channel 37
with `findScanRsp` (SnifferAPI).

### `ble_adv_report.py`

`reverse/tools/ble_adv_report.py` summarizes the advertising in a capture
(needs `tshark` in `PATH`; a missing or failing tshark is reported as a one-line
error):

```bash
python3 reverse/tools/ble_adv_report.py adv.pcapng --adva xx:xx:xx:xx:xx:xx
```

Output: `per_channel` packet counts, `crc_bad`, `pdu_types`, and
`interval_ms_mean` / `min` / `max`. Notes:

- The CRC flag comes from `nordic_ble.crcok`. tshark 4.x prints it as
  `True`/`False`, older versions as `1`/`0`; the script handles both and treats
  an empty field as good.
- The interval is computed from consecutive good channel 37 packets, so missed
  events inflate it (the sniffer hops and misses some).
- If the sniffer re-syncs during a capture, `frame.time_relative` restarts and
  the minimum interval is bogus.

Tests: `cd reverse/tools && python3 -m unittest test_ble_adv_report test_openll_stats`.

## Host tests

```bash
zmk/drivers/bluetooth/openll/tests/run_host_tests.sh
```

Builds and runs with the host gcc (`-Wall -Wextra -Werror`), 41 binaries: the
per-link suites for `LL_MAX_CONN` 1, 3 and 5, and the LLCP/HCI suites also with
a supported maximum of 27 octets (`_sup27`):

| Test | Covers |
|---|---|
| `test_hci` | Opcode handling, event encoding and masks, parameter validation, unknown opcodes, handles, ACL framing and fragmentation, DLE/PHY/APTO/CPR commands |
| `test_pdu` | PDU encoding (ChSel, TxAdd), SCAN_REQ match, CONNECT_IND parsing against captured bytes |
| `test_scanrsp`, `test_scanrsp_s50` | SCAN_RSP decision and trigger lead for the default and a non-default settle |
| `test_adv` | Advertising state machine against a fake radio: random address, advertising while connected, slicing, flash window |
| `test_arb` | Event arbiter, multilink and advertising scenarios, chains of flash operations against 1 to 3 links |
| `test_csa1`, `test_csa2` | CSA #1 against hand-computed sequences, CSA #2 against the Core Spec sample data |
| `test_crypt` | AES-CCM and session key against the Core Spec sample data (software AES reference) |
| `test_txq`, `test_txq_safe` | TX queue against a fake FIFO implementing the measured hardware model, including forced NACKs, the placeholder case, pointer wrap, other links' events in between, long PDUs |
| `test_fifo` | Record placement and word copies |
| `test_rxq` | RX queue, data-only queuing, decryption, MIC failure, room checks, the reception event |
| `test_conn` | Connection timing (transmit window, widening, anchor rule), latency rules, instants (also late), supervision, termination, RX flow control, DLE spans, flash window |
| `test_llcp` | Every LLCP PDU, encryption start, LENGTH, PHY, Ping, Connection Parameters Request, owed PDUs, lock discipline (no AES under the IRQ lock) |
| `test_credit` | Per-link credits and generations |
| `test_radio_mode` | Advertising register snapshot across restores |

## Roadmap

- **Slice 5b: battery measurement and SoC suspend.** Overnight comparison with
  the blob, then suspend between connection events and 32 kHz RC calibration.
- **Multi-host test** (slice 6a Task 7) and **more centrals:** Windows, macOS,
  Android and iOS hosts.
- **Optional:** HCI LE Connection Update (initiator) and Peripheral-initiated
  Feature Exchange; 2M PHY if a register source becomes available.
- **In parallel:** ask Telink for permission to redistribute the blob (and for
  the 2M register sequence).
