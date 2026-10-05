# Open BLE Controller (openll)

Our own BLE link layer for the Telink B91. It replaces the proprietary
controller blob `liblt_9518_zephyr.a`. Tracking issue:
[#13](https://github.com/scholzri/rainy75-zmk/issues/13).

**Status:** slices 1 to 5 are done. A keyboard running the open controller
advertises, accepts a connection from a bonded host, starts link encryption with
the existing bond and works as a BLE HID keyboard. No blob is linked. A 33-minute
encrypted soak under traffic ended with 0 disconnects. Slice 5 added peripheral
latency, an event-driven controller thread and coordination with deep sleep
(see [Power management](#power-management)). The blob stays the default build
until the battery comparison of slice 5b is done. Only one central has been
tested so far (Intel controller with Linux/BlueZ).

## Why

The firmware links `liblt_9518_zephyr.a`, a prebuilt controller library from
Telink. It is proprietary (see [NOTICE](../NOTICE)), so it is not committed to
this repository and no prebuilt firmware image can be published. Every user has
to build locally, and `fetch_ble_blob.sh` downloads the blob at build time.

The link map shows the blob as the only prebuilt binary in the image. Zephyr,
the BT host, mbedTLS, picolibc, hal_telink (including `rf.c`, `aes.c`, `trng.c`
and `stimer.c`) and our drivers all build from source. The per-chip RF
calibration at flash `0xFE000` is factory data, not code. With the open
controller the firmware can be built and shipped without any binary blob.

## Using it

### Build

```bash
./build.sh -p --iso --openll          # or --ansi
grep -c liblt build/zephyr/zmk.map    # 0 for the open controller, 48 for the blob
```

`--openll` appends `conf/openll.conf` (`CONFIG_BT_HCI_B91_CTLR_OPEN=y`) to the
app config. `fetch_ble_blob.sh` is not run for an `--openll` build.

Deep sleep works with the open controller and needs no extra configuration:
`./build.sh --openll` keeps `CONFIG_ZMK_SLEEP=y` (15 minute idle timeout). The
old advice to build with a local `conf/nosleep.conf` no longer applies.

Image sizes from the "Memory region" summary (ISO, `conf/app.conf`, deep sleep
on, nothing else changed):

| Variant | ROM | RAM | RAM_ILM |
|---|---|---|---|
| Blob (`./build.sh -p --iso`) | 328120 B | 85416 B | 40288 B |
| Open (`./build.sh -p --iso --openll`) | 277700 B | 85988 B | 6236 B |

### Flash over USB (mcumgr)

The same as any other app update (see [zmk-firmware.md](zmk-firmware.md)):

```bash
M="$HOME/go/bin/mcumgr --conntype serial --connstring dev=/dev/ttyACM0,baud=115200"
$M image upload build-openll/zephyr/zmk.signed.bin
$M image list                 # note the hash of the new image in slot 1
$M image test <slot-1 hash>
$M reset
# after the reboot, once BLE typing works:
$M image confirm
```

A bonded host reconnects by itself and keeps its bond: the identity address
(MAC from flash) and the host's bond storage are the same for both controllers.
The boot log shows `open link layer up`, `AES self-test (FIPS-197 C.1): pass`
and, once the host connects, `connected: interval ...`.

### Switching back to the blob

MCUboot keeps the previous image in slot 1. Either `image test <slot-1 hash>`
plus `reset`, or build the blob (`./build.sh -p --iso`) and upload it as above.
The bond survives both ways.

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
| `BT_HCI_B91_CTLR_BLOB` (default) | Telink blob. |
| `BT_HCI_B91_CTLR_OPEN` | Open link layer from `openll/`. The blob is not linked. |

Wiring:

- `zmk/CMakeLists.txt` links `liblt_9518_zephyr.a` only under `BT_HCI_B91_CTLR_BLOB`.
- `zmk/drivers/bluetooth/CMakeLists.txt` builds `hci_b91.c` and `b91_mac.c` for
  both, `b91_bt.c` for the blob, and `openll/*.c` plus hal_telink's `rf.c`,
  `aes.c` (and `trng.c` unless the Zephyr TRNG driver already provides it) for
  the open controller.
- `b91_mac.c` holds the MAC-from-flash logic (read at `0xFF000`, random static
  fallback), shared by both controllers. Both derive the same public address.
- `patches/hal_telink/0002-...` builds hal_telink's `sys.c` unless the blob is
  selected. The blob ships its own `sys_init`, the open controller needs the
  HAL one.

### Files (`zmk/drivers/bluetooth/openll/`)

Everything except `ll_glue.c`, `ll_radio.c` and `ll_sched.c` is pure C without
Zephyr or hardware dependencies and is tested on the host with gcc.

| File | Responsibility |
|---|---|
| `ll_glue.c` | Implements `b91_bt.h`. Init (TRNG, MAC, radio, scheduler, AES self-test), controller thread, ACL in and out, HCI flow control (Number Of Completed Packets), connection and disconnection handling, platform hooks (`ll_plat.h`), periodic statistics |
| `ll_hci.c` / `.h` | HCI command parser and dispatcher, Command Complete/Status builders, all events toward the host (LE Connection Complete, Disconnection Complete, Number Of Completed Packets, LE LTK Request, Encryption Change, LE Connection Update Complete), ACL framing |
| `ll_pdu.c` / `.h` | Advertising PDUs and SCAN_RSP, SCAN_REQ match, CONNECT_IND parsing |
| `ll_scanrsp.h` | SCAN_REQ -> SCAN_RSP decision and STX trigger tick, inline for the RX ISR in RAM |
| `ll_adv.c` / `.h` | Advertising state machine; hands a CONNECT_IND for us to `ll_conn_start()` |
| `ll_conn.c` / `.h` | Connection state machine: transmit window, window widening, anchor re-sync, event counter, CSA #1 channel, instants (connection update, channel map), supervision timeout, termination |
| `ll_csa1.c` / `.h` | Channel Selection Algorithm #1 |
| `ll_txq.c` / `.h` | Software model of the hardware TX FIFO: backlog, ring refill, placeholder rule, SN/NESN init per event, ack detection by read pointer, completion callbacks |
| `ll_rxq.c` / `.h` | RX queue: ISR-filled ring of received data PDUs, decryption in the consumer |
| `ll_llcp.c` / `.h` | Responder LL control procedures, encryption start, the single encrypt-and-push point for all outgoing data PDUs |
| `ll_crypt.c` / `.h` | BLE AES-CCM (encrypt and decrypt one PDU, nonce from packet counter, direction and IV), session key derivation |
| `ll_radio.c` / `.h` | The only RF-touching file. Wraps hal_telink `rf.c`: advertising (STX2RX, STX), connection events (BRX), TX FIFO pointers, RX DMA ring, timestamps, T_IFS monitor, return to advertising, counters |
| `ll_sched.c` / `.h` | One-shot alarm and a guard alarm at absolute system timer (stimer, 16 MHz) ticks, callbacks in ISR context |
| `ll_defs.h` | Shared constants (HCI status codes, PDU types, LLIDs, features, version, ACL buffer sizes, ticks per us) |
| `ll_plat.h` | Platform hooks used by the pure code: random number, IRQ lock, TX mutex, AES-128 block |
| `tests/` | Host tests (see [Host tests](#host-tests)) |

### Execution contexts

- **ISR (RF and stimer interrupts):** `ll_adv`, `ll_conn`, `ll_txq` (refill at
  event start, ack processing at event end) and the producer side of `ll_rxq`.
  The RX ISR copies every received data PDU out of the RX DMA ring before the
  DMA can reuse the entry. Callbacks toward the glue only record what happened
  (pending bits, counters) and wake the controller thread.
- **Controller thread** (`CONFIG_BT_HCI_B91_RX_PRIO`, preemptible): consumes
  `ll_rxq` (decrypt, LLCP to `ll_llcp`, ACL to the host), sends host ACL through
  `ll_llcp_tx()`, emits LE Connection Complete, Connection Update Complete,
  Number Of Completed Packets and Disconnection Complete, and resets `ll_rxq` and
  `ll_llcp` after a disconnect, before advertising may be enabled again.
- **HCI thread** (the host's TX path): HCI commands. Host ACL is only parsed and
  queued here. The LTK reply and HCI Disconnect queue control PDUs from this
  thread.
- **Locks:** `ll_plat_lock()` (interrupts off) protects state shared with the
  ISRs and is held only briefly: around `ll_txq_push()` and around the switch of
  the RX decryption context. `ll_plat_tx_lock()` is a recursive `k_mutex` with
  priority inheritance that serializes the two TX producers (controller thread
  and HCI thread). Encryption and push of one PDU run under it, so the CCM
  packet counter order equals the FIFO order. AES never runs with interrupts
  off. The mutex may block, so it is never taken from an ISR or with the IRQ
  lock held.

### Connection event flow

1. **Alarm** (`ll_sched`), `LL_CONN_ARM_LEAD_US` = 500 us before the RX window
   opens. If the alarm runs less than 120 us before the window, the event is
   skipped (counted as `late`).
2. **Prepare** (`ll_conn`): channel from CSA #1, RX window = anchor - widening -
   margin, first-RX timeout = 2 x (widening + margin) + 40 us (the transmit
   window size is added for window events). `ll_txq_event_start()` programs SN
   and NESN init and refills the ring.
3. **BRX** (`ll_radio`): one BRX command (`0x82`) with TX settle 86 us,
   triggered 80 us (RX settle) before the window opens. The hardware receives
   the central's packet, answers after its own turnaround and keeps chaining
   RX/TX exchanges while either side has MD set.
4. **RX IRQ, per packet:** the PDU goes to `ll_txq_rx()` (SN tracking) and
   `ll_rxq_isr_put()`. Only the event's first packet re-anchors and refreshes
   the supervision timer (see "Anchor rule" below).
5. **Event end:** CMD_DONE or a first-RX timeout. `ll_txq_event_end()` reads the
   read pointer and completes acked entries, the event counter advances,
   supervision and termination are checked, instants are applied and the next
   event is planned. A guard alarm ends any event that has no end IRQ by
   RX open + an interval-based cap (interval minus the widening growth, the
   500 us alarm lead and a 300 us safety, at least the first RX window + 1 ms),
   so a long MD burst of the central may use almost the whole interval but
   never overruns the next event. Three guard-ended events in a row that
   received no valid packet end the link with 0x08 (radio wedged); a guard
   that cuts a burst with packets resets that count.

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
- Otherwise it sends entry `rptr & 3` at base + 64 x (1 + entry). MD on air is
  set by the hardware when more entries wait behind the one being sent. The MD,
  SN and NESN bits in the buffer are ignored.
- An entry that is not acked is resent with the same SN.

`ll_txq` keeps a backlog of 8 PDUs and copies them into the ring only at event
start, never while an event is on air. An entry is complete when the read
pointer has passed it: ACL entries count toward Number Of Completed Packets,
control PDUs notify `ll_llcp` and `ll_conn` (LL_TERMINATE_IND ack).

**Placeholder rule.** At the first RX of a BRX command the hardware judges the
central's ack against the programmed SN init, but it cannot know whether our
last packet was the base buffer or a ring entry. If our last packet was the base
empty PDU, `ll_txq` therefore writes an empty placeholder entry in front of new
data. If the central acked the base, the first pop removes the placeholder
unsent, which is correct. If not, the placeholder is resent with the base's SN
and the same empty content, a correct retransmission. Verified on the device
under forced NACKs (TX settle raised to 102 us, about 18 % of our responses not
received by the central): 2686 of 2686 encrypted SMP echoes matched, about 12000
encrypted data PDUs, 0 MIC failures. On an encrypted link a duplicate or a lost
PDU would have shifted the CCM packet counter and ended the link with 0x3D. A
fallback (`LL_TXQ_SAFE_MODE`, one data PDU per event) exists in the code and is
host-tested, but is not needed.

### RX path

The RX DMA is a 4 x 64 byte ring, configured once at boot. The hardware writes
only new packets into it: a retransmission of the central (SN not the expected
one) is acked through NESN but not written, and its RX IRQ finds no new entry.
Retransmissions are therefore filtered in hardware; there is no software
duplicate check. This was verified under forced NACKs: 10 595 central
retransmissions in one run, none reached software.

The RX ISR copies each CRC-valid data PDU into `ll_rxq`, a 16-entry ring that
holds data PDUs only (empty PDUs are not queued). The controller thread drains
it and decrypts in RX order. A MIC failure ends the link with 0x3D.

An overflow of `ll_rxq` cannot be recovered: the hardware has already acked the
PDU, so the central will never resend it. `ll_conn` then ends the link with
0x08 at the end of the event instead of continuing with a hole in the L2CAP
stream. With data-only queuing no overflow has been seen since; before, empty
PDUs filled the ring while the host processed LE Connection Complete (about
300 ms, see "Measured results").

### LLCP (responder) and encryption

| Received | Action |
|---|---|
| LL_FEATURE_REQ | LL_FEATURE_RSP. Features: LE Encryption, Extended Reject Indication (`0x05`) |
| LL_VERSION_IND | LL_VERSION_IND (version 0x09, company 0xFFFF, subversion 1), once per connection |
| LL_CONNECTION_UPDATE_IND | Applied at the instant with the new transmit window; LE Connection Update Complete if interval, latency or timeout changed |
| LL_CHANNEL_MAP_IND | Applied at the instant. Fewer than 2 used channels end the link with 0x1E |
| LL_TERMINATE_IND | Disconnection Complete with the reason from the PDU |
| LL_ENC_REQ | LL_ENC_RSP (SKDs, IVs) right away, then LE LTK Request to the host. On an already encrypted link (or during the procedure): rejected with 0x24. If LL_ENC_RSP cannot be queued, the link ends with 0x1F |
| LL_START_ENC_RSP | TX encryption on, encrypted LL_START_ENC_RSP, Encryption Change to the host |
| LL_PAUSE_ENC_REQ | Rejected (0x1A), key refresh not supported |
| Unsupported requests (LENGTH, PHY, PING, PERIPHERAL_FEATURE, CONN_PARAM, ...) and known requests with a wrong length | LL_UNKNOWN_RSP |
| Response opcodes (UNKNOWN_RSP, FEATURE_RSP, REJECT_IND, REJECT_EXT_IND, PING_RSP, LENGTH_RSP, PHY_RSP, CONN_PARAM_RSP, PAUSE_ENC_RSP) and LL_START_ENC_RSP outside the procedure | Dropped silently |
| own: HCI Disconnect | LL_TERMINATE_IND, Disconnection Complete (0x16) after the ack or the supervision timeout |

Encryption start (Core Spec Vol 6 Part B 5.1.3.1), peripheral side: on the
host's LTK reply the session key `SK = e(LTK, SKDs || SKDm)` is derived, RX
decryption is switched on and LL_START_ENC_REQ is queued in plaintext. The
central's LL_START_ENC_RSP arrives encrypted and turns TX encryption on. A
negative reply sends LL_REJECT_EXT_IND (or LL_REJECT_IND if the central lacks
Extended Reject) with 0x06. The LLCP response timeout is 40 s (0x22).

`ll_crypt` implements AES-CCM in software on top of the hardware AES-128 block
of the B91 (hal `aes_encrypt()`). It reproduces the Core Spec sample data
(Vol 6 Part C) in the host tests, and the glue runs a FIPS-197 C.1 self-test at
every boot. One AES block takes 31 to 43 us on the device; a 27-byte PDU needs
a few blocks.

### HCI subset

Reset, Set Event Mask, LE Set Event Mask, Read Local Version Information, Read
Local Supported Commands, Read Local Supported Features, LE Read Local Supported
Features, LE Read Buffer Size (27 bytes, 3 packets), Read BD_ADDR, LE Rand
(hardware TRNG), LE Set Advertising Parameters, LE Set Advertising Data, LE Set
Scan Response Data, LE Set Advertising Enable, Disconnect, LE Long Term Key
Request Reply and Negative Reply.

Invalid parameters return 0x12, unknown opcodes 0x01 (logged once per opcode).
The supported-commands bitmap was cross-checked against Zephyr's `ll_sw`
`hci.c`. The feature bits advertise no 2M PHY, no Data Length Extension, no LL
privacy and no extended advertising, so the host does not try them. ACL toward
the host uses connection handle 0x0000.

### Return to advertising

After a disconnect, `ll_radio_adv_restore()` resets the baseband and
re-initializes advertising. The TX read pointer survives the reset (it cannot be
written either), but advertising works anyway once the DMA is no longer
reconfigured at runtime: up to 11 disconnects and restores in one boot, no
reboot, and the host reconnects within seconds.

## Measured results

All on the keyboard with a bonded Linux/BlueZ PC (Intel controller). Numbers
come from the device statistics (logged every 2 s) and the nRF sniffer.

### T_IFS

On air, sniffer, one connection, n = 542, 0 CRC errors:

| T_IFS | 148 us | 149 us | 150 us | 151 us | 152 us | 153 us | 154 us |
|---|---|---|---|---|---|---|---|
| share | 15.3 % | 46.5 % | 24.9 % | 5.4 % | 5.0 % | 2.8 % | 0.2 % |

86.7 % at or below 150 us, 3.0 % above 152 us. The blob shows the same mode
(148/149 us). The device's own estimate from the TX timestamp reads about 1 us
higher than the sniffer; over the 33-minute soak it gave 71.7 % at or below
150 us, 27.5 % at 151 to 152 us and 1.3 % above. The central accepted every
response.

### Soak

33 minutes connected, encrypted, with an SMP echo every 5 s (ACL in both
directions, multi-PDU L2CAP, MD chains) and the host switching between 15 ms
and 7.5 ms intervals:

| Counter | Value |
|---|---|
| disconnects | 0 |
| connection events | 233 628 |
| events with RX | 217 813 |
| missed (central did not send) | 15 815 |
| late / guard / CRC errors / RX queue overflow | 0 / 0 / 0 / 0 |
| window widening max | 43 us |
| connection updates applied | 756 |
| SMP echoes matched | 378 / 378 |

### Interrupt lock times

| | Before (encrypt under the IRQ lock) | After (TX mutex) |
|---|---|---|
| ACL encrypt + push, IRQ lock | 360 to 398 us | 42 to 55 us |
| Longest IRQ lock while connected | 360 to 398 us | 68 to 86 us |

The longest lock overall, 206 to 223 us, is the baseband reset at the return to
advertising, while no connection exists.

### Connection updates

The PC sends LL_CONNECTION_UPDATE_IND about 5 s after every connect (parameters
unchanged, interval 12, latency 30, timeout 400; only the anchor shifts by the
window offset). During GATT/SMP activity the host asks for 7.5 ms, latency 0,
and later returns to 15 ms. Both are applied at their instant; at 7.5 ms the
link runs at 133 events per second with an RX in almost every event.

### Late events

Under traffic the stimer alarm often starts 150 to 270 us late, mostly because
the USB ISR (higher interrupt priority) is still running. With a 300 us lead
about 0.4 % of the events at 7.5 ms were skipped; with 500 us, 0 in a 10-minute
run. Flash writes (bond, CCC and settings storage after a connect) disable
interrupts for up to 3.8 ms and can still skip a single event; the central
covers that with a retransmission.

### Stack usage (soak, used / size)

| Thread | Bytes |
|---|---|
| openll controller | 888 / 2048 |
| BT RX WQ | 1056 / 2208 |
| BT LW WQ | 1080 / 1408 |
| logging | 824 / 1024 |
| ISR | 608 / 2048 |

`conf/app.conf` raises the log thread stack to 1024 bytes for both controllers
(the default 768 was 99 % used).

## Power management

Slice 5 does not suspend the SoC between connection events (that is slice 5b,
below). It removes everything that made the CPU or the radio work for nothing:

- **Peripheral latency.** The latency from CONNECT_IND or from the last
  connection update is honoured. After an event that re-anchored, the planner
  skips up to `latency` events and arms the next listen directly (counter and
  CSA #1 channel advance over the skipped events). Skipping is refused while
  there is queued or unacknowledged TX data, while an LLCP or encryption
  procedure runs (`ll_llcp_busy()`), while a local terminate runs, after a
  missed or late event, and while a channel map or connection update instant
  lies within the skip window. Window widening keeps using real time since the
  last sync.
- **Kick.** Every successful TX push (ACL data, LLCP response, encryption PDU)
  calls `ll_conn_kick()`. If the planned listen is a skipped-ahead one, it is
  re-planned to the first event whose alarm is still in the future, so queued
  data leaves within one connection interval instead of waiting out the skip
  window. Kicks during an event are no-ops: the next plan sees the backlog.
  A connection update or channel map instant that lands in the skip window
  re-plans to the first reachable event as well, and every event up to the
  instant is then listened to, so a kick never waits for an instant.
- **Quiet controller thread.** The thread blocks with `K_FOREVER` and is woken
  only by events (queued data PDUs, a CONNECT_IND log entry, TX
  acknowledgements, host ACL and HCI traffic, link establishment, parameter
  updates and link ends, the host's LTK negative reply, HCI Reset, and the
  LLCP and optional stats timers). The 40 s LLCP response timeout is a
  delayable work item that exists only while a procedure is pending. There is
  no polling.
- **Deep sleep.** `z_sys_poweroff()` calls `b91_bt_controller_poweroff()`
  first. The open controller clears both scheduler slots, masks the system
  timer compare, stops the radio, clears the RF interrupt state and disables
  both PLIC sources, all interrupt-lock safe. Wake is a cold boot through
  MCUboot as before. The blob variant of this hook is a no-op.

### Power counters

The controller counts planned listens, skipped events, kicks, misses and
controller thread wakeups. They are exposed through the custom mcumgr group 66
(command 0, read only) and read with `reverse/tools/openll_stats.py`:

```bash
reverse/tools/openll_stats.py                 # one read over USB serial
reverse/tools/openll_stats.py -n 6 -i 10      # six reads, 10 s apart, with deltas
reverse/tools/openll_stats.py --ble           # over BLE (needs bleak)
```

Reply fields: `up` (ms), `idle` (CPU idle ms, from
`CONFIG_THREAD_RUNTIME_STATS`), `plan` (listen alarms armed), `listen`
(events listened to), `skip` (events skipped by latency), `kick`, `ev`, `miss`
(events without any CRC-valid packet, plus late alarms), `wake` (controller
thread wakeups) and `mv` (battery millivolts, 0 if unavailable). The tool prints deltas, idle
percentage and the share of skipped events. `ev`, `miss` and `skip` are counted
when planned or closed, so `skip` may overstate by up to the latency when a
link ends. Over BLE the read itself is traffic: the host raises the link to
7.5 ms with latency 0 for a few seconds, so use USB for idle measurements.

The periodic stats log line (every 2 s) is opt-in with
`CONFIG_BT_HCI_B91_OPENLL_STATS_LOG=y`; it adds the planned, listened, skipped
and kick counts.

### Measured results (power)

All on the keyboard with the bonded Linux/BlueZ PC, link at interval 12
(15 ms), latency 30, timeout 400.

| Check | Result |
|---|---|
| Idle cadence on air (sniffer, 24 s window) | we answer every 31st event (steps of 31, 62, 93) |
| Idle skipped events (USB counters, 10 to 20 s windows) | 96 to 97 % skipped, controller wakeups 0.0 to 0.2 per s |
| CPU idle share | 91.8 % shortly after boot, 94.1 % after 30 min |
| Kick to RX open of the listen | 1.2 to 13.7 ms, always below one interval; the data PDU is on air in that event |
| 30 min connected soak, SMP echo every 30 s | 60/60 echoes, 0 disconnects, 120 connection updates |
| Deep sleep | USB detached and advertising stopped after the idle timeout (0 ADV packets from the keyboard in a 45 s scan) |
| Wake by keypress | cold boot, bonded host reconnected by itself about 13 s after boot (about 14 s after the key press), encrypted echo 10/10 |

### Slice 5b (not done)

- An overnight battery comparison, blob versus open controller, with RGB off,
  logging `mv` and the counters over BLE. If the open build is clearly worse,
  the counters show where.
- SoC suspend between connection events (Zephyr PM states for the B91,
  `pm_policy_event` hook from the link layer). This is where the remaining
  battery gap to the blob would be closed. Risks are suspend reliability on
  the chip revision, system timer recovery, GPIO wake, RGB DMA and USB.

## Hardware findings

These are measured on this board and do not appear in the Telink
documentation. They may help anyone writing a B91 link layer.

- **BRX turnaround.** The BRX command (`0x80140a00 = 0x82`, register sequence of
  hal `rf_start_brx()`) receives and answers in hardware. TX settle 86 us gives
  an on-air T_IFS of 148/149 us, like the blob. BRX (and RX2TX) answer
  whatever they receive, so they are not used for SCAN_RSP (see "SCAN_RSP
  timing" below).
- **Chaining and MD.** One BRX command chains RX/TX exchanges while either side
  has MD. Our MD bit comes from the TX FIFO occupancy, not from the buffer.
- **SN/NESN in hardware.** The baseband sets SN and NESN on air. Before every
  BRX it reads SN init and NESN init from `ll_ctrl_1` (`0x80140a03` bits 4 and
  5) for the first exchange of the command. Both must be programmed per
  command: SN init = SN of our last packet, NESN init = SN of the central's last
  new packet XOR 1. With NESN init left at 0, every central packet with SN 1 was
  acked but dropped. `reset_sn_nesn()` (`0x80140a01 = 1`) once per connection.
- **5-bit pointers.** The TX FIFO read/write pointers and the RX DMA write
  pointer are 5-bit counters (wrap at 32). 8-bit arithmetic stalled the TX queue
  after the first wrap.
- **Read pointer not resettable.** Writing `0x80` or `0x00` to the TX read
  pointer has no effect. A baseband reset does not reset it either, yet
  advertising works after a connection as long as the DMA geometry is not
  changed at runtime.
- **`rf_set_rx_dma()` on a live radio freezes the SoC** within microseconds,
  before any fault handler runs (watchdog reset). Configure the RX and TX DMA
  once at boot. The mechanism is not understood, only avoided.
- **Timestamps.** The RX DMA trailer timestamp marks the end of the access
  address. With `0x80140830 |= 0x08` the register `0x80140850` holds the TX
  start time, which gives an on-device T_IFS monitor.
- **Kernel clock.** The board ran kernel time 6.67 % fast:
  `SYS_CLOCK_TICKS_PER_SEC` 10000 against a 32 kHz mtime gave 3 instead of 3.2
  cycles per tick. Fixed by a tick rate of 32000 (affects every ZMK build:
  debounce, tap-hold and sleep timeouts were 6.25 % short). The link layer uses
  only the stimer anyway.

## Known limitations

- **No SoC suspend between events (slice 5b).** The CPU idles but the SoC is
  not suspended, so battery life is likely still shorter than with the blob.
  The overnight comparison is not done yet.
- **About 8 % of the listens are misses** (the central did not transmit in
  that event, for example the two events after each echo response). Each miss
  costs one extra listen because there is no skip after a miss.
- **Latency applies from event 1 after connect.** Early LLCP or GATT requests
  from the central, before we have anything to send, can wait up to one skip
  window (up to 31 x 15 ms = 465 ms). A kick only helps when we have TX data.
- **Interrupt latency from outside the link layer.** USB interrupts and flash
  writes with interrupts off can still skip single connection events.
- **A reconnect to a bonded host takes about 1.1 s until encryption, on the
  host side.** On LE Connection Complete the Zephyr host loads the peer's CCC
  values from settings (`CONFIG_BT_SETTINGS_CCC_LAZY_LOADING`, default y).
  That scans the whole NVS storage, every read with interrupts off, inside the
  cooperative BT RX work queue, so no thread runs for about 1.07 s (PC
  sampling: 99.8 % `bt_workq` in `settings_nvs_load` and the NVS ATE reads).
  Our controller thread, which delivered the event, answers the central's
  LL_FEATURE_REQ only then (on air 1.09 s after CONNECT_IND, encryption at
  1.15 s). The controller's own part is 1.3 ms (CONNECT_IND to the event
  handed to the host). With `CONFIG_BT_SETTINGS_CCC_LAZY_LOADING=n` (CCCs
  loaded at boot, +312 B RAM) the host returns after 6 ms and encryption is
  on after about 0.12 s. The time grows with the NVS contents (about 0.3 s in
  slice 2).
- **Not supported:** CSA #2 (we advertise ChSel 0, so the central uses CSA #1),
  2M and Coded PHY, Data Length Extension, LL privacy (LE Set Random Address,
  RPA), LE Ping (answered with LL_UNKNOWN_RSP, which the tested central
  accepts), encryption pause and key refresh, multiple connections.
- **A SCAN_REQ whose RX interrupt comes too late is not answered** (about 1 %
  in the measurement, counted in `rsp_late`); the scanner retries. Discovery
  is not affected: the name is in ADV_IND.
- **About 3 % of the responses have a T_IFS above 152 us** (first exchange of
  an event, cause unknown). The tested central accepts them; stricter centrals
  might not.
- **One central tested.** Only an Intel controller with Linux/BlueZ. Other
  operating systems and controllers are untested.
- **Host-tested only, not exercised on the device:** fresh pairing (SMP
  without an existing bond; the device tests used the bonded PC), the LTK
  negative reply path, and HCI Reset during a connection.
- **LL_ENC_REQ on an encrypted link is rejected (0x24)**: encryption pause and
  key refresh are not supported.
- **No advertising while connected.** LE Set Advertising Enable is refused while
  a connection exists (one connection only), so ZMK profile switching to a
  second host does not advertise until the current link is gone.
- **Long central MD bursts** (SMP image upload, rgb_mgmt writes) are cut by the
  guard at the interval-based cap; the central resends the rest in the next
  event. Such cut events do not count toward the radio-wedge rule.
- **A plaintext LL_TERMINATE_IND between our LTK reply and the central's
  receipt of LL_START_ENC_REQ** fails the MIC (RX decryption is already on), so
  the host sees 0x3D instead of the central's reason. The link ends either way.
- After LE Connection Complete the Zephyr host's cooperative RX work queue holds
  the CPU for about 300 ms, so our first LLCP answers leave that late. Harmless
  (the central allows 40 s).

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

Read the device log at the same time over the CDC ACM console (do not run
mcumgr while reading):

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
  device log (`CONNECT_IND ... AA 0x...`).
- The sniffer cannot follow the hop sequence after an encrypted
  LL_CONNECTION_UPDATE_IND, since it cannot decrypt it.
- Our responses with a T_IFS well above 152 us fall outside the sniffer's
  capture window and do not appear in the capture at all.
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
a loop during the capture, then `bluetoothctl unblock <addr>`.

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

Tests: `cd reverse/tools && python3 -m unittest test_ble_adv_report`.

## Host tests

```bash
zmk/drivers/bluetooth/openll/tests/run_host_tests.sh
```

Builds and runs with the host gcc (`-Wall -Wextra -Werror`):

| Test | Covers |
|---|---|
| `test_hci` | Opcode handling, event encoding, parameter validation, unknown opcodes, connection events, ACL framing |
| `test_pdu` | PDU encoding, SCAN_REQ match, CONNECT_IND parsing against captured bytes |
| `test_adv` | Advertising state machine against a fake radio |
| `test_csa1` | CSA #1 against hand-computed sequences |
| `test_crypt` | AES-CCM and session key against the Core Spec sample data (software AES reference) |
| `test_txq`, `test_txq_safe` | TX queue against a fake FIFO implementing the measured hardware model, including forced NACKs, the placeholder case and pointer wrap |
| `test_rxq` | RX queue, data-only queuing, decryption, MIC failure |
| `test_conn` | Connection timing (transmit window, widening, anchor rule), instants, supervision, termination, RX queue loss |
| `test_llcp` | Every LLCP PDU, encryption start, lock discipline (no AES under the IRQ lock) |

## SCAN_RSP timing (slice 7)

The RX interrupt answers a SCAN_REQ itself, before the link layer callback:
a CRC-valid SCAN_REQ for the AdvA and TxAdd of the prepared SCAN_RSP gets a
single scheduled TX (`rf_start_stx()`, TX settle 50 us) triggered 41 us after
the request's end, so the first bit is on air 150 us after the request
(trigger + settle + a fixed 59 us TX path delay, measured). The decision and
the trigger run from RAM (`.ram_code`, `ll_scanrsp.h` inline): through the
callback chain, or with flash-resident helpers, the decision came 50 to 90 us
after the request, too late. The CPU then holds on the cycle counter until the
response is on air, which removes a 2 to 5 us jitter of the TX start (as in
the connection turnaround). A decision later than 3 us before the trigger is
not answered (`rsp_late`). Nothing else is ever answered.

Why not the hardware turnaround: BRX and RX2TX transmit after whatever they
receive (a CONNECT_IND, another advertiser's SCAN_REQ). Only a CPU veto in the
RX interrupt could stop that, and the interrupt can be held off by
interrupt-locked sections (a flash erase takes 13 to 26 ms), so the controller
would sometimes answer a packet it must not answer. The scheduled TX fails
safe: no answer unless the CPU decided in time.

Before slice 7 the RX interrupt reached the decision through the link layer
callbacks 60 to 90 us after the request and triggered with the hal's settle of
78 us: on air at about 209 us, outside the scanner's window (the tested Intel
scanner never took one; it takes the 150 us responses).

## Roadmap

- **Slice 5: power management (done).** Peripheral latency, kick, quiet
  controller thread, power counters, deep sleep coordination.
- **Slice 5b: battery measurement and SoC suspend.** Overnight comparison with
  the blob, then suspend between connection events and 32 kHz RC calibration.
- **Slice 6: privacy and extras.** LE Set Random Address and RPA (the blob hangs
  on this today), optional 2M PHY, Data Length Extension, CSA #2.
- **More centrals:** test with Windows, macOS, Android and iOS hosts.
- **In parallel:** ask Telink for permission to redistribute the blob.
