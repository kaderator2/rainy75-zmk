# Open BLE Controller (openll)

Our own BLE link layer for the Telink B91, replacing the proprietary controller
blob `liblt_9518_zephyr.a`. Tracking issue: [#13](https://github.com/scholzri/rainy75-zmk/issues/13).

This is slice 1 of a multi-slice project. Slice 1 brings up Zephyr's Bluetooth
host (`bt_enable()` succeeds) and does legacy connectable advertising on the
open controller. It does not follow connections yet, so a keyboard running the
open controller cannot be used over BLE. The blob stays the default build.

## Why

The firmware links `liblt_9518_zephyr.a`, a prebuilt controller library from
Telink. It is proprietary (see [NOTICE](../NOTICE)), so it is not committed to
this repository and no prebuilt firmware image can be published. Every user has
to build locally, and `fetch_ble_blob.sh` downloads the blob at build time.

The link map shows the blob as the only prebuilt binary in the image. Zephyr,
the BT host, mbedTLS, picolibc, hal_telink (including `rf.c`, `trng.c`,
`stimer.c`) and our drivers all build from source. The per-chip RF calibration
at flash `0xFE000` is factory data, not code. Once the open controller can do
everything the keyboard needs, the firmware can be built and shipped without
any binary blob.

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
| `BT_HCI_B91_CTLR_BLOB` (default) | Telink blob, full peripheral feature set. Unchanged behaviour. |
| `BT_HCI_B91_CTLR_OPEN` | Open link layer from `openll/`. The blob is not linked. |

Wiring:

- `zmk/CMakeLists.txt` links `liblt_9518_zephyr.a` only under `BT_HCI_B91_CTLR_BLOB`.
- `zmk/drivers/bluetooth/CMakeLists.txt` builds `hci_b91.c` and `b91_mac.c` for
  both, `b91_bt.c` for the blob, and `openll/*.c` plus hal_telink's `rf.c` (and
  `trng.c` unless the Zephyr TRNG driver already provides it) for the open
  controller.
- `b91_mac.c` holds the MAC-from-flash logic (read at `0xFF000`, random static
  fallback), shared by both controllers. Both derive the same public address.
- `patches/hal_telink/0002-...` builds hal_telink's `sys.c` unless the blob is
  selected. The blob ships its own `sys_init`, the open controller needs the
  HAL one.
- `conf/openll.conf` contains only `CONFIG_BT_HCI_B91_CTLR_OPEN=y`.

### Files (`zmk/drivers/bluetooth/openll/`)

| File | Responsibility | Touches hardware |
|---|---|---|
| `ll_glue.c` | Implements `b91_bt.h`. Init (TRNG, MAC, radio, scheduler), controller thread, event queue to the host, CONNECT_IND logging, periodic radio stats | no (Zephyr glue) |
| `ll_hci.c` / `.h` | HCI command parser and dispatcher, Command Complete builders, parameter validation. Pure logic: actions go through `struct ll_hci_ops`, events leave through a sink as H4 packets | no, host-tested |
| `ll_pdu.c` / `.h` | Build ADV_IND / ADV_SCAN_IND / ADV_NONCONN_IND and SCAN_RSP, match SCAN_REQ, parse CONNECT_IND into `struct ll_connect_ind` | no, host-tested |
| `ll_adv.c` / `.h` | Advertising state machine: interval plus 0..10 ms advDelay, channels 37 -> 38 -> 39, TX then a 300 us RX window per channel, SCAN_REQ and CONNECT_IND handling | no, host-tested against a fake radio |
| `ll_sched.c` / `.h` | One one-shot alarm at an absolute system timer tick (16 MHz), callback in ISR context | stimer |
| `ll_radio.c` / `.h` | The only RF-touching file. Wraps hal_telink `rf.c`: BLE 1M mode, advertising access address and CRC init, channel and whitening, TX/RX DMA buffers, RF IRQ, RX timestamps, counters | RF |
| `ll_defs.h` | Shared constants (HCI status codes, PDU types, version and company ID, ACL buffer sizes, ticks per us, T_IFS) | no |
| `ll_plat.h` | Platform hooks used by the pure code: random number, IRQ lock/unlock | no |
| `tests/` | Host gcc tests `test_hci.c`, `test_pdu.c`, `test_adv.c` and `run_host_tests.sh` | no |

### Interfaces

- **Radio** (`ll_radio.h`): `ll_radio_set_adv_channel(ch)`,
  `ll_radio_tx_then_rx(pdu, len, start_tick, rx_window_us)` (hardware TX then
  RX, `rf_start_stx2rx`), `ll_radio_prepare_rsp()` plus `ll_radio_tx_rsp_at(tick)`
  for SCAN_RSP (returns false and sends nothing when the TX trigger would be
  less than 10 us in the future; the advertiser then moves to the next channel
  at once), `ll_radio_stop()`, `ll_radio_now()`. Completions come back
  through one callback in ISR context with `TX_DONE`, `RX_OK` (PDU, length and
  the tick at the end of the packet), `RX_TIMEOUT` or `RX_CRC_ERR`.
- **Scheduler** (`ll_sched.h`): `ll_sched_at(tick, cb)` and `ll_sched_cancel()`.
- **HCI** (`ll_hci.h`): `ll_hci_init(ops, sink)` and `ll_hci_cmd(cmd, len)`.
- **Advertising** (`ll_adv.h`): parameter, data and enable setters called by
  the HCI layer, `ll_adv_radio_evt()` registered as the radio callback, and a
  connect callback that receives every parsed CONNECT_IND.

### Data flow and threading

- Host to controller: `hci_b91_send()` -> `b91_bt_host_send_packet()` ->
  `ll_hci_cmd()` -> state changes in `ll_adv`. ACL packets are dropped with a
  warning (no connections in slice 1).
- Controller to host: events go into a `k_msgq`. The controller thread
  delivers them through the registered `host_read_packet` callback, the same
  thread context the blob shim uses.
- Radio and scheduler callbacks run in ISR context. A parsed CONNECT_IND is
  pushed into a second queue and logged from the controller thread.
- The controller thread polls the event queue with a 100 ms timeout and logs
  the radio counters every 2 s.

### HCI subset

Exactly the commands `bt_enable()` plus legacy advertising need: Reset, Set
Event Mask, LE Set Event Mask, Read Local Version Information, Read Local
Supported Commands, Read Local Supported Features, LE Read Local Supported
Features, LE Read Buffer Size, Read BD_ADDR, LE Rand (hardware TRNG), LE Set
Advertising Parameters, LE Set Advertising Data, LE Set Scan Response Data, LE
Set Advertising Enable.

Invalid parameters return status `0x12`. Unknown opcodes return `0x01` and are
logged once per opcode. The supported-commands bitmap was cross-checked against
Zephyr's `ll_sw` `hci.c`. The feature bits advertise no 2M PHY, no Data Length
Extension, no LL privacy and no extended advertising, so the host does not try
them. The controller reports HCI version 5.0, manufacturer `0xffff`.

### Robustness measures

Added before the first hardware run, since a radio bug shows up as silence:

- `rx_buf` is 288 bytes, large enough for a full 255-byte noise packet plus DMA
  header and trailer, and that size is passed to `rf_set_rx_dma()` (hardware
  enforcement of the max length is unverified).
- `ll_adv` ignores radio events that arrive after an advertising event ended
  (no double scheduling), re-bases the next event on "now" after a stall instead
  of firing a burst of late events, and rejects NULL data with a nonzero length.
- `ll_radio` counts `tx2rx`, `rx_ok`, `crc`, `timeout`, `rsp` (SCAN_RSP TX
  triggered, not confirmed on air) and `rsp_late` (SCAN_RSP refused because the
  TX trigger tick was less than 10 us ahead or already past; a trigger in the
  past would leave the radio waiting and stall advertising). `ll_glue` logs them
  every 2 s as `radio: tx2rx N rx_ok N crc N timeout N rsp N rsp_late N` and
  warns `radio stalled` when advertising is enabled but `tx2rx` does not
  advance.
- Oversized events are dropped with an error, dropped CONNECT_INDs are counted.
- CONNECT_IND logging is rate limited: the first one and then at most one full
  dump every 10 s, with `N since last dump`. Without this the log flood
  overflowed the log buffer and made mcumgr time out on the shared CDC ACM port.

## Building

```bash
./build.sh -p --iso --openll      # or --ansi
grep -c liblt build/zephyr/zmk.map   # 0 for the open controller, 48 for the blob
```

`--openll` appends `conf/openll.conf` to the app config. Flash with mcumgr as
usual (`image upload`, `image test`, `reset`). MCUboot keeps the previous image
in slot 1, so going back to the blob is `image test <slot-1 hash>` plus
`reset`, or a fresh `./build.sh -p --iso` and upload.

Image sizes from the "Memory region" summary of the final builds (ISO,
`conf/app.conf`, deep sleep on, nothing else changed):

| Variant | ROM | RAM | RAM_ILM |
|---|---|---|---|
| Blob (`./build.sh -p --iso`) | 328088 B | 85160 B | 40288 B |
| Open (`./build.sh -p --iso --openll`) | 263396 B | 83416 B | 6204 B |

Practical notes:

- With the open controller the keyboard advertises but cannot hold a
  connection, so a bonded host keeps retrying (several CONNECT_INDs per second)
  and BLE typing does not work. Use USB.
- The default config enables deep sleep after 15 minutes, which stops
  advertising until a keypress. For long sniffer sessions build with
  `CONFIG_ZMK_SLEEP=n` in an extra config file (for example by adding it to
  `EXTRA_CONF_FILE` in a manual `west build`).
- If `ZEPHYR_TOOLCHAIN_VARIANT` is set but empty in your environment, the
  build fails to find the toolchain. `export ZEPHYR_TOOLCHAIN_VARIANT=zephyr`
  first.

## Status (slice 1)

Measured on the keyboard on 2026-10-02.

| Check | Result |
|---|---|
| Blob not linked | Pass. `grep -c liblt zmk.map` = 0 (blob build: 48). |
| `bt_enable()` | Pass on first flash. Identity address matches the MAC from flash. Only unsupported opcode seen: `0xfc01` (Zephyr vendor Read Version Info, warning only). |
| ADV_IND on air | Pass. Channels 37/38/39, CRC valid (CRC24 recomputed independently). 0 CRC errors in one 30 s capture, 6 isolated ones (0.6 %) in another, with the other channels of the same events good. AdvData, AdvA and length byte-identical to the blob. |
| Advertising interval | 100 ms + 0..10 ms advDelay: mean 105.07 ms, min 99.80, max 110.00 over 228 single intervals. ZMK requests 100..150 ms; the blob picks 150 ms (mean 152.75 ms). Both are valid. |
| Channel step inside an event | 0.906 ms (blob 0.725 ms). Each ADV_IND is about 376 us on air. |
| Scanners see the device | Pass. `bluetoothctl` lists "Rainy 75 Pro". The name is in ADV_IND (ZMK's scan response is empty), so this does not depend on SCAN_RSP. |
| SCAN_RSP | Partial. On air with valid CRC, but T_IFS is about 209 us instead of 150 +/- 2 us. See below. |
| CONNECT_IND parsing | Pass on all fields observable so far. The bonded PC sends about 2.6 CONNECT_INDs per second and every one is decoded. Static fields match the sniffer capture (after CRC reconstruction, see below). A capture of the same CONNECT_IND by both sniffer and device log (matched Access Address) is still missing because follow mode is unreliable. |
| Blob build regression | Default blob build flashed after the work and reconnects to the bonded host by itself. Confirmation that BLE typing works is pending. |

Fields decoded by the device and seen on air, from a Linux/BlueZ central:
Interval 12 (15 ms), Latency 30, Timeout 400 (4 s), WinSize 1, WinOffset
0..11, ChM `ff ff ff ff 1f`, Hop 5..15, SCA 1, ChSel 1, with a fresh Access
Address and CRCInit on every attempt.

Radio counters after about 7 minutes with the bonded PC retrying (before
`rsp_late` existed):
`tx2rx 10076 rx_ok 1114 crc 87 timeout 8875 rsp 5`. Most RX windows time out,
which is normal for advertising. RX CRC errors are mostly other devices'
packets or collisions in our window.

### SCAN_RSP turnaround spike

T_IFS (150 us between the end of SCAN_REQ and the start of SCAN_RSP) was the
main technical risk of slice 1. The current code is software timed: the RX IRQ
parses the SCAN_REQ and starts a single TX with `rf_start_stx()` at
`packet end + 150 us - 78 us` TX settle.

Result: SCAN_RSP is on air with a valid CRC, but starts about **209 us** after
the SCAN_REQ (about 59 us late). Measured indirectly with the sniffer: SCAN_REQ
end to the next ADV on channel 38 is 577 us, SCAN_RSP start to the next ADV
start is 368 us in both captured responses, so 577 - 368 = 209 us.

Why it cannot be fixed in software:

- The RX IRQ reaches the ISR 19..57 us after the packet end, and the TX trigger
  is programmed about 46..75 us after the packet end.
- Air start = trigger tick + 78 us TX settle + about 59 us fixed TX path delay.
  To get 150 us the trigger would have to be at packet end + 13 us. Pulling the
  trigger earlier (iteration 1, -59 us) put the start tick in the past.
- Iteration 2 tried the hardware RX->TX turnaround (FSM `0x88` rx2tx with the
  SCAN_RSP preloaded). Advertising kept running and SCAN_REQs were matched, but
  no SCAN_RSP appeared on air in 180 s. Not converged within the timebox, not
  committed.

Implication for slice 2: connection events need a response within T_IFS every
interval, so the connection path must use the hardware brx/btx automatic
turnaround of the B91 FSM, not a software-triggered TX. Unexplored options for
SCAN_RSP: a shorter TX settle on the STX path, `txwait`/`tx_stl` tuning for the
rx2tx path, and a RAM-resident ISR.

A response whose TX trigger tick would be less than 10 us ahead is now not
started at all and is counted in `rsp_late` instead. The trigger slack at
baseline was estimated at about +26..-3 us (from the slack measured with a
-59 us correction), so part of the SCAN_REQs now get no response. This trades
some responses for an advertising state machine that cannot stall on a
trigger in the past.

Scanners treat our late responses as missing and back off (Core Vol 6 Part B
4.4.3.2), so a PC sends few SCAN_REQs to the keyboard (about one per minute
visible on air). This does not affect discovery, since the name is in ADV_IND.

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

### Capturing

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

`--follow <ADDRESS IN CAPS>` makes the sniffer follow one advertiser, which is
needed to see CONNECT_INDs addressed to it. From the CLI it is unreliable: it
worked about once in nine tries in one session and zero times in four tries in
another. Usually the sniffer stalls (0 to 21 packets in 25 to 30 s) or keeps hopping
without catching the CONNECT_IND. Without follow mode, no CONNECT_IND to the
keyboard was ever captured, even while the device logged several per second.
The Wireshark GUI (extcap "Device" control) is the next thing to try.

### Sniffer display quirks

- The sniffer rewrites some fields in what it reports: the ChSel bit in ADV_IND
  and CONNECT_IND headers, and ChM, Hop and SCA in CONNECT_IND. The displayed
  bytes then fail the CRC. Reconstruct the real values by searching for the
  combination that makes the CRC match (CRC24, polynomial `0x65B`, init
  `0x555555` on advertising channels). This is how the blob's ADV_IND header
  `20 21` (ChSel = 1) was found, which the sniffer shows as `00 21`.
- For timing between packets use `nordic_ble.delta_time`, which is end to
  start (T_IFS). `nordic_ble.delta_time_ss` is start to start.

### Producing SCAN_REQs

While a bonded host keeps trying to connect, the kernel pauses discovery and
`bluetoothctl scan on` sends no SCAN_REQs at all. What works:
`bluetoothctl block <addr>`, then restart `bluetoothctl --timeout 5 scan on` in
a loop during the capture, then `bluetoothctl unblock <addr>`.

### `ble_adv_report.py`

`reverse/tools/ble_adv_report.py` summarizes the advertising in a capture
(needs `tshark` in `PATH`):

```bash
python3 reverse/tools/ble_adv_report.py adv.pcapng --adva xx:xx:xx:xx:xx:xx
```

Output: `per_channel` packet counts, `crc_bad`, `pdu_types`, and
`interval_ms_mean` / `min` / `max`. Notes:

- The CRC flag comes from `nordic_ble.crcok`. tshark 4.x prints it as
  `True`/`False`, older versions as `1`/`0`; the script handles both and treats
  an empty field as good.
- The interval is computed from consecutive good channel 37 packets, so missed
  events inflate it (the sniffer hops and misses some). For the real
  advInterval, compute event starts and keep only deltas below 1.5 x the
  minimum interval, as done for the table above.
- If the sniffer re-syncs during a capture, `frame.time_relative` restarts and
  the minimum interval is bogus.

Tests: `cd reverse/tools && python3 -m unittest test_ble_adv_report`.

## Host tests

```bash
zmk/drivers/bluetooth/openll/tests/run_host_tests.sh
```

Builds and runs `test_hci` (opcode handling, event encoding, parameter
validation, unknown opcodes), `test_pdu` (PDU encoding, SCAN_REQ match,
CONNECT_IND parsing against captured bytes) and `test_adv` (state machine
against a fake radio: channel sequence, interval and delay, SCAN_RSP trigger,
refused late SCAN_RSP, CONNECT_IND callback, late events and stall
catch-up) with the host gcc.

## Known limitations

- No connections. A CONNECT_IND is parsed and logged, then advertising
  resumes. A bonded host retries endlessly, so BLE typing does not work with
  this build.
- SCAN_RSP is about 59 us late (T_IFS about 209 us), see above. Discovery still
  works because the name is in ADV_IND.
- We advertise with ChSel = 0 (CSA #1 only). The blob sets ChSel = 1 (CSA #2
  support). Valid either way; the decision belongs to slice 2.
- Timing between log timestamps and wall clock disagrees by about 6 %: the
  "every 2 s" radio reports appear 1.88 s apart in log time and the 10 s
  CONNECT_IND dumps 9.4..9.8 s apart, while the stimer-based advertising
  timing is correct on air. To be explained before connection timing work.
- Under a heavy log flood, aborting an mcumgr upload once wedged the CDC ACM
  port (the keyboard kept advertising). A host-side USB port reset
  (`USBDEVFS_RESET`) recovered it. Possibly a `usb_dc_b91` or CDC TX issue,
  separate from openll.
- No power management: the radio runs whenever advertising is enabled.
- An AA-matched comparison of one CONNECT_IND between sniffer and device log is
  still open (sniffer follow mode).

## Roadmap

Each slice gets its own spec, plan and build cycle once the previous one works.

- **Slice 2: connection follow and empty-PDU keepalive.** Anchor tracking,
  window widening from SCA, CSA #1 channel hopping, SN/NESN, supervision
  timeout, LL_TERMINATE_IND, LE Connection Complete and Disconnection Complete
  events. Prerequisites from slice 1: use the hardware brx/btx turnaround,
  explain the 6 % clock discrepancy, decide ChSel, get one AA-matched
  CONNECT_IND capture.
- **Slice 3: LLCP subset and ACL data.** LL_VERSION_IND, LL_FEATURE_REQ/RSP,
  LL_CONNECTION_UPDATE_IND, LL_CHANNEL_MAP_IND, LL_UNKNOWN_RSP for everything
  else (including PHY update, so 2M is avoided), ACL data path, HCI flow control
  (Number of Completed Packets).
- **Slice 4: link encryption.** LL_ENC_REQ/RSP, LL_START_ENC, AES-CCM with the
  B91 hardware AES, LE Long Term Key Request reply. After this, SMP pairing and
  HID over GATT work end to end.
- **Slice 5: power management.** CPU sleep between connection events,
  coordination with deep sleep (`poweroff.c`), 32k RC calibration.
- **Slice 6: privacy and extras.** LE Set Random Address and RPA (the blob hangs
  on this today), optional 2M PHY, Data Length Extension.
- **In parallel:** ask Telink for permission to redistribute the blob.
