#!/usr/bin/env python3
"""
Read the open BLE controller's power counters (mcumgr group 66, command 0).

Works over USB CDC-ACM serial (default) or BLE SMP GATT (--ble). The firmware
must be built with the open controller (CONFIG_OPENLL_MGMT, enabled by
conf/openll.conf). BLE needs `bleak`; reuses the helpers of rainy75_rgb.py and
rainy75_rgb_ble.py (connected-peripheral handling: the device is resolved via
BlueZ and an already-live link is never disconnected).

Usage:
    python3 openll_stats.py                 # one read over USB serial
    python3 openll_stats.py --ble           # one read over BLE
    python3 openll_stats.py --ble -n 5 -i 10  # 5 reads, 10 s apart, with deltas
    python3 openll_stats.py --port /dev/ttyACM0 --address XX:XX:XX:XX:XX:XX

Fields: up (ms), idle (CPU idle ms, if built in), plan/listen/skip (connection events planned / listened /
skipped by peripheral latency), kick, ev, miss, wake (controller thread
wakeups), mv (battery, 0 = unavailable). Counters are uint32, cumulative
since boot; deltas are computed modulo 2**32.
"""

import argparse
import asyncio
import base64
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

GROUP = 66          # MGMT_GROUP_ID_PERUSER + 2
CMD_STATS = 0
SMP_OP_READ = 0
FIELDS = ("up", "idle", "plan", "listen", "skip", "kick", "ev", "miss", "wake", "mv")
COUNTERS = ("idle", "plan", "listen", "skip", "kick", "ev", "miss", "wake")


# --- pure helpers (unit tested) --------------------------------------------

def build_request(seq, group=GROUP, cmd=CMD_STATS):
    """SMP read request with an empty CBOR map: 8-byte header + 0xA0."""
    return struct.pack(">BBHHBB", SMP_OP_READ, 0, 1, group, seq & 0xFF, cmd) + b"\xa0"


def parse_response(frame):
    """Raw SMP frame (header + CBOR) -> stats dict. Raises on rc != 0."""
    from rainy75_rgb import _cbor_decode
    if len(frame) < 9:
        raise ValueError("short SMP frame")
    body, _ = _cbor_decode(frame[8:])
    if not isinstance(body, dict):
        raise ValueError("SMP body is not a map")
    rc = body.get("rc", 0)
    if rc != 0:
        raise RuntimeError(f"device rc={rc}")
    return body


def delta(new, old):
    """Per-field difference of two stats dicts; counters wrap at 2**32."""
    return {k: (new[k] - old[k]) & 0xFFFFFFFF for k in ("up",) + COUNTERS
            if k in new and k in old}


def format_stats(s, d=None):
    idle = (f"idle {100.0 * s['idle'] / s['up']:.1f}%  "
            if "idle" in s and s["up"] else "")
    line = (f"up {s['up'] / 1000:.1f}s  {idle}plan {s['plan']}  listen {s['listen']}  "
            f"skip {s['skip']}  kick {s['kick']}  ev {s['ev']}  miss {s['miss']}  "
            f"wake {s['wake']}  batt {s['mv']} mV")
    if d and d.get("up"):
        secs = d["up"] / 1000.0
        tot = d.get("listen", 0) + d.get("skip", 0)
        ratio = f"{100.0 * d['skip'] / tot:.0f}% skipped" if tot else "no events"
        if "idle" in d:
            line += f"\n  delta idle {100.0 * d['idle'] / d['up']:.1f}%"
        line += (f"\n  delta {secs:.1f}s: listen {d.get('listen', 0)} "
                 f"skip {d.get('skip', 0)} ({ratio}) wake {d.get('wake', 0)} "
                 f"({d.get('wake', 0) / secs:.1f}/s) miss {d.get('miss', 0)}")
    return line


# --- serial transport ------------------------------------------------------

def read_serial(port=None, timeout=3.0):
    import rainy75_rgb as r
    kb = r.Rainy75(port)
    try:
        # Reuse its SMP serial framing with our group id.
        old = r.RGB_GROUP
        r.RGB_GROUP = GROUP
        try:
            return kb._request(SMP_OP_READ, CMD_STATS, [], timeout=timeout)
        finally:
            r.RGB_GROUP = old
    finally:
        kb.close()


# --- BLE transport ---------------------------------------------------------

async def read_ble(address=None, timeout=10.0):
    import rainy75_rgb_ble as b
    kb = b.Rainy75BLE(address=address, timeout=timeout)
    await kb.connect()
    try:
        old = b.RGB_GROUP
        b.RGB_GROUP = GROUP
        try:
            return await kb._request(SMP_OP_READ, CMD_STATS, [])
        finally:
            b.RGB_GROUP = old
    finally:
        # Never disconnects a link that was already up (live HID session).
        await kb.disconnect()


def main():
    ap = argparse.ArgumentParser(description="Read open-controller power counters")
    ap.add_argument("--ble", action="store_true", help="use BLE instead of USB serial")
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    ap.add_argument("--address", help="BLE address (default: by name via BlueZ)")
    ap.add_argument("-n", type=int, default=1, help="number of reads (default 1)")
    ap.add_argument("-i", type=float, default=5.0, help="seconds between reads")
    args = ap.parse_args()

    prev = None
    for k in range(args.n):
        if k:
            time.sleep(args.i)
        s = asyncio.run(read_ble(args.address)) if args.ble else read_serial(args.port)
        print(format_stats(s, delta(s, prev) if prev else None), flush=True)
        prev = s


if __name__ == "__main__":
    main()
