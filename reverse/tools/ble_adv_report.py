#!/usr/bin/env python3
"""Summarize BLE advertising captured with nRF Sniffer.

Runs tshark to extract per-packet fields, then reports per-channel counts,
advertising event interval (from channel 37 packets) and CRC failures.
Used to compare the blob controller with the open controller.
"""
import argparse
import subprocess
import sys

FIELDS = ["frame.time_relative", "nordic_ble.channel",
          "btle.advertising_header.pdu_type", "btle.advertising_address",
          "nordic_ble.crcok"]


def parse_fields_line(line):
    t, ch, pdu, adva, crc = (line.rstrip("\n").split("\t") + [""] * 5)[:5]
    # nordic_ble.crcok: tshark 4.x prints the boolean field as "True"/"False";
    # older tshark releases printed "1"/"0". Treat "0" or "false" (any case) as
    # a bad CRC, everything else (including empty) as good.
    crc_ok = crc.strip().lower() not in ("0", "false")
    return {"t": float(t), "ch": int(ch), "pdu": int(pdu, 0),
            "adva": adva.lower(), "crc_ok": crc_ok}


def summarize(rows, adva=None):
    if adva:
        adva = adva.lower()
        rows = [r for r in rows if r["adva"] == adva]
    per_channel = {}
    for r in rows:
        per_channel[r["ch"]] = per_channel.get(r["ch"], 0) + 1
    t37 = sorted(r["t"] for r in rows if r["ch"] == 37 and r["crc_ok"])
    deltas = [(b - a) * 1000.0 for a, b in zip(t37, t37[1:])]
    return {
        "per_channel": per_channel,
        "crc_bad": sum(1 for r in rows if not r["crc_ok"]),
        "pdu_types": sorted({r["pdu"] for r in rows}),
        "interval_ms_mean": sum(deltas) / len(deltas) if deltas else None,
        "interval_ms_min": min(deltas) if deltas else None,
        "interval_ms_max": max(deltas) if deltas else None,
    }


def run_tshark(path):
    cmd = ["tshark", "-r", path, "-Y", "btle.advertising_header", "-T", "fields",
           "-e", FIELDS[0], "-e", FIELDS[1], "-e", FIELDS[2], "-e", FIELDS[3],
           "-e", "nordic_ble.crcok"]
    out = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
    rows = []
    for line in out.splitlines():
        if not line.strip():
            continue
        rows.append(parse_fields_line(line))
    return rows


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("pcap")
    ap.add_argument("--adva", help="filter on advertiser address")
    a = ap.parse_args(argv)
    try:
        rows = run_tshark(a.pcap)
    except FileNotFoundError:
        print("error: tshark not found (install Wireshark's tshark)", file=sys.stderr)
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        msg = (e.stderr or "").strip() or f"exit status {e.returncode}"
        print(f"error: tshark failed: {msg}", file=sys.stderr)
        sys.exit(1)
    s = summarize(rows, a.adva)
    for k, v in s.items():
        print(f"{k}: {v}")


if __name__ == "__main__":
    main(sys.argv[1:])
