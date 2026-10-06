#!/usr/bin/env python3
"""Headless nRF Sniffer capture that follows one BLE device into its connections.

The Wireshark extcap only exposes "follow this device" as a GUI toolbar control, and
`nrfutil ble-sniffer sniff --follow` gives up if the device is not advertising at the
moment it starts. This script drives Nordic's SnifferAPI directly instead.

Requires the SnifferAPI from the AUR package `nrf-sniffer-ble` (installed under
/usr/lib/nrf-sniffer-ble/pymodule) and pyserial. Run it in the arch distrobox:

    python3 reverse/tools/ble_follow_capture.py --follow AA:BB:CC:DD:EE:FF \
        --seconds 60 --out reverse/captures/blob-conn.pcapng

The address is written MSB first, as bluetoothctl prints it. Add --random for a
random address. The output is a classic pcap file that tshark/Wireshark read
(the .pcapng name is only a convention here).
"""
import argparse
import glob
import os
import sys
import time

SNIFFER_API = "/usr/lib/nrf-sniffer-ble/pymodule"
sys.path.insert(0, SNIFFER_API)

PROTOCOL_VERSION = 3  # packet protocol of sniffer firmware 4.x


def default_port():
    links = glob.glob("/dev/serial/by-id/*nRF_Sniffer*")
    return os.path.realpath(links[0]) if links else None


def parse_address(text, random):
    octets = [int(x, 16) for x in text.split(":")]
    if len(octets) != 6:
        raise argparse.ArgumentTypeError("address must have 6 octets")
    return octets + [1 if random else 0]


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--follow", required=True, help="device address, MSB first")
    ap.add_argument("--random", action="store_true", help="address is random")
    ap.add_argument("--port", default=default_port(), help="sniffer tty")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", required=True, help="output pcap path")
    a = ap.parse_args(argv)

    if not a.port:
        sys.exit("no nRF Sniffer found under /dev/serial/by-id")
    try:
        from SnifferAPI import Devices, Pcap, Sniffer, UART
    except ImportError:
        sys.exit(f"SnifferAPI not found in {SNIFFER_API} (install AUR nrf-sniffer-ble)")

    # An idle sniffer sends nothing, so a passive probe misses it: ping it then.
    rates = UART.find_sniffer_baudrates(a.port)
    if rates is None:
        rates = UART.find_sniffer_baudrates(a.port, write_data=True)
    if rates is None:
        sys.exit(f"{a.port} does not answer like an nRF Sniffer")

    out = open(a.out, "wb")
    out.write(Pcap.get_global_header())
    count = {"packets": 0}

    def on_packet(notification):
        p = notification.msg["packet"]
        out.write(Pcap.create_packet(bytes([p.boardId] + p.getList()), p.time))
        count["packets"] += 1

    sniffer = Sniffer.Sniffer(a.port, rates["default"])
    sniffer.subscribe("NEW_BLE_PACKET", on_packet)
    sniffer.setAdvHopSequence([37, 38, 39])
    sniffer.setSupportedProtocolVersion(PROTOCOL_VERSION)
    sniffer.getFirmwareVersion()
    sniffer.getTimestamp()
    sniffer.start()
    sniffer.scan()

    device = Devices.Device(address=parse_address(a.follow, a.random), name='""', RSSI=0)
    sniffer.addDevice(device)
    sniffer.follow(device)
    print(f"following {a.follow} on {a.port} for {a.seconds:.0f} s", flush=True)

    try:
        time.sleep(a.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        sniffer.doExit()
        out.close()
    print(f"wrote {count['packets']} packets to {a.out} "
          f"(missed over UART: {sniffer.missedPackets})")


if __name__ == "__main__":
    main(sys.argv[1:])
