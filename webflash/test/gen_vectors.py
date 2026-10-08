#!/usr/bin/env python3
"""Reference vectors for proto.js from the Python tools (run from the repo root)."""
import json, os, struct, sys, io
sys.path.insert(0, 'reverse/tools')
import restore_original as smp
import ota_flasher as ota

def frames_hex(frames): return [f.hex() for f in frames]
def req(op, group, seq, cmd, pairs):
    payload = smp.cbor_encode_map(pairs)
    return smp.smp_serial_encode(smp.smp_build_header(op, 0, len(payload), group, seq, cmd) + payload)

data256 = bytes((i * 7 + 3) & 0xFF for i in range(256))
data420 = bytes((i * 13 + 1) & 0xFF for i in range(420))
h = bytes(range(32))
v = {}
v['flash_write'] = frames_hex(req(2, 64, 5, 1, [("off", smp.cbor_encode_uint(0x80100)), ("data", smp.cbor_encode_bstr(data256))]))
v['flash_erase'] = frames_hex(req(2, 64, 0, 0, [("off", smp.cbor_encode_uint(0x80000)), ("len", smp.cbor_encode_uint(0x5D000))]))
v['flash_read'] = frames_hex(req(0, 64, 255, 2, [("off", smp.cbor_encode_uint(0)), ("len", smp.cbor_encode_uint(4))]))
v['flash_commit'] = frames_hex(req(2, 64, 7, 3, [("stg", smp.cbor_encode_uint(0x80000)), ("len", smp.cbor_encode_uint(383128))]))
v['img_upload_first'] = frames_hex(req(2, 1, 1, 1, [("image", smp.cbor_encode_uint(0)), ("len", smp.cbor_encode_uint(313000)), ("off", smp.cbor_encode_uint(0)), ("data", smp.cbor_encode_bstr(data420))]))
v['img_upload_next'] = frames_hex(req(2, 1, 2, 1, [("off", smp.cbor_encode_uint(420)), ("data", smp.cbor_encode_bstr(data420))]))
v['img_test'] = frames_hex(req(2, 1, 3, 0, [("hash", smp.cbor_encode_bstr(h)), ("confirm", b"\xf4")]))
v['img_confirm_running'] = frames_hex(req(2, 1, 4, 0, [("confirm", b"\xf5")]))
v['os_reset'] = frames_hex(req(2, 0, 9, 5, []))
v['os_echo'] = frames_hex(req(2, 0, 10, 0, [("d", smp.cbor_encode_tstr("hi"))]))
v['img_list'] = frames_hex(req(0, 1, 11, 0, []))

# A response the device could send: image list with an array of maps and bools,
# plus a log line in front and a CR after a frame.
images = (bytes([0x81]) + bytes([0xA5]) + smp.cbor_encode_tstr("slot") + smp.cbor_encode_uint(0)
          + smp.cbor_encode_tstr("version") + smp.cbor_encode_tstr("0.3.2")
          + smp.cbor_encode_tstr("hash") + smp.cbor_encode_bstr(h)
          + smp.cbor_encode_tstr("active") + b"\xf5"
          + smp.cbor_encode_tstr("pending") + b"\xf4")
body = bytes([0xA2]) + smp.cbor_encode_tstr("images") + images + smp.cbor_encode_tstr("splitStatus") + smp.cbor_encode_uint(0)
pkt = smp.smp_build_header(1, 0, len(body), 1, 11, 0) + body
rsp_frames = smp.smp_serial_encode(pkt)
stream = b"[00:00:01.000,000] <inf> zmk: hello\n" + rsp_frames[0] + b"".join(f[:-1] + b"\r\n" for f in rsp_frames[1:])
v['rsp_stream'] = stream.hex()
v['rsp_expect'] = {"slot": 0, "version": "0.3.2", "hash": h.hex(), "active": True, "pending": False}

# OTA packets for a 100-byte firmware (7 segments -> 3 packets) captured from ota_flasher.
fw = bytes((i * 3 + 1) & 0xFF for i in range(100))
captured = []
ota.os.write = lambda fd, pkt: captured.append(bytes(pkt).hex())
ota.read_response = lambda fd, timeout_ms=0: bytes([5, 2, 3, 0, 6, 0xFF, 0])
ota.flash_firmware(0, fw)
v['ota_packets'] = captured   # start, 3 data, end (each with report id 5 in front)
v['ota_fw'] = fw.hex()
v['ota_version_pkt'] = (bytes([5, 1, 0, 0]) + bytes([0xFF] * 60)).hex()
v['crc16_telink_abc'] = ota.crc16_telink(b"123456789")
v['crc16_xmodem_abc'] = smp.crc16_itu_t(b"123456789")
json.dump(v, open('webflash/test/vectors.json', 'w'), indent=1)
print("vectors ok", len(captured), "ota packets")
