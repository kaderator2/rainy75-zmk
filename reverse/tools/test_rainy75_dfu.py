"""Unit tests for rainy75_dfu.py (no hardware needed)."""
import hashlib
import struct
import unittest

import rainy75_dfu as dfu


class ImageHash(unittest.TestCase):
    def test_header_and_body_without_tlvs(self):
        hdr = bytearray(32)
        struct.pack_into("<I", hdr, 0, 0x96f3b83d)   # IMAGE_MAGIC
        struct.pack_into("<H", hdr, 8, 32)           # header size
        struct.pack_into("<I", hdr, 12, 5)           # image size
        data = bytes(hdr) + b"BODY!" + b"TLV-AREA"
        self.assertEqual(dfu.image_hash(data),
                         hashlib.sha256(bytes(hdr) + b"BODY!").digest())


if __name__ == "__main__":
    unittest.main()
