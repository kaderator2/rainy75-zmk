import os, sys, unittest
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import openll_stats as o
from rainy75_rgb import _cbor_map, _cbor_uint


class T(unittest.TestCase):
    def test_request(self):
        r = o.build_request(5)
        self.assertEqual(r, bytes([0, 0, 0, 1, 0, 66, 5, 0, 0xA0]))

    def test_parse(self):
        pairs = [("rc", _cbor_uint(0))] + [(k, _cbor_uint(i * 1000)) for i, k in enumerate(o.FIELDS)]
        cbor = _cbor_map(pairs)
        frame = bytes([1, 0, 0, len(cbor), 0, 66, 1, 0]) + cbor
        s = o.parse_response(frame)
        self.assertEqual(s["skip"], 4000)
        self.assertEqual(s["mv"], 9000)

    def test_rc(self):
        cbor = _cbor_map([("rc", _cbor_uint(8))])
        with self.assertRaises(RuntimeError):
            o.parse_response(bytes(8) + cbor)

    def test_delta_wrap(self):
        d = o.delta({"up": 5, "skip": 3}, {"up": 2, "skip": 0xFFFFFFFF})
        self.assertEqual(d, {"up": 3, "skip": 4})

    def test_format(self):
        s = dict(zip(o.FIELDS, [10000, 9000, 1, 2, 3, 4, 5, 6, 7, 3900]))
        self.assertIn("3900 mV", o.format_stats(s))
        self.assertIn("% skipped", o.format_stats(s, {"up": 1000, "listen": 1, "skip": 9, "wake": 2}))

    def test_group_matches_firmware(self):
        src = open(os.path.join(os.path.dirname(__file__), "../../zmk/src/openll_mgmt.c")).read()
        self.assertIn("MGMT_GROUP_ID_PERUSER + 2", src)
        self.assertEqual(o.GROUP, 66)


if __name__ == "__main__":
    unittest.main()
