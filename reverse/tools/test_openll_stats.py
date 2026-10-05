import os, sys, unittest
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import openll_stats as o
from rainy75_rgb import _cbor_map, _cbor_uint, _cbor_decode


def _cbor_tstr(t):
    b = t.encode()
    return bytes([0x60 | len(b)]) + b


def _indef_array(items):
    # zcbor (non-canonical, as in the firmware) encodes lists indefinite
    return b"\x9f" + b"".join(items) + b"\xff"


def _indef_map(pairs):
    return b"\xbf" + b"".join(_cbor_tstr(k) + v for k, v in pairs) + b"\xff"


def _link(up, listen, skip, coll, miss):
    return _indef_map([("up", _cbor_uint(up)), ("listen", _cbor_uint(listen)),
                       ("skip", _cbor_uint(skip)), ("coll", _cbor_uint(coll)),
                       ("miss", _cbor_uint(miss))])


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

    def test_cbor_arrays(self):
        self.assertEqual(_cbor_decode(bytes([0x83, 1, 2, 3]))[0], [1, 2, 3])
        self.assertEqual(_cbor_decode(_indef_array([_cbor_uint(7), _cbor_uint(300)]))[0],
                         [7, 300])
        self.assertEqual(_cbor_decode(bytes([0x80]))[0], [])

    def _frame(self):
        pairs = [("rc", _cbor_uint(0))] + [(k, _cbor_uint(i * 1000))
                                            for i, k in enumerate(o.FIELDS)]
        body = b"\xbf" + b"".join(_cbor_tstr(k) + v for k, v in pairs)
        body += _cbor_tstr("links") + _cbor_uint(2)
        body += _cbor_tstr("link") + _indef_array([
            _link(1, 100, 3000, 5, 2), _link(1, 50, 1500, 0, 1), _link(0, 0, 0, 0, 0)])
        body += _cbor_tstr("adv") + _indef_map([
            ("ev", _cbor_uint(40)), ("slid", _cbor_uint(9)), ("drop", _cbor_uint(1)),
            ("cut", _cbor_uint(0)), ("stuck", _cbor_uint(0))])
        body += b"\xff"
        return bytes([1, 0, 0, 0, 0, 66, 1, 0]) + body

    def test_parse_links(self):
        s = o.parse_response(self._frame())
        self.assertEqual(s["links"], 2)
        self.assertEqual(len(s["link"]), 3)
        self.assertEqual(s["link"][0], {"up": 1, "listen": 100, "skip": 3000,
                                        "coll": 5, "miss": 2})
        self.assertEqual(s["adv"]["slid"], 9)

    def test_link_delta_wrap(self):
        new = {"up": 10, "link": [{"listen": 5, "skip": 1, "coll": 0, "miss": 0}],
               "adv": {"ev": 3, "slid": 0, "drop": 0, "cut": 0, "stuck": 0}}
        old = {"up": 5, "link": [{"listen": 0xFFFFFFFF, "skip": 0, "coll": 0, "miss": 0}],
               "adv": {"ev": 1, "slid": 0, "drop": 0, "cut": 0, "stuck": 0}}
        d = o.delta(new, old)
        self.assertEqual(d["link"][0]["listen"], 6)
        self.assertEqual(d["adv"]["ev"], 2)

    def test_format_links(self):
        s = o.parse_response(self._frame())
        txt = o.format_stats(s)
        self.assertIn("links 2", txt)
        self.assertIn("link 0: up listen 100 skip 3000 coll 5 miss 2", txt)
        self.assertIn("link 2: down", txt)
        self.assertIn("adv: ev 40 slid 9 drop 1 cut 0 stuck 0", txt)
        d = o.delta(s, s)
        d["up"] = 1000   # deltas are printed for a non-empty interval only
        txt = o.format_stats(s, d)
        self.assertIn("delta link 0: listen 0 skip 0 coll 0 miss 0", txt)

    def test_format_old_firmware(self):
        # a slice 5 reply has neither "links" nor "link" nor "adv"
        s = dict(zip(o.FIELDS, [10000, 9000, 1, 2, 3, 4, 5, 6, 7, 3900]))
        txt = o.format_stats(s)
        self.assertNotIn("link 0", txt)

    def test_format_flash(self):
        # flash window counters (and the RX ring overrun count) when present
        s = dict(zip(o.FIELDS, [10000, 9000, 1, 2, 3, 4, 5, 6, 7, 3900]))
        s["flash"] = {"win": 120, "wait": 3, "force": 0, "wmax": 8000, "hmax": 13000,
                      "pause": 40, "cut": 2, "fkick": 7, "abort": 5, "pskip": 0}
        txt = o.format_stats(s)
        self.assertIn("flash: win 120 wait 3 force 0 wmax 8000 hmax 13000 pause 40 cut 2 "
                      "fkick 7 abort 5 pskip 0", txt)
        old = dict(s, flash=dict(s["flash"], win=100, wmax=7000, hmax=12000, pskip=0))
        d = o.delta(s, old)
        self.assertEqual(d["flash"]["win"], 20)
        # wmax / hmax are maxima since boot, not counters: no deltas
        self.assertNotIn("wmax", d["flash"])
        self.assertNotIn("hmax", d["flash"])
        d["up"] = 1000
        txt = o.format_stats(s, d)
        self.assertIn("delta flash: win 20", txt)
        self.assertNotIn("delta flash: win 20 wait 0 force 0 wmax", txt)
        s2 = dict(zip(o.FIELDS, [10000, 9000, 1, 2, 3, 4, 5, 6, 7, 3900]))
        self.assertNotIn("flash:", o.format_stats(s2))

    def test_group_matches_firmware(self):
        with open(os.path.join(os.path.dirname(__file__), "../../zmk/src/openll_mgmt.c")) as f:
            src = f.read()
        self.assertIn("MGMT_GROUP_ID_PERUSER + 2", src)
        self.assertEqual(o.GROUP, 66)


if __name__ == "__main__":
    unittest.main()
