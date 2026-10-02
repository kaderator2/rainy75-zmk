import unittest
from ble_adv_report import parse_fields_line, summarize

class ParseTest(unittest.TestCase):
    def test_parse_line(self):
        r = parse_fields_line("1.250000\t37\t0x00\taa:bb:cc:dd:ee:ff\t1")
        self.assertEqual(r, {"t": 1.25, "ch": 37, "pdu": 0,
                             "adva": "aa:bb:cc:dd:ee:ff", "crc_ok": True})

    def test_parse_line_crc_bad_and_missing_adva(self):
        r = parse_fields_line("2.0\t38\t0x03\t\t0")
        self.assertEqual(r["adva"], "")
        self.assertFalse(r["crc_ok"])

    def test_parse_line_crc_text_true_false(self):
        # tshark 4.x prints nordic_ble.crcok as "True"/"False" rather than "1"/"0"
        good = parse_fields_line("3.0\t37\t0x00\taa:bb:cc:dd:ee:ff\tTrue")
        bad = parse_fields_line("3.0\t37\t0x00\taa:bb:cc:dd:ee:ff\tFalse")
        self.assertTrue(good["crc_ok"])
        self.assertFalse(bad["crc_ok"])

class SummarizeTest(unittest.TestCase):
    def rows(self):
        a = "aa:bb:cc:dd:ee:ff"
        out = []
        # three events, 100 ms apart, each on 37/38/39 one ms apart
        for ev in range(3):
            for i, ch in enumerate((37, 38, 39)):
                out.append({"t": ev * 0.1 + i * 0.001, "ch": ch, "pdu": 0,
                            "adva": a, "crc_ok": True})
        out.append({"t": 0.05, "ch": 37, "pdu": 0, "adva": "11:22:33:44:55:66",
                    "crc_ok": True})
        out.append({"t": 0.06, "ch": 38, "pdu": 3, "adva": "", "crc_ok": False})
        return out

    def test_filters_by_adva_and_counts_channels(self):
        s = summarize(self.rows(), adva="AA:BB:CC:DD:EE:FF")
        self.assertEqual(s["per_channel"], {37: 3, 38: 3, 39: 3})
        self.assertEqual(s["crc_bad"], 0)
        self.assertEqual(s["pdu_types"], [0])

    def test_event_interval_from_channel_37(self):
        s = summarize(self.rows(), adva="aa:bb:cc:dd:ee:ff")
        self.assertAlmostEqual(s["interval_ms_mean"], 100.0, places=3)
        self.assertAlmostEqual(s["interval_ms_min"], 100.0, places=3)
        self.assertAlmostEqual(s["interval_ms_max"], 100.0, places=3)

    def test_unfiltered_counts_crc_bad(self):
        s = summarize(self.rows())
        self.assertEqual(s["crc_bad"], 1)

if __name__ == "__main__":
    unittest.main()
