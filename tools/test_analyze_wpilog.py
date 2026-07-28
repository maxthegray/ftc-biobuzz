import os
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import analyze_wpilog


class AnalyzeWpiLogTest(unittest.TestCase):
    def test_truncated_final_record_is_reported_without_crashing(self):
        data = b"WPILOG" + struct.pack("<H", 0x0100) + struct.pack("<I", 0)
        name = b"battery"
        type_name = b"double"
        start = (
            b"\x00"
            + struct.pack("<I", 1)
            + struct.pack("<I", len(name))
            + name
            + struct.pack("<I", len(type_name))
            + type_name
            + struct.pack("<I", 0)
        )
        data += bytes((0, 0, len(start), 0)) + start
        payload = struct.pack("<d", 12.5)
        data += bytes((0, 1, len(payload), 1)) + payload
        data += b"\x00\x01"
        with tempfile.NamedTemporaryFile(delete=False) as log:
            log.write(data)
            path = log.name
        try:
            records, channel_types, truncated = analyze_wpilog.parse_wpilog(path)
        finally:
            os.unlink(path)

        self.assertEqual([(1, 12.5)], records["battery"])
        self.assertEqual("double", channel_types["battery"])
        self.assertTrue(truncated)

    def test_fault_summary_includes_localizer_and_telemetry_faults(self):
        records = {
            "events": [
                (1, "LOCALIZER FAULT: frozen pose"),
                (2, "TELEMETRY FAULT: dashboard"),
                (3, "ordinary marker"),
            ],
        }

        report = analyze_wpilog.build_report(records, {"events": "string"}, "test.wpilog")

        self.assertEqual(
            ["LOCALIZER FAULT: frozen pose", "TELEMETRY FAULT: dashboard"],
            [fault["text"] for fault in report["faults"]],
        )


if __name__ == "__main__":
    unittest.main()
