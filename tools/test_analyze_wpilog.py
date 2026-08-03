import os
import struct
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(__file__))
import analyze_wpilog


class AnalyzeWpiLogTest(unittest.TestCase):
    def test_default_path_uses_run_timestamp_instead_of_pull_order(self):
        with tempfile.TemporaryDirectory() as directory:
            older = os.path.join(
                directory,
                "FrameworkSmokeTestTeleOp-20260803-141807-927-1.wpilog",
            )
            newer = os.path.join(
                directory,
                "FrameworkSmokeTestTeleOp-20260803-142223-270-2.wpilog",
            )
            open(older, "wb").close()
            open(newer, "wb").close()
            os.utime(older, (200, 200))
            os.utime(newer, (100, 100))

            with mock.patch.object(analyze_wpilog.glob, "glob", return_value=[older, newer]):
                resolved = analyze_wpilog.resolve_paths([])

        self.assertEqual([newer], resolved)

    def test_default_path_falls_back_to_mtime_for_unrecognized_names(self):
        with tempfile.TemporaryDirectory() as directory:
            older = os.path.join(directory, "older.wpilog")
            newer = os.path.join(directory, "newer.wpilog")
            open(older, "wb").close()
            open(newer, "wb").close()
            os.utime(older, (100, 100))
            os.utime(newer, (200, 200))

            with mock.patch.object(analyze_wpilog.glob, "glob", return_value=[older, newer]):
                resolved = analyze_wpilog.resolve_paths([])

        self.assertEqual([newer], resolved)

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

    def test_loop_maxima_prefer_recorder_window_peaks(self):
        records = {
            "loop/totalNanos": [(1, 10), (2, 20)],
            "loop/windowMaxTotalNanos": [(1, 100), (2, 200)],
            "loop/controlNanos": [(1, 3), (2, 4)],
            "loop/windowMax/controlNanos": [(1, 30), (2, 40)],
        }

        report = analyze_wpilog.build_report(records, {}, "test.wpilog")

        self.assertEqual(200, report["loop"]["maxNs"])
        self.assertEqual(40, report["loop"]["phases"][0]["maxNs"])


if __name__ == "__main__":
    unittest.main()
