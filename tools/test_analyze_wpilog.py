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

    def test_logs_without_command_history_say_so_instead_of_reporting_no_commands(self):
        records = {
            "events": [(1, "COMMAND FAULT: IllegalStateException: boom (commands cleared, subsystems halted)")],
            "driveMode": [(1, "TELEOP"), (2, "ROBOT_CENTRIC_FALLBACK")],
        }

        report = analyze_wpilog.build_report(records, {}, "test.wpilog")
        bundle = analyze_wpilog.to_json_dict(report)

        self.assertFalse(bundle["commandHistoryRecorded"])
        self.assertNotIn("commands", bundle)
        self.assertEqual(1, len(bundle["faults"]))
        self.assertIn("ROBOT_CENTRIC_FALLBACK", bundle["driveModeTimeSec"])

    def test_pre_ivy_logs_keep_their_command_set_transitions(self):
        records = {"commands/running": [(1, "teleop drive"), (2, "follow\nmarker")]}

        bundle = analyze_wpilog.to_json_dict(analyze_wpilog.build_report(records, {}, "old.wpilog"))

        self.assertTrue(bundle["commandHistoryRecorded"])
        self.assertEqual("all scheduled", bundle["commandHistoryCoverage"])
        self.assertEqual(["follow", "marker"], bundle["commands"][1]["running"])
        self.assertNotIn("commandExecutions", bundle)

    def test_migration_logs_report_no_coverage(self):
        records = {"events": [(1, "init TeleOp")]}

        bundle = analyze_wpilog.to_json_dict(analyze_wpilog.build_report(records, {"events": "string"}, "mid.wpilog"))

        self.assertEqual("none", bundle["commandHistoryCoverage"])
        self.assertFalse(bundle["commandHistoryRecorded"])
        self.assertNotIn("commandHistoryIntact", bundle)

    def test_instrumented_logs_rebuild_executions_and_never_claim_all_commands(self):
        records = {
            "commands/events": [
                (1_000_000, "START #1 Driver sticks"),
                (2_000_000, "INTERRUPT #1 Driver sticks"),
                (2_000_001, "START #2 Route"),
                (2_000_002, "START #3 Follow line"),
                (2_500_000, "SUSPEND #2 Route"),
                (2_600_000, "RESUME #2 Route"),
                (3_000_000, "FAIL #3 Follow line in execute: IllegalStateException: boom: detail"),
                (3_000_001, "FAIL #2 Route in execute: IllegalStateException: boom: detail (from #3)"),
                (3_000_002, "FAIL #- Later step in end: IllegalStateException: late"),
                (4_000_000, "START #4 Same name"),
                (4_000_001, "START #5 Same name"),
                (5_000_000, "ABORT #4 Same name: op-mode stop"),
            ],
            "commands/active": [(0, ""), (1_000_000, "#1 Driver sticks"), (4_000_001, "#4 Same name\n#5 Same name")],
            "commands/lost": [(0, 0)],
        }
        types = {"commands/events": "string", "commands/active": "string", "commands/lost": "int64"}

        bundle = analyze_wpilog.to_json_dict(analyze_wpilog.build_report(records, types, "new.wpilog"))

        self.assertEqual("instrumented", bundle["commandHistoryCoverage"])
        self.assertTrue(bundle["commandHistoryIntact"])
        runs = {run["id"]: run for run in bundle["commandExecutions"]}
        self.assertEqual(("INTERRUPT", 1.0, 2.0), (runs[1]["outcome"], runs[1]["startSec"], runs[1]["endSec"]))
        self.assertEqual("in execute: IllegalStateException: boom: detail (from #3)", runs[2]["detail"])
        self.assertEqual(1, runs[2]["suspensions"])
        self.assertEqual("op-mode stop", runs[4]["detail"])
        self.assertEqual(("OPEN", None), (runs[5]["outcome"], runs[5]["endSec"]))
        self.assertEqual("Same name", runs[5]["name"])
        self.assertEqual(3, len(bundle["commandFailures"]))
        self.assertEqual(["#4 Same name", "#5 Same name"], bundle["commandActive"][-1]["active"])

    def test_lost_records_mark_instrumented_history_incomplete(self):
        records = {
            "commands/events": [(1, "START #1 A"), (5, "HISTORY INCOMPLETE: 12 command records lost")],
            "commands/lost": [(0, 0), (5, 12)],
        }

        bundle = analyze_wpilog.to_json_dict(analyze_wpilog.build_report(records, {}, "lossy.wpilog"))

        self.assertFalse(bundle["commandHistoryIntact"])
        self.assertEqual(12, bundle["commandHistoryLostRecords"])
        self.assertEqual(1, len(bundle["commandExecutions"]))

    def test_instrumented_log_without_any_traced_command_still_reports_coverage(self):
        records = {"events": [(1, "init TeleOp")], "commands/active": [(0, "")], "commands/lost": [(0, 0)]}
        types = {"events": "string", "commands/events": "string", "commands/active": "string", "commands/lost": "int64"}

        bundle = analyze_wpilog.to_json_dict(analyze_wpilog.build_report(records, types, "idle.wpilog"))

        self.assertEqual("instrumented", bundle["commandHistoryCoverage"])
        self.assertEqual([], bundle["commandExecutions"])


if __name__ == "__main__":
    unittest.main()
