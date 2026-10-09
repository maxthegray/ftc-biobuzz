import gzip
import http.client
import json
import math
import struct
import sys
import tempfile
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from log_viewer import LogLibrary, ThreadingHTTPServer, camera_channels, handler_for, json_safe, match_info, problems, scan_run


def fixture(truncated=False):
    data = bytearray(b"WPILOG\x00\x01\x00\x00\x00\x00")

    def record(entry, timestamp, payload):
        data.extend(b"\x3f" + struct.pack("<III", entry, len(payload), timestamp) + payload)

    def string(value):
        encoded = value.encode()
        return struct.pack("<I", len(encoded)) + encoded

    channels = [("pose", "double[]"), ("Field/Robot", "struct:Pose2d"),
                ("battery", "double"), ("commands/events", "string"),
                ("commands/lost", "int64"), ("events", "string")]
    for i, (name, kind) in enumerate(channels, 1):
        record(0, 0, b"\x00" + struct.pack("<I", i) + string(name) + string(kind) + string(""))
    record(1, 1_000_001, struct.pack("<ddd", 8, 56, math.pi / 2))
    record(2, 1_000_001, struct.pack("<ddd", -0.5, 1.3, 3.14))
    record(1, 2_000_001, struct.pack("<ddd", 20, 60, float("nan")))
    record(3, 1_000_001, struct.pack("<d", 12.4))
    record(4, 1_000_001, b"START #1 Drive hold")
    record(4, 1_000_002, b"FINISH #1 Drive hold")
    record(5, 1, struct.pack("<q", 0))
    record(6, 1_000_002, b"start")
    if truncated:
        data.extend(b"\x3f\x01")
    return data


class ViewerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.logs = self.root / "logs"
        self.uploads = self.root / "uploads"
        self.logs.mkdir()
        self.uploads.mkdir()
        self.path = self.logs / "Native-20260916-120000-000-1.wpilog"
        self.path.write_bytes(fixture(truncated=True))
        self.library = LogLibrary(self.logs, self.uploads)
        self.ident = self.library.catalog()[0]["id"]

    def tearDown(self):
        self.temp.cleanup()

    def test_native_pose_and_microsecond_command_times_survive(self):
        run = self.library.overview(self.ident)
        self.assertEqual([1.000001, [8, 56, math.pi / 2]], run["playback"]["pose"][0])
        self.assertNotIn("Field/Robot", run["playback"])
        self.assertTrue(run["report"]["truncated"])
        self.assertEqual("instrumented", run["report"]["commandHistoryCoverage"])
        execution = run["report"]["commandExecutions"][0]
        self.assertEqual(1.000001, execution["startSec"])
        self.assertEqual(1.000002, execution["endSec"])
        self.assertEqual(2.000001, run["endSec"])

    def test_nonfinite_values_are_gaps_in_valid_json(self):
        result = self.library.series(self.ident, ["pose", "missing"])
        encoded = json.dumps(json_safe(result), allow_nan=False)
        decoded = json.loads(encoded)
        self.assertIsNone(decoded["pose"][1][1][2])
        self.assertEqual([], decoded["missing"])

    def test_several_runs_stay_decoded_for_overlays(self):
        other = self.logs / "Other-20260916-130000-000-1.wpilog"
        other.write_bytes(fixture())
        second = self.library.register(other)["id"]
        self.library.overview(self.ident)
        self.library.overview(second)
        self.assertEqual({self.ident, second}, set(self.library.cache))
        for n in range(8):
            extra = self.logs / f"Extra-20260916-14000{n}-000-1.wpilog"
            extra.write_bytes(fixture())
            self.library.overview(self.library.register(extra)["id"])
        self.assertEqual(6, len(self.library.cache))
        self.assertNotIn("problems", self.library.overview(self.ident)["report"])
        self.assertEqual({"Pose jump": 1, "Log ends early": 1}, self.library.problems(self.ident)["counts"])

    def test_arbitrary_file_paths_cannot_be_requested(self):
        with self.assertRaises(KeyError):
            self.library.overview("../../README.md")
        outside = self.root / "outside.wpilog"
        outside.write_bytes(fixture())
        (self.logs / "escape.wpilog").symlink_to(outside)
        self.assertEqual(1, len(self.library.catalog()))

    def test_empty_and_wrong_format_files(self):
        empty = self.logs / "empty.wpilog"
        empty.write_bytes(b"WPILOG\x00\x01\x00\x00\x00\x00")
        result = self.library.overview(self.library.register(empty)["id"])
        self.assertTrue(result["report"]["empty"])
        self.assertEqual(0, result["endSec"])
        wrong = self.logs / "wrong.wpilog"
        wrong.write_bytes(b"not a log file")
        with self.assertRaises(ValueError):
            self.library.overview(self.library.register(wrong)["id"])

    def test_local_http_load_upload_and_origin_boundary(self):
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(self.library))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        connection = http.client.HTTPConnection("127.0.0.1", server.server_port)
        try:
            connection.request("GET", f"/api/log?id={self.ident}", headers={"Accept-Encoding": "gzip"})
            response = connection.getresponse()
            data = response.read()
            if response.getheader("Content-Encoding") == "gzip":
                data = gzip.decompress(data)
            self.assertEqual(200, response.status)
            self.assertEqual([8, 56, math.pi / 2], json.loads(data)["playback"]["pose"][0][1])
            connection.request("POST", "/api/upload?name=Imported.wpilog", body=fixture())
            response = connection.getresponse()
            imported = json.loads(response.read())
            self.assertEqual(200, response.status)
            self.assertEqual("Imported.wpilog", self.library.overview(imported["id"])["name"])
            connection.request("GET", "/api/logs", headers={"Origin": "https://example.com"})
            response = connection.getresponse()
            response.read()
            self.assertEqual(403, response.status)
            connection.request("POST", "/api/upload?name=bad.wpilog", body=b"invalid")
            response = connection.getresponse()
            response.read()
            self.assertEqual(400, response.status)
            self.assertEqual(1, len(list(self.uploads.iterdir())))
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == "__main__":
    unittest.main()


class CameraChannelTest(unittest.TestCase):
    def test_any_subsystems_detection_channels_are_sent_for_playback(self):
        records = {"pose": [], "BallCamera/candidates/xPx": [], "IntakeCam/target/horizontalDeg": [],
                   "IntakeCam/mount/heightIn": [], "Limelight/target/txDegrees": [], "Lift/heightIn": [],
                   "commands/active": []}
        self.assertEqual(
            ("BallCamera/candidates/xPx", "IntakeCam/target/horizontalDeg", "IntakeCam/mount/heightIn"),
            camera_channels(records),
        )


def seconds(*points):
    return [(round(t * 1e6), v) for t, v in points]


class ProblemTest(unittest.TestCase):
    def kinds(self, records, report=None):
        return [(p["kind"], p["severity"], p["startSec"], p["endSec"], p["detail"]) for p in problems(records, report or {})[0]]

    def test_battery_sag_merges_bursts_and_ignores_dead_readings(self):
        records = {"battery": seconds((0, 12.5), (1, 0.0), (2, 9.2), (3, 12.0), (6, 7.9), (6.5, 9.0), (7, 12.1),
                                      (20, 9.4), (21, 12.0))}
        self.assertEqual([("Battery sag", "fault", 2, 7, "min 7.90 V · 2.00 s under 9.5 V in 2 dips"),
                          ("Battery sag", "warn", 20, 21, "min 9.40 V · 1.00 s under 9.5 V")],
                         self.kinds(records))

    def test_loop_spikes_skip_the_startup_loops(self):
        records = {"loop/totalNanos": seconds((0, 9e6), (0.01, 90e6), (0.02, 9e6), (5, 64e6), (5.01, 9e6))}
        self.assertEqual([("Loop spike", "warn", 5, 5.01, "peak 64 ms")], self.kinds(records))
        records = {"loop/totalNanos": seconds((0, 9e6), (0.01, 9e6), (5, 64e6), (5.01, 70e6), (5.02, 9e6))}
        self.assertEqual([("Loop spike", "warn", 5, 5.02, "peak 70 ms · 2 slow loops")], self.kinds(records))

    def test_disconnects_count_only_after_a_connection(self):
        records = {"Limelight/connected": seconds((0, False), (1, True), (2, False), (4, True)),
                   "BallCamera/connected": seconds((0, False), (9, False))}
        self.assertEqual([("Limelight disconnected", "fault", 2, 4, "for 2.00 s")], self.kinds(records))

    def test_pose_jumps_and_word_bounded_fault_events(self):
        records = {"pose": seconds((0, [0.0, 0.0, 0.0]), (0.01, [1.0, 0.0, 0.0]), (0.02, [float("nan"), 0.0, 0.0]),
                                   (0.03, [9.0, 0.0, 0.0]), (0.04, [13.0, 0.0, 0.0])),
                   "loop/totalNanos": seconds((0, 9e6)),
                   "events": seconds((1, "schedule default Drive: teleop drive"), (2, "LOCALIZER FAULT: Pinpoint status NOT_READY"),
                                     (3, "LOOP OVERRUN: 146 ms"))}
        self.assertEqual([("Pose jump", "warn", 0.04, 0.04, "4.0 in"),
                          ("Fault event", "fault", 2, 2, "LOCALIZER FAULT: Pinpoint status NOT_READY")], self.kinds(records))

    def test_failed_commands_and_cut_off_logs(self):
        report = {"commandFailures": [{"tSec": 3.5, "text": "#2 Lift up: timeout"}], "truncated": True}
        self.assertEqual([("Command failed", "fault", 3.5, 3.5, "#2 Lift up: timeout"),
                          ("Log ends early", "warn", 4, 4, "the last record was cut off")],
                         self.kinds({"battery": seconds((4, 12.0))}, report))


class RunSummaryTest(unittest.TestCase):
    def test_header_scan_finds_the_end_and_op_mode_start_without_decoding(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "Run.wpilog"
            path.write_bytes(fixture(truncated=True))
            self.assertEqual({"endSec": 2.000001, "startSec": 1.000002}, scan_run(path))
            path.write_bytes(b"not a log")
            self.assertEqual({"endSec": None, "startSec": None}, scan_run(path))

    def test_library_lists_run_length_and_flags_runs_that_never_started(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "Driven-20260916-120000-000-1.wpilog").write_bytes(fixture())
            (root / "Init-20260916-110000-000-1.wpilog").write_bytes(b"WPILOG\x00\x01\x00\x00\x00\x00")
            library = LogLibrary(root, root)
            library.catalog()
            for _ in range(200):
                if all(log["scanned"] for log in library.catalog()):
                    break
                threading.Event().wait(0.01)
            runs = {log["name"].split("-")[0]: log for log in library.catalog()}
            self.assertAlmostEqual(0.999999, runs["Driven"]["drivenSec"])
            self.assertTrue(runs["Driven"]["short"])
            self.assertIsNone(runs["Init"]["drivenSec"])
            self.assertTrue(runs["Init"]["short"])

    def test_match_kind_comes_from_the_op_mode_name(self):
        self.assertEqual(("auto", 2.0), (lambda m: (m["kind"], m["startSec"]))(match_info("TwoTipAuto-20260916-120000-0-1.wpilog", 2.0)))
        self.assertEqual("teleop", match_info("DriveOnlyTeleOp-20260916-120000-0-1.wpilog", 2.0)["kind"])
        self.assertIsNone(match_info("X.wpilog", None)["startSec"])

    def test_viewer_keeps_log_time_and_shares_the_analyzers_start(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "Run-20260916-120000-000-1.wpilog").write_bytes(fixture())
            library = LogLibrary(root, root)
            run = library.overview(library.catalog()[0]["id"])
            self.assertEqual("log start", run["report"]["clock"])
            self.assertEqual(1.000002, run["match"]["startSec"])
            self.assertEqual(1.000002, run["report"]["events"][0]["tSec"])
