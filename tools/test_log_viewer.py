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
from log_viewer import LogLibrary, ThreadingHTTPServer, handler_for, json_safe


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

    def test_choosing_another_folder_switches_the_library(self):
        other = self.root / "season-logs"
        (other / "nested").mkdir(parents=True)
        (other / "nested" / "Auto-20260917-090000-000-1.wpilog").write_bytes(fixture())
        stale = self.ident
        self.assertEqual(other.resolve(), self.library.set_directory(other))
        catalog = self.library.catalog()
        self.assertEqual(["Auto-20260917-090000-000-1.wpilog"], [entry["name"] for entry in catalog])
        self.assertEqual(1.000001, self.library.overview(catalog[0]["id"])["playback"]["pose"][0][0])
        with self.assertRaises(KeyError):
            self.library.overview(stale)

    def test_imports_survive_a_folder_change(self):
        imported = self.uploads / "Imported.wpilog"
        imported.write_bytes(fixture())
        ident = self.library.register(imported, "Imported.wpilog")["id"]
        self.library.set_directory(self.root / "uploads")
        self.assertEqual("Imported.wpilog", self.library.overview(ident)["name"])

    def test_unusable_folders_are_refused(self):
        for candidate in (self.root / "missing", self.path, ""):
            with self.assertRaises(ValueError):
                self.library.set_directory(candidate)
        self.assertEqual(self.logs.resolve(), self.library.directory)

    def test_folder_endpoint_accepts_and_rejects_over_http(self):
        other = self.root / "elsewhere"
        other.mkdir()
        (other / "Teleop-20260917-100000-000-1.wpilog").write_bytes(fixture())
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(self.library))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        connection = http.client.HTTPConnection("127.0.0.1", server.server_port)
        try:
            connection.request("POST", "/api/folder", body=json.dumps({"path": str(other)}))
            response = connection.getresponse()
            result = json.loads(response.read())
            self.assertEqual(200, response.status)
            self.assertEqual({"path": str(other.resolve()), "count": 1}, result)
            connection.request("GET", "/api/folder")
            response = connection.getresponse()
            self.assertEqual(str(other.resolve()), json.loads(response.read())["path"])
            connection.request("POST", "/api/folder", body=json.dumps({"path": str(self.root / "nope")}))
            response = connection.getresponse()
            self.assertIn("No such folder", json.loads(response.read())["error"])
            self.assertEqual(400, response.status)
            connection.request("POST", "/api/folder", body=b"not json")
            response = connection.getresponse()
            response.read()
            self.assertEqual(400, response.status)
            self.assertEqual(other.resolve(), self.library.directory)
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            thread.join()

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
