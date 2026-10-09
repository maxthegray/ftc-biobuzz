#!/usr/bin/env python3
"""Serve the offline flight-log viewer on loopback using only Python's stdlib."""

import argparse
import gzip
import hashlib
import json
import math
import queue
import re
import struct
import sys
import tempfile
import threading
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "logs"))
import analyze_wpilog  # noqa: E402
import traction  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
ASSETS = Path(__file__).with_name("viewer")
MAX_UPLOAD = 256 * 1024 * 1024
# Decoded runs stay cached so overlays don't re-parse on every graph change: up to CACHE_RUNS, and
# while their files total CACHE_BYTES (decoded, a run takes about 8x its file size in memory).
CACHE_RUNS = 6
CACHE_BYTES = 64 * 1024 * 1024
PLAYBACK_CHANNELS = ("pose", "driveMode", "gamepad1/axes", "gamepad1/buttons",
                     "gamepad2/axes", "gamepad2/buttons", "commands/active", "commands/running",
                     "BallAssist/tx", "BallAssist/ty", "BallAssist/targetTy",
                     "Limelight/target/visible", "Limelight/target/txDegrees", "Limelight/target/tyDegrees",
                     "Hive/RED/state", "Hive/BLUE/state")

# Any camera subsystem's detections, whatever it is called: <Subsystem>/<one of these>.
CAMERA_CHANNEL_SUFFIXES = tuple(
    f"candidates/{name}" for name in ("xPx", "yPx", "radiusPx", "horizontalDeg", "verticalDeg", "rejections", "selectedIndex")
) + tuple(f"frame/{name}" for name in ("widthPx", "heightPx", "status")) + (
    "target/horizontalDeg", "target/verticalDeg",
) + tuple(f"mount/{name}" for name in ("measured", "heightIn", "pitchDownDeg", "forwardIn", "leftIn", "yawDeg"))


def camera_channels(records):
    return tuple(name for name in records
                 if name.partition("/")[0] and name.partition("/")[2] in CAMERA_CHANNEL_SUFFIXES)


# Problem thresholds. The robot's own LOOP OVERRUN event fires at 100 ms; 50 ms is already a
# missed control update. 9.5 V matches analyze_wpilog's sag warning; under 8 V risks a brownout.
SAG_VOLTS = 9.5
BROWNOUT_VOLTS = 8.0
MIN_VALID_VOLTS = 1.0
LOOP_SPIKE_MS = 50
POSE_JUMP_IN = 3.0
# The last pose before the op-mode's stop event is often a reset, not motion.
STOP_SETTLE_SEC = 0.1
MERGE_GAP_SEC = 1.0
SAG_MERGE_GAP_SEC = 5.0
STARTUP_LOOPS = 2
MAX_PER_KIND = 200
# Whole words, so "schedule default Drive" isn't a FAULT.
FAULT_EVENT = re.compile(r"\b(?:FAULT|CRASH|FAIL(?:ED|URE)?|LOST|INCOMPLETE|DISABLED|OVERRUN)\b", re.I)


def spans(series, predicate, gap_us=MERGE_GAP_SEC * 1e6):
    """Runs of samples matching predicate, as [start, end, values, active_us, dips]. Runs closer
    than gap_us merge into one span; active_us is only the time actually matching, summed over
    the merged dips. A dip lasts until the first sample that no longer matches."""
    result, current, since = [], None, None
    for ts, value in series:
        if predicate(value):
            if since is None:
                if current and ts - current[1] <= gap_us:
                    current[4] += 1
                else:
                    current = [ts, ts, [], 0, 1]
                    result.append(current)
                since = ts
            current[1] = ts
            current[2].append(value)
        elif since is not None:
            current[1] = ts
            current[3] += ts - since
            since = None
    if since is not None:
        current[3] += current[1] - since
    return result


def lasted(active_us):
    return f"{active_us / 1e6:.2f} s"


def problems(records, report):
    """Moments worth a look, from channels the recorder already writes. Missing channels add nothing."""
    found = []

    def add(kind, severity, start_us, end_us, detail):
        found.append({"kind": kind, "severity": severity, "startSec": start_us / 1e6,
                      "endSec": end_us / 1e6, "detail": detail})

    battery = [(ts, v) for ts, v in records.get("battery", []) if isinstance(v, float) and v > MIN_VALID_VOLTS]
    # Sag comes in bursts with each hard acceleration; one span per stretch of hard driving.
    for start, end, values, active, dips in spans(battery, lambda v: v < SAG_VOLTS, SAG_MERGE_GAP_SEC * 1e6):
        low = min(values)
        add("Battery sag", "fault" if low < BROWNOUT_VOLTS else "warn", start, end,
            f"min {low:.2f} V · {lasted(active)} under {SAG_VOLTS} V" + (f" in {dips} dips" if dips > 1 else ""))

    # The first loops after start time the op-mode starting up, not a control hitch.
    loop = records.get("loop/totalNanos", [])[STARTUP_LOOPS:]
    for start, end, values, _, _ in spans(loop, lambda v: v > LOOP_SPIKE_MS * 1e6):
        slow = f" · {len(values)} slow loops" if len(values) > 1 else ""
        add("Loop spike", "warn", start, end, f"peak {max(values) / 1e6:.0f} ms{slow}")

    for name in sorted(records):
        subsystem, _, leaf = name.rpartition("/")
        if leaf != "connected" or not subsystem:
            continue
        series = records[name]
        first = next((i for i, (_, v) in enumerate(series) if v is True), None)
        if first is not None:
            for start, end, _, active, _ in spans(series[first:], lambda v: v is False):
                add(f"{subsystem} disconnected", "fault", start, end, f"for {lasted(active)}")

    for name, kind in (("Localizer/faulted", "Localizer fault"), ("Drive/odometryFallback", "Odometry fallback")):
        for start, end, _, active, _ in spans(records.get(name, []), lambda v: v is True):
            add(kind, "fault", start, end, f"for {lasted(active)}")

    stop = next((ts for ts, text in records.get("events", []) if text == "stop"), None)
    previous = None
    for ts, pose in records.get("pose", []):
        valid = isinstance(pose, (list, tuple)) and len(pose) >= 2 and all(map(math.isfinite, pose[:2]))
        if valid and previous:
            jump = math.hypot(pose[0] - previous[0], pose[1] - previous[1])
            if jump > POSE_JUMP_IN and not (stop is not None and ts >= stop - STOP_SETTLE_SEC * 1e6):
                add("Pose jump", "warn", ts, ts, f"{jump:.1f} in")
        previous = pose if valid else None

    # The loop channel already covers overruns, with the true peak rather than a throttled event.
    skip_overruns = bool(records.get("loop/totalNanos"))
    for ts, text in records.get("events", []):
        if FAULT_EVENT.search(text) and not (skip_overruns and text.startswith("LOOP OVERRUN")):
            add("Fault event", "fault", ts, ts, text)

    for kind, start, end, detail in traction.problems(records, report.get("traction")):
        add(kind, "warn", start, end, detail)

    for failure in report.get("commandFailures", []):
        at = failure["tSec"] * 1e6
        add("Command failed", "fault", at, at, failure["text"])

    lost = next((ts for ts, v in records.get("commands/lost", []) if v), None)
    if lost is not None:
        add("Command records lost", "warn", lost, lost, "later command history is incomplete")

    if report.get("truncated"):
        end = max((series[-1][0] for series in records.values() if series), default=0)
        add("Log ends early", "warn", end, end, "the last record was cut off")

    found.sort(key=lambda item: item["startSec"])
    counts = {}
    kept = []
    for item in found:
        counts[item["kind"]] = counts.get(item["kind"], 0) + 1
        if counts[item["kind"]] <= MAX_PER_KIND:
            kept.append(item)
    return kept, counts


# BIOBUZZ timing comes from the robot code so the viewer and controller cues agree.
SHORT_RUN_SEC = 5


def match_timing():
    config = ROOT / "TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/RobotConfig.kt"
    try:
        source = config.read_text()
    except OSError:
        source = ""
    timing = {}
    for key, constant, fallback in (
        ("autoSec", "AUTO_NANOS", 30.0),
        ("driverSec", "DRIVER_NANOS", 120.0),
        ("flowerUnlockSec", "FLOWER_UNLOCK_NANOS", 60.0),
        ("finalTwentySec", "FINAL_TWENTY_NANOS", 100.0),
    ):
        match = re.search(rf"\bconst val {constant}\s*=\s*([\d_]+)L\b", source)
        timing[key] = int(match[1].replace("_", "")) / 1e9 if match else fallback
    return timing


def match_info(name, start):
    kind = "auto" if re.search(r"auto", Path(name).stem.split("-")[0], re.I) else "teleop"
    return {"kind": kind, "startSec": start, **match_timing()}


def scan_run(path):
    """Last timestamp and op-mode start time from record headers alone, without decoding any
    samples, so the run list can show every run's length. About 30 MB/s."""
    data = path.read_bytes()
    if data[:6] != b"WPILOG" or len(data) < 12:
        return {"endSec": None, "startSec": None}
    pos, end = 12 + struct.unpack_from("<I", data, 8)[0], len(data)
    last, events, start = 0, None, None
    while pos < end:
        header = data[pos]
        pos += 1
        entry_len, size_len, time_len = (header & 3) + 1, ((header >> 2) & 3) + 1, ((header >> 4) & 7) + 1
        if pos + entry_len + size_len + time_len > end:
            break
        entry = int.from_bytes(data[pos:pos + entry_len], "little")
        pos += entry_len
        size = int.from_bytes(data[pos:pos + size_len], "little")
        pos += size_len
        ts = int.from_bytes(data[pos:pos + time_len], "little")
        pos += time_len
        if pos + size > end:
            break
        if entry == 0:
            if size >= 9 and data[pos] == 0:
                ident, length = struct.unpack_from("<II", data, pos + 1)
                if data[pos + 9:pos + 9 + length] == b"events":
                    events = ident
        else:
            last = max(last, ts)
            if start is None and entry == events and data[pos:pos + size] == b"start":
                start = ts
        pos += size
    return {"endSec": last / 1e6 if last else None, "startSec": start / 1e6 if start is not None else None}


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def field_length():
    config = ROOT / "TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/RobotConfig.kt"
    match = re.search(r"const val LENGTH_INCHES\s*=\s*([\d.]+)", config.read_text())
    if not match:
        raise ValueError("Could not read RobotConfig.Field.LENGTH_INCHES")
    return float(match[1])


def series_seconds(records, names):
    # Preserve microsecond timestamps and native units; never use Field/Robot.
    return {name: [[ts / 1e6, value] for ts, value in records.get(name, [])] for name in names}


class LogLibrary:
    def __init__(self, directory, upload_directory):
        self.directory = Path(directory).resolve()
        self.upload_directory = Path(upload_directory)
        self.paths = {}
        self.names = {}
        self.cache = OrderedDict()
        self.found = {}
        self.lock = threading.RLock()
        # Run lengths for the library, scanned in the background and keyed by file identity.
        self.summaries = {}
        self.scan_queue = queue.Queue()
        self.queued = set()
        threading.Thread(target=self.scan_worker, daemon=True).start()

    def scan_worker(self):
        while True:
            identity, path = self.scan_queue.get()
            try:
                summary = scan_run(path)
            except (OSError, struct.error):
                summary = {"endSec": None, "startSec": None}
            self.summaries[identity] = summary

    def register(self, path, name=None):
        path = path.resolve()
        stat = path.stat()
        identity = f"{path}:{stat.st_mtime_ns}:{stat.st_size}"
        ident = hashlib.sha256(identity.encode()).hexdigest()[:24]
        self.paths[ident] = path
        self.names[ident] = name or path.name
        if identity not in self.summaries and identity not in self.queued:
            self.queued.add(identity)
            self.scan_queue.put((identity, path))
        summary = self.summaries.get(identity)
        driving = summary and summary["endSec"] is not None and summary["startSec"] is not None
        return {"id": ident, "name": self.names[ident], "bytes": stat.st_size,
                "modified": stat.st_mtime, "uploaded": path.parent == self.upload_directory,
                "scanned": summary is not None,
                "endSec": summary and summary["endSec"],
                "drivenSec": summary["endSec"] - summary["startSec"] if driving else None,
                "short": summary is not None and (not driving or summary["endSec"] - summary["startSec"] < SHORT_RUN_SEC)}

    def catalog(self):
        with self.lock:
            paths = [p for p in self.directory.rglob("*.wpilog")
                     if p.is_file() and p.resolve().is_relative_to(self.directory)]
            paths.sort(key=lambda p: (analyze_wpilog.run_timestamp_from_filename(p) or "",
                                      p.stat().st_mtime), reverse=True)
            return [self.register(p) for p in paths]

    def load(self, ident):
        if ident not in self.paths:
            raise KeyError("Unknown log; refresh the run library")
        if ident in self.cache:
            self.cache.move_to_end(ident)
            return self.cache[ident]
        path = self.paths[ident]
        if path.stat().st_size > MAX_UPLOAD:
            raise ValueError("This draft supports logs up to 256 MiB")
        with path.open("rb") as stream:
            header = stream.read(8)
        if header[:6] != b"WPILOG":
            raise ValueError("This file is not a WPILOG")
        if len(header) == 8 and header[6:8] != b"\x00\x01":
            raise ValueError("This draft supports WPILOG version 1.0")
        size = path.stat().st_size
        while self.cache and (len(self.cache) >= CACHE_RUNS
                              or sum(self.paths[i].stat().st_size for i in self.cache) + size > CACHE_BYTES):
            evicted, _ = self.cache.popitem(last=False)
            self.found.pop(evicted, None)
        records, types, truncated = analyze_wpilog.parse_wpilog(path)
        for series in records.values():
            if any(series[i][0] < series[i - 1][0] for i in range(1, len(series))):
                series.sort(key=lambda point: point[0])
        raw = analyze_wpilog.build_report(records, types, self.names[ident], truncated)
        # Derived after the report, so the analyzer's own summary only describes recorded channels.
        derived, raw["traction"] = traction.analyze(records)
        for name, (kind, series) in derived.items():
            records[name] = series
            types[name] = kind
        # MaxScope keeps log time internally and shows op-mode-start time, as the analyzer prints.
        report = analyze_wpilog.to_json_dict(raw, from_start=False)
        # The CLI rounds timestamps for readability. The viewer needs full precision.
        if not report.get("empty"):
            report["events"] = [{"tSec": ts / 1e6, "text": value}
                                for ts, value in records.get("events", [])]
            report["commandExecutions"], _ = analyze_wpilog.command_executions(records)
            report["traction"] = raw["traction"]
        self.cache[ident] = records, types, report, raw
        return self.cache[ident]

    def overview(self, ident):
        with self.lock:
            records, types, report, raw = self.load(ident)
            timestamps = [series[-1][0] for series in records.values() if series]
            channels = []
            for name, series in sorted(records.items()):
                length = max((len(v) for _, v in series), default=0) if types.get(name) == "double[]" else 0
                channels.append({"name": name, "type": types.get(name), "count": len(series),
                                 "components": length})
            return {"id": ident, "name": self.names[ident], "report": report,
                    "match": match_info(self.names[ident], raw.get("opModeStartSec")),
                    "endSec": max(timestamps, default=0) / 1e6,
                    "fieldLengthIn": field_length(), "channels": channels,
                    "playback": series_seconds(records, PLAYBACK_CHANNELS + camera_channels(records)),
                    "commandEvents": [{"tSec": ts / 1e6, "text": value}
                                      for ts, value in records.get("commands/events", [])]}

    def series(self, ident, names):
        if len(names) > 8:
            raise ValueError("Request at most eight channels at once")
        with self.lock:
            records, _, _, _ = self.load(ident)
            return series_seconds(records, names)

    def problems(self, ident):
        # Only scanned when asked for, then kept with the decoded run.
        with self.lock:
            records, _, _, raw = self.load(ident)
            if ident not in self.found:
                found, counts = problems(records, raw)
                self.found[ident] = {"problems": found, "counts": counts}
            return self.found[ident]


def handler_for(library):
    class Handler(BaseHTTPRequestHandler):
        def send_bytes(self, body, content_type, status=200):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self' 'unsafe-inline'; object-src 'none'; frame-ancestors 'none'")
            if len(body) > 4096 and "gzip" in self.headers.get("Accept-Encoding", ""):
                body = gzip.compress(body, compresslevel=1)
                self.send_header("Content-Encoding", "gzip")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def send_json(self, value, status=200):
            self.send_bytes(json.dumps(json_safe(value), allow_nan=False, separators=(",", ":")).encode(),
                            "application/json", status)

        def local_request(self):
            host = self.headers.get("Host", "")
            valid = {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
            origin = self.headers.get("Origin")
            if host not in valid or (origin is not None and origin != f"http://{host}"):
                self.send_json({"error": "Only same-origin local requests are accepted"}, 403)
                return False
            return True

        def do_GET(self):
            if not self.local_request():
                return
            url = urlsplit(self.path)
            query = parse_qs(url.query)
            try:
                if url.path == "/api/logs":
                    self.send_json(library.catalog())
                elif url.path == "/api/log":
                    self.send_json(library.overview(query.get("id", [""])[0]))
                elif url.path == "/api/problems":
                    self.send_json(library.problems(query.get("id", [""])[0]))
                elif url.path == "/api/series":
                    self.send_json(library.series(query.get("id", [""])[0], query.get("channel", [])))
                else:
                    files = {"/": ("index.html", "text/html; charset=utf-8"),
                             "/app.js": ("app.js", "text/javascript"),
                             "/ui.js": ("ui.js", "text/javascript"),
                             "/layout.js": ("layout.js", "text/javascript"),
                             "/charts.js": ("charts.js", "text/javascript"),
                             "/field-camera.js": ("field-camera.js", "text/javascript"),
                             "/overlays.js": ("overlays.js", "text/javascript"),
                             "/field-elements.js": ("field-elements.js", "text/javascript"),
                             # The experimental 3D page and its vendored three.js; see vendor/three/README.md.
                             "/3d": ("field3d.html", "text/html; charset=utf-8"),
                             "/field3d.js": ("field3d.js", "text/javascript"),
                             "/field3d.css": ("field3d.css", "text/css"),
                             "/vendor/three/three.module.js": ("vendor/three/three.module.js", "text/javascript"),
                             "/vendor/three/three.core.js": ("vendor/three/three.core.js", "text/javascript"),
                             "/vendor/three/OrbitControls.js": ("vendor/three/OrbitControls.js", "text/javascript"),
                             "/core.mjs": ("core.mjs", "text/javascript"),
                             "/theme.js": ("theme.js", "text/javascript"),
                             "/style.css": ("style.css", "text/css")}
                    if url.path not in files:
                        self.send_json({"error": "Not found"}, 404)
                        return
                    name, mime = files[url.path]
                    self.send_bytes((ASSETS / name).read_bytes(), mime)
            except KeyError as error:
                self.send_json({"error": str(error)}, 404)
            except (ValueError, OSError) as error:
                self.send_json({"error": str(error)}, 400)

        def do_POST(self):
            if not self.local_request():
                return
            if urlsplit(self.path).path != "/api/upload":
                self.send_json({"error": "Not found"}, 404)
                return
            temp_path = None
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= MAX_UPLOAD:
                    raise ValueError("Choose a WPILOG between 1 byte and 256 MiB")
                query = parse_qs(urlsplit(self.path).query)
                name = Path(query.get("name", ["Imported.wpilog"])[0]).name
                self.connection.settimeout(30)
                with tempfile.NamedTemporaryFile(dir=library.upload_directory, suffix=".wpilog", delete=False) as stream:
                    temp_path = Path(stream.name)
                    remaining = length
                    while remaining:
                        chunk = self.rfile.read(min(remaining, 1024 * 1024))
                        if not chunk:
                            raise ValueError("Upload ended before the file was complete")
                        stream.write(chunk)
                        remaining -= len(chunk)
                with temp_path.open("rb") as stream:
                    if stream.read(6) != b"WPILOG":
                        raise ValueError("This file is not a WPILOG")
                with library.lock:
                    result = library.register(temp_path, name)
                self.send_json(result)
            except (ValueError, OSError) as error:
                if temp_path:
                    temp_path.unlink(missing_ok=True)
                self.send_json({"error": str(error)}, 400)

        def log_message(self, format, *args):
            if args and str(args[1]) != "200":
                super().log_message(format, *args)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8008)
    parser.add_argument("--logs", type=Path, default=ROOT / "robot-logs")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="biobuzz-viewer-") as uploads:
        library = LogLibrary(args.logs, uploads)
        server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(library))
        print(f"MaxScope → http://127.0.0.1:{server.server_port}", flush=True)
        print(f"Reading {args.logs}. Ctrl+C to stop. Imported files are temporary.", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()


if __name__ == "__main__":
    main()
