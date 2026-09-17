#!/usr/bin/env python3
"""Serve the offline flight-log viewer on loopback using only Python's stdlib."""

import argparse
import gzip
import hashlib
import json
import math
import re
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import analyze_wpilog

ROOT = Path(__file__).resolve().parents[1]
ASSETS = Path(__file__).with_name("viewer")
MAX_UPLOAD = 256 * 1024 * 1024
PLAYBACK_CHANNELS = ("pose", "driveMode", "gamepad1/axes", "gamepad1/buttons",
                     "gamepad2/axes", "gamepad2/buttons", "commands/active", "commands/running",
                     "BallAssist/tx", "BallAssist/ty", "BallAssist/targetTy",
                     "Limelight/target/visible", "Limelight/target/txDegrees", "Limelight/target/tyDegrees")

# Any camera subsystem's detections, whatever it is called: <Subsystem>/<one of these>.
CAMERA_CHANNEL_SUFFIXES = tuple(
    f"candidates/{name}" for name in ("xPx", "yPx", "radiusPx", "horizontalDeg", "verticalDeg", "rejections", "selectedIndex")
) + tuple(f"frame/{name}" for name in ("widthPx", "heightPx", "status")) + (
    "target/horizontalDeg", "target/verticalDeg",
) + tuple(f"mount/{name}" for name in ("measured", "heightIn", "pitchDownDeg", "forwardIn", "leftIn", "yawDeg"))


def camera_channels(records):
    return tuple(name for name in records
                 if name.partition("/")[0] and name.partition("/")[2] in CAMERA_CHANNEL_SUFFIXES)


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def field_length():
    config = ROOT / "TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/core/runtime/RobotConfig.kt"
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
        self.cached_id = None
        self.cached = None
        self.lock = threading.RLock()

    def register(self, path, name=None):
        path = path.resolve()
        stat = path.stat()
        identity = f"{path}:{stat.st_mtime_ns}:{stat.st_size}"
        ident = hashlib.sha256(identity.encode()).hexdigest()[:24]
        self.paths[ident] = path
        self.names[ident] = name or path.name
        return {"id": ident, "name": self.names[ident], "bytes": stat.st_size,
                "modified": stat.st_mtime, "uploaded": path.parent == self.upload_directory}

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
        if self.cached_id != ident:
            self.cached = None
            self.cached_id = None
            path = self.paths[ident]
            if path.stat().st_size > MAX_UPLOAD:
                raise ValueError("This draft supports logs up to 256 MiB")
            with path.open("rb") as stream:
                header = stream.read(8)
            if header[:6] != b"WPILOG":
                raise ValueError("This file is not a WPILOG")
            if len(header) == 8 and header[6:8] != b"\x00\x01":
                raise ValueError("This draft supports WPILOG version 1.0")
            records, types, truncated = analyze_wpilog.parse_wpilog(path)
            for series in records.values():
                if any(series[i][0] < series[i - 1][0] for i in range(1, len(series))):
                    series.sort(key=lambda point: point[0])
            report = analyze_wpilog.to_json_dict(
                analyze_wpilog.build_report(records, types, self.names[ident], truncated))
            # The CLI rounds timestamps for readability. The viewer needs full precision.
            if not report.get("empty"):
                report["events"] = [{"tSec": ts / 1e6, "text": value}
                                    for ts, value in records.get("events", [])]
                report["commandExecutions"], _ = analyze_wpilog.command_executions(records)
            self.cached = records, types, report
            self.cached_id = ident
        return self.cached

    def overview(self, ident):
        with self.lock:
            records, types, report = self.load(ident)
            timestamps = [series[-1][0] for series in records.values() if series]
            channels = []
            for name, series in sorted(records.items()):
                length = max((len(v) for _, v in series), default=0) if types.get(name) == "double[]" else 0
                channels.append({"name": name, "type": types.get(name), "count": len(series),
                                 "components": length})
            return {"id": ident, "name": self.names[ident], "report": report,
                    "endSec": max(timestamps, default=0) / 1e6,
                    "fieldLengthIn": field_length(), "channels": channels,
                    "playback": series_seconds(records, PLAYBACK_CHANNELS + camera_channels(records)),
                    "commandEvents": [{"tSec": ts / 1e6, "text": value}
                                      for ts, value in records.get("commands/events", [])]}

    def series(self, ident, names):
        if len(names) > 8:
            raise ValueError("Request at most eight channels at once")
        with self.lock:
            records, _, _ = self.load(ident)
            return series_seconds(records, names)


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
                elif url.path == "/api/series":
                    self.send_json(library.series(query.get("id", [""])[0], query.get("channel", [])))
                else:
                    files = {"/": ("index.html", "text/html; charset=utf-8"),
                             "/app.js": ("app.js", "text/javascript"),
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
