#!/usr/bin/env python3
"""Post-match one-pager for the flight recorder's WPILOG files.

Usage:
    python3 tools/analyze_wpilog.py [file.wpilog ...]
    python3 tools/analyze_wpilog.py --json [--channel battery,Lift/outputPower] [file ...]

--json emits a compact machine-readable diagnostic bundle (the same metrics as
the text one-pager, plus a channel manifest, command-set transitions, and the
full event timeline) for programmatic post-match debugging. --channel adds the
full [tSec, value] series for the named channels.

With no arguments, analyzes the newest .wpilog under ./robot-logs (where
`make pull-logs` drops them). Ten minutes between matches is the real
debugging budget — this prints the questions worth asking first: loop-rate
percentiles, which phase spiked, battery sag, follower error, contained
faults, and the event timeline.
"""

import glob
import os
import statistics
import struct
import sys


def read_varint(data, pos, length):
    value = 0
    for i in range(length):
        value |= data[pos + i] << (8 * i)
    return value


def parse_wpilog(path):
    with open(path, "rb") as f:
        data = f.read()

    if data[:6] != b"WPILOG":
        raise ValueError(f"{path}: not a WPILOG file")
    extra_len = struct.unpack_from("<I", data, 8)[0]
    pos = 12 + extra_len

    entries = {}   # id -> (name, type)
    records = {}   # name -> list[(timestamp_us, value)]

    while pos < len(data):
        header = data[pos]
        pos += 1
        entry_len = (header & 0x3) + 1
        size_len = ((header >> 2) & 0x3) + 1
        time_len = ((header >> 4) & 0x7) + 1

        entry_id = read_varint(data, pos, entry_len)
        pos += entry_len
        payload_size = read_varint(data, pos, size_len)
        pos += size_len
        timestamp = read_varint(data, pos, time_len)
        pos += time_len
        payload = data[pos:pos + payload_size]
        pos += payload_size

        if entry_id == 0:
            if payload and payload[0] == 0:  # start record
                eid = struct.unpack_from("<I", payload, 1)[0]
                name_len = struct.unpack_from("<I", payload, 5)[0]
                name = payload[9:9 + name_len].decode("utf-8")
                type_off = 9 + name_len
                type_len = struct.unpack_from("<I", payload, type_off)[0]
                type_name = payload[type_off + 4:type_off + 4 + type_len].decode("utf-8")
                entries[eid] = (name, type_name)
            continue

        if entry_id not in entries:
            continue
        name, type_name = entries[entry_id]
        if type_name == "double":
            value = struct.unpack("<d", payload)[0]
        elif type_name == "int64":
            value = struct.unpack("<q", payload)[0]
        elif type_name == "double[]":
            value = list(struct.unpack(f"<{len(payload) // 8}d", payload))
        elif type_name == "string":
            value = payload.decode("utf-8", errors="replace")
        elif type_name == "boolean":
            value = payload[0] != 0
        else:
            continue
        records.setdefault(name, []).append((timestamp, value))

    channel_types = {name: type_name for name, type_name in entries.values()}
    return records, channel_types


def percentile(sorted_values, fraction):
    if not sorted_values:
        return 0.0
    index = min(int(len(sorted_values) * fraction), len(sorted_values) - 1)
    return sorted_values[index]


def fmt_ms(nanos):
    return f"{nanos / 1e6:.2f} ms"


PHASES = [
    ("clearCaches", "loop/clearCachesNanos"),
    ("periodic", "loop/periodicNanos"),
    ("input", "loop/inputNanos"),
    ("control", "loop/controlNanos"),
    ("scheduler", "loop/schedulerNanos"),
    ("writeHardware", "loop/writeHardwareNanos"),
    ("telemetry", "loop/telemetryNanos"),
    ("record", "loop/recordNanos"),
]

_SCALAR_TYPES = ("double", "int64")


def build_report(records, channel_types, path):
    """Compute every metric once; both the text and JSON outputs read this."""
    report = {"file": os.path.basename(path)}
    all_ts = [ts for series in records.values() for ts, _ in series]
    if not all_ts:
        report["empty"] = True
        return report
    report["empty"] = False
    end_ts = max(all_ts)
    report["durationSec"] = (end_ts - min(all_ts)) / 1e6
    report["recordCount"] = len(all_ts)
    report["channelCount"] = len(records)

    # --- loop timing -------------------------------------------------------
    totals = sorted(v for _, v in records.get("loop/totalNanos", []) if v > 0)
    if totals:
        p50, p90, p99 = (percentile(totals, f) for f in (0.50, 0.90, 0.99))
        phases = []
        for label, key in PHASES:
            series = [v for _, v in records.get(key, [])]
            if series:
                phases.append({"label": label, "maxNs": max(series),
                               "meanNs": statistics.mean(series)})
        report["loop"] = {"p50Ns": p50, "p90Ns": p90, "p99Ns": p99,
                          "maxNs": totals[-1], "phases": phases}

    # --- battery -----------------------------------------------------------
    battery = [v for _, v in records.get("battery", [])]
    if battery:
        report["battery"] = {"startV": battery[0], "minV": min(battery),
                             "endV": battery[-1], "sagWarning": min(battery) < 9.5}

    # --- follower error ----------------------------------------------------
    trans = [v for _, v in records.get("follow/translationalErrorIn", [])]
    heading = [abs(v) for _, v in records.get("follow/headingErrorRad", [])]
    if trans:
        report["following"] = {
            "samples": len(trans),
            "transMaxIn": max(trans),
            "transMeanIn": statistics.mean(trans),
            "headingMaxRad": max(heading) if heading else 0.0,
            "headingMeanRad": statistics.mean(heading) if heading else 0.0,
        }

    # --- drive mode time ---------------------------------------------------
    modes = records.get("driveMode", [])
    if modes:
        time_in = {}
        for i, (ts, mode) in enumerate(modes):
            seg_end = modes[i + 1][0] if i + 1 < len(modes) else end_ts
            time_in[mode] = time_in.get(mode, 0) + (seg_end - ts)
        report["driveModeTimeSec"] = {m: time_in[m] / 1e6 for m in sorted(time_in)}

    # --- commands ----------------------------------------------------------
    running = records.get("commands/running", [])
    report["commandChanges"] = len(running)
    report["commands"] = [
        {"tSec": ts / 1e6, "running": [c for c in text.split("\n") if c]}
        for ts, text in running
    ]

    # --- events ------------------------------------------------------------
    events = records.get("events", [])
    faults = [(ts, text) for ts, text in events
              if "COMMAND FAULT" in text or "CRASH" in text]
    corrections = [(ts, text) for ts, text in events if "pose correction" in text]
    report["events"] = [{"tSec": ts / 1e6, "text": text} for ts, text in events]
    report["faults"] = [{"tSec": ts / 1e6, "text": text} for ts, text in faults]
    if corrections:
        applied = sum(1 for _, t in corrections if "applied" in t)
        report["poseCorrections"] = {"applied": applied,
                                     "rejected": len(corrections) - applied}

    # --- channel manifest --------------------------------------------------
    manifest = []
    for name in sorted(records):
        series = records[name]
        ctype = channel_types.get(name, "?")
        entry = {"name": name, "type": ctype, "count": len(series),
                 "tFirstSec": series[0][0] / 1e6, "tLastSec": series[-1][0] / 1e6}
        if ctype in _SCALAR_TYPES:
            values = [v for _, v in series]
            entry["minV"] = min(values)
            entry["maxV"] = max(values)
        manifest.append(entry)
    report["channels"] = manifest
    return report


def print_text_report(report):
    print(f"=== {report['file']} ===")
    if report["empty"]:
        print("  (empty log)")
        return
    print(f"duration: {report['durationSec']:.1f} s, "
          f"{report['recordCount']} records, {report['channelCount']} channels")

    loop = report.get("loop")
    if loop:
        print(f"\nloop: {1e9 / loop['p50Ns']:.0f} Hz median "
              f"(p50 {fmt_ms(loop['p50Ns'])}, p90 {fmt_ms(loop['p90Ns'])}, "
              f"p99 {fmt_ms(loop['p99Ns'])}, max {fmt_ms(loop['maxNs'])})")
        print("phase maxima:")
        for phase in loop["phases"]:
            print(f"  {phase['label']:<14} max {fmt_ms(phase['maxNs']):>10}   "
                  f"mean {fmt_ms(phase['meanNs']):>10}")

    battery = report.get("battery")
    if battery:
        print(f"\nbattery: start {battery['startV']:.2f} V, min {battery['minV']:.2f} V, "
              f"end {battery['endV']:.2f} V")
        if battery["sagWarning"]:
            print("  WARNING: deep sag below 9.5 V — check battery health / brownout risk")

    following = report.get("following")
    if following:
        import math
        print(f"\nfollowing ({following['samples']} samples): "
              f"translational max {following['transMaxIn']:.2f} in, "
              f"mean {following['transMeanIn']:.2f} in")
        print(f"  heading max {math.degrees(following['headingMaxRad']):.1f} deg, "
              f"mean {math.degrees(following['headingMeanRad']):.1f} deg")

    drive_mode = report.get("driveModeTimeSec")
    if drive_mode:
        parts = ", ".join(f"{m} {t:.1f}s" for m, t in drive_mode.items())
        print(f"\ndrive mode time: {parts}")

    if report["commandChanges"]:
        print(f"\ncommand-set changes: {report['commandChanges']}")

    if report["faults"]:
        print(f"\n!! {len(report['faults'])} fault/crash event(s):")
        for fault in report["faults"]:
            print(f"  [{fault['tSec']:8.2f}s] {fault['text'].splitlines()[0]}")
    corrections = report.get("poseCorrections")
    if corrections:
        print(f"\npose corrections: {corrections['applied']} applied, "
              f"{corrections['rejected']} rejected")
    if report["events"]:
        print(f"\nevent timeline ({len(report['events'])}):")
        for event in report["events"]:
            print(f"  [{event['tSec']:8.2f}s] {event['text'].splitlines()[0]}")
    print()


def to_json_dict(report):
    """Reshape the internal report into the compact, documented JSON bundle."""
    out = {"file": report["file"], "empty": report["empty"]}
    if report["empty"]:
        return out
    out["durationSec"] = round(report["durationSec"], 1)
    out["recordCount"] = report["recordCount"]
    out["channelCount"] = report["channelCount"]

    loop = report.get("loop")
    if loop:
        out["loop"] = {
            "hzMedian": round(1e9 / loop["p50Ns"]),
            "p50Ms": round(loop["p50Ns"] / 1e6, 3),
            "p90Ms": round(loop["p90Ns"] / 1e6, 3),
            "p99Ms": round(loop["p99Ns"] / 1e6, 3),
            "maxMs": round(loop["maxNs"] / 1e6, 3),
            "phaseMaxMs": {p["label"]: round(p["maxNs"] / 1e6, 3) for p in loop["phases"]},
            "phaseMeanMs": {p["label"]: round(p["meanNs"] / 1e6, 3) for p in loop["phases"]},
        }

    battery = report.get("battery")
    if battery:
        out["battery"] = {"startV": round(battery["startV"], 2),
                          "minV": round(battery["minV"], 2),
                          "endV": round(battery["endV"], 2),
                          "sagWarning": battery["sagWarning"]}

    following = report.get("following")
    if following:
        import math
        out["following"] = {
            "samples": following["samples"],
            "transMaxIn": round(following["transMaxIn"], 3),
            "transMeanIn": round(following["transMeanIn"], 3),
            "headingMaxDeg": round(math.degrees(following["headingMaxRad"]), 1),
            "headingMeanDeg": round(math.degrees(following["headingMeanRad"]), 1),
        }

    if "driveModeTimeSec" in report:
        out["driveModeTimeSec"] = {m: round(t, 1) for m, t in report["driveModeTimeSec"].items()}

    out["commands"] = [{"tSec": round(c["tSec"], 3), "running": c["running"]}
                       for c in report["commands"]]
    out["events"] = [{"tSec": round(e["tSec"], 3), "text": e["text"]}
                     for e in report["events"]]
    out["faults"] = [{"tSec": round(f["tSec"], 3), "text": f["text"]}
                     for f in report["faults"]]
    out["poseCorrections"] = report.get("poseCorrections", {"applied": 0, "rejected": 0})

    out["channels"] = []
    for ch in report["channels"]:
        entry = {"name": ch["name"], "type": ch["type"], "count": ch["count"],
                 "tFirstSec": round(ch["tFirstSec"], 3), "tLastSec": round(ch["tLastSec"], 3)}
        if "minV" in ch:
            entry["minV"] = round(ch["minV"], 4) if isinstance(ch["minV"], float) else ch["minV"]
            entry["maxV"] = round(ch["maxV"], 4) if isinstance(ch["maxV"], float) else ch["maxV"]
        out["channels"].append(entry)
    return out


def dump_channels(records, names):
    out = {}
    for name in names:
        series = records.get(name)
        if series is None:
            out[name] = None
            continue
        out[name] = [[round(ts / 1e6, 4), v] for ts, v in series]
    return out


def resolve_paths(paths):
    if paths:
        return paths
    candidates = glob.glob("robot-logs/**/*.wpilog", recursive=True)
    if not candidates:
        return []
    return [max(candidates, key=os.path.getmtime)]


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Post-match WPILOG analyzer.")
    parser.add_argument("paths", nargs="*", help="log file(s); default: newest under robot-logs/")
    parser.add_argument("--json", action="store_true",
                        help="emit a machine-readable diagnostic bundle instead of the text one-pager")
    parser.add_argument("--channel", metavar="NAME[,NAME...]",
                        help="with --json, also dump full [tSec, value] series for these "
                             "comma-separated channels (e.g. battery,Lift/outputPower)")
    args = parser.parse_args()

    paths = resolve_paths(args.paths)
    if not paths:
        print("no .wpilog files under ./robot-logs — run `make pull-logs` first", file=sys.stderr)
        return 1

    if args.json:
        import json
        bundles = []
        for path in paths:
            records, channel_types = parse_wpilog(path)
            bundle = to_json_dict(build_report(records, channel_types, path))
            if args.channel:
                names = [n for n in args.channel.split(",") if n]
                bundle["series"] = dump_channels(records, names)
            bundles.append(bundle)
        print(json.dumps(bundles[0] if len(bundles) == 1 else bundles, indent=2))
        return 0

    for path in paths:
        records, channel_types = parse_wpilog(path)
        print_text_report(build_report(records, channel_types, path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
