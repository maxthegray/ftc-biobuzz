#!/usr/bin/env python3
"""Post-match one-pager for the flight recorder's WPILOG files.

Usage:
    python3 tools/analyze_wpilog.py [file.wpilog ...]

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

    return records


def percentile(sorted_values, fraction):
    if not sorted_values:
        return 0.0
    index = min(int(len(sorted_values) * fraction), len(sorted_values) - 1)
    return sorted_values[index]


def fmt_ms(nanos):
    return f"{nanos / 1e6:.2f} ms"


def analyze(path):
    records = parse_wpilog(path)
    print(f"=== {os.path.basename(path)} ===")
    all_ts = [ts for series in records.values() for ts, _ in series]
    if not all_ts:
        print("  (empty log)")
        return
    duration_s = (max(all_ts) - min(all_ts)) / 1e6
    print(f"duration: {duration_s:.1f} s, {len(all_ts)} records, {len(records)} channels")

    # --- loop timing -------------------------------------------------------
    totals = sorted(v for _, v in records.get("loop/totalNanos", []) if v > 0)
    if totals:
        p50, p90, p99 = (percentile(totals, f) for f in (0.50, 0.90, 0.99))
        print(f"\nloop: {1e9 / p50:.0f} Hz median "
              f"(p50 {fmt_ms(p50)}, p90 {fmt_ms(p90)}, p99 {fmt_ms(p99)}, max {fmt_ms(totals[-1])})")
        phases = [
            ("clearCaches", "loop/clearCachesNanos"),
            ("periodic", "loop/periodicNanos"),
            ("input", "loop/inputNanos"),
            ("control", "loop/controlNanos"),
            ("scheduler", "loop/schedulerNanos"),
            ("writeHardware", "loop/writeHardwareNanos"),
            ("telemetry", "loop/telemetryNanos"),
            ("record", "loop/recordNanos"),
        ]
        print("phase maxima:")
        for label, key in phases:
            series = [v for _, v in records.get(key, [])]
            if series:
                print(f"  {label:<14} max {fmt_ms(max(series)):>10}   "
                      f"mean {fmt_ms(statistics.mean(series)):>10}")

    # --- battery -----------------------------------------------------------
    battery = [v for _, v in records.get("battery", [])]
    if battery:
        print(f"\nbattery: start {battery[0]:.2f} V, min {min(battery):.2f} V, "
              f"end {battery[-1]:.2f} V")
        if min(battery) < 9.5:
            print("  WARNING: deep sag below 9.5 V — check battery health / brownout risk")

    # --- follower error ----------------------------------------------------
    trans = [v for _, v in records.get("follow/translationalErrorIn", [])]
    heading = [v for _, v in records.get("follow/headingErrorRad", [])]
    if trans:
        import math
        print(f"\nfollowing ({len(trans)} samples): translational max {max(trans):.2f} in, "
              f"mean {statistics.mean(trans):.2f} in")
        print(f"  heading max {math.degrees(max(abs(v) for v in heading)):.1f} deg, "
              f"mean {math.degrees(statistics.mean([abs(v) for v in heading])):.1f} deg")

    # --- drive mode time ---------------------------------------------------
    modes = records.get("driveMode", [])
    if modes:
        time_in = {}
        for i, (ts, mode) in enumerate(modes):
            end = modes[i + 1][0] if i + 1 < len(modes) else max(all_ts)
            time_in[mode] = time_in.get(mode, 0) + (end - ts)
        parts = ", ".join(f"{m} {t / 1e6:.1f}s" for m, t in sorted(time_in.items()))
        print(f"\ndrive mode time: {parts}")

    # --- commands ----------------------------------------------------------
    running = records.get("commands/running", [])
    if running:
        print(f"\ncommand-set changes: {len(running)}")

    # --- events ------------------------------------------------------------
    events = records.get("events", [])
    faults = [e for e in events if "COMMAND FAULT" in e[1] or "CRASH" in e[1]]
    corrections = [e for e in events if "pose correction" in e[1]]
    if faults:
        print(f"\n!! {len(faults)} fault/crash event(s):")
        for ts, text in faults:
            print(f"  [{ts / 1e6:8.2f}s] {text.splitlines()[0]}")
    if corrections:
        applied = sum(1 for _, t in corrections if "applied" in t)
        print(f"\npose corrections: {applied} applied, {len(corrections) - applied} rejected")
    if events:
        print(f"\nevent timeline ({len(events)}):")
        for ts, text in events:
            first_line = text.splitlines()[0]
            print(f"  [{ts / 1e6:8.2f}s] {first_line}")
    print()


def main():
    paths = sys.argv[1:]
    if not paths:
        candidates = glob.glob("robot-logs/**/*.wpilog", recursive=True)
        if not candidates:
            print("no .wpilog files under ./robot-logs — run `make pull-logs` first", file=sys.stderr)
            return 1
        paths = [max(candidates, key=os.path.getmtime)]
    for path in paths:
        analyze(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
