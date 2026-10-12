"""How much the battery sags per amp: the resistance of the battery and its wiring.

The hub reads voltage = resting voltage − resistance × current. The resting
voltage drifts through a run (a fresh charge settles, a weak battery fades), so
it is tracked as a rolling median of what the voltage would be with no load,
and the resistance is the slope of the sag below it against total current.

Current and voltage come from the Control Hub (`hubs/control/currentAmps` and
`volts`, read together): everything it powers. Older logs only have the drive
motors' current and the `battery` channel, so other loads go uncounted there.

There is no good or bad threshold: the resistance depends on the battery, its
wiring and the robot, so it is for comparing runs, not for flagging one.
"""

import bisect
import math
import statistics

WHEELS = ("leftFront", "rightFront", "leftRear", "rightRear")
PREFIX = "MaxScope/battery/"
MIN_VALID_VOLTS = 1.0
BROWNOUT_VOLTS = 8.0
# Drive motors are read one at a time; a total needs all four read this recent.
MAX_CURRENT_AGE_US = 300_000
REST_WINDOW_SEC = 20.0
REST_GRID_SEC = 1.0
# The resting voltage depends on the resistance and the resistance on the resting voltage;
# alternating the two converges on the one pair that agrees.
FIT_MAX_ITERATIONS = 100
FIT_TOLERANCE_OHMS = 1e-5
# A resistance needs real load to measure: this many totals at or above LOADED_AMPS. Fits from
# light driving still read low.
LOADED_AMPS = 10.0
FIT_MIN_LOADED = 20


def finite(value):
    return isinstance(value, float) and math.isfinite(value)


def totals(series):
    """(ts, source, total amps) at each reading where every source was read recently."""
    samples = sorted((ts, name, amps) for name, points in series.items() for ts, amps in points if finite(amps))
    latest, result = {}, []
    for ts, name, amps in samples:
        latest[name] = (ts, amps)
        if len(latest) == len(series) and all(ts - t <= MAX_CURRENT_AGE_US for t, _ in latest.values()):
            result.append((ts, name, sum(a for _, a in latest.values())))
    return result


def hub_rows(records):
    """(ts, amps, volts) from the Control Hub's own readings. Expansion Hubs are left out: if the
    Control Hub's reading includes what it passes through to them, adding theirs double-counts."""
    volts = dict(records.get("hubs/control/volts", []))
    return [(ts, amps, volts[ts]) for ts, amps in records.get("hubs/control/currentAmps", [])
            if finite(amps) and finite(volts.get(ts)) and volts[ts] > MIN_VALID_VOLTS]


def drive_rows(records):
    """(ts, total drive amps, volts) for logs without hub readings: the four drive motors'
    current, each paired with the latest `battery` reading."""
    battery = [(ts, v) for ts, v in records.get("battery", []) if finite(v) and v > MIN_VALID_VOLTS]
    battery_ts = [ts for ts, _ in battery]
    rows = []
    for ts, _, amps in totals({w: records.get(f"Drive/motors/{w}/currentAmps", []) for w in WHEELS}):
        i = bisect.bisect_right(battery_ts, ts) - 1
        if i >= 0:
            rows.append((ts, amps, battery[i][1]))
    return rows


def rolling_rest(rows, resistance):
    """Resting voltage at each row: the rolling median of voltage + resistance × current,
    evaluated on a coarse grid and interpolated, so long runs stay fast."""
    times = [ts for ts, _, _ in rows]
    unloaded = [v + resistance * amps for _, amps, v in rows]
    half, step = REST_WINDOW_SEC * 1e6 / 2, REST_GRID_SEC * 1e6
    grid, values = [], []
    t = times[0]
    while t <= times[-1] + step:
        lo, hi = bisect.bisect_left(times, t - half), bisect.bisect_right(times, t + half)
        if hi > lo:
            grid.append(t)
            values.append(statistics.median(unloaded[lo:hi]))
        t += step
    rest = []
    for ts in times:
        i = bisect.bisect_right(grid, ts)
        if i == 0 or i == len(grid):
            rest.append(values[0] if i == 0 else values[-1])
        else:
            f = (ts - grid[i - 1]) / (grid[i] - grid[i - 1])
            rest.append(values[i - 1] + f * (values[i] - values[i - 1]))
    return rest


def analyze(records):
    """Derived channels (name -> (type, series)) and the fit, or ({}, None) without current logs."""
    rows, source = hub_rows(records), "hubs"
    if not rows:
        rows, source = drive_rows(records), "drive motors"
    if not rows:
        return {}, None
    channels = {PREFIX + "currentAmps": ("double", [(ts, amps) for ts, amps, _ in rows])}
    if sum(amps >= LOADED_AMPS for _, amps, _ in rows) < FIT_MIN_LOADED:
        return channels, {"ok": False, "source": source, "reason": "not enough driving load to measure"}

    resistance = 0.0
    for _ in range(FIT_MAX_ITERATIONS):
        rest = rolling_rest(rows, resistance)
        previous = resistance
        resistance = sum(a * (r - v) for (_, a, v), r in zip(rows, rest)) / sum(a * a for _, a, _ in rows)
        if abs(resistance - previous) < FIT_TOLERANCE_OHMS:
            break
    rest = rolling_rest(rows, resistance)
    channels[PREFIX + "restingVolts"] = ("double", [(ts, r) for (ts, _, _), r in zip(rows, rest)])
    lowest_rest = min(rest)
    return channels, {
        "ok": True,
        "source": source,
        "resistanceOhms": resistance,
        "restingStartVolts": rest[0],
        "restingEndVolts": rest[-1],
        # Total current at which the weakest resting voltage of the run would sag to brownout.
        "headroomAmps": (lowest_rest - BROWNOUT_VOLTS) / resistance if resistance > 0 else None,
    }

