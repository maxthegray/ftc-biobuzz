"""Wheels vs Pinpoint: did the robot move the way its wheels turned?

The drive encoders and the Pinpoint both measure the robot's velocity. Where
they disagree, either the wheels turned without the robot following (slipping,
or pushing into something) or the robot moved without its wheels (pushed by
something). The wheel-to-inch scales are fitted from each log, so nothing here
depends on wheel size, gearing or which robot recorded it.
"""

import math
import statistics

WHEELS = "Drive/motors/velocityTps"
PREFIX = "MaxScope/traction/"

# Fitting: an axis is fitted from samples where it turns at least this share of its own
# 95th-percentile speed, so standing still and encoder noise don't vote.
FIT_MIN_SHARE = 0.25
FIT_MIN_SAMPLES = 50
# A fit whose typical mismatch in ordinary driving is above this share of wheel speed is not
# trustworthy (an encoder reversed or unplugged), so no problems are reported from it.
FIT_MAX_TYPICAL_ERROR = 0.3
SMOOTH_SAMPLES = 5

# Initial guesses, to be tuned on a run with known hits and pushes.
MISMATCH_MIN_IN_S = 8.0
MISMATCH_SHARE_OF_SPEED = 0.25
MIN_EVENT_SEC = 0.1
EVENT_MERGE_SEC = 0.5


def wheel_axes(tps):
    """Pedro's mixer order [leftFront, rightFront, leftRear, rightRear] to [forward, left, turn ccw]."""
    fl, fr, bl, br = tps
    return (fl + fr + bl + br) / 4, (-fl + fr + bl - br) / 4, (-fl + fr - bl + br) / 4


def robot_frame(pose, velocity):
    """Field-frame [vx, vy, omega] to [forward, left, omega] at the pose's heading."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return velocity[0] * c + velocity[1] * s, -velocity[0] * s + velocity[1] * c, velocity[2]


def finite(values, length):
    return isinstance(values, (list, tuple)) and len(values) >= length and all(map(math.isfinite, values[:length]))


def paired(records):
    """(ts, wheel axes, ground axes) for every tick that logged all three channels."""
    poses = {ts: v for ts, v in records.get("pose", []) if finite(v, 3)}
    velocities = {ts: v for ts, v in records.get("velocity", []) if finite(v, 3)}
    return [(ts, wheel_axes(tps), robot_frame(poses[ts], velocities[ts]))
            for ts, tps in records.get(WHEELS, [])
            if finite(tps, 4) and ts in poses and ts in velocities]


def fit_scale(wheel, ground):
    """Ground units per wheel unit on one axis: the median ratio, so hits and slips don't skew it."""
    moving = sorted(abs(w) for w in wheel)
    if not moving:
        return None
    floor = FIT_MIN_SHARE * moving[int(0.95 * (len(moving) - 1))]
    ratios = [g / w for w, g in zip(wheel, ground) if floor > 0 and abs(w) >= floor]
    return statistics.median(ratios) if len(ratios) >= FIT_MIN_SAMPLES else None


def smooth(values):
    half = SMOOTH_SAMPLES // 2
    return [statistics.fmean(values[max(0, i - half):i + half + 1]) for i in range(len(values))]


def analyze(records):
    """Derived channels (name -> (type, series)) and the fit, or ({}, None) without wheel encoders."""
    samples = paired(records)
    if not samples:
        return {}, None
    wheel = list(zip(*(w for _, w, _ in samples)))
    ground = list(zip(*(g for _, _, g in samples)))
    forward = fit_scale(wheel[0], ground[0])
    if forward is None:
        return {}, {"ok": False, "reason": "the robot never drove forward or back far enough to fit"}
    # Mecanum strafing loses some speed to the rollers, so it gets its own scale when there is
    # enough strafing to fit one; otherwise the ideal mecanum assumption, the forward scale.
    left = fit_scale(wheel[1], ground[1]) or forward
    turn = fit_scale(wheel[2], ground[2])
    # Inches a wheel travels per radian of robot turn: turn mismatch in the same in/s as the rest.
    radius = abs(forward / turn) if turn else None

    times, wheels_in, ground_in, raw = [], [], [], []
    for ts, w, g in samples:
        mine = (w[0] * forward, w[1] * left, w[2] * turn if turn else math.nan)
        times.append(ts)
        wheels_in.append(mine)
        ground_in.append(g)
        raw.append([g[0] - mine[0], g[1] - mine[1], (g[2] - mine[2]) * radius if radius else 0.0])
    diff = list(zip(*(smooth(list(axis)) for axis in zip(*raw))))
    mismatch = [math.hypot(*d) for d in diff]
    speed = [math.hypot(w[0], w[1], w[2] * radius if radius else 0.0) for w in wheels_in]
    threshold = [MISMATCH_MIN_IN_S + MISMATCH_SHARE_OF_SPEED * s for s in speed]

    moving = [m / s for m, s in zip(mismatch, speed) if s > MISMATCH_MIN_IN_S]
    typical = statistics.median(moving) if moving else None
    fit = {"ok": typical is not None and typical <= FIT_MAX_TYPICAL_ERROR,
           "forwardInPerTick": forward, "leftInPerTick": left,
           "turnRadPerTick": turn, "wheelRadiusIn": radius, "typicalError": typical}
    if not fit["ok"]:
        fit["reason"] = ("the robot barely moved" if typical is None
                         else f"wheels and Pinpoint disagree by {typical:.0%} in ordinary driving; "
                              "check encoder directions")

    channels = {
        PREFIX + "wheels": ("double[]", [(ts, list(v)) for ts, v in zip(times, wheels_in)]),
        PREFIX + "pinpoint": ("double[]", [(ts, list(v)) for ts, v in zip(times, ground_in)]),
        PREFIX + "mismatchInPerS": ("double", list(zip(times, mismatch))),
        PREFIX + "thresholdInPerS": ("double", list(zip(times, threshold))),
        # Positive: the wheels went faster than the robot (slipping or pushing something).
        # Negative: the robot went faster than its wheels (pushed).
        PREFIX + "wheelsMinusPinpointInPerS": ("double", [
            (ts, s - math.hypot(g[0], g[1], g[2] * radius if radius else 0.0))
            for ts, s, g in zip(times, speed, ground_in)]),
    }
    return channels, fit


def problems(records, fit):
    """(kind, start_us, end_us, detail) for each stretch where the mismatch stays over threshold."""
    if fit is None:
        return []
    if not fit["ok"]:
        first = min((series[0][0] for series in records.values() if series), default=0)
        return [("Traction check off", first, first, fit["reason"])]
    mismatch = records.get(PREFIX + "mismatchInPerS", [])
    threshold = dict(records.get(PREFIX + "thresholdInPerS", []))
    balance = dict(records.get(PREFIX + "wheelsMinusPinpointInPerS", []))
    found, current = [], None
    for ts, m in mismatch:
        if m > threshold[ts]:
            if current and ts - current["end"] <= EVENT_MERGE_SEC * 1e6:
                current["end"] = ts
            else:
                current = {"start": ts, "end": ts, "peak": 0.0, "balance": 0.0}
                found.append(current)
            current["peak"] = max(current["peak"], m)
            current["balance"] += balance[ts]
    result = []
    for span in found:
        if span["end"] - span["start"] < MIN_EVENT_SEC * 1e6:
            continue
        kind = "Wheels slipping" if span["balance"] > 0 else "Pushed"
        detail = (f"peak {span['peak']:.0f} in/s mismatch · {(span['end'] - span['start']) / 1e6:.2f} s · "
                  + ("wheels faster than the robot" if span["balance"] > 0 else "robot moved without its wheels"))
        result.append((kind, span["start"], span["end"], detail))
    return result
