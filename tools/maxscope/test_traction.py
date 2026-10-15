import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import traction
from log_viewer import problems

IN_PER_TICK = 0.02
STRAFE_IN_PER_TICK = 0.017
RAD_PER_TICK = 0.0025
TICK_US = 10_000


def run(events=(), samples=3000, flip_front_left=False):
    """A synthetic drive: forward, strafe and turn mixes on a turning heading. events maps a
    sample range to (wheels, ground) overrides in robot-frame [forward in/s, left in/s, omega]."""
    records = {"pose": [], "velocity": [], traction.WHEELS: [], traction.POWER: []}
    heading = 0.0
    for i in range(samples):
        t = i * TICK_US
        wanted = (40 * math.sin(i / 90), 25 * math.sin(i / 140 + 1), 1.5 * math.sin(i / 60 + 2))
        wheels = ground = wanted
        for (first, last), (w, g) in events:
            if first <= i < last:
                wheels, ground = w, g
        wiggle = 0.3 * math.sin(i * 1.7)
        f, l, w = wheels[0] / IN_PER_TICK, wheels[1] / STRAFE_IN_PER_TICK, wheels[2] / RAD_PER_TICK
        tps = [f - l - w, f + l + w, f + l - w, f - l + w]
        if flip_front_left:
            tps[0] = -tps[0]
        vx = ground[0] * math.cos(heading) - ground[1] * math.sin(heading) + wiggle
        vy = ground[0] * math.sin(heading) + ground[1] * math.cos(heading)
        records["pose"].append((t, [0.0, 0.0, heading]))
        records["velocity"].append((t, [vx, vy, ground[2]]))
        records[traction.WHEELS].append((t, tps))
        records[traction.POWER].append((t, [v / 4000 for v in tps]))
        heading += ground[2] * TICK_US / 1e6
    return records


def analyzed(records):
    channels, fit = traction.analyze(records)
    records.update({name: series for name, (_, series) in channels.items()})
    return records, fit


class TractionTest(unittest.TestCase):
    def test_scales_are_fitted_from_the_log(self):
        _, fit = analyzed(run())
        self.assertTrue(fit["ok"])
        self.assertAlmostEqual(fit["forwardInPerTick"], IN_PER_TICK, delta=IN_PER_TICK * 0.02)
        self.assertAlmostEqual(fit["leftInPerTick"], STRAFE_IN_PER_TICK, delta=STRAFE_IN_PER_TICK * 0.02)
        self.assertAlmostEqual(fit["turnRadPerTick"], RAD_PER_TICK, delta=RAD_PER_TICK * 0.02)

    def test_clean_driving_reports_nothing(self):
        records, fit = analyzed(run())
        self.assertEqual([], [p for p in problems(records, {"traction": fit})[0]])

    def test_spinning_wheels_and_a_shove_are_told_apart(self):
        spinning = ((40.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        shoved = ((0.0, 0.0, 0.0), (0.0, -30.0, 0.0))
        records, fit = analyzed(run(events=[((1000, 1050), spinning), ((2000, 2030), shoved)]))
        found = problems(records, {"traction": fit})[0]
        self.assertEqual(["Wheels slipping", "Pushed"], [p["kind"] for p in found])
        self.assertAlmostEqual(found[0]["startSec"], 10.0, delta=0.05)
        self.assertAlmostEqual(found[0]["endSec"], 10.5, delta=0.05)
        self.assertAlmostEqual(found[1]["startSec"], 20.0, delta=0.05)

    def test_skids_while_driving_are_not_flagged(self):
        overrun = ((20.0, 0.0, 0.0), (45.0, 0.0, 0.0))
        spin_while_moving = ((60.0, 0.0, 0.0), (30.0, 0.0, 0.0))
        records, fit = analyzed(run(events=[((1000, 1050), overrun), ((2000, 2050), spin_while_moving)]))
        self.assertEqual([], problems(records, {"traction": fit})[0])

    def test_a_blip_shorter_than_an_event_is_ignored(self):
        blip = ((40.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        records, fit = analyzed(run(events=[((1000, 1004), blip)]))
        self.assertEqual([], problems(records, {"traction": fit})[0])

    def test_a_reversed_encoder_turns_the_check_off_instead_of_flagging_everything(self):
        records, fit = analyzed(run(flip_front_left=True))
        self.assertFalse(fit["ok"])
        found = problems(records, {"traction": fit})[0]
        self.assertEqual(["Traction check off"], [p["kind"] for p in found])

    def test_logs_without_wheel_encoders_add_nothing(self):
        records = run()
        del records[traction.WHEELS]
        channels, fit = traction.analyze(records)
        self.assertEqual(({}, None), (channels, fit))
        self.assertEqual([], problems(records, {})[0])


if __name__ == "__main__":
    unittest.main()
