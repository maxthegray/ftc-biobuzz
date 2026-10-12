import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import battery
from log_viewer import problems

TICK_US = 10_000
WHEELS = battery.WHEELS


def run(ohms=0.09, peak_amps=8.0, seconds=150, rest_start=13.6, rest_settled=12.4, hubs=(), other_amps=0.0):
    """A battery whose resting voltage settles like a fresh charge, under drive current that
    swings up to peak_amps per motor, plus up to other_amps for a mechanism that runs harder
    the harder the robot drives, which only the hubs see. Drive current is
    sampled round-robin, one motor every 30 ms; each of `hubs` is read in turn every 50 ms, the
    first powering the drive and the rest drawing other_amps."""
    records = {"battery": [], **{f"Drive/motors/{w}/currentAmps": [] for w in WHEELS}}
    for hub in hubs:
        records[f"hubs/{hub}/currentAmps"] = []
        records[f"hubs/{hub}/volts"] = []
    for i in range(seconds * 100):
        t = i * TICK_US
        sec = t / 1e6
        rest = rest_settled + (rest_start - rest_settled) * math.exp(-sec / 30)
        per_motor = [max(0.0, peak_amps * math.sin(sec / 2 + k * 0.3) ** 2) for k in range(4)]
        other = other_amps * sum(per_motor) / (4 * peak_amps)
        volts = rest - ohms * (sum(per_motor) + other) + 0.02 * math.sin(i * 1.3)
        records["battery"].append((t, volts))
        if i % 3 == 0:
            k = (i // 3) % 4
            records[f"Drive/motors/{WHEELS[k]}/currentAmps"].append((t, per_motor[k]))
        if hubs and i % 5 == 0:
            k = (i // 5) % len(hubs)
            amps = sum(per_motor) + (other if len(hubs) == 1 else 0.0) if k == 0 else other
            records[f"hubs/{hubs[k]}/currentAmps"].append((t, amps))
            records[f"hubs/{hubs[k]}/volts"].append((t, volts - 0.05 * k))
    return records


class BatteryTest(unittest.TestCase):
    def test_resistance_is_measured_while_the_resting_voltage_settles(self):
        channels, fit = battery.analyze(run(ohms=0.09))
        self.assertTrue(fit["ok"])
        self.assertAlmostEqual(fit["resistanceOhms"], 0.09, delta=0.009)
        self.assertAlmostEqual(fit["restingStartVolts"], 13.6, delta=0.25)
        self.assertAlmostEqual(fit["restingEndVolts"], 12.4, delta=0.1)
        self.assertAlmostEqual(fit["headroomAmps"], (fit["restingEndVolts"] - 8.0) / fit["resistanceOhms"], delta=3)
        self.assertEqual({battery.PREFIX + "currentAmps", battery.PREFIX + "restingVolts"}, set(channels))
        self.assertEqual("drive motors", fit["source"])

    def test_a_total_needs_all_four_motors_sampled_recently(self):
        records = run()
        self.assertTrue(battery.drive_rows(records))
        records["Drive/motors/rightRear/currentAmps"] = [(0, 1.0)]
        self.assertEqual([], [r for r in battery.drive_rows(records) if r[0] > battery.MAX_CURRENT_AGE_US])

    def test_hub_readings_count_loads_the_drive_motors_miss(self):
        records = run(ohms=0.09, hubs=("control",), other_amps=12.0)
        _, fit = battery.analyze(records)
        self.assertEqual("hubs", fit["source"])
        self.assertAlmostEqual(fit["resistanceOhms"], 0.09, delta=0.0045)
        del records["hubs/control/currentAmps"], records["hubs/control/volts"]
        _, drive_only = battery.analyze(records)
        self.assertEqual("drive motors", drive_only["source"])
        self.assertGreater(drive_only["resistanceOhms"], 0.11)

    def test_hubs_are_summed_against_the_control_hubs_voltage(self):
        records = run(ohms=0.09, hubs=("control", "expansion2"), other_amps=12.0)
        rows = battery.hub_rows(records)
        control_volts = dict(records["hubs/control/volts"])
        self.assertTrue(rows)
        self.assertTrue(all(control_volts[ts] == v for ts, _, v in rows))
        _, fit = battery.analyze(records)
        self.assertAlmostEqual(fit["resistanceOhms"], 0.09, delta=0.009)

    def test_light_driving_reports_no_resistance(self):
        channels, fit = battery.analyze(run(peak_amps=1.5))
        self.assertFalse(fit["ok"])
        self.assertEqual({battery.PREFIX + "currentAmps"}, set(channels))
        self.assertEqual([], battery.problems(fit))

    def test_healthy_and_failing_batteries(self):
        self.assertEqual([], battery.problems(battery.analyze(run(ohms=0.09))[1]))
        # Like the runs that browned out: about 15 A sagging a charged battery to 5-6 V.
        records = run(ohms=0.45, peak_amps=4.0)
        _, fit = battery.analyze(records)
        self.assertGreater(min(v for _, v in records["battery"]), 4.0)
        [(kind, severity, at, detail)] = battery.problems(fit)
        self.assertEqual(("High battery resistance", "fault"), (kind, severity))
        self.assertEqual(min(records["battery"], key=lambda p: p[1])[0], at)
        self.assertRegex(detail, r"^4[45]\d mΩ battery")
        _, fit = battery.analyze(run(ohms=0.18))
        self.assertEqual("warn", battery.problems(fit)[0][1])

    def test_problems_scan_includes_the_battery_model(self):
        records = run(ohms=0.45, peak_amps=4.0)
        _, fit = battery.analyze(records)
        found, _ = problems(records, {"batteryModel": fit})
        self.assertIn("High battery resistance", [p["kind"] for p in found])

    def test_logs_without_motor_current_add_nothing(self):
        records = {"battery": run()["battery"]}
        self.assertEqual(({}, None), battery.analyze(records))
        self.assertEqual([], battery.problems(None))


if __name__ == "__main__":
    unittest.main()
