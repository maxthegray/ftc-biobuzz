# Lab findings

## August 3, 2026

### Setup

REV Color Sensor V3 ×2, REV 2m Distance Sensor ×1, SRS Hub ×1.

**Direct:** Color sensors on Control Hub I2C ports 1 and 2 as `testColor` and
`colorSensor2`; distance sensor on port 3 as `distance`; SRS Hub unplugged.

**SRS:** SRS Hub on Control Hub I2C port 3 as `srsHub`; color sensors on SRS
buses 1 and 2; distance sensor on SRS bus 3.

### Framework smoke test

Passed OpMode lifecycle, PS5 input, command scheduling/interruption/sequences,
intentional fault containment, Driver Station telemetry, Panels over USB,
config persistence, and WPILOG recording/pulling/analysis.

The long run lasted 167.5 seconds and recorded 4,968,159 records across 25
channels, including 13 command-set changes and 16 events. The intentional fault
was logged as `FAULTED`; the default command returned and the OpMode continued.

### WPILOG size

The unpaced smoke test ran at a median 1,899 Hz. Its 167.5-second log was
93,450,172 bytes. After adding a 20 ms diagnostic delay and limiting continuous
channels to 100 Hz, an 18.7-second run at a median 44 Hz produced 19,807 records
across 34 channels and a 334,255-byte log.

Before: `93,450,172 / 167.5 = 557,912 bytes/second`.

After: `334,255 / 18.7 = 17,875 bytes/second`.

Reduction: `1 - 17,875 / 557,912 = 96.80%`, or 31.2× less data per second.
Projected 150-second size dropped from 83.7 MB to 2.68 MB. This includes both
diagnostic pacing and recorder rate limiting.

### One Color Sensor V3

Both routes returned changing RGB/proximity data with no read failures. SRS
disconnect reporting also worked.

**Direct periodic:** median 6.796 ms, mean 8.022 ms, p90 12.023 ms, p99 14.391
ms, maximum 17.829 ms.

**SRS periodic:** median 4.418 ms, mean 4.477 ms, p90 5.151 ms, p99 6.236 ms,
maximum 7.978 ms.

SRS was 34.99% lower at the median, 44.19% lower at the mean, 57.15% lower at
p90, 56.67% lower at p99, and 55.25% lower at the maximum.

Mean reduction: `(8.022 - 4.477) / 8.022 = 44.19%`.

### Three sensors: data validity

**Direct:** 28.7-second run, 26.5 seconds after START, 513 samples per sensor,
zero read failures, zero faults/crashes, and 12.40–12.41 V battery voltage.

Median direct reads were 8.996 ms for color 1, 8.892 ms for color 2, and 6.954
ms for distance. Distance produced 192 distinct values; 509 of 513 were
nonzero/non-`65535`, with three startup zeroes, one `65535`, and a 119 mm
non-sentinel median.

Direct normalized ranges were red 0.000015–0.001129, green
0.000015–0.001129, blue 0.000015–0.000565, alpha 0.000015–0.054844, and
proximity 0–2047 for `testColor`. For `colorSensor2`, they were red
0.000015–0.003128, green 0–0.003311, blue 0–0.001617, alpha
0.000002–0.320958, and proximity 0–2047.

**SRS:** 45.8-second run, 31.0 seconds after START, 1,032 samples per sensor,
zero read failures, zero disconnects, zero CRC mismatches, zero faults/crashes,
and 12.40 V battery voltage.

SRS distance produced 134 distinct values from 0–324 mm with a 21 mm median.
SRS bus 1 raw ranges were red 0–84, green 0–537, blue 0–134, and proximity
164–2047. Bus 2 ranges were red 2–346, green 1–386, blue 1–119, and proximity
71–2047. SRS values are raw integers, so they are not numerically comparable
to the direct normalized values.

### Three sensors: timing

Both OpModes used the same additional 20 ms delay.

```text
Timing                    Direct I2C     SRS Hub       SRS improvement
Periodic median            23.223 ms      4.075 ms      82.46% lower
Periodic mean              23.581 ms      4.377 ms      81.44% lower
Periodic p90               29.642 ms      5.702 ms      80.76% lower
Periodic p99               33.596 ms      7.099 ms      78.87% lower
Periodic maximum           36.423 ms     10.856 ms      70.19% lower

Whole-loop median          51.717 ms     29.035 ms      43.86% lower
Whole-loop p90             62.226 ms     34.474 ms      44.60% lower
Whole-loop p99             75.866 ms     54.161 ms      28.61% lower
Whole-loop maximum         89.058 ms     60.962 ms      31.54% lower
Median loop rate            19.34 Hz      34.44 Hz      78.12% higher
```

Direct median loop rate: `1000 / 51.717 = 19.34 Hz`.

SRS median loop rate: `1000 / 29.035 = 34.44 Hz`.

Rate increase: `(34.44 - 19.34) / 19.34 = 78.12%`, or 1.78×.

Sensor-time reduction: `(23.581 - 4.377) / 23.581 = 81.44%`.

The SRS Hub saved 19.204 ms per loop and is the better route for this
three-sensor setup.

### Source logs

Raw WPILOGs remain local in ignored `robot-logs/`. Primary comparison files:
`DirectThreeSensorStressTestTeleOp-20260803-155704-319-908873903.wpilog` and
`SrsThreeSensorStressTestTeleOp-20260803-160228-167-1623428490.wpilog`.
