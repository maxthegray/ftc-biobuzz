# Logging

Logging is automatic. Students expose public read-only subsystem state and use
logged command factories and autonomous steps; Robot owns recording, timing,
events, hub power and command history. Ivy remains the only scheduler.

## Compatibility rules

- Never rename a channel read by `tools/`: lookups use exact names and a rename
  silently empties graphs or removes fault flags. Preserve explicit channel
  layouts for drive, localizer, Limelight, ball camera and HIVE tracking.
- Never remove, bypass or weaken `logged(...)`. It implements command fault
  containment: only the failing command ends, its requirements receive
  `onCommandFault()`, and a parent sequential continues to its next step.
  Keep wrappers in subsystem factories and around autonomous steps; preserve
  `LoggedCommand` exception handling in `core/logging/CommandHistory.kt`.

## Flight recorder and WPILOG channels

`FlightRecorder` writes `/sdcard/FIRST/logs/<OpMode>-<timestamp>-<n>.wpilog`
(30 files kept). Channels (sampled at ≤ 100 Hz unless noted):

| Channel | Type | Meaning |
|---|---|---|
| `Field/Robot` | `struct:Pose2d` | Pose in the FTC field frame for AdvantageScope's 2D field: metres about the centre, axes and heading turned by `RobotConfig.Field.FIELD_VIEW_QUARTER_TURNS` (display only) |
| `pose` | `double[]` | `[x in, y in, heading rad]`, raw Pedro frame |
| `velocity` | `double[]` | `[vx in/s, vy in/s, omega rad/s]`, field frame |
| `driveMode` | string | `IDLE`, `TELEOP`, `FOLLOWING`, `HOLDING`, `ROBOT_CENTRIC_FALLBACK` |
| `follow/translationalErrorIn`, `follow/headingErrorRad` | double | Foresight errors, only while following/holding |
| `gamepad1/axes`, `gamepad2/axes` | `double[]` | `[lx, ly(+up), rx, ry(+up), L2, R2]` after deadband |
| `gamepad1/buttons`, `gamepad2/buttons` | int64 | bits: Cross Circle Square Triangle L1 R1 up down left right Options Create L3 R3 Touchpad |
| `battery` | double | volts: the Control Hub's input voltage from `Robot.hubPower` |
| `loop/totalNanos`, `loop/<phase>Nanos`, `loop/windowMax…` | int64 | loop timing and per-window peaks |
| `<Subsystem>/…` | any | `SubsystemBase.logState` channels |
| `Drive/motors/power`, `Drive/motors/velocityTps` | `double[]` | every tick, `[leftFront, rightFront, leftRear, rightRear]` (Pedro's mixer order): the power Pedro commanded (0 once the drivetrain stops) and the encoder velocity in ticks/s from the bulk read |
| `Drive/motors/<wheel>/currentAmps` | double | one motor sampled every 200 ms round-robin (each every 800 ms), logged once at the next sample; `getCurrent` is a real Lynx transaction |
| `hubs/<hub>/currentAmps`, `hubs/<hub>/volts` | double | `<hub>` is `control` or `expansion<address>`: each hub's input current and voltage, read back to back by `Robot.hubPower`, one hub every 50 ms round-robin, logged once at the next sample; a 0 V reading (the hub after stop) is dropped. Counts everything the hub powers; MaxScope's battery analysis uses it. The Control Hub's voltage also feeds `battery` |
| `BallCamera/candidates/…` | `double[]` | every blob that passed the filters in the current frame, largest first: `xPx`, `yPx`, `radiusPx`, `areaPx`, `circularity`, `horizontalDeg`, `verticalDeg` (+ below the axis), and `selectedIndex` (0, or −1 when none) |
| `BallCamera/mount/…` | double/boolean | `BallCamera.mount*` as recorded: `measured`, `heightIn`, `pitchDownDeg`, `forwardIn`, `leftIn`, `yawDeg` |
| `Hive/<RED\|BLUE>/…` | mixed | `state` (`AUDIENCE_RAISED`, `FAR_RAISED`, `UNKNOWN`), `stateAssumed`, `tipCount`, and the goal: `visible`, `cell`, `turretBearingDeg`, `robotBearingDeg`, `distanceIn` (horizontal, from the turret axis), `heightIn`, `spreadIn`, `ageMs` (since estimated capture), `tagIds`, `reprojected`; the aim goal: `aimSource` (`VISION`/`ODOMETRY`), `aimTurretBearingDeg`, `aimRobotBearingDeg`; `fieldGoalErrorIn` (vision goal vs `HiveField`) |
| `Hive/tags/…` | `double[]` | every season tag of the latest processed frame: `id`, `heightIn`, `class` (1 raised, −1 lowered, 0 ambiguous), `deviationIn` (implied goal's distance from its CELL's fused goal) |
| `Hive/mount/…`, `Hive/settings/raisedMinHeightIn`, `Hive/frames/…` | mixed | `HiveConfig` camera mount and threshold as recorded; frames processed and dropped for want of a turret angle |
| `events` | string | explicit events with their own timestamps (not sampled); strictly increasing, a same-microsecond event moves 1 µs later |
| `commands/events` | string | lifecycle records of `logged` commands, own timestamps, strictly increasing (see below) |
| `commands/active` | string | traced executions open after each record, `#id name` per line, `(suspended)` suffix |
| `commands/lost` | int64 | records dropped from the bounded queue; 0 at open, rewritten when it grows |

Subsystem state logging is automatic: public read-only scalar getters declared on
the subsystem class become property-name channels. Double, Float, Int, Long,
Boolean, String and enum getters are supported; enums use their names and nulls
are skipped. Private, inherited, static, synthetic and publicly writable state
is excluded. Discovery uses cached Java reflection, without kotlin-reflect.
`getPower` becomes `power`, and `isReady` becomes `ready`. A throwing getter's
Exception skips that value and the remaining getters still run; Errors retain
the normal lifecycle policy. Override `logState(log)` only for arrays or specific channel names;
an override replaces the automatic logging. Record events with `Events.record("text")`. `put` also takes a `DoubleArray` for one row of
parallel values (a per-frame detection list); its length may change per tick,
and it is written immediately, so a cached array can be passed again.

### Command history

```kotlin
fun raise(): Command = logged("Lift raise", Command.build().requiring(this).setDone { atTop })
drive.followCommand(path, name = "Drive to bar")   // drive factories are already logged
```

- `logged` returns a `LoggedCommand` that forwards requirements, priority, the
  three behaviours and normal return values to the wrapped command. Exceptions
  are contained by the command fault policy; Errors still propagate. Schedule, cancel, bind and compare **the returned instance**;
  Ivy never sees the inner one. `logged` on a logged command renames it.
  `SubsystemBase.defaultCommand` takes any `Command` and sets CANCEL on the
  builder inside a `LoggedCommand`.
- Records: `START #id name`, `FINISH` (its `done` returned true, not
  arrival), `INTERRUPT`, `SUSPEND`/`RESUME` (Ivy's SUSPEND behaviour; resume is
  seen at the next `execute`), `FAIL #id name in <start|execute|done|end>:
  <Exception>: <message>`, `FAIL #- name …` when a command threw outside a traced
  run, and `ABORT #id name: command fault | op-mode stop | restarted without an
  end` for runs Ivy dropped without `end`.
- Ivy quirks handled: `end` without `start` and a second `end` record nothing;
  `loop`/`repeat` restarting the same instance is a new id; `Scheduler.reset`
  skips `end`, so `Robot` closes open runs with ABORT **after** subsystems are
  halted (fault policy) or stopped (op-mode stop). ABORT never calls command
  code.
- `CommandHistory` is static like Ivy's scheduler and reset by every `Robot`.
  Its open runs (≤ 64) are the truth; each change is timestamped on the robot
  clock and queued (≤ 512) with the resulting active set, then written by the
  recorder each loop and at close. Overflow and tracing errors increment
  `lost` and write `HISTORY INCOMPLETE: N command records lost`; the command
  itself is unaffected by tracing failures. Command exceptions follow the
  fault policy above.
- **Coverage is instrumented commands only.** A logged group does not expose
  unlogged children. There are no blocked-schedule events and no interruption
  causes: Ivy's public API does not reveal them. Never describe
  `commands/events` as a complete command history.
- Instrumented today: the five drive factories (default names `Drive
  teleop`, `Drive follow`, `Drive hold`, `Drive turn`, `Drive robot-centric
  fallback`), TeleOpBase's default `Drive teleop`, `Drive robot-centric
  fallback` and `Reset heading`,
  Localization Test's moves and `Driver takeover`, Example Auto's routine and
  steps. Instrument new mechanism commands and autonomous steps the same way;
  don't wrap trivial `instant`s or marker internals.

Failure isolation: an I/O failure disables the recorder for the run; a
non-I/O exception in one subsystem's `logState` disables that subsystem's
channels and records why; anything else escaping the recorder closes it. The
loop keeps running.

- `#id` numbers each run from 1 per op-mode, so two runs of the same command,
  or two commands with the same name, stay apart.
- **FINISH means the command's own `done` returned true**, not that the robot
  arrived: a hold that timed out, or a path at its parametric end, also
  finishes. Check pose and `follow/*` for arrival.
- **FAIL** is the command whose `start`, `execute`, `done` or `end` threw; a
  parent sequential continues after that child ends. **ABORT** is
  bookkeeping for commands that were still running when the fault policy or
  the op-mode stop cleared Ivy: they did not fail, and their end handlers did
  not run.
- Nothing records why a command was interrupted (driver takeover, a group's
  timeout, cancel) or that a schedule was rejected: Ivy doesn't say. Read the
  surrounding records and `events`.

## Events and settings

`Events` is static like Ivy's Scheduler and CommandHistory. Robot resets its
sink at construction, wires it when the recorder opens, and clears it at close
or stop. `Robot.recordEvent` delegates to `Events.record`. Tests without a Robot
can set the internal sink and reset it in setup/teardown. Event text is a tool
interface: preserve parsed prefixes such as `LOCALIZER FAULT`, `LOOP OVERRUN`
and `TELEMETRY FAULT` verbatim.

`SettingsChangeLog` records a config section initially, then only changed
fields, using the robot clock. Slider edits are merged to at most one event
per second; an edit reverted within the window emits nothing, and an edit in
the last second before stop can be missing.

**Tuning in the flight log.** Both OpModes write the `visionDiagnostics`
values as an event at init. The ball camera writes every `ballVision` value
at init, then an event naming only the fields that changed (at most one per
second; slider drags are merged). An edit in the last second before stop can
be missing from the log.

The ball camera's `BallCamera/mount/*` channels record measured status, height,
pitch, forward/left offset and yaw each tick. MaxScope uses them to place
candidates on the field. Copy tuned values into config code before restarting;
Panels edits are not persisted.

## Hub power sampling

`Robot.hubPower` reads one hub every 50 ms, round-robin, current and voltage
back to back. The Control Hub voltage supplies `battery` and drive voltage
compensation. Each sample is logged once; 0 V after stop is dropped.
`hubs/<hub>/currentAmps` includes everything powered by that hub, while drive
motor current uses a separate 200 ms round-robin (800 ms per motor).
MaxScope's battery analysis uses hub samples; traction uses drive power,
encoder velocity and motor current. These slow Lynx transactions stay out
of student code.

## Review a WPILOG in MaxScope

Run `make viewer` and open `http://127.0.0.1:8008`. Choose an already-pulled
run or **Open WPILOG** to inspect another file. Field playback, graphs,
commands, events and gamepads share one time cursor. The field reads raw
Pedro `pose` in inches/radians: origin at the audience-left corner, +X right,
+Y away, heading counterclockwise. No AdvantageScope coordinate conversion
is needed. Robot logging remains compatible with both viewers.

MaxScope runs locally with Python 3.10+ and no extra dependencies.
See [MaxScope](../tools/maxscope/README.md) for controls, limits and tests.

## Download a WPILOG and open it in AdvantageScope

1. Connect to the Control Hub over USB or join its Wi-Fi.
2. `make pull-logs 2` pulls the newest match's logs into `robot-logs/`;
   `make pull-logs` copies all of them. `make viewer` summarizes them.
3. Open the `.wpilog` in AdvantageScope (File → Open Log).
4. **2D field:** drag `Field/Robot` onto a 2D Field tab with an FTC field
   (default *Center/Rotated* coordinates). It is already in the FTC frame:
   Pedro's (0, 0) draws at the bottom-left corner as seen from the audience,
   and heading 0 points right along the audience wall. Don't drag `pose`:
   AdvantageScope reads its inches as metres. **Graphs:** use `pose` (inches),
   `velocity`, `driveMode`, `follow/translationalErrorIn`, `battery`,
   `loop/totalNanos`, and your subsystem channels. `events` is the timeline.
5. **What was the robot trying to do?** Open `commands/events` in a table
   (or drag it onto a line graph) next to `driveMode`, `pose` and motor
   channels, and scrub to the moment in question; `commands/active` shows
   every traced command running then. MaxScope (`make viewer`) lists each execution
   with start, end and outcome, and failures first.

## Post-match debugging

`make pull-logs [N]` pulls the newest N (default all 30 the hub keeps);
`make viewer` opens MaxScope; `python3 tools/logs/analyze_wpilog.py --json`
emits the JSON bundle (default: newest log under `robot-logs/`). It reports `commandHistoryCoverage`:
`"all scheduled"` (pre-Ivy logs, `commands/running`), `"none"` (Ivy logs before
command tracing) or `"instrumented"` (current logs, `logged` commands only,
with `commandExecutions`, `commandFailures`, `commandActive` and
`commandHistoryIntact`/`commandHistoryLostRecords`). All its times, like MaxScope's, count from the
op-mode start (`clock`, `opModeStartLogSec`; init is negative), so a time the user reads off
MaxScope matches the bundle. When a log doesn't determine the cause, give ranked
hypotheses and the one channel or reproduction that decides it.

### Assistant workflow

When the user is plugged into the hub and reports a match problem, run
**`make pull-logs 2`**, then
`python3 tools/logs/analyze_wpilog.py --json robot-logs/<file>`, anchor on the stated symptom, and drill into channels with
`python3 tools/logs/analyze_wpilog.py --json --channel <name,name> robot-logs/<file>`.
The auton run is usually the second pulled file. For "what was it doing":
read `commandFailures`, then the `commandExecutions` overlapping the symptom
time, then `--channel commands/active,driveMode,pose` around it. State the
coverage: an absent command may simply be unlogged.

### Watching the robot on AdvantageScope's 2D field

Drag **`Field/Robot`** onto the 2D Field tab and pick an FTC field; its default
coordinate system (**Center/Rotated**) is the one the channel is written for.
`Field/Robot` is a WPILib `Pose2d` struct in the FTC field frame: metres about
the field centre, rotated from Pedro's axes by
`RobotConfig.Field.FIELD_VIEW_QUARTER_TURNS`. Graph `pose` (raw Pedro inches)
when comparing against `Constants.java`; never put `pose` on the field, where
its inches are read as metres.

**Which way is which.** Pedro's frame is fixed by the audience: (0, 0) is the
corner on the audience's left, +X runs right along the audience wall, +Y runs
away from the audience. The FTC frame is fixed by the red wall: +Y points from
the red wall to the blue wall, and AdvantageScope draws it with the audience
at the bottom of the screen. In BIOBUZZ the red wall is on the audience's left
(the alliances are swapped from the usual layout), so the two frames differ by
one quarter turn counter-clockwise and the constant is `1`. With the usual
layout (red on the right) it is `-1`. Set it when the game launches, next to
`SYMMETRY`. On screen a BIOBUZZ log shows Pedro (0, 0) at the bottom-left
corner and heading 0 pointing right; before this rotation existed the robot
drew at the top-left facing down.

**Verify the axes once on a real field.** Park the robot in Pedro's origin
corner (audience side, red side in BIOBUZZ) facing along the audience wall,
start a teleop, drive one tile forward. The drawn robot must sit in the
bottom-left corner pointing right and move right. A quarter-turn error means
`FIELD_VIEW_QUARTER_TURNS` is wrong for this field; anything else is the
localizer. The rotation is display only — autonomous start poses, paths and
`Alliance` stay in Pedro's frame and need no change when it changes.

### Flight recorder validation

Three separate levels; passing one says nothing about the next.

1. **Decoder (host, automated).** `FlightRecorderDriveIntegrationTest` runs
   teleop, a path, a driver takeover, a command fault, an explicit event, pose
   and heading changes and shutdown through the real `Robot`, Ivy and Pedro
   drive, decodes the WPILOG, and checks every `Field/Robot` sample against
   `(-(y − c) · 0.0254, (x − c) · 0.0254, heading + π/2)` of its `pose` sample
   (`c = FIELD_CENTRE_INCHES` for BIOBUZZ), the
   last sample against the last real pose, event order and the channel set,
   and the command history of that session: exact `commands/events` order and
   ids, strictly increasing timestamps aligned with `driveMode` and the
   `COMMAND FAULT` event, the active set during the nested route and the
   takeover, and nothing active or lost at the end. `CommandHistoryTest` covers
   the Ivy lifecycle cases (cancel before start, deadline double end, suspend,
   repeat, identical names, faults, stop, bounded loss, recorder failure).
   To keep that log, run the test with an output directory outside the repo
   (`--rerun`, because Gradle does not see the variable change):

   ```sh
   WPILOG_SAMPLE_DIR=../biobuzz-command-history-validation ./gradlew :TeamCode:testDebugUnitTest \
     --tests '*FlightRecorderDriveIntegrationTest' --rerun
   python3 tools/logs/analyze_wpilog.py ../biobuzz-command-history-validation/IntegrationSample.wpilog
   ```
2. **AdvantageScope GUI (manual).** Open `IntegrationSample.wpilog` in
   AdvantageScope (File → Open Log). Expected:
   - The sidebar lists `Field/Robot` with `translation`/`rotation` children,
     `pose`, `driveMode`, `events`, `follow/*`, `gamepad1/*`, `loop/*`,
     `Drive/*`, `Localizer/*`.
   - Add a **2D Field** tab, pick an FTC field, keep **Center/Rotated**, and
     drag `Field/Robot` on. Scrub the timeline (values are the FTC frame,
     BIOBUZZ quarter turn applied): the robot starts at (0.41, −1.63) m
     facing 90° (Pedro (8, 56) in), near the left wall below the centre
     line with the arrow pointing right; moves to (0.38, −1.32) m; turns to
     120° at (0.56, −1.07) m; and ends at (0.81, 0.46) m facing 180°
     (Pedro (90, 40) in). It does not jump to the field centre at the end.
   - A **Line Graph** of `driveMode` and the `events` list show TELEOP →
     FOLLOWING → TELEOP → IDLE → TELEOP and the four events `init
     IntegrationTest`, `COMMAND FAULT: …`, `marker`, `stop` in that order.
   - Add a **Table** tab with `commands/events`, `commands/active` and
     `driveMode`. Fourteen `commands/events` rows, none missing: `START #1
     Driver sticks` at 0.010 s; `INTERRUPT #1`, `START #2 Route`, `START #3
     Follow line` at 0.040 s (the first FOLLOWING row); `INTERRUPT #3`,
     `INTERRUPT #2 Route`, `START #4 Driver takeover` at 0.080 s; `INTERRUPT
     #4`, `START #5 Driver sticks` at 0.090 s; `INTERRUPT #5`, `START #6
     Faulty step`, `FAIL #6 Faulty step in execute: IllegalStateException:
     boom` at 0.100 s (with `COMMAND FAULT` in `events`); `START #7 Driver
     sticks` at 0.110 s; `ABORT #7 Driver sticks: op-mode stop` at 0.120 s.
     `commands/active` reads `#2 Route` / `#3 Follow line` while following,
     `#4 Driver takeover` during the takeover, and is empty at the end;
     `commands/lost` is 0.
3. **Hardware (manual).** Park the robot at a known field pose, init an
   op-mode, drive one tile forward and turn 90°, stop, pull the log, and
   confirm the 2D field shows the same start, direction and end (see "Verify
   the axes" above). In Localization Test, press Triangle, move a stick mid-path, and
   confirm `commands/events` shows `START … Localization test path`, then
   `INTERRUPT` of it with `START … Driver takeover` at the moment the sticks
   moved, and `Drive teleop` resuming after release.

### Symptom triage

| Symptom | First evidence to check |
|---|---|
| Op-mode stopped | Driver Station exception, `LOOP CRASHED` event, loop phase maxima, minimum battery |
| A mechanism stopped / auto skipped a step | `COMMAND FAULT` or `SUBSYSTEM FAULT` event and the Health counts; the `FAIL` record in `commands/events` names the traced command and phase |
| All mechanisms twitched off / auto stopped mid-routine | `COMMAND FAULT: … (commands cleared, subsystems halted)`: something threw outside every `logged()` step |
| Auto did the wrong step / stopped early | `commands/events` and `commands/active` around the moment (only `logged` steps appear); FINISH of a hold may be its timeout |
| Path "finished" short of the target | `follow/translationalErrorIn` at the end; follow ends at the parametric end, add `holdCommand` |
| Auton wrong only on BLUE | Headings not through `Alliance.mirror`, or `PoseFactory.mirrorX` used (it reflects; this season rotates) |
| Paths refuse to start | `FORESIGHT_TUNED` false, or `ROBOT_CENTRIC_FALLBACK` after a localizer fault |
| Auton drifted | `follow/translationalErrorIn`: small error means localization; large means following |
| Sudden pose jump | `pose correction applied` events and correction gates |
| Field-centric wrong | Robot faced elsewhere at INIT (heading starts at 0): Create+Triangle |
| Driving robot-centric unexpectedly | `LOCALIZER FAULT` event, `Drive/odometryFallback` |
| Loop rate collapsed | Phase maxima: `writeHardware` usually Pinpoint, `telemetry` Panels, `periodic` season I/O |
| Tuned config reverted | Registration, public primitive `@JvmField`, config schema |
| Pinpoint unhealthy | Init Health status (`waiting for start pose` = pose write not landing), I²C cable, robot still at power-up (IMU calibration) |
| Ball target flickers or lingers | `BallCamera/frame/ageMs`, `rate/processedFps`, `target/status`, `candidates/accepted` |
| Tag diagnostic shows nothing | Limelight health (pipeline index/type), `Limelight/fiducial/count`, Full 3D and marker size |
| HIVE goal missing or on the wrong CELL | `Hive/<ALLIANCE>/state`, `Hive/tags/heightIn` and `tags/class` against `settings/raisedMinHeightIn`, `mount/*`, `frames/withoutTurretAngle` |
| HIVE goal spread large | `Hive/tags/deviationIn` per tag: one tag far off is an outlier; all off together is the mount or rotation convention |

- Repeated `LOOP OVERRUN` events matter; one at init/stop is usually warm-up.
- Swap a battery below 12.0 V resting. Normal operation should not sag below ~10 V.

## Hardware recording checklist

Hardware checks remain unverified; record results in [Progress](PROGRESS.md).

- [ ] `IntegrationSample.wpilog` passes the AdvantageScope GUI checks under
      "Flight recorder validation".
- [ ] A 2-minute teleop log opens in AdvantageScope; `Field/Robot` draws the
      robot in the right place and orientation on the FTC field, starting from
      a known parked pose (axis check above: Pedro's origin corner draws
      bottom-left, heading 0 points right).
- [ ] `pose`, `velocity`, `driveMode`, `follow/*`, `battery`, `gamepad1/*`,
      `loop/*`, `Drive/*`, `Localizer/*` and season subsystem channels plot
      with sensible values; events line up with what happened.
- [ ] `make pull-logs 2` pulls the newest Auto + TeleOp logs and MaxScope summarises them.
- [ ] A teleop log with a driver takeover shows `Drive teleop` interrupted and
      resumed in `commands/events`, `commands/lost` stays 0, and the analyzer
      reports `instrumented commands only`.
- [ ] Log size stays bounded (a host test run wrote ~24 kB/s at 50 Hz) and the file is intact
      after stopping normally and after a battery pull.
- [ ] Second-run heading drill ([Operations](OPERATIONS.md), step 4): a second init reads (0, 0, 0), and
      teleop after an autonomous does not inherit the autonomous pose.
- [ ] The last `Field/Robot` sample of a stopped log is where the robot
      actually stopped, not the field origin.
