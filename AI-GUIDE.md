# AI Guide

Canonical instructions for every AI assistant working in this repository.
Read this entire file before changing code.

Human documentation has three entry points:

- `README.md` — setup and repository map
- `DEVELOPMENT.md` — student workflows: subsystems, buttons, autos, logging
- `OPERATIONS.md` — hardware bring-up, AutoTune, logs, diagnosis, validation

## What this repo is

BioBuzz's robot code for the current FTC season, forked from
`maxthegray/ftc-starter` — itself built on an unmodified clone of
`FIRST-Tech-Challenge/FtcRobotController` (now 11.2.1).

Season code **does** belong here. The boundary that matters is a directory one:

- `core/` is season- and chassis-agnostic framework. Fixes made here get
  cherry-picked back to `ftc-starter`, so keep it free of game logic,
  season constants, and this year's mechanism names.
- Everything else — `opmodes/`, `vision/`, season subsystems, `RobotConfig`,
  `pedro/` — is this season's and never flows upstream.

The code currently runs on a **sensorbot**: a temporary chassis to develop
against while the competition robot is built. The competition robot replaces
it in place — re-run AutoTune into `pedro/Constants.java`, update
`RobotConfig`, bump `RobotConfig.CONFIG_SCHEMA`. There is no sensorbot branch
and no robot profile switch; the last sensorbot commit gets tagged
`sensorbot-final`.

## Stack (exact versions; these are load-bearing)

| Library | Coordinate | Version |
|---|---|---|
| FTC SDK | `org.firstinspires.ftc:*` | 11.2.1 |
| Android Gradle Plugin / Gradle | `com.android.tools.build:gradle` | 8.7.0 / 8.9 |
| Kotlin Android plugin | `org.jetbrains.kotlin:kotlin-gradle-plugin` | 2.0.21 |
| Pedro Pathing core (pulled by revhub) | `com.pedropathing:core` | 3.0.0 |
| Pedro Pathing REV hub drivetrains/localizers | `com.pedropathing:revhub` | 3.0.0 |
| Pedro AutoTune | `com.pedropathing:tuning` | 1.0.0 |
| Ivy command scheduler | `com.pedropathing.ivy:core` | 1.1.1 |
| Sloth-compatible Panels | `com.bylazar.sloth:fullpanels` | 0.2.4+1.0.12 |
| Sloth / Load plugin | `dev.frozenmilk.sinister:Sloth`, `dev.frozenmilk:Load` | 0.2.4 |

`TeamCode/build.gradle` holds two strict constraints. Keep them unless the
whole toolchain moves together:

- **Sloth strictly 0.2.4.** `tuning:1.0.0` requests Sloth 0.3.0, which needs
  Load 0.3.0 (AGP 8.13, Kotlin 2.4 stdlib) and has no published
  Sloth-compatible Panels build. The Sinister API AutoTune calls
  (`Scanner`, `NarrowSearch`, `SinisterRegisteredOpModes`, app hooks) exists
  unchanged in Sinister 2.2.0.
- **kotlin-stdlib strictly 2.1.20.** revhub/tuning declare stdlib 2.3.21 but
  contain no Kotlin bytecode; the Kotlin 2.0.21 compiler cannot read 2.3
  metadata.

AutoTune needs SDK ≥ 11.2 (`OpModeMeta.Flavor.UTILITY`). Ivy's
`com.pedropathing.ivy:pedro` helpers are deliberately not used (see
**Drive commands**).

Maven repositories: `mavenCentral()` (Pedro, Ivy), `google()`,
`https://mymaven.bylazar.com/releases`, `https://repo.dairy.foundation/releases`.
Pedro is **not** on `maven.pedropathing.com`. If you need to bump any version,
verify the artifact exists in its real repository and check its POM/module
dependencies first. Don't guess.

## Library workarounds (re-check on every Pedro or Ivy upgrade)

Each workaround below exists because of a behaviour pinned in
`TeamCode/src/test/kotlin/.../core/LibraryContractTest.kt`. After a version
bump, run the unit tests: a failing contract test means the library changed,
and its failure message points here. Confirm the new behaviour in the
library's source, then do the listed cleanup in the same change. Don't just
flip the assertion.

| Workaround | Library behaviour (upstream) | Contract test | When the test fails |
|---|---|---|---|
| `core/util/LinearHeading.kt` (`linearHeading`) | Pedro 3.0.0 `Interpolator.linear`/`longLinear`/`piecewise` use `Curve.pathCompletion`, which returns the fraction *remaining* on `Line` and `CompoundCurve`: headings run backwards and `endPose()` gets the start heading. [Pedro-Pathing/PedroPathing#176](https://github.com/Pedro-Pathing/PedroPathing/issues/176), open as of 2026-09-14 | `pedroLinearHeadingRunsBackwardsOnLinesButNotOnCurves` | Check `.linear`, `longLinear`, `piecewise` and `endPose()` on a line and a compound path. If all are fixed: replace every `.heading(linearHeading(a, b))` with `.linear(a, b)` (`git grep linearHeading`), delete `LinearHeading.kt` and `LinearHeadingTest.kt`, turn the contract test into a check that `.linear` is correct, and remove the bug notes here, in the Pedro section, `DEVELOPMENT.md` and `OPERATIONS.md` (triage row and checklist). `linearHeading` stays correct until then, so there is no hurry. |
| `core/util/MonotonicWait.kt` (`monotonicWaitMs`) | Ivy 1.1.1 `Commands.waitMs` uses `System.currentTimeMillis()`. Not reported upstream | `ivyWaitMsIsTimedByTheWallClock` | If Ivy's wait is now monotonic: replace `monotonicWaitMs(ms)` with `waitMs(ms)`, delete the helper and `MonotonicWaitTest.kt`, update docs. Tests that need a fake clock may still want the helper. |
| `Alliance.poses()` / `Alliance.mirror` instead of `PoseFactory.mirrorX` | `mirrorX` maps heading to −h; this field's reflection is π−h. A convention, not a bug | `pedroPoseFactoryMirrorXIsNotTheFieldReflection` | Keep `Alliance`, which also handles `FieldSymmetry.ROTATE`. Only if `mirrorX` now gives π−h, consider using it for MIRROR seasons. |
| `MecanumDriveSubsystem.halt()` stops the drivetrain directly | Pedro 3.0.0 `Follower.stop()` changes mode only; motors update on the next `update()` | `pedroStopOnlyChangesModeUntilTheNextUpdate` | If `stop()` now zeroes motors immediately, the direct `drivetrain.stop()` is redundant but harmless. |
| Drive commands instead of Ivy's `PedroCommands` (no `com.pedropathing.ivy:pedro` dependency) | Ivy 1.1.1 `follow` has no requirement and no interruption cleanup; `hold` reports no arrival | None (the artifact isn't a dependency) | On an Ivy upgrade, read `PedroCommands` in the new `ivy:pedro` sources before considering it. The drive commands also carry the logging and completion semantics. |
| Once-per-start guards in drive commands; `LoggedCommand` ignoring unstarted or repeated ends; ABORT after `Scheduler.reset` | Ivy 1.1.1 ends unstarted group children, ends deadline children twice, forwards a lazy's end, skips `end` on reset, executes a command interrupted earlier in the tick | the `ivy…` tests | Re-check `MecanumDriveSubsystem`, `CommandHistory.kt`, `DriveCommandCancellationTest` and `CommandHistoryTest` against the new scheduler and groups. The guards stay harmless if Ivy stops doing this. |

Checking upstream: `gh issue view 176 -R Pedro-Pathing/PedroPathing`, and the
newest versions at
`https://repo1.maven.org/maven2/com/pedropathing/<core|revhub|tuning|ivy/core>/maven-metadata.xml`.

## Optimize for the best decision, not the cheapest

When a decision forks between a pragmatic compromise and the genuinely better
engineering answer, take the better one. This governs the *quality* of
decisions, not the *quantity* of work: it is **not** licence to over-engineer
or add speculative abstractions. Prefer the libraries' public APIs over
custom infrastructure.

## Ivy (the only scheduler)

Ivy's `Scheduler` is **static**. `Robot` resets it when constructed and at
stop, and ticks it once per loop. Op-modes never call `Scheduler.execute()`.

```kotlin
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.*   // instant, waitUntil, infinite, lazy, conditional (not waitMs)
import org.firstinspires.ftc.teamcode.core.util.monotonicWaitMs
import com.pedropathing.ivy.groups.Groups.*       // sequential, parallel, race, deadline, repeat, loop

Command.build()
    .requiring(subsystem)
    .setPriority(CommandPriorities.DRIVER_ACTION)
    .setStart { }             // once, immediately inside schedule()
    .setExecute { }           // every tick
    .setDone { false }        // checked after execute at top level
    .setEnd { condition -> }  // NATURALLY or INTERRUPTED

Scheduler.schedule(cmd); Scheduler.cancel(cmd); Scheduler.isScheduled(cmd)
```

Semantics verified against the 1.1.1 artifact (`LibraryContractTest` pins them):

- A command with a **strictly higher** priority holder is blocked (dropped by
  default). Equal priority **overrides** by default; lower-priority holders
  are interrupted.
- `Scheduler.reset()` drops everything **without calling end handlers**.
- `cancel` of a group ends its unfinished children with INTERRUPTED,
  **including children that never started** (every later step of a
  `sequential`). `cancel` of a queued command ends it without a start, and a
  `lazy` ended before it starts forwards the end to its previous command.
- `deadline` ends an unfinished child twice (INTERRUPTED, then NATURALLY).
- So an end handler can run without a start, or twice. It may only make
  things safe (zero a target, reset state); never energize hardware or
  treat NATURALLY as success in it. When cleanup must match a real run,
  guard it with a flag set in `setStart` (the drive commands do this).
- A command preempted *from inside* `Scheduler.execute()` still executes once
  more that tick. Schedule from bindings (input phase), `periodic()` (e.g.
  fault policies) or `onLoop()`, not from another command's execute.
- Groups take the union of child requirements and the max child priority;
  nothing stops two drive commands in one `parallel`. Don't do that.
- `waitMs` uses `System.currentTimeMillis()`, which jumps when the hub's
  wall clock is set. Use `monotonicWaitMs(ms)` (`core/util`), the same command
  on `System.nanoTime()`; pass a `Clock` in tests.
- There are no command names, no running-command registry and no lifecycle
  hooks. The one exception this repo adds is `logged(name, command)`
  (`core/logging/CommandHistory.kt`), a forwarding `Command` that traces its
  lifecycle into the flight log (see **Command history**).

Priority ladder (`CommandPriorities`): defaults `0` < auton routines and
assists `10` < driver actions `20` < overrides `30`. Keep priorities ≥ 0.

Default commands: `subsystem.defaultCommand = builder`. The setter sets
`ConflictBehavior.CANCEL`, and `Robot` schedules it only when it is not
already scheduled, so a default never preempts an explicit command of equal
priority, while explicit commands at priority ≥ 0 still preempt the default.

## Pedro Pathing 3 (geometry, paths, follower)

Pedro's types are used directly everywhere: `com.pedropathing.math.Pose`
(immutable; heading normalized to [0, 2π)), `Velocity` (field frame),
`Vector2D`, `com.pedropathing.api.Paths`, `PoseFactory`,
`com.pedropathing.paths.Path`. Units: inches, radians, CCW-positive.

```kotlin
val p = alliance.poses()                      // PoseFactory in degrees, RED coords → this alliance
val start = p.of(8.0, 56.0, 0.0)
val out = p.of(32.0, 56.0, 0.0)
Paths.line(start, out).constant(start)
Paths.line(a, b).heading(linearHeading(a, b)) // turn along a path (core/util), never .linear
Paths.curve(a, control, b).constant(a)        // ≥ 3 points
Paths.path(first, second)                     // compound
path.with(Constants.foresightConfig.maxPathSpeed.at(0.5))
```

Verified 3.0.0 behaviour to respect:

- **`linear` heading runs backwards on `Paths.line` and on compound paths**
  ([#176](https://github.com/Pedro-Pathing/PedroPathing/issues/176); see
  **Library workarounds** for removing the workaround once fixed)
  (t=0 gets the end heading) because the default `Curve.pathCompletion`
  returns the fraction remaining; only `BezierCurve` overrides it. Use
  `path.heading(linearHeading(a, b))` from `core/util` on every path kind: it
  interpolates by distance travelled, turns the short way like Pedro's, and
  follows alliance-mapped poses. Pedro's `Interpolator.piecewise()` breakpoints
  have the same problem on lines and compound paths.
- `Paths.curve` rejects fewer than three points (the docs show two).
- `PoseFactory.mirrorX` maps heading to −h, which is not this repo's field
  reflection (π−h). Use `Alliance.poses()` / `Alliance.mirror`.
- The follower leaves FOLLOW mode at the **parametric end** of the path (then
  HOLD with `holdEnd`, else IDLE). That is not arrival.
- `isBusy` is only cleared by a converged or timed-out hold; with `holdEnd`
  false it stays true. It is not a completion signal.
- `Follower.stop()` changes mode only; motors are written on the next `update()`.
- `Follower.update()` reads the localizer and writes motors. It runs exactly
  once per tick, in `MecanumDriveSubsystem.writeHardware()`.

Configuration lives in `pedro/Constants.java` in AutoTune's layout
(`drivetrainConfig`, `localizerConfig`, `foresightConfig`, `create()`).
`FORESIGHT_TUNED` is false until AutoTune's Foresight output is pasted; until
then the follower has no algorithm, manual driving works, and path, hold and
turn commands throw at start. AutoTune procedures in `pedro/procedures/` are
copied unmodified from Pedro-Pathing/Quickstart b4312385; `pedro/Tuning.java`
registers the Mecanum, Pinpoint, Foresight and Tests procedures.

## Drive commands

`MecanumDriveSubsystem` is the single drive owner. Every drive command
requires it and stops the follower when interrupted. Ivy's
`PedroCommands.follow` is not used: in 1.1.1 it has no requirement and no
interruption cleanup (a cancelled path keeps driving), and `hold` reports
nothing about arrival.

```kotlin
drive.teleopCommand(priority = 0) { TeleopInput(fwd, strafe, turn, precision, turnPower, forwardPower) }
drive.followCommand(path, holdEnd = false)       // ends when Pedro leaves FOLLOW (parametric end)
drive.holdCommand(pose, timeoutMs = 2000.0)      // ends on measured arrival, or timeout (bounded wait)
drive.turnToCommand(radians, timeoutMs = 2000.0) // ends on measured heading; timeout THROWS
drive.robotCentricFallbackCommand { input }      // localizer-fault policy, priority Int.MAX_VALUE
drive.pathProgress()                             // latched 0..1 over all segments, for markers
drive.pose / drive.velocity / drive.atPose(target) / drive.toggleFieldCentric()
```

- Arrival uses `DriveConfig.holdToleranceInches/Radians`.
- Teleop input is staged by the command and applied in `writeHardware()`.
  Field-centric rotation uses the measured heading; a non-finite heading is
  never used. `forwardPower` forces robot-centric for that tick.
- A mid-path marker is plain Ivy: `deadline(drive.followCommand(path),
  sequential(waitUntil { drive.pathProgress() >= 0.5 }, instant { … }))`.
  It fires once, and is dropped if the path ends or is cancelled first.
- A step timeout is `race(step, monotonicWaitMs(ms))`; the loser is interrupted.
- `halt()` (stop, command fault) writes zero power immediately.

## Lifecycle rules (enforced by Robot/OpModeBase)

1. **Bulk reads are MANUAL.** Caches clear once per tick at the top.
2. **`periodic()` reads. Commands decide. `writeHardware()` flushes.**
   `initPeriodic()` (defaults to `periodic()`) runs in init; no commands run
   and nothing is written before start.
3. **Loop order:** clear caches → `periodic()` → gamepad input and trigger
   bindings → `onLoop()` → default commands + `Scheduler.execute()` →
   `writeHardware()` → telemetry → flight recorder.
4. **Bindings are init-only.** Wire `GamepadEx` triggers in `configure()`;
   they lock at start. A button held through init does not fire at start.
5. **Register the drive before the localizer** (`registerAfter` enforces it):
   the localizer samples pose history after `Follower.update()`.
6. **Fault policy.** An `Exception` from bindings or command execution
   clears Ivy (no end handlers run), calls `onCommandFault()` on **every**
   subsystem, records `COMMAND FAULT: …` in the WPILOG, and the loop
   continues with defaults. An autonomous routine is then no longer scheduled
   and the op-mode stops itself. An `Error` is not contained, wherever it is
   thrown: Kotlin's `TODO()` throws `NotImplementedError`, so a `TODO()` left
   in a command or binding **ends the op-mode**, as do exceptions from
   `periodic()`, `onLoop()` and `writeHardware()`. `OpModeBase` then calls
   `Robot.stopAfterCrash`: every subsystem is stopped first, `LOOP CRASHED`
   and the stack trace are recorded, the log closes, and the error is
   rethrown to the Driver Station. Telemetry and recorder failures are
   contained. Use `error("…")` (an `IllegalStateException`) for a command
   that should fail without ending the op-mode.
7. **Localizer faults.** `LocalizerSubsystem.periodic()` trips on a non-finite
   pose, a pose frozen while following, or a bad Pinpoint status, and calls
   `onFault` once. Teleop (`TeleOpBase`) schedules the robot-centric fallback
   and makes it the default: no follower update, no odometry read, no
   field-centric rotation, and paths/holds/turns refuse. Auton cancels the
   routine and stays put.
8. **Fresh localization.** `LocalizerSubsystem.init()` writes `startingPose`
   (default `Pose.zero()`; an autonomous passes its field start pose) to the
   Pinpoint at every INIT. A read that disagrees before one confirms it is a
   pre-reset sample: the pose is written again, and `ready` stays false until
   a read confirms it within 0.5 in / 1° (fault after five seconds). The pose
   is written, not recalibrated: `Constants` sets Pedro's Pinpoint
   `ResetMode.NONE`, so the IMU keeps its power-up calibration and INIT has
   no stationary requirement. No pose is restored from a previous op-mode.
9. **Shutdown.** `Robot.stop()` stops every subsystem first, then runs the
   crash-report callback, clears Ivy without end handlers (so cleanup cannot
   re-energize hardware), closes the recorder (the last logged pose is the
   last real one), and saves dirty config. Nothing else carries over. `stop()` must zero actuators and
   avoid storage I/O.

## Config persistence (ConfigStore) + Sloth hot reload + Panels

Use the Sloth-compatible `com.bylazar.sloth:fullpanels` artifact; plain
`com.bylazar:fullpanels` does not track Sloth's replacement classes.

Panels writes `@Configurable` statics, which die with the process and are
re-initialised by Sloth reloads. `ConfigStore` (`core/runtime/`) persists
registered objects to `/sdcard/FIRST/config/tuning.properties` (~1 Hz when
dirty, atomically) and reloads them at every op-mode init.

`OpModeBase` registers `DriveConfig` and `LocalizerConfig`. Season code
registers its own in `configure()`:

```kotlin
ConfigStore.register("lift", LiftConfig, LiftConfig::resetDefaults)
```

Only public `@JvmField` mutable primitive/String fields persist, keyed
`<section>.<field>`. Each config supplies `resetDefaults()` from compiled
constants; never capture live values. Every load resets before applying
overrides. Bump `RobotConfig.CONFIG_SCHEMA` when tuned values stop applying.
`DriveConfig.brakeOnTeleop` is copied into Pedro's `manualBrakeMode` when the
follower is created (next init).

There are no `@Pinned` classes. Don't pin config objects. Ivy's
static scheduler is reset by every `Robot`, so commands created before a
reload never run.

## Flight recorder (WPILOG for AdvantageScope)

`FlightRecorder` writes `/sdcard/FIRST/logs/<OpMode>-<timestamp>-<n>.wpilog`
(30 files kept). Channels (sampled at ≤ 100 Hz unless noted):

| Channel | Type | Meaning |
|---|---|---|
| `Field/Robot` | `struct:Pose2d` | Pose in the FTC field frame for AdvantageScope's 2D field: metres about the centre, axes and heading turned by `RobotConfig.Field.FIELD_VIEW_QUARTER_TURNS` (display only) |
| `pose` | `double[]` | `[x in, y in, heading rad]`, raw Pedro frame |
| `velocity` | `double[]` | `[vx in/s, vy in/s, omega rad/s]`, field frame |
| `driveMode` | string | `IDLE`, `TELEOP`, `FOLLOWING`, `HOLDING`, `ROBOT_CENTRIC_FALLBACK` |
| `follow/translationalErrorIn`, `follow/headingErrorRad` | double | Foresight errors, only while following/holding |
| `gamepad1/axes`, `gamepad2/axes` | `double[]` | `[lx, ly(+up), rx, ry(+up), lt, rt]` after deadband |
| `gamepad1/buttons`, `gamepad2/buttons` | int64 | bits: A B X Y LB RB up down left right start back LS RS |
| `battery` | double | volts |
| `loop/totalNanos`, `loop/<phase>Nanos`, `loop/windowMax…` | int64 | loop timing and per-window peaks |
| `<Subsystem>/…` | any | `SubsystemBase.logState` channels |
| `events` | string | explicit events with their own timestamps (not sampled); strictly increasing, a same-microsecond event moves 1 µs later |
| `commands/events` | string | lifecycle records of `logged` commands, own timestamps, strictly increasing (see below) |
| `commands/active` | string | traced executions open after each record, `#id name` per line, `(suspended)` suffix |
| `commands/lost` | int64 | records dropped from the bounded queue; 0 at open, rewritten when it grows |

Log values with `logState(log)` (`log.put("name", value)`) and events with
`robot.recordEvent("text")`.

### Command history

```kotlin
fun raise(): Command = logged("Lift raise", Command.build().requiring(this).setDone { atTop })
drive.followCommand(path, name = "Drive to bar")   // drive factories are already logged
```

- `logged` returns a `LoggedCommand` that forwards requirements, priority, the
  three behaviours, return values and exceptions to the wrapped command on
  every call. Schedule, cancel, bind and compare **the returned instance**;
  Ivy never sees the inner one. `logged` on a logged command renames it.
  `SubsystemBase.defaultCommand` takes any `Command` and sets CANCEL on the
  builder inside a `LoggedCommand`.
- Records: `START #id name`, `FINISH` (its `done` returned true, not
  arrival), `INTERRUPT`, `SUSPEND`/`RESUME` (Ivy's SUSPEND behaviour; resume is
  seen at the next `execute`), `FAIL #id name in <start|execute|done|end>:
  <Exception>: <message>` with `(from #child)` when a logged parent sees its
  child's exception, `FAIL #- name …` when a command threw outside a traced
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
  itself is unaffected, and its exception is always rethrown unchanged.
- **Coverage is instrumented commands only.** A logged group does not expose
  unlogged children. There are no blocked-schedule events and no interruption
  causes: Ivy's public API does not reveal them. Never describe
  `commands/events` as a complete command history.
- Instrumented today: the five drive factories (default names `Drive
  teleop`, `Drive follow`, `Drive hold`, `Drive turn`, `Drive robot-centric
  fallback`), TeleOpBase's `Driver sticks`, fallback and `Reset heading`,
  Localization Test's moves and `Driver takeover`, Example Auto's routine and
  steps. Instrument new mechanism commands and autonomous steps the same way;
  don't wrap trivial `instant`s or marker internals.

Failure isolation: an I/O failure disables the recorder for the run; a
non-I/O exception in one subsystem's `logState` disables that subsystem's
channels and records why; anything else escaping the recorder closes it. The
loop keeps running.

Retired with the migration (don't reintroduce): `commands/running` (it meant
every scheduled command; the analyzer still reads it in old logs),
`COMMAND STARTED/FINISHED/INTERRUPTED/FAULTED` events in `events`,
blocked-schedule and first-default-resume events, `TRIGGER FAULT` quarantine,
the recent-events ring, `lastcrash.txt`.

## Things AI assistants get wrong often

- **There is one scheduler and it is Ivy's static `Scheduler`.** There is no
  `robot.scheduler`, no `core/command`. Names exist only on `logged` commands.
- **Don't use Ivy's `PedroCommands`** for driving; use the drive's commands.
- **Parametric end ≠ arrival.** Use `holdCommand` or `atPose` when arrival matters.
- **`linear` on `Paths.line` is backwards in Pedro 3.0.0.**
- **Mirroring:** `Alliance.poses()` / `Alliance.mirror`, field length
  `RobotConfig.Field.LENGTH_INCHES` (**141.5**), symmetry
  `RobotConfig.Field.SYMMETRY`. Never `PoseFactory.mirrorX`.
- **Foresight constants are not tuned.** Never invent values; run AutoTune.
  The Pedro docs' example ForesightConfig numbers are another robot's.
- **Only `Field/Robot` is rotated.** Pedro's frame (origin at the corner on
  the audience's left, +X along the audience wall) and the FTC frame
  AdvantageScope draws (origin at the centre, +Y from the red wall to the
  blue wall) differ by a quarter turn whose sign depends on which wall is red
  that season: `RobotConfig.Field.FIELD_VIEW_QUARTER_TURNS` (+1 for DECODE,
  −1 when red is on the audience's right). `WpiStruct` applies it to the
  field-view channel and nothing else: `pose`, `Constants.java`, autonomous
  start poses and `Alliance` are Pedro's frame and never change for it. A
  robot drawn a quarter turn off means that constant, not a path.
- **Pinpoint offsets map as** Pedro 2 `forwardPodY → xPodOffset`,
  `strafePodX → yPodOffset` (same `setOffsets(x, y)` call).
- **Subsystem writes in `periodic`.** Don't.
- **VisionPortal processors are single-use.** Build a new processor every
  run. Never let an exception leave `processFrame`.
- **Panels values never reach camera threads directly.** Copy into an
  immutable snapshot on the robot loop (see `BallCameraSubsystem`); only
  `CameraControlWorker` calls blocking USB camera controls.
- **Vision timestamps are not interchangeable.** Limelight `staleness` is
  receipt age, `ts` is device clock (identity only), VisionPortal capture
  time is `System.nanoTime()`. Label which age a number is.
- **A BIOBUZZ tag sighting is not HIVE state.**

## When the user asks you to add a subsystem

Follow `DEVELOPMENT.md` → *Add a subsystem*: extend `SubsystemBase`, resolve
hardware in `init` through `DeviceReaders`, read in `periodic()`, flush in
`writeHardware()`, expose Ivy command factories that `requiring(this)`, zero
actuators in `stop()` and `onCommandFault()`, log in `logState`, register in
`configure()`. Season mechanisms go under `teamcode/subsystems/`, not `core/`.
There is deliberately no generic mechanism base class.

## When the user asks you to add an I²C sensor (or touch SRSHub)

Read the sensor section in `DEVELOPMENT.md` first. Pinpoint stays direct; all
other I²C goes on one SRSHub read **inline** in `periodic()`. Do not
background the SRSHub by default.

## When the user asks you to add a path or auton routine

Copy `opmodes/skeletons/ExampleAuto.kt` (routine and meaningful steps `logged`,
drive steps named): RED poses through `alliance.poses()`,
paths from `Paths`, the routine as Ivy groups of drive commands, `race`
timeouts, `deadline` markers, start gates (`FORESIGHT_TUNED`,
`localizer.ready`, schedule accepted), stop when the routine is no longer
scheduled. One `@Autonomous` class per alliance and routine; the BLUE copy
overrides `initialAlliance` only. Start delay via `StartDelay` on dpad in init.

## Naming op-modes

Currently enabled: Drive Only, Ball Tracking Test, Limelight AprilTag Test.
AutoTune is a web page, not a Driver Station op-mode. `opmodes/archived/` and
`opmodes/skeletons/` hold `@Disabled` op-modes; keep them disabled unless the
user asks.

No team or season prefix. `"Match"` or `"Diagnostics"` groups. Title Case, no
`TeleOp`/`Auto` suffix. Alliance variants end in the alliance
(`"Far Side RED"`). Class names keep their suffix.

## Things not to do unless explicitly asked

- Don't rename hardware-map strings (`frontLeftMotor`, `pinpoint`, …).
- Don't move files between `java/` and `kotlin/` source roots.
- Don't bump FTC SDK, Pedro, Ivy, Kotlin, AGP, Sloth or Panels versions.
  When asked to, work through **Library workarounds** as part of the bump.
- Don't edit the copied AutoTune procedures; re-copy them from the Quickstart.
- Don't add wrappers, DSLs or aliases over Ivy or Pedro APIs. `logged` is the
  single sanctioned command wrapper; don't grow it into a framework.
- Don't invent game-specific mechanisms or put season code in `core/`.
- Don't enable disabled op-modes, change field dimensions or symmetry, or
  rewrite vision algorithms.

## Workflow

- **Full install** (`make install`): first deploy of a session, and after
  changing `@Pinned` classes, dependencies, the manifest, `res/`, or anything
  outside TeamCode.
- **Hot reload** (`make hot`, `./gradlew deploySloth`): ordinary TeamCode
  iteration, including `pedro/Constants.java`.

Logs: `make debug` (newest Auto + TeleOp, JSON bundle), `make pull-logs`,
`make analyze`. `tools/analyze_wpilog.py` reports `commandHistoryCoverage`:
`"all scheduled"` (pre-Ivy logs, `commands/running`), `"none"` (Ivy logs before
command tracing) or `"instrumented"` (current logs, `logged` commands only,
with `commandExecutions`, `commandFailures`, `commandActive` and
`commandHistoryIntact`/`commandHistoryLostRecords`). When a log doesn't determine the cause, give ranked
hypotheses and the one channel or reproduction that decides it.

### Post-match debugging (the AI runs this)

When the user is plugged into the hub and reports a match problem, run
**`make debug`**, anchor on the stated symptom, and drill into channels with
`python3 tools/analyze_wpilog.py --json --channel <name,name> robot-logs/<file>`.
The auton run is usually the second pulled file. For "what was it doing":
read `commandFailures`, then the `commandExecutions` overlapping the symptom
time, then `--channel commands/active,driveMode,pose` around it. State the
coverage: an absent command may simply be unlogged.

## Running the project

```
JAVA_HOME="/Users/maximilianreich/Library/Java/JavaVirtualMachines/corretto-17.0.13/Contents/Home" \
  ./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
python3 -m unittest tools/test_analyze_wpilog.py
```
