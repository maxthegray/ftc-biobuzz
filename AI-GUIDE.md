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
import com.pedropathing.ivy.commands.Commands.*   // instant, waitMs, waitUntil, infinite, lazy, conditional
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
- `cancel` of a group ends its unfinished children with INTERRUPTED.
- `deadline` ends an unfinished child twice (INTERRUPTED, then NATURALLY).
  End handlers must be idempotent and must not treat NATURALLY as success.
- A command preempted *from inside* `Scheduler.execute()` still executes once
  more that tick. Schedule from bindings (input phase), `periodic()` (e.g.
  fault policies) or `onLoop()`, not from another command's execute.
- Groups take the union of child requirements and the max child priority;
  nothing stops two drive commands in one `parallel`. Don't do that.
- `waitMs` uses `System.currentTimeMillis()`.
- There are no command names, no running-command registry and no lifecycle
  hooks.

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
Paths.curve(a, control, b).linear(a, b)       // ≥ 3 points
Paths.path(first, second)                     // compound
path.with(Constants.foresightConfig.maxPathSpeed.at(0.5))
```

Verified 3.0.0 behaviour to respect:

- **`linear` heading runs backwards on `Paths.line` and on compound paths**
  (t=0 gets the end heading). It is correct on a `Paths.curve` segment. Use
  `constant`/`tangent`/`facingPoint` on lines, or a 3-point curve with a
  collinear control point when a straight segment must rotate.
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
- A step timeout is `race(step, waitMs(ms))`; the loser is interrupted.
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
6. **Fault policy.** An exception from bindings or command execution clears
   Ivy (no end handlers run), calls `onCommandFault()` on **every** subsystem,
   records `COMMAND FAULT: …` in the WPILOG, and the loop continues with
   defaults. An autonomous routine is then no longer scheduled and the
   op-mode stops itself. Exceptions from `periodic()`, `onLoop()` and
   `writeHardware()` end the op-mode (hardware stopped first, stack trace
   recorded). Telemetry and recorder failures are contained.
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
| `Field/Robot` | `struct:Pose2d` | Pose in metres about the field centre (AdvantageScope 2D field) |
| `pose` | `double[]` | `[x in, y in, heading rad]`, raw Pedro frame |
| `velocity` | `double[]` | `[vx in/s, vy in/s, omega rad/s]`, field frame |
| `driveMode` | string | `IDLE`, `TELEOP`, `FOLLOWING`, `HOLDING`, `ROBOT_CENTRIC_FALLBACK` |
| `follow/translationalErrorIn`, `follow/headingErrorRad` | double | Foresight errors, only while following/holding |
| `gamepad1/axes`, `gamepad2/axes` | `double[]` | `[lx, ly(+up), rx, ry(+up), lt, rt]` after deadband |
| `gamepad1/buttons`, `gamepad2/buttons` | int64 | bits: A B X Y LB RB up down left right start back LS RS |
| `battery` | double | volts |
| `loop/totalNanos`, `loop/<phase>Nanos`, `loop/windowMax…` | int64 | loop timing and per-window peaks |
| `<Subsystem>/…` | any | `SubsystemBase.logState` channels |
| `events` | string | explicit events with their own timestamps (not sampled) |

Log values with `logState(log)` (`log.put("name", value)`) and events with
`robot.recordEvent("text")`. Nothing records command starts or ends: there is
**no complete command history**. Do not describe sampled channels as one.

Failure isolation: an I/O failure disables the recorder for the run; a
non-I/O exception in one subsystem's `logState` disables that subsystem's
channels and records why; anything else escaping the recorder closes it. The
loop keeps running.

Retired with the migration (don't reintroduce): `commands/running`,
`COMMAND STARTED/FINISHED/INTERRUPTED/FAULTED` events, blocked-schedule and
first-default-resume events, `TRIGGER FAULT` quarantine, the recent-events
ring, `lastcrash.txt`.

## Things AI assistants get wrong often

- **There is one scheduler and it is Ivy's static `Scheduler`.** There is no
  `robot.scheduler`, no `core/command`, no command names.
- **Don't use Ivy's `PedroCommands`** for driving; use the drive's commands.
- **Parametric end ≠ arrival.** Use `holdCommand` or `atPose` when arrival matters.
- **`linear` on `Paths.line` is backwards in Pedro 3.0.0.**
- **Mirroring:** `Alliance.poses()` / `Alliance.mirror`, field length
  `RobotConfig.Field.LENGTH_INCHES` (**141.5**), symmetry
  `RobotConfig.Field.SYMMETRY`. Never `PoseFactory.mirrorX`.
- **Foresight constants are not tuned.** Never invent values; run AutoTune.
  The Pedro docs' example ForesightConfig numbers are another robot's.
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

Copy `opmodes/skeletons/ExampleAuto.kt`: RED poses through `alliance.poses()`,
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
- Don't edit the copied AutoTune procedures; re-copy them from the Quickstart.
- Don't add wrappers, DSLs or aliases over Ivy or Pedro APIs.
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
`make analyze`. `tools/analyze_wpilog.py` reports `commandHistoryRecorded:
false` for current logs. When a log doesn't determine the cause, give ranked
hypotheses and the one channel or reproduction that decides it.

### Post-match debugging (the AI runs this)

When the user is plugged into the hub and reports a match problem, run
**`make debug`**, anchor on the stated symptom, and drill into channels with
`python3 tools/analyze_wpilog.py --json --channel <name,name> robot-logs/<file>`.
The auton run is usually the second pulled file.

## Running the project

```
JAVA_HOME="/Users/maximilianreich/Library/Java/JavaVirtualMachines/corretto-17.0.13/Contents/Home" \
  ./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
python3 -m unittest tools/test_analyze_wpilog.py
```
