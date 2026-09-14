# Development Guide

Short workflows for the things you do most, then the rules behind them.
Robot code uses [Ivy](https://pedropathing.com/docs/ivy) for commands and
[Pedro Pathing 3](https://pedropathing.com/docs/pathing) for poses, paths and
the drivetrain, directly. `AI-GUIDE.md` has the full contract.

Paths below are relative to `TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/`.

## Add a subsystem

Create `subsystems/<area>/IntakeSubsystem.kt` (season code never goes in `core/`):

```kotlin
class IntakeSubsystem : SubsystemBase("Intake") {
    private lateinit var roller: MotorIO
    private var rollerPower = 0.0
    private var ballSeen = false

    override fun init(hardwareMap: HardwareMap) {
        roller = RealMotorIO(DeviceReaders.motor(hardwareMap, "intakeRoller", DcMotorSimple.Direction.REVERSE))
    }

    override fun periodic() {
        ballSeen = false // read sensors here; never command motors here
    }

    override fun writeHardware() {
        roller.setPower(rollerPower) // the one place motor power is written
    }

    fun grab(): Command = logged( // named in the flight log's command history
        "Intake grab",
        Command.build()
            .requiring(this)
            .setPriority(CommandPriorities.DRIVER_ACTION)
            .setStart { rollerPower = 1.0 }
            .setDone { ballSeen }
            .setEnd { rollerPower = 0.0 }, // only ever makes the roller safe
    )

    override fun onCommandFault() { rollerPower = 0.0 }
    override fun stop() { rollerPower = 0.0; roller.setPower(0.0) }

    override fun logState(log: StateLog) {
        log.put("rollerPower", rollerPower)
        log.put("ballSeen", ballSeen)
    }
}
```

Register it in your op-mode's `configure()` (or `configureTeleop()`):
`val intake = robot.register(IntakeSubsystem())`.

Rules:

- `periodic()` reads, commands decide, `writeHardware()` writes.
- Every command that touches the subsystem says `requiring(this)`, so Ivy
  lets only one of them run. Reset per-run state in `setStart`; use
  `Commands.lazy { … }` when the command depends on state known only when it
  starts.
- End handlers run on natural end and on interruption, **not** when the
  op-mode stops or a command faults. `stop()` and `onCommandFault()` are what
  make the mechanism safe then.
- An end handler can also run for a command that **never started** (a later
  step of a cancelled `sequential`) and can run **twice** (a `deadline`
  child). Write it so that is harmless: set targets to zero or reset state,
  never start a motor or log "done" there. If cleanup must match a real
  run, set a flag in `setStart`:

  ```kotlin
  fun spitOut(): Command {
      var running = false
      return Command.build()
          .requiring(this)
          .setStart { running = true; rollerPower = -0.5 }
          .setDone { !ballSeen }
          .setEnd {
              if (running) lastSpitOut = it // counts only runs that happened
              running = false
              rollerPower = 0.0
          }
  }
  ```
- Bench rigs without the drivetrain override `requiredDevices`:
  `get() = listOf(Preflight.Requirement("liftMotor", DcMotorEx::class.java))`.

## Bind a button

In `configure()` / `configureTeleop()`, using Ivy commands:

```kotlin
driver.button(Button.A).onTrue(intake.grab())                 // schedule on press
driver.button(Button.LEFT_BUMPER).whileTrue(intake.eject())   // cancel on release
driver.button(Button.X).toggleOnTrue(lift.raise())
driver.trigger { driver.rightTrigger > 0.5 }.whileTrue(shooter.spinUp())
(driver.button(Button.BACK) and driver.button(Button.Y)).onTrue(instant { … })
```

Bindings lock at start. Read sticks directly (`driver.leftStickY` is +up).
`TeleOpBase` already binds Back+Y (heading reset), Back+B (field-centric) and
installs stick driving as the drive's default command.

## Compose an autonomous

Copy `opmodes/skeletons/ExampleAuto.kt`. The shape:

```kotlin
private val poses get() = alliance.poses()          // degrees, RED coordinates
private val start get() = poses.of(8.0, 56.0, 0.0)
private val score get() = poses.of(32.0, 56.0, 0.0)

private fun toScore(): Path = Paths.line(start, score).constant(start)

private fun routine(): Command = logged(
    "Score preload",
    race(
        sequential(
            logged("Start delay", monotonicWaitMs(startDelay.millis.toDouble())),
            race(
                drive.followCommand(toScore(), name = "Drive to score"),
                logged("Drive to score time limit", monotonicWaitMs(4_000.0)), // step timeout
            ),
            drive.holdCommand(score, name = "Settle at score"),               // wait for arrival
            instant { robot.recordEvent("AUTO: scored") },
            drive.turnToCommand(alliance.mirror(Math.toRadians(90.0)), name = "Face the wall"),
        ),
        logged("Routine time limit", monotonicWaitMs(29_000.0)),             // whole-routine timeout
    ),
)
```

Schedule, check and cancel the returned `routine()` instance. Drive commands
take a `name`; wrap everything else worth seeing with `logged(...)`.

Pass the start pose to the localizer in `configure()`:
`LocalizerSubsystem(follower, ..., startingPose = start)`. It is written to the
Pinpoint at INIT, so place the robot before pressing INIT. In `onStart`: refuse
to run if `!Constants.FORESIGHT_TUNED` or `!localizer.ready` (start pose not
yet confirmed), `Scheduler.schedule(...)` and check `Scheduler.isScheduled(...)`. In `onLoop`, stop the op-mode once the
routine is no longer scheduled.

- **Waits:** use `monotonicWaitMs(ms)` from `core/util`, not Ivy's `waitMs`,
  which is timed by the wall clock and ends early or stalls if the hub's
  time is set mid-match.
- **Markers** (do something part-way along a path, once):
  `deadline(drive.followCommand(path), sequential(waitUntil { drive.pathProgress() >= 0.5 }, instant { lift.up() }))`.
  If the path ends or is cancelled first, the marker is dropped.
- **Completion:** `followCommand` ends at Pedro's *parametric end*, which is
  not arrival. Follow it with `holdCommand(pose)` when arrival matters.
  `turnToCommand` throws on timeout, which aborts the whole routine.
- **Turning along a path:** in Pedro 3.0.0, `.linear(...)` rotates backwards
  on `Paths.line` and compound paths. Use
  `path.heading(linearHeading(a, b))` (`core/util`; see `backPath()` in the
  example), or `.constant(...)` to keep one heading.
- **Alliances:** one `@Autonomous` class per alliance/routine; the BLUE copy
  overrides `initialAlliance` only. Never use `PoseFactory.mirrorX`.
- **Relocalization:** `localizer.applyCorrection(measured, timestampNanos, …)`
  with the camera frame's capture time. It is gated, blended and scaled down
  while following.

## Log a value

Anything a subsystem knows goes in `logState`; it becomes `<Subsystem>/<name>`
in the WPILOG (at most 100 samples a second):

```kotlin
override fun logState(log: StateLog) {
    log.put("goalTicks", goal)        // double, long, boolean or string
    log.put("state", state.name)      // strings are only written when they change
}
```

For a one-off moment, record an event with its exact time:
`robot.recordEvent("AUTO: preload scored")`.

## Trace commands in the log

Wrap a command with `logged` (`core/logging`) to put its runs in the WPILOG:

```kotlin
import org.firstinspires.ftc.teamcode.core.logging.logged

fun raise(): Command = logged("Lift raise", Command.build().requiring(this).setDone { atTop })
val auto = logged("Left auto", sequential(drive.followCommand(path, name = "Drive to bar"), lift.raise()))
Scheduler.schedule(auto)   // schedule, cancel, bind and compare this instance
```

- The drive's `teleopCommand`, `followCommand`, `holdCommand`,
  `turnToCommand` and `robotCentricFallbackCommand` are already logged; give
  them a `name` instead of wrapping them again.
- **Only wrapped commands appear.** A logged group does not show its children:
  wrap each step you want to see. Skip trivial internals (`instant` markers,
  a `waitUntil` inside a marker); record an event for a moment instead.
- Keep the instance: `Scheduler.cancel(inner)` or `isScheduled(inner)` on the
  unwrapped command does nothing, because Ivy only knows the wrapper.
  `logged` on an already logged command renames it.

Channels:

| Channel | Meaning |
|---|---|
| `commands/events` | One record per change: `START #7 Drive to bar`, `FINISH #7 …`, `INTERRUPT #7 …`, `SUSPEND`/`RESUME #7 …`, `FAIL #7 … in execute: IllegalStateException: …`, `ABORT #7 …: command fault` / `op-mode stop` |
| `commands/active` | The traced executions running after each record, one `#id name` per line (`(suspended)` when suspended) |
| `commands/lost` | Records dropped because too much happened between two loops; above 0 the history is incomplete |

- `#id` numbers each run from 1 per op-mode, so two runs of the same command,
  or two commands with the same name, stay apart.
- **FINISH means the command's own `done` returned true**, not that the robot
  arrived: a hold that timed out, or a path at its parametric end, also
  finishes. Check pose and `follow/*` for arrival.
- **FAIL** is the command whose `start`, `execute`, `done` or `end` threw; a
  logged group it ran inside also fails, marked `(from #child)`. **ABORT** is
  bookkeeping for commands that were still running when the fault policy or
  the op-mode stop cleared Ivy: they did not fail, and their end handlers did
  not run.
- Nothing records why a command was interrupted (driver takeover, a group's
  timeout, cancel) or that a schedule was rejected: Ivy doesn't say. Read the
  surrounding records and `events`.

## Download a WPILOG and open it in AdvantageScope

1. Connect to the Control Hub (USB, or its Wi-Fi then `make connect`).
2. `make debug` pulls the newest match's logs into `robot-logs/` and prints a
   summary; `make pull-logs` copies all of them.
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
   every traced command running then. `make analyze` lists each execution
   with start, end and outcome, and failures first.

## Commands, priorities and faults

- Ladder: defaults `0` < autos and assists `10` < driver actions `20` <
  overrides `30`. A command is blocked by a strictly higher priority holder;
  equal priority takes over.
- A default command never takes over from an explicit command of equal
  priority; it resumes when the subsystem is free.
- **If a command throws an `Exception`** (for example `error("…")` or
  `require(...)`), the robot clears every command, halts every subsystem
  (`onCommandFault()`), records `COMMAND FAULT` in the log, and keeps looping
  with defaults. In auto that means the routine is gone and the op-mode stops.
  Health telemetry shows the count.
- **An `Error` ends the op-mode.** `TODO()` throws `NotImplementedError`,
  which is an `Error`: a command or binding still containing `TODO()` stops
  every motor, records `LOOP CRASHED` with the stack trace, and ends the run.
  Replace every `TODO()` before driving.
- **If the localizer fails** in teleop, the driver keeps robot-centric sticks
  for the rest of the run; paths, holds and turns refuse to start.

## What the custom code is for

Everything else is Ivy or Pedro. Each remaining helper has one job:

| Helper | Why it exists |
|---|---|
| `core/runtime/Robot`, `OpModeBase` | Loop order (bulk reads → reads → input → commands → writes → telemetry → log), init lockout, Ivy reset, fault policy, shutdown order |
| `SubsystemBase` | The read/write/stop/log lifecycle and passive default commands |
| `MecanumDriveSubsystem` | The one drive owner: stick shaping, field-centric, drive commands with requirements, interruption cleanup and measured completion |
| `LocalizerSubsystem`, `PoseEstimator`, `PoseHistory` | Start pose written at every INIT, Pinpoint readiness/fault watchdog, latency-compensated vision corrections |
| `GamepadEx`, `Trigger` | Deadbanded sticks, edges, and button bindings that schedule Ivy commands (Ivy has none) |
| `Alliance` | RED→BLUE transform with the season's symmetry (Pedro's `mirrorX` uses a different heading convention) |
| `FlightRecorder`, `WpiLogWriter`, `WpiStruct`, `StateLog` | WPILOG files for AdvantageScope |
| `logged`, `CommandHistory` | Command history for traced commands (Ivy has no lifecycle hooks or names) |
| `FieldView`, `TelemetryBag` | Panels field drawing and throttled DS/Panels telemetry |
| `ConfigStore` | Tuning that survives restarts and hot reloads |
| `DeviceReaders`, `Preflight`, `BulkReadManager`, `MotorIO`, `LoopProfile`, `StartDelay`, `MatchTimer`, `PIDFController` | Named hardware errors, missing-device listing, manual bulk caching, testable motors, loop timing, start delay, endgame rumble, gains |
| `pedro/Constants.java`, `pedro/Tuning.java` | Pedro's configuration and AutoTune registration, in the Quickstart layout |

## Config objects

Live-tunable values go in an `@Configurable` object with `@JvmField` vars,
registered in `configure()`:

```kotlin
@Configurable
object ShooterConfig {
    private const val DEFAULT_TARGET_RPM = 3200.0
    @JvmField var targetRpm: Double = DEFAULT_TARGET_RPM
    fun resetDefaults() { targetRpm = DEFAULT_TARGET_RPM }
}
ConfigStore.register("shooter", ShooterConfig, ShooterConfig::resetDefaults)
```

Values persist to `/sdcard/FIRST/config/tuning.properties` and survive power
cycles, installs and hot reloads. Don't `@Pinned` config objects. Keep
`resetDefaults()` covering every field. Add `safe*` clamping getters where a
bad Panels edit could hurt (see `DriveConfig`).

## Mechanisms

There is no generic mechanism base class. Build each lift/arm/turret as a
plain `SubsystemBase` against the real hardware, with `PIDFController` +
`PIDFGains` and the `MotorIO` seam (host tests can inject
`SimMotorIO(clock, …)`). Add homing, soft limits or profiles only once you can
validate them on the mechanism.

## Sensors and I²C

1. Keep Pinpoint direct on its own Control Hub I²C port. Pedro reads it inside
   `Follower.update()`.
2. Put auxiliary I²C sensors on one SRSHub and read it inline from
   `SRSHubSubsystem.periodic()`:
   `val srs = robot.register(SRSHubSubsystem()); val color = srs.color(bus = 1)`.
3. Don't background the SRSHub unless measurements prove the inline read is
   the loop-time problem. A background thread still shares the Lynx serial
   link with motor writes; that was tried and reverted for Pinpoint.

Before changing the policy: baseline direct Pinpoint loop time from a full
battery down to ~11 V; add the SRSHub inline and log `hub.update()` duration
and CRC mismatches; only then consider a bounded worker, and check motor-write
timing does not regress. Never share the SRSHub's in-place decoded objects
across threads.

## Season rollover

- Keep game-specific subsystems, paths and op-modes in the season fork.
- Set `RobotConfig.Field.SYMMETRY` from the game manual and verify the field length.
- Change `RobotConfig.CONFIG_SCHEMA`.
- Re-run AutoTune when the chassis, weight, wheels or odometry change.

## Migration notes (Ivy 1.1.1 + Pedro 3.0.0, September 2026)

Replaced: the repo's own scheduler/commands/groups (now Ivy), `PathDSL` and
`PedroAutoRunner` (now Pedro `Paths` + Ivy groups + drive commands),
`Pose2d`/`Vector2d` (now Pedro `Pose`/`Vector2D`/`Velocity`), Pedro 2
constants and the Panels tuning op-mode (now `pedro/Constants.java` and the
AutoTune web page), FTC SDK 11.1.0 (now 11.2.1, required by AutoTune).

Removed features:

- Flight log: `commands/running` (every scheduled command) and per-command
  `COMMAND STARTED/FINISHED/INTERRUPTED/FAULTED` events; `schedule blocked`
  and `schedule default` events. Ivy has no names, registry or lifecycle
  hooks. Command history came back later as `commands/events`,
  `commands/active` and `commands/lost`, covering `logged` commands only.
- `lastcrash.txt` and the recent-events ring (loop crashes still write their
  stack trace to `events`).
- Per-trigger fault quarantine (`TRIGGER FAULT`).
- Virtual-time autonomous simulation (`SimFollower`, `SimHarness`).
- Pedro 2 voltage compensation (no Pedro 3 equivalent).
- The command `maxPower` follow overload (use Foresight `maxPathSpeed`).

Changed behaviour:

- **Fault policy:** teleop used to end only the faulting command; auton used
  to crash the op-mode. Both now clear all commands, halt every subsystem,
  record the reason and continue (auton then stops because its routine is gone).
- **Shutdown:** command end handlers no longer run at op-mode stop.
- **Hold arrival** is measured against `DriveConfig` tolerances instead of
  Pedro 2 path constraints.
- **Paths refuse to run** until Foresight is tuned (`FORESIGHT_TUNED`).
- **Fresh localization every run:** every op-mode starts at its
  `startingPose` (teleop: 0, 0, 0). The autonomous-to-teleop pose handoff
  (`PersistedPose`) is gone. The Pinpoint's pose is written at INIT; its IMU is
  not recalibrated (`ResetMode.NONE`), so it relies on the power-up calibration.
- Drive mode values and all other WPILOG channel names and types are unchanged.
