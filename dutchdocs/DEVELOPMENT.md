# Development Guide

Short workflows for the things you do most, then the rules behind them.
Robot code uses [Ivy](https://pedropathing.com/docs/ivy) for commands and
[Pedro Pathing 3](https://pedropathing.com/docs/pathing) for poses, paths and
the drivetrain, directly. `AI-GUIDE.md` has the full contract.

Paths below are relative to `TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/`.

## How the code thinks

**A subsystem is one mechanism**: the intake, the transfer, the drive. It
owns its motors and sensors, and nothing else touches them. It keeps a
*target*, like `power = 0.0`.

**A command is something a subsystem is doing**: "run the intake", "drive
this path", "hold the blocker closed". A command has four parts, all
optional:

```kotlin
Command.build()
    .requiring(intake)        // which subsystem it uses
    .setStart { }             // once, when it starts
    .setExecute { }           // every loop while it runs
    .setDone { false }        // when true, it finishes (default: never)
    .setEnd { }               // once, when it finishes or gets replaced
```

Most mechanism commands are "do this until something else takes over", and
Ivy has a shortcut for that:

```kotlin
infinite { power = 1.0 }.requiring(this).setEnd { power = 0.0 }
```

**One command per subsystem at a time, and the newest one wins.** Press the
eject button while the intake is collecting: the collect command ends (its
`setEnd` sets the power to 0), and eject starts. That's all of the conflict
handling. The single exception is the drive's localizer-fault fallback,
which nothing can replace.

**A default command is what a subsystem does when nothing else is using
it**: stick driving for the drive, "blocker closed" for the transfer. It
comes back by itself when the other command ends.

**Every loop runs in the same order:**

1. Each subsystem reads its sensors (`periodic()`).
2. Button bindings start and stop commands.
3. Every running command updates its subsystem's target.
4. Each subsystem writes its target to the hardware (`writeHardware()`).

So the rule is: **subsystems read and write, commands decide.** A command
never touches a motor; it sets the target, and the subsystem writes it.

**Autos are commands too**, glued together: `sequential(a, b, c)` runs one
after another, `parallel(a, b)` runs them together, and `race(a, b)` stops
both when either finishes (that's how a timeout works).

## Add a subsystem

Create `subsystems/IntakeSubsystem.kt`:

```kotlin
class IntakeSubsystem : SubsystemBase("Intake") {
    private lateinit var roller: DcMotorEx
    var rollerPower = 0.0
        private set
    var ballSeen = false
        private set

    override fun init(hardwareMap: HardwareMap) {
        roller = hardwareMap.get(DcMotorEx::class.java, "intakeRoller").apply {
            direction = DcMotorSimple.Direction.REVERSE
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    override fun periodic() {
        ballSeen = false // read sensors here; never command motors here
    }

    override fun writeHardware() {
        roller.power = rollerPower // the one place motor power is written
    }

    fun grab(): Command = logged( // named in the flight log's command history
        "Intake grab",
        Command.build()
            .requiring(this)
            .setStart { rollerPower = 1.0 }
            .setDone { ballSeen }
            .setEnd { rollerPower = 0.0 }, // only ever makes the roller safe
    )

    override fun onCommandFault() { rollerPower = 0.0 }
    override fun stop() { rollerPower = 0.0; roller.power = 0.0 }
}
```

Register it in your op-mode's `configure()` (or `configureTeleop()`):
`val intake = robot.register(IntakeSubsystem())`.

Rules:

- `periodic()` reads, commands decide, `writeHardware()` writes.
- Public read-only scalar state logs automatically. Override `logState` only
  for arrays or specific channel names.
- Every command that touches the subsystem says `requiring(this)`, so Ivy
  lets only one of them run. Use `Commands.lazy { … }` when the command
  depends on state known only when it starts.
- `setEnd` runs when the command finishes, is replaced, or throws (a
  `logged()` command only), but **not** when the op-mode stops or an
  unlogged command crashes: `stop()` and `onCommandFault()` make the
  mechanism safe then.
- Only make things safe in `setEnd` (power to 0, blocker closed). Ivy can
  run it for a command that never started, or twice, so it must not start
  anything or count anything. AI-GUIDE.md shows the pattern if you need to.
- Bench rigs without the drivetrain override `requiredDevices`:
  `get() = listOf(Preflight.Requirement("liftMotor", DcMotorEx::class.java))`.

## Bind a button

In `configure()` / `configureTeleop()`, using Ivy commands:

```kotlin
driver.button(Button.CROSS).onTrue(intake.grab())             // schedule on press
driver.button(Button.L1).whileTrue(intake.eject())            // cancel on release
driver.trigger { driver.r2 > 0.5 }.whileTrue(shooter.spinUp())
(driver.button(Button.CREATE) and driver.button(Button.TRIANGLE)).onTrue(instant { … })
```

We only use PS5 controllers, so buttons have PS5 names. Bindings lock at
start. Read sticks directly (`driver.leftStickY` is +up), and buttons with
`driver.held(Button.X)` / `driver.pressed(Button.X)` (edge since last loop).
`TeleOpBase` already binds Create+Triangle (heading reset), Create+Circle
(field-centric) and installs stick driving as the drive's default command.
Match teleop also buzzes both controllers twice at 1:00 left (FLOWER
ownership unlocks) and three times at 0:20 left (`TeleOpBase.onStart`).

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
            logged("Start delay", waitMs(startDelaySec * 1000.0)),
            race(
                drive.followCommand(toScore(), name = "Drive to score"),
                logged("Drive to score time limit", waitMs(4_000.0)), // step timeout
            ),
            drive.holdCommand(score, name = "Settle at score"),               // wait for arrival
            instant { robot.recordEvent("AUTO: scored") },
            drive.turnToCommand(alliance.mirror(Math.toRadians(90.0)), name = "Face the wall"),
        ),
        logged("Routine time limit", waitMs(29_000.0)),             // whole-routine timeout
    ),
)
```

Schedule, check and cancel the returned `routine()` instance. Drive commands
take a `name`; wrap everything else worth seeing with `logged(...)`.

Pass the start pose to the localizer in `configure()`:
`LocalizerSubsystem(follower, ..., startingPose = start)`. It is written to the
Pinpoint at INIT, so place the robot before pressing INIT; nudging it into
place during init is fine, the pose snaps back to the start pose. Pass
`onStatusFault = {}` so a Pinpoint status problem is only logged and the auto
keeps driving; only a lost pose (`onFault`) cancels it. In `onStart`: refuse to
run if `!Constants.FORESIGHT_TUNED`. Then, from `onLoop`, start the routine once
`localizer.ready`, or after one second anyway unless `localizer.fault` is set
(START pressed right after INIT): `Scheduler.schedule(...)` and check
`Scheduler.isScheduled(...)`. Stop the op-mode once the routine is no longer
scheduled.

- **Waits:** Ivy's `waitMs(ms)`.
- **Markers** (do something part-way along a path, once):
  `deadline(drive.followCommand(path), sequential(waitUntil { drive.pathProgress() >= 0.5 }, instant { lift.up() }))`.
  If the path ends or is cancelled first, the marker is dropped.
- **Completion:** `followCommand` ends at Pedro's *parametric end*, which is
  not arrival. Follow it with `holdCommand(pose)` when arrival matters.
  `turnToCommand` ends on its timeout too and the routine moves on.
- **Turning along a path:** `.linear(a, b)` (see `backPath()` in the
  example), or `.constant(...)` to keep one heading.
- **Alliances:** one `@Autonomous` class per alliance/routine; the BLUE copy
  overrides `initialAlliance` only. Map poses with `alliance.poses()`, not
  `PoseFactory.mirrorX`.
- **Where was the robot when the camera saw that?** `localizer.poseAt(captureNanos)`
  interpolates the last few seconds of poses.

## Logging

Logging is automatic; see [dutchdocs/LOGGING.md](LOGGING.md) for state logging,
WPILOG channels, command history, settings, viewers and post-match debugging.

## Commands and faults

- The newest command for a subsystem replaces the running one. Don't set
  priorities; the localizer-fault fallback is the only command with one.
- A default command never replaces a running command; it comes back when the
  subsystem is free.
- **If a `logged()` command throws an `Exception`** (for example
  `error("…")` or `require(...)`), only that command ends. Its subsystems are
  halted (`onCommandFault()`), the log gets `COMMAND FAULT: <name>: …`, and an
  auto moves on to its next step. Wrap every routine step in `logged()`; the
  drive commands already are.
- An exception from a binding or an unlogged command clears every command
  and halts every subsystem instead. Defaults come back; in auto the routine
  is gone and the op-mode stops.
- **If a subsystem's `periodic()` or `writeHardware()` throws,** the log gets
  `SUBSYSTEM FAULT` and the other subsystems keep running. After a failed
  write that subsystem's outputs are zeroed (`onWriteFault()`). The drive
  keeps following its path, so a one-tick glitch costs one tick.
- Health telemetry shows both counts.
- **An `Error` ends the op-mode,** and so does an exception from `onLoop()`.
  `TODO()` throws `NotImplementedError`, which is an `Error`: code still
  containing `TODO()` stops every motor, records `LOOP CRASHED` with the stack
  trace, and ends the run. Replace every `TODO()` before driving.
- **If the localizer fails** in teleop, the driver keeps robot-centric sticks
  for the rest of the run; paths, holds and turns refuse to start.

## What the custom code is for

Everything else is Ivy or Pedro. Each remaining helper has one job:

| Helper | Why it exists |
|---|---|
| `core/Robot`, `OpModeBase` | Loop order (bulk reads → reads → input → commands → writes → telemetry → log), init lockout, Ivy reset, fault policy, shutdown order |
| `SubsystemBase` | The read/write/stop/log lifecycle and passive default commands |
| `MecanumDriveSubsystem` | The one drive owner: stick shaping, field-centric, drive commands with requirements, interruption cleanup and measured completion |
| `LocalizerSubsystem`, `PoseHistory` | Start pose written at every INIT, Pinpoint readiness/fault watchdog, pose history for camera latency |
| `GamepadEx`, `Trigger` | Deadbanded sticks, edges, and button bindings that schedule Ivy commands (Ivy has none) |
| `Alliance` | RED→BLUE transform with the season's symmetry (mirror or rotate; Pedro's `mirrorX` only mirrors) |
| `TelemetryBag`, `FieldView` | Throttled DS/Panels telemetry and Panels field drawing |
| `Preflight`, `LoopProfile` | Missing-device listing, loop timing |
| `pedro/Constants.java`, `pedro/Tuning.java` | Pedro's configuration and AutoTune registration, in the Quickstart layout |

## Config objects

Live-tunable values go in an `@Configurable` object with `@JvmField` vars:

```kotlin
@Configurable
object ShooterConfig {
    @JvmField var targetRpm = 3200.0
}
```

Panels edits these while the robot runs. **They are not saved**: an op-mode
restart, app restart or hot reload puts back what is in the code. When a value is right,
type it into the file. The code is always the truth about how the robot is
tuned. Don't `@Pinned` config objects.

## Mechanisms

Season mechanisms live in `subsystems/` next to the drive: `IntakeSubsystem`,
`TransferSubsystem`, `ShooterSubsystem` and `TurretSubsystem`.

**Intake and Transfer are implemented** against placeholder hardware:
- Names are in `RobotConfig.Intake` / `RobotConfig.Transfer`. The motor names
  are lists, so a second motor is one more name.
- Powers, blocker positions and the blocker travel time are in
  `IntakeConfig` / `TransferConfig`: placeholders to tune in Panels.
- Intake: `collect()` / `eject()`.
- Transfer owns the middle motors and the blocker servo together, so motors
  never push into a closed blocker:
  - `hold()` is the default (blocker closed, motors off);
  - `stage()` pushes balls up against the closed blocker;
  - `feed()` opens the blocker and runs the motors only after
    `blockerTravelMs`;
  - `reverse()` runs the motors backwards with the blocker closed.
- Collect-and-stage is `parallel(intake.collect(), transfer.stage())`.
- Neither is registered in any op-mode yet: the sensorbot has no such
  hardware, and `init` fails on a missing device.
- `ballStaged` is null until a ball sensor is fitted.

**Shooter and Turret are still unconfigured skeletons**; their `init` fails
explicitly.
- Shooter will accept target RPM from `ShotModel.rpmForDistance` (in `ShooterSubsystem.kt`), a
  distance-to-speed function still to be fitted from shots at measured
  distances (it returns null until then).
- Turret owns all mechanically linked turret servos. It will keep a short
  timestamped history of its servo-encoder angle and give `HiveTracker` a
  lookup into it, so each camera frame is placed at its capture time.
- Once CAD defines their actuators: resolve them with `hardwareMap.get` in
  `init`, add logged Ivy command factories requiring the subsystem, implement
  reads/writes and fault/stop cleanup, then register in `configure()`.

There is no generic mechanism base class. Build each lift/arm/turret as a
plain `SubsystemBase` using the SDK's `DcMotorEx` and `Servo` directly. Unit
tests put fake devices (`MotorProbe`, `ServoProbe` in `test/…/core/sim/`)
into a `FakeHardwareMap`. Add closed-loop control, homing, soft limits or profiles only once you can
validate them on the mechanism.

## Vision layout

Season vision code that runs in a match is in `vision/`:

- `BallCamera*`, `LensIntrinsics`: the USB ball camera on the SDK's color-blob processor, its Panels
  settings, and lens calibration.
- `BiobuzzAprilTags`: season tag identities and FIRST's cluster geometry.
- `Hive*`, `GoalGeometry`: HIVE goal tracking for the turret-mounted Limelight. `HiveConfig`
  holds the camera mount and thresholds; `GoalGeometry` places each tag's
  implied goal (its CELL's opening centre) on the robot through that mount and
  the turret angle at capture time; `HiveState` infers which CELL of each HIVE
  is raised from tag heights, with hysteresis and tip counts; `HiveTracker`, a
  subsystem, fuses each alliance's goal and exposes both alliances to op-modes.
Tests mirror these packages. The reusable Limelight device adapter is
`subsystems/LimelightSubsystem.kt`. The Vision Test op-mode and its tooling
(tag-sighting history, survey memory, `VisionTestConfig`) live together in
`opmodes/diagnostics/VisionTestTeleOp.kt`.

## Sensors and I²C

1. Keep Pinpoint direct on its own Control Hub I²C port. Pedro reads it inside
   `Follower.update()`.
2. For several auxiliary I²C sensors, use an SRS Hub read inline in
   `periodic()`: in the August lab test it saved about 19 ms a loop over
   direct I²C (`PROGRESS.md`). Its driver and subsystem were removed until a
   sensor needs them; restore `SRSHub.java` and `SRSHubSubsystem.kt` from
   commit `33e20b6`.
3. Don't read sensors on a background thread unless measurements prove the
   inline read is the loop-time problem. A background thread still shares the
   Lynx serial link with motor writes; that was tried and reverted for
   Pinpoint.

## Season rollover

- Keep `core/`, the drive, localizer and Limelight; replace the season's mechanisms, vision and op-modes.
- Set `RobotConfig.Field.SYMMETRY` from the game manual and verify the field length.
- Re-run AutoTune when the chassis, weight, wheels or odometry change.
