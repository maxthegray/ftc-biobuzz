# Development Guide

How to add subsystems, commands, configuration, autonomous routines, and
sensors — the framework contracts and the patterns worth copying, without
repeating all of `AI-GUIDE.md`.

## A season subsystem, end to end

```kotlin
class IntakeSubsystem : SubsystemBase("Intake") {
    private lateinit var roller: MotorIO
    private lateinit var gate: Servo
    private var ballSeen = false

    override fun init(hardwareMap: HardwareMap) {
        roller = RealMotorIO(DeviceReaders.motor(hardwareMap, "intakeRoller",
            DcMotorSimple.Direction.REVERSE))
        gate = DeviceReaders.servo(hardwareMap, "intakeGate")
    }

    override fun periodic() {
        // READS ONLY. Bulk cache is fresh; never command actuators here.
        ballSeen = /* sensor read */ false
    }

    private var rollerPower = 0.0
    override fun writeHardware() {
        // The single flush point. Whatever commands decided this tick.
        roller.setPower(rollerPower)
    }

    fun grab(): Command = Command.build()
        .setName("intake grab")
        .requiring(this)
        .setPriority(CommandPriorities.DRIVER_ACTION)
        .setStart { rollerPower = 1.0 }
        .setDone { ballSeen }
        .setEnd { rollerPower = 0.0 }   // ALWAYS runs — cancel, fault, natural

    override fun onCommandFault() { rollerPower = 0.0 }  // safety net; never throw
    override fun health(): String? = if (ballSeen) "holding" else null

    override fun logState(log: StateLog) {
        log.put("rollerPower", rollerPower)
        log.put("ballSeen", ballSeen)
    }
}
```

The contract, compressed:

- **`init` resolves hardware** via `DeviceReaders` (missing devices throw
  `HardwareConfigError` with the name baked in). Add op-mode-level names to
  `requiredDevices` for the preflight listing.
- **`periodic()` reads. Commands decide. `writeHardware()` flushes.** The
  scheduler's requirements system only protects you if actuator state is
  written once, in `writeHardware`, from fields that commands set.
- **End handlers always run.** Cleanup belongs in `setEnd`;
  `onCommandFault()` is only the net for when the end handler itself is the
  buggy code.
- **`logState` is your tuning view** — channels land in the .wpilog as
  `Intake/rollerPower` etc. Log goals, setpoints, measurements, outputs.
- **Register in `configure()`** (`robot.register(IntakeSubsystem())`), wire
  bindings there too (they lock at start). TeleOps extend `TeleOpBase` and
  use `configureTeleop()`.

Bench op-modes: the default `requiredDevices` is `Preflight.standard`
(drive motors + Pinpoint), so a mechanism-only test rig refuses to init.
Override it with just what the rig has:

```kotlin
override val requiredDevices: List<Preflight.Requirement>
    get() = listOf(Preflight.Requirement("liftMotor", DcMotorEx::class.java))
```

## Single-motor mechanisms

There is no generic mechanism base class in this repo, on purpose. A
`ProfiledMotorSubsystem` (profile + PIDF + soft limits + stall homing) and a
`TrapezoidProfile` were written during the 2026 offseason, never got wired to
a real mechanism, and were deleted before kickoff rather than carried as
untested surface. Code review kept generating findings against them that no
match could ever hit.

When this season has an actual lift/arm/turret: build it as a plain
`SubsystemBase`, against the real hardware, with `PIDFController` +
`PIDFGains` from `core/control/` and the `MotorIO` seam from `core/io/`
(inject `SimMotorIO(clock, …)` to host-test it). Homing, soft limits, and
motion profiling are worth adding only once you can validate each on the
mechanism itself. `git log --diff-filter=D` will surface the old
implementation if it turns out to fit.

The live-holder config pattern below is still the right shape for tunables:
primitive `@JvmField` fields are what Panels discovers and `ConfigStore`
persists; non-primitive holders get synced from them in `periodic()`.
`ConfigStoreTest.primitiveMechanismConfigRoundTripsIntoLiveHolders` locks
that down.

## Commands and priorities

- Build with `Command.build()` or the `Commands` helpers; compose with
  `Groups`. Always `setName(...)` — it's what the flight log shows.
- The ladder: defaults `0` < auton/assists `10` < driver actions `20` <
  panic overrides `30`. Blocked only by *strictly higher*; equal preempts.
- Command instances are reusable but **`setStart` must fully reset per-run
  state**. If the command depends on state known only at run time (a path
  from the current pose), use `Commands.defer(requirements) { build() }`.
- Waits: `Commands.waitMs(ms, robot.clock)` — inject the clock and the
  routine simulates. `PedroAutoRunner.wait()` already does.

## Config objects

Live-tunable values go in an `@Configurable` object with `@JvmField` vars,
registered with the store in `configure()`:

```kotlin
@Configurable
object ShooterConfig { @JvmField var targetRpm: Double = 3200.0 }
// in configure():
ConfigStore.register("shooter", ShooterConfig)
```

Tuned values persist to `/sdcard/FIRST/config/tuning.properties` and restore
at every init — power cycles, installs, and hot reloads included. Do **not**
`@Pinned` config objects. Add `safe*` clamping getters for values where a
fat-fingered Panels edit could hurt (see `DriveConfig` for the pattern).

## Auton

`ExampleAuto` is the copyable skeleton. The pieces:

- **Poses in RED coordinates**, as `Pose2d`. The `path` DSL and
  `Alliance.mirror` transform for BLUE — including heading interpolation
  args and `turnTo` targets, which pose mirroring alone misses. Set
  `RobotConfig.Field.SYMMETRY` (MIRROR vs ROTATE) when the game launches.
- **Sequence with `autoRoutine(robot, drive, robot::recordEvent) { … }`** —
  follows, holds, turns, waits, parallel/race/deadline groups, and
  mid-path **markers**:

  ```kotlin
  follow(toScore) {
      at(0.3) { lift.setGoal(HIGH) }
      at(0.85, "deploy") { intake.deploy() }
  }
  ```

- **Alliance and routine**: one `@Autonomous` op-mode each — no init-loop
  menu. The Driver Station dropdown already shows the selection in large text
  with nothing to confirm, which beats a telemetry line plus a lock button
  under match pressure. A BLUE variant copies the RED op-mode and overrides
  `initialAlliance`; everything else mirrors automatically.
- **Start delay**: `StartDelay(telemetryBag)` on dpad left/right in
  `onInitLoop`, 0–10 s. The one choice that can't be a separate op-mode —
  it's decided in the alliance meeting to dodge a partner's auto.
- **Start gate**: before setting the starting pose or scheduling, abort loudly
  if `localizer.fault` is already latched. Also check
  `PedroAutoRunner.schedule()`; false means the routine never started.
- **Relocalization**: feed vision through
  `localizer.applyCorrection(measured, timestampNanos, …)` — gated,
  blended, axis-weighted, scaled down automatically mid-path. Use the camera
  frame-acquisition timestamp, not the time processing finished.

## Sim before carpet

Every routine deserves a `SimAutonRoutineTest`-style test: real path
geometry, real scheduling, real waits in virtual time, RED/BLUE mirror
symmetry, marker timing, pose handoff. A routine that's wrong in sim is
wrong on carpet; the reverse isn't guaranteed (sim doesn't model Pedro's
control quality), but it catches the whole class of sequencing/mirroring
bugs for free.

## Sensors and I²C

The default wiring policy is deliberate:

1. Keep Pinpoint direct on its own Control Hub I²C port. Pedro reads it inline
   inside `Follower.update()`.
2. Put auxiliary I²C sensors on one SRSHub and read the hub inline from
   `SRSHubSubsystem.periodic()`.
3. Do not background the SRSHub unless measurements prove the inline read is
   the loop-time problem.

The SRSHub helps because downstream sensor retries happen on the hub and the
Control Hub performs one bounded register read. A background thread does not
remove that read from the Control Hub's Lynx serial link; it can instead
contend with motor writes and leave the follower using stale data. That exact
failure mode was tried and reverted for Pinpoint.

Integration rules:

- Register the drive before the localizer so pose history is sampled after
  `Follower.update()`.
- Pinpoint through the SRSHub is not supported. The direct connection preserves
  the raw device-status watchdog and keeps Pedro's tuning op-modes identical to
  the competition localizer.
- If threading auxiliary sensors is ever justified, publish one immutable
  snapshot and poll near the main-loop rate. Never share the SRSHub's in-place
  decoded objects across threads.

Measure in this order before changing the policy:

1. Baseline direct Pinpoint from a full battery down to roughly 11 V. Compare
   loop time against voltage.
2. Add the SRSHub with auxiliary sensors only, still inline. Log
   `hub.update()` duration and CRC mismatches.
3. Only if the inline SRSHub read itself stretches the loop, evaluate moving
   it to a bounded background worker — and verify that motor-write timing
   does not regress. (An unused `I2CBusThread` was deleted before kickoff;
   write the smallest thing the measurement justifies.)

Typical auxiliary-sensor setup:

```kotlin
val srs = robot.register(SRSHubSubsystem())
val intakeColor = srs.color(bus = 1)
val frontDistance = srs.distance(bus = 2)
```

Register SRSHub devices before robot initialization, then consume their latest
values from the owning subsystem's `periodic()`.

## Season rollover

- Keep game-specific subsystems, paths, and op-modes in the season fork.
- Set `RobotConfig.Field.SYMMETRY` from the game manual and verify the field
  length before writing paths.
- Change `RobotConfig.CONFIG_SCHEMA` so stale tuning files from the previous
  season are ignored.
- Recalibrate every placeholder in `pedroPathing/Constants.java` when the
  chassis, weight, wheel setup, or odometry geometry changes.
- Keep config objects unpinned and registered with `ConfigStore`.

## Deploy + diagnose

- `make hot` for iteration; full install after dependency/manifest/@Pinned
  changes.
- `make analyze` after every surprising run; `OPERATIONS.md` maps symptoms to
  channels. Watch the `Health` telemetry section during driver practice —
  contained faults show up there before they become match failures.
