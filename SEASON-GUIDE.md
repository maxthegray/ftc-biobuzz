# Season Guide

How to build a season on this framework. ARCHITECTURE.md explains *why* the
framework is shaped this way; this is the *how* for the code you'll write in
a fork: subsystems, commands, config, auton. Expert-oriented — it states
contracts, not tutorials.

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

## Single-motor mechanisms: don't hand-roll

A lift/arm/turret is `ProfiledMotorSubsystem` — profile + PIDF + soft
limits + stall-detect homing + hold-at-goal + log channels, already tested:

```kotlin
object LiftConfig {
    @JvmField var gains = PIDFGains(kP = 0.1, kV = 0.02, kG = 0.08)
    @JvmField var constraints = ProfileConstraints(maxVelocity = 30.0, maxAcceleration = 60.0)
}

val lift = robot.register(ProfiledMotorSubsystem(
    "Lift", "liftMotor", ProfiledController(LiftConfig.constraints, LiftConfig.gains),
    ticksPerUnit = 83.7,
    softMinUnits = 0.0, softMaxUnits = 26.0,
))
operator.button(GamepadEx.Button.Y).onTrue(lift.goToCommand(24.0, toleranceUnits = 0.5))
operator.button(GamepadEx.Button.BACK)
    .onTrue(lift.homeCommand(power = -0.3, stallVelocityUnitsPerSec = 1.0))
```

Host-test season mechanisms by injecting `io = SimMotorIO(clock, …)` — see
`ProfiledMotorSubsystemTest` for the pattern (including a homing run against
a simulated hard stop).

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

- **Selector**: `AutonSelector` on dpad in `onInitLoop` (A locks; last
  locked selection is restored as the default after a re-init).
- **Relocalization**: feed vision through
  `localizer.applyCorrection(measured, timestampNanos, …)` — gated,
  blended, axis-weighted, scaled down automatically mid-path. Wall contact:
  `WallSnap.pose(...)` with `blend = 1.0`. The AprilTag subsystem spec is in
  ARCHITECTURE.md (Localisation Hooks).

## Sim before carpet

Every routine deserves a `SimAutonRoutineTest`-style test: real path
geometry, real scheduling, real waits in virtual time, RED/BLUE mirror
symmetry, marker timing, pose handoff. A routine that's wrong in sim is
wrong on carpet; the reverse isn't guaranteed (sim doesn't model Pedro's
control quality), but it catches the whole class of sequencing/mirroring
bugs for free. `MechanismReplayTest` is the pattern for "did my refactor
change control outputs?".

## Deploy + diagnose

- `make hot` for iteration; full install after dependency/manifest/@Pinned
  changes (see CLAUDE.md Workflow).
- `make analyze` after every run that surprised you; `RUNBOOK.md` maps
  symptoms to channels. Watch the `Health` telemetry section during driver
  practice — contained faults show up there long before they show up as a
  dead mechanism in a match.
