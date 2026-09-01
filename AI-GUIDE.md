# AI Guide

Canonical instructions for every AI assistant working in this repository.
Read this entire file before changing code.

Human documentation has three entry points:

- `README.md` — setup and repository map
- `DEVELOPMENT.md` — team coding patterns, auton, configuration, and sensors
- `OPERATIONS.md` — hardware bring-up, tuning, logging, and diagnosis

## What this repo is

BioBuzz's robot code for the current FTC season, forked from
`maxthegray/ftc-starter` — itself built on an unmodified clone of
`FIRST-Tech-Challenge/FtcRobotController` 11.1.0.

Season code **does** belong here; that is what this repo is for. The
boundary that matters is a directory one:

- `core/` is season- and chassis-agnostic framework. Fixes made here get
  cherry-picked back to `ftc-starter`, so keep it free of game logic,
  season constants, and this year's mechanism names.
- Everything else — `opmodes/`, season subsystems, `RobotConfig`,
  `pedroPathing/Constants.java` — is this season's and never flows upstream.

The code currently runs on a **sensorbot**: a temporary chassis to develop
against while the competition robot is built. The competition robot replaces
it in place — re-measure `pedroPathing/Constants.java`, update `RobotConfig`,
bump `RobotConfig.CONFIG_SCHEMA`. There is no sensorbot branch and no robot
profile switch: the swap is one commit on `main`, with the last sensorbot
commit tagged `sensorbot-final`.

Stack (exact versions; these are load-bearing):

| Library               | Coordinate                                | Version |
|-----------------------|-------------------------------------------|---------|
| FTC SDK               | `org.firstinspires.ftc:*`                 | 11.1.0  |
| Kotlin Android plugin | `org.jetbrains.kotlin:kotlin-gradle-plugin` | 2.0.21  |
| Pedro Pathing core    | `com.pedropathing:core`                   | 2.1.1   |
| Pedro Pathing FTC     | `com.pedropathing:ftc`                    | 2.1.1   |
| Pedro Telemetry       | `com.pedropathing:telemetry`              | 1.0.0   |
| Bylazar fullpanels    | `com.bylazar:fullpanels`                  | 1.0.12  |
| Sloth (Sinister)      | `dev.frozenmilk.sinister:Sloth`           | 0.2.4   |

Maven repositories (also load-bearing):
- `mavenCentral()` — Pedro
- `google()` — AndroidX + AGP
- `https://mymaven.bylazar.com/releases` — fullpanels
- `https://repo.dairy.foundation/releases` — Sloth + transitively Sinister

If you need to bump any version, **verify** the new version exists in the
corresponding repo before editing `build.dependencies.gradle` /
`TeamCode/build.gradle`. Don't guess.

## Optimize for the best decision, not the cheapest

This project has no constraints on time, budget, or engineering skill.
When a decision forks between a pragmatic compromise and the genuinely
better engineering answer, take the better one — "more work", "harder to
implement", or "the quick way is fine for now" are not reasons to
compromise here. State the optimal approach plainly; it will get built.

This governs the *quality* of decisions, not the *quantity* of work. It is
**not** licence to over-engineer, gold-plate, or add speculative
abstractions — the "don't do this unless asked" rules below still hold,
and the best engineers ship the right thing, not the most thing. Optimal
means: at a real fork, choose what's better long-term over what's easier
today.

## Critical API cheatsheet

These are the framework / Pedro calls used throughout the codebase. They
are stable as of the versions above and wrong in at least one piece of
publicly-searchable documentation.

### Command scheduler (`core/command/` — ours, not a library)

The scheduler is **instance-scoped and owned by `Robot`** — there is no
global scheduler. Access it as `robot.scheduler` (op-modes) or take it as a
constructor parameter (framework components).

```kotlin
robot.scheduler.execute()        // one tick; Robot.loop() already calls this
robot.scheduler.reset()          // interrupt everything (end handlers RUN)
robot.scheduler.schedule(cmd)    // returns false if blocked or start faulted
robot.scheduler.cancel(cmd)      // end handler runs with INTERRUPTED
robot.scheduler.isScheduled(cmd)
robot.scheduler.runningCommandNames()  // native introspection, no reflection
```

Semantics: a command is blocked only by a *strictly higher*-priority holder
of one of its requirements; equal priority preempts. End handlers always run
(natural end, cancel, reset, preemption, fault). A lifecycle exception ends
only the faulting command (`EndCondition.FAULTED`) and is routed to
`Robot.containCommandFaults` policy — teleop contains it surgically, auton
rethrows.

### Building a command

```kotlin
Command.build()
    .setStart { ... }            // called once when scheduled
    .setExecute { ... }          // called every tick while running
    .setDone { returnBoolean }   // called every tick; true → end
    .setEnd { endCondition -> ... }  // ALWAYS runs (NATURALLY/INTERRUPTED/FAULTED)
    .requiring(driveSubsystem)   // vararg requirements
    .setPriority(10)             // higher priority interrupts lower
    .setName("score preload")    // shows up in flight-log commands/running
```

Or use the helpers:

```kotlin
Commands.instant { action() }            // runs once and completes
Commands.waitMs(500.0)                   // Double and Long overloads exist
Commands.waitMs(500.0, robot.clock)      // inject the clock → simulable waits
Commands.waitUntil { condition }
Commands.infinite { action() }
Commands.defer(drive) { buildCommand() } // construct at schedule time
```

And composition via `Groups`:

```kotlin
Groups.sequential(a, b, c)
Groups.parallel(a, b, c)
Groups.race(a, b, c)
Groups.deadline(deadlineCmd, a, b)
```

Framework priority ladder (higher interrupts lower; gaps are for season levels):

```kotlin
CommandPriorities.DEFAULT         // 0:  subsystem default commands
CommandPriorities.AUTON_ROUTINE   // 10: auton routines & teleop auto-assists
CommandPriorities.DRIVER_ACTION   // 20: driver-triggered actions (beat assists)
CommandPriorities.DRIVER_OVERRIDE // 30: panic / manual-override bindings
```

### Geometry (`core/geometry/` — ours)

Framework code speaks `Pose2d` / `Vector2d` (inches, radians, CCW-positive,
Pedro field frame). Pedro's `Pose` appears **only** in the adapter layer:
`core/pathing/PedroConversions.kt` (`toPedro()` / `toCore()`), `PathDSL`,
`MecanumDriveSubsystem` internals, the Pedro `Localizer` implementations,
and the vendored `pedroPathing/` files. Don't import
`com.pedropathing.geometry` anywhere else.

```kotlin
Pose2d(x, y, heading)                  // data class; .withHeading, .lerp, .distanceTo
pose.relativeTo(origin)                // this pose in origin's frame
origin.transformBy(delta)              // inverse of relativeTo
pose.mirror(symmetry, fieldLength)     // RED→BLUE; symmetry from RobotConfig.Field
normalizeAngle(rad)                    // [0, 2π)
shortestAngleDelta(from, to)           // (-π, π], half-turn resolves to +π
```

### Pedro Follower (adapter-layer use only)

The follower is `internal` to `MecanumDriveSubsystem` — op-modes use the
subsystem's `pose` / `velocity` / commands, the localizer's
`applyCorrection`, and `DriveTelemetrySource` for logging. Inside the
adapter layer:

```kotlin
follower.update()                        // MUST run every tick (in writeHardware)
follower.pose                            // Pose getter
follower.velocity                        // Pose getter
follower.setPose(p)                      // hard snap — useful for field-pose correction
follower.setStartingPose(p)              // treats p as the origin; prior movement shifts
follower.startTeleopDrive(brakeMode)     // switch to manual driving mode
follower.setTeleOpDrive(fwd, strafe, turn, isRobotCentric)
follower.followPath(chain, holdEnd)      // auton
follower.holdPoint(pose)                 // pin position
follower.breakFollowing()                // cancel a path
follower.isBusy                          // true while a path is running
follower.atPose(pose, xTol, yTol, headingTol)
follower.pathBuilder()                   // returns a fresh PathBuilder
```

Important: `Follower.update()` is heavy (runs localiser + PIDs + writes
motors). Call it exactly once per tick, from `MecanumDriveSubsystem.writeHardware()`.

### PathBuilder

```kotlin
follower.pathBuilder()
    .addPath(BezierLine(Pose(0.0, 0.0), Pose(24.0, 0.0)))
    .addPath(BezierCurve(listOf(p1, p2, p3)))
    .curveThrough(0.5, p1, p2, p3)
    .setLinearHeadingInterpolation(startRadians, endRadians)
    .setConstantHeadingInterpolation(radians)
    .setTangentHeadingInterpolation()
    .setReversed()
    .build()  // -> PathChain
```

The Kotlin `PathDSL` wraps these with cleaner names — prefer it.

### Drive subsystem defaults

Teleop should be a default command, not an imperative `onLoop()` call:

```kotlin
drive.defaultCommand = drive.teleopCommand {
    MecanumDriveSubsystem.TeleopInput(
        driver.leftStickY,
        driver.leftStickX,
        driver.rightStickX,
        precision = driver.rightTrigger > 0.1,
    )
}
```

`driveRaw()` remains public for tuning/bring-up only. The old imperative
drive/path helpers are internal; use `followCommand`, `holdCommand`,
`turnToCommand`, `PedroAutoRunner`, and trigger bindings for real op-modes.

Season teleops should extend `TeleOpBase` instead of wiring this by hand:
it registers drive + localizer, installs the teleop default command,
restores the persisted auton pose, and wires the standard driver chords —
**Back+Y** heading reset and **Back+B** field-centric toggle (Back, not
Start: Start+A/B are the Driver Station's gamepad re-bind chords). Wire
season subsystems and bindings in `configureTeleop()`.

Useful runtime hooks:

```kotlin
localizer.restorePersistedPose()                 // auton → teleop pose handoff
                                                 // (survives RC process restart via
                                                 //  /sdcard/FIRST/persisted-pose.txt)
localizer.applyCorrection(measured, timestampNs) // latency-compensated vision seam;
                                                 // returns CorrectionResult (APPLIED /
                                                 // STALE / NO_HISTORY / REJECTED_JUMP),
                                                 // gated + blended via LocalizerConfig;
                                                 // translationWeight/headingWeight for
                                                 // partial corrections; auto-scaled by
                                                 // followingBlendScale while pathing
val startDelay = StartDelay(telemetryBag)       // dpad left/right in the init loop
robot.recordEvent("marker")                      // also writes to WPILOG
autoRoutine(robot, drive, robot::recordEvent) { ... }  // optional event sink → labelled
                                                 // per-step timeline in the WPILOG
```

### Mechanism control toolkit (`core/control/`)

```kotlin
val gains = PIDFGains(kP = 0.1, kV = 0.02, kG = 0.08)   // mutable; Panels-tunable
val constraints = ProfileConstraints(maxVelocity = 30.0, maxAcceleration = 60.0)
val lift = robot.register(
    ProfiledMotorSubsystem("Lift", "liftMotor", ProfiledController(constraints, gains),
        ticksPerUnit = 83.7,
        softMinUnits = 0.0, softMaxUnits = 26.0),   // clamp goals, gate open-loop
)
operator.button(GamepadEx.Button.Y).onTrue(lift.goToCommand(24.0, toleranceUnits = 0.5))
operator.button(GamepadEx.Button.BACK).onTrue(
    lift.homeCommand(power = -0.3, stallVelocityUnitsPerSec = 1.0,
        timeoutMs = 3_000.0))  // timeout faults; it never silently declares zero
lift.openLoop(0.3)   // bring-up only; setGoal()/goToCommand() for real control
```

`ProfiledMotorSubsystem` holds the last goal after a command ends (gravity
hold) — no default command needed. The encoder is not reset on init by
default so lift position survives the auton → teleop handoff; `homeCommand`
re-establishes a true zero (velocity-stall detection, software offset).
A fresh mechanism is UNHOMED: open-loop movement and homing are allowed, but
closed-loop goals and coordinate-based soft limits are rejected until a zero
is established or restored.

Hardware goes through the `MotorIO` seam (`core/io/`): real op-modes
resolve a `RealMotorIO` automatically; host tests inject
`SimMotorIO(clock, …)` via the `io` constructor parameter and the whole
mechanism — profile, PIDF, soft limits, homing — runs headless
(`ProfiledMotorSubsystemTest`, `MechanismReplayTest`).

Two packages sound alike and are not interchangeable. `core/io/` is the
**abstraction seam** — interfaces plus their real implementations, so
subsystem code can run against hardware or a fake. `core/hardware/` is
**concrete device drivers** (`SRSHubSubsystem`, `I2CBusThread`) that only
work on a real robot. Every test double lives in `core/sim/` under the test
source root — `SimFollower`, `SimHarness`, `SimMotorIO`, `FakeClock`,
`FakeSink` — so nothing fake ever ships in the APK. Doubles used by a single
test stay nested in that test.

Subsystems log tuning channels by overriding
`logState(log: StateLog)` — the flight recorder prefixes them with
`<subsystem name>/` (e.g. `Lift/goalUnits`, `Lift/setpointUnits`,
`Lift/outputPower`) for AdvantageScope.

### Panels telemetry

```kotlin
val tm = PanelsTelemetry.telemetry   // TelemetryManager
tm.addLine("hello")
tm.addData("key", value)
tm.update()                                    // flush to dashboard
tm.wrapper                                     // an FTC Telemetry facade
JoinedTelemetry(telemetry, tm.wrapper)         // wrap both DS + Panels as one Telemetry
```

### Observe-only failure policy

Telemetry and logging must never kill the robot. `TelemetryBag` isolates the
Driver Station and Panels sinks, permanently disables a sink that throws, and
clears its buffers in `finally`. `FlightRecorder` disables only the throwing
subsystem's `logState` channels for non-I/O season-code faults; recorder I/O or
other recorder-phase faults close the recorder while the op-mode continues.
Keep framework I/O paths fail-fast inside the recorder so those faults reach
this containment boundary.

### Gamepad triggers

`GamepadEx` binds buttons (and any boolean condition) to commands on the
robot's scheduler. Wire bindings once in `configure()` — don't poll
`*Pressed` flags in `onLoop()` and call `robot.scheduler.schedule` by hand.

```kotlin
driver.button(GamepadEx.Button.A).onTrue(intake.grab())
driver.button(GamepadEx.Button.LEFT_BUMPER).whileTrue(intake.eject())
driver.button(GamepadEx.Button.X).toggleOnTrue(lift.raise())
driver.trigger { driver.rightTrigger > 0.5 }.whileTrue(drive.slowMode())
(driver.button(GamepadEx.Button.START) and driver.button(GamepadEx.Button.A))
    .onTrue(resetHeading())
```

- `button(...)` is cached per button; `trigger { }` and `and`/`or`/`!`
  each make a fresh one. All are polled in `GamepadEx.update()`, which
  `OpModeBase` already calls every tick before the scheduler runs. Each base
  condition is sampled once; composed triggers reuse those samples.
- `onTrue` / `whileTrue` skip scheduling if the command is already
  scheduled; `whileTrue`'s falling-edge cancel is safe on a command that
  already finished.
- A throwing condition or binding quarantines only that trigger and later
  triggers still poll. Quarantining an active `whileTrue` trigger cancels
  its command so a dead sensor cannot strand an actuator action.
- The raw `driver.aPressed` / `driver.a` flags still exist — use them for
  continuous reads (drive sticks), use triggers for command scheduling.

## Lifecycle rules (enforced by OpModeBase)

1. **Bulk reads are MANUAL.** Every Lynx module is in
   `BulkCachingMode.MANUAL` at init. The main loop clears caches once
   per tick at the top. Do **not** read motor/encoder values outside of
   the main tick window; if you must (e.g. from an I2CBusThread), use
   the raw Lynx APIs and accept stale data.

2. **`periodic()` reads. Commands write. `writeHardware()` flushes.**
   A subsystem's `periodic` must not set motor power — if you feel the
   urge, write a command instead. The whole point of the scheduler's
   requirements system is to prevent two chunks of code from commanding
   the same hardware simultaneously.

3. **Don't call `Follower.update()` from anywhere other than
   `MecanumDriveSubsystem.writeHardware()`.** Calling it twice per tick
   will double the PID step and gives you weird oscillation.

4. **Teleop mode starts from the default command.** TeleOp op-modes set
   `drive.defaultCommand = drive.teleopCommand { ... }` in `configure()`.
   Do not call `drive.drive(...)` from `onLoop()`; command requirements
   are what let teleop resume cleanly after a path or driver action.

5. **Bindings are init-only.** Wire `GamepadEx` triggers in `configure()`.
   The init loop updates gamepad edges with trigger polling disabled, and
   `OpModeBase` locks bindings at start so per-loop binding creation throws.

6. **Register the drive before the localizer.** `LocalizerSubsystem`
   samples its pose history in `writeHardware()`, which must run right
   after the drive's `writeHardware()` (where `Follower.update()` measures
   the pose). Registering them the other way round re-introduces a
   one-loop-period timestamp skew into every vision correction.

7. **Teleop contains command faults; auton does not.** `TeleOpBase` sets
   `Robot.containCommandFaults = true`: a lifecycle exception ends only the
   faulting command, calls `onCommandFault()` on its required subsystems, and
   lets freed defaults resume next tick. Auton rethrows command faults.
   Trigger condition/binding faults are separately quarantined per trigger in
   every mode. `periodic()`, `writeHardware()`, and `onLoop()` always fail
   fast.

## Config persistence (ConfigStore) + Sloth hot reload + Panels

Panels live-tuning writes into `@Configurable` statics, which die with the
process (and Sloth hot reloads re-run static initialisers). The framework
answer is **`ConfigStore`** (`core/runtime/`): registered config objects
are persisted to `/sdcard/FIRST/config/tuning.properties` (~1 Hz when
dirty, atomically) and reloaded into the statics at every op-mode init.
Tuned values therefore survive power cycles, full installs, *and* hot
reloads — and config objects no longer need `@Pinned`, so their code
hot-reloads normally.

`OpModeBase` registers `DriveConfig` and `LocalizerConfig` itself. Season
forks register their own objects in `configure()`:

```kotlin
ConfigStore.register("lift", LiftConfig)
```

Only public `@JvmField` mutable primitive/String fields are persisted,
keyed `<section>.<field>`. Delete the file to fall back to compiled
defaults. Bump `RobotConfig.CONFIG_SCHEMA` whenever the tuned values stop
applying — a new season fork, and the sensorbot → competition-robot swap,
since the Control Hub usually moves between chassis and carries its tuning
file with it. Files recorded under a different schema are ignored. Tunables on `@Configurable` *op-modes* (the Pedro `Tuning`
op-mode, `LocalizationTestTeleOp`) are deliberately not persisted — move
anything that must persist into a registered config object.

The only `@Pinned` class left is `PersistedPose` (it carries live state
across op-modes in the same process, which is exactly what pinning is
for). Don't pin config objects.

## Things AI assistants get wrong often

- **The scheduler is not global.** It lives on `Robot`
  (`robot.scheduler`); there is no static `Scheduler` and no Ivy dependency
  anymore. The tick method is `execute()`, never `run()`.
- **`Commands.waitMs` overloads.** Double and Long overloads both exist —
  keep both. Pass `robot.clock` as the second argument anywhere a routine
  might run in the sim (PedroAutoRunner's `wait()` already does).
- **Pedro maven repository.** Pedro is on **Maven Central**, not
  `maven.pedropathing.com`. That domain returns a 302 to a 404 and
  breaks the build. Do not "fix" `build.dependencies.gradle` to use it.
- **Pedro calibration values are placeholders.** The numeric values in
  `pedroPathing/Constants.java` must be measured for the physical chassis
  before autonomous paths run; follow `OPERATIONS.md` in order.
- **Pose coordinates for mirrored paths.** Mirroring runs through
  `Alliance.mirror` / `Pose2d.mirror` with the field length from
  `RobotConfig.Field.LENGTH_INCHES` (**141.5 inches**, not 144) and the
  season's `FieldSymmetry` (MIRROR vs ROTATE — check the game manual).
  Don't change either unless the user explicitly asks.
- **Kotlin property access on Java getters.** `follower.pose` works
  (maps to `getPose()`) but `follower.startingPose = p` does **not** (no
  `getStartingPose()` exists). Use `follower.setStartingPose(p)`.
- **Subsystem writes in `periodic`.** Don't. That's what commands +
  `writeHardware` are for.

## When the user asks you to add a subsystem

(`DEVELOPMENT.md` has the full worked example with the same contract.)

For a single profiled motor (lift, arm, turret), extend or instantiate
`ProfiledMotorSubsystem` instead of hand-rolling the control loop — it
already implements the lifecycle below plus profile + PIDF + hold-at-goal.
For everything else:

1. Extend `SubsystemBase(name = "…")`.
2. Resolve hardware in `init(hardwareMap)` using `DeviceReaders.motor`
   etc. so a missing device surfaces as `HardwareConfigError` with the
   name baked in.
3. Read in `periodic()`, write targets in `writeHardware()`.
4. Expose a clean API. Commands that touch this subsystem declare
   `requiring(this@MySubsystem)`.
5. Register it on the `Robot` from the op-mode's `configure()` hook —
   not from anywhere else.

## When the user asks you to add an I²C sensor (or touch SRSHub / I2CBusThread)

**Read the sensor section in `DEVELOPMENT.md` first.** Pinpoint stays direct;
all other I²C goes on one SRSHub read **inline** in `periodic()`. Do not
background the SRSHub by default: it shares the Control Hub's Lynx serial link
with motor writes, the same trap that was tried and reverted for Pinpoint
(postmortem in `pedroPathing/Constants.java`). The development guide contains
the hazards, integration rules, and measurement sequence for any SRSHub
threading decision.

## When the user asks you to add a path or auton routine

Prefer the `PathDSL` / `PedroAutoRunner` DSLs in `core/pathing/`. They
already handle the Pedro API correctly and add alliance mirroring and
parallel/race groups on top. Don't drop back to raw `PathBuilder` unless
you have a reason — and if you do, remember `Alliance.mirror(heading)`
for the heading-interpolation arguments; pose mirroring alone doesn't
cover them.

There is no init-loop auton menu. Every alliance/routine combination is its
own `@Autonomous` class, so the Driver Station dropdown is the selector —
the chosen name is displayed in large text and there is nothing to confirm.
A BLUE variant is a copy of the RED op-mode that overrides `initialAlliance`
and changes nothing else. Only the start delay is chosen at init, via
`StartDelay` on dpad left/right, because it is picked minutes before the
match to avoid an alliance partner's routine.

`ExampleAuto` is the copyable skeleton: RED-coordinate poses mirrored by the
DSL, routine built at start, sequenced with `autoRoutine`, final pose
persisted automatically for teleop to restore. Its `onStart` aborts before
scheduling when the localizer already faulted and treats a rejected schedule
as an auton abort; preserve both gates in season copies.

Mid-path actions use progress markers instead of parallel/waitUntil
contortions:

```kotlin
follow(toScore) {
    at(0.3) { lift.setGoal(HIGH) }          // fires once at 30% of the chain
    at(0.85, "deploy") { intake.deploy() }  // label shows in the flight log
}
```

Markers ride `drive.pathProgress()` (0..1 across the whole chain), fire
through completion, and are dropped if the path never reaches them
(deadline semantics). They work in the sim — `SimFollower` emulates
progress.

## Things not to do unless explicitly asked

- Don't rename hardware-map strings (`frontLeftMotor`, `pinpoint`,
  etc.). Those match the user's actual robot config.
- Don't move files between `java/` and `kotlin/` source roots.
- Don't bump FTC SDK, Pedro, Kotlin, or AGP versions.
- Don't remove or rewrite `pedroPathing/Constants.java` — Pedro reads
  from that exact package path.
- Don't add Kalman filters, PID overhauls, or "cleaner architecture"
  refactors.
- Don't invent game-specific mechanisms nobody asked for. Season subsystems
  belong in this repo now, but build the one requested — not a speculative
  intake/shooter/lift set alongside it.
- Don't put season code in `core/`. That directory is what gets cherry-picked
  back to `ftc-starter`; game logic in it makes the merge manual forever.

## Workflow

Code is written in this repo, then deployed to the robot one
of two ways:

- **Full install** — Android Studio's normal Run / install (`installDebug`).
  A full APK build + install. Use this for the first deploy of a session,
  and after changing anything Sloth can't hot-reload: `@Pinned` classes
  (`PersistedPose`), non-teamcode code, dependencies, or the manifest.
- **Hot reload** — `./gradlew deploySloth` (or a Gradle run configuration
  in Android Studio pointed at it). Pushes only teamcode, ~1s. Use this for
  ordinary iteration on subsystems, op-modes, and command logic.

The Load plugin (`dev.frozenmilk.sinister.sloth.load`, applied in
`TeamCode/build.gradle`) auto-wires `removeSlothRemote` into `installDebug`/
`installRelease`, so a normal Android Studio install always clears any
staged hot-reload jar first — the two paths don't fight each other.

Because hot reload is live, `@Pinned` and ConfigStore matter — see the
**Config persistence** section above.

Logs live on the robot at `/sdcard/FIRST/logs`. Use `make pull-logs` and
open `.wpilog` files in AdvantageScope, or `make analyze` for a quick
post-match summary (loop percentiles, phase maxima, battery, follower
error, events). `OPERATIONS.md` maps competition symptoms to log channels.
Continuous channels are capped at 100 Hz independently of the control-loop
rate; events and command transitions remain immediate, and per-window timing
maxima preserve spikes between samples.

### Post-match debugging (the AI runs this)

When the user reports a match problem and is connected to the hub ("I'm
plugged in", "figure out what went wrong this match", "the lift didn't lift
this match") — run **`make debug`**. It reaches the hub over USB (wifi
fallback), pulls only the newest match's log(s) — Auto + TeleOp, not all 30 —
and prints a JSON diagnostic bundle (loop timing, battery, follower error,
drive-mode time, full command-set transitions, full event/fault timeline,
pose-correction tally, and a channel manifest). Anchor on the stated symptom,
then drill into specific channels with:

```
python3 tools/analyze_wpilog.py --json --channel <name,name> robot-logs/<file>
```

The auton run is the second pulled file; `make debug`'s JSON defaults to the
newest (TeleOp), so point at `robot-logs/Auto-*.wpilog` for auton symptoms.
Don't fall back to `make pull-logs` here — it drags all 30 logs over the link.

When a log doesn't uniquely determine the cause, don't assert one. Give the
ranked hypotheses with what each predicts in the data, and name the one
channel to pull or the one reproduction that decides it — then ask for it.

Auton routines can run headless before touching the robot: see
`core/sim/SimAutonRoutineTest` (test sources) — `SimHarness` + `SimFollower`
execute full `PedroAutoRunner` routines against real path geometry in
JUnit — including `wait()` steps and `timeout()`s, which run on the
harness's virtual clock.

## Running the project

Run the host unit tests and Android debug assemble with JDK 17:

```
JAVA_HOME="/Users/maximilianreich/Library/Java/JavaVirtualMachines/corretto-17.0.13/Contents/Home" \
  ./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```
