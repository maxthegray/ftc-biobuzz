# CLAUDE.md

Guidance for AI-assisted work in this repository. Read this first — it will
save you (and your user) both time and embarrassment.

## What this repo is

A Kotlin FTC starter template built on top of an unmodified clone of
`FIRST-Tech-Challenge/FtcRobotController` 11.1.0. The user forks it every
season. Season-specific code does **not** live here — keep this repo
clean so the next fork inherits a stable base.

Stack (exact versions; these are load-bearing):

| Library               | Coordinate                                | Version |
|-----------------------|-------------------------------------------|---------|
| FTC SDK               | `org.firstinspires.ftc:*`                 | 11.1.0  |
| Kotlin Android plugin | `org.jetbrains.kotlin:kotlin-gradle-plugin` | 2.0.21  |
| Pedro Pathing core    | `com.pedropathing:core`                   | 2.1.1   |
| Pedro Pathing FTC     | `com.pedropathing:ftc`                    | 2.1.1   |
| Pedro Ivy             | `com.pedropathing:ivy`                    | 1.0.0   |
| Pedro Telemetry       | `com.pedropathing:telemetry`              | 1.0.0   |
| Bylazar fullpanels    | `com.bylazar:fullpanels`                  | 1.0.12  |
| Sloth (Sinister)      | `dev.frozenmilk.sinister:Sloth`           | 0.2.4   |

Maven repositories (also load-bearing):
- `mavenCentral()` — Pedro + Ivy
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

These are the Ivy / Pedro calls used throughout the codebase. They are
stable as of the versions above and wrong in at least one piece of
publicly-searchable documentation.

### Ivy command scheduler

```kotlin
// Ticking the scheduler (called once per loop from Robot.loop()):
Scheduler.execute()      // NOT Scheduler.run() — that name does not exist
Scheduler.reset()        // clear all running / queued / suspended commands
Scheduler.schedule(cmd)  // schedule a command
Scheduler.cancel(cmd)    // cancel a command in any state
Scheduler.isScheduled(cmd)
Scheduler.isRunning(cmd)
```

### Building a command

```kotlin
Command.build()
    .setStart { ... }            // called once when scheduled
    .setExecute { ... }          // called every tick while running
    .setDone { returnBoolean }   // called every tick; true → end
    .setEnd { endCondition -> ... }
    .requiring(driveSubsystem)   // Object... varargs
    .setPriority(10)             // higher priority interrupts lower
```

Or use the helpers:

```kotlin
Commands.instant { action() }          // runs once and completes
Commands.waitMs(500.0)                 // takes Double, NOT Long
Commands.waitUntil { condition }
Commands.infinite { action() }
```

And composition via `Groups`:

```kotlin
Groups.sequential(a, b, c)
Groups.parallel(a, b, c)
Groups.race(a, b, c)
Groups.deadline(deadlineCmd, a, b)
```

Framework priority convention:

```kotlin
CommandPriorities.DEFAULT       // 0: subsystem default commands
CommandPriorities.DRIVER_ACTION // 10: driver-triggered actions
CommandPriorities.AUTON_ROUTINE // 10: autonomous routines
```

### Pedro Follower

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

Useful runtime hooks:

```kotlin
localizer.restorePersistedPose()                 // auton → teleop pose handoff
localizer.applyCorrection(measured, timestampNs) // latency-compensated vision seam
val selector = AutonSelector(robot, telemetryBag)
robot.recordEvent("marker")                      // also writes to WPILOG
```

### Panels telemetry

```kotlin
val tm = PanelsTelemetry.telemetry   // TelemetryManager
tm.addLine("hello")
tm.addData("key", value)
tm.update()                                    // flush to dashboard
tm.wrapper                                     // an FTC Telemetry facade
JoinedTelemetry(telemetry, tm.wrapper)         // wrap both DS + Panels as one Telemetry
```

### Gamepad triggers

`GamepadEx` binds buttons (and any boolean condition) to Ivy commands.
Wire bindings once in `configure()` — don't poll `*Pressed` flags in
`onLoop()` and call `Scheduler.schedule` by hand.

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
  `OpModeBase` already calls every tick before the scheduler runs.
- `onTrue` / `whileTrue` skip scheduling if the command is already
  scheduled; `whileTrue`'s falling-edge cancel is safe on a command that
  already finished.
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
   urge, write a command instead. The whole point of Ivy's requirements
   system is to prevent two chunks of code from commanding the same
   hardware simultaneously.

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

## Sloth hot reload + Panels: pin `@Configurable` classes

Sloth only hot-reloads classes under `org.firstinspires.ftc.teamcode`,
and each reload re-runs their static initialisers. Panels live-tuning
writes into `@Configurable` statics — so a reload silently resets every
tuned value back to its compiled-in default.

The fix: annotate any `@Configurable` class whose tuned values must
survive a reload with `@dev.frozenmilk.sinister.loading.Pinned`. A pinned
class is loaded exactly once into Sloth's root classloader and never
re-initialised. `DriveConfig` is already pinned for this reason.

Trade-off: edits to a pinned class's *code* are not hot-reloaded — you
need a full install for them to take effect. That's fine for config
objects (they change rarely), and it's why `@Configurable` *op-modes*
(the Pedro `Tuning` op-mode, `LocalizationTestTeleOp`) are deliberately
left unpinned — you want their logic hot-reloadable. Their tunables just
don't persist across a reload of that op-mode; move anything that must
persist into a pinned config object.

## Things Claude gets wrong often

- **Ivy tick method name.** It is `Scheduler.execute()`, never
  `Scheduler.run()`. If you see `.run()` in a message about this codebase,
  correct it.
- **`Commands.waitMs` parameter type.** It is `double`, not `long`. In
  Kotlin we provide both overloads — keep both.
- **Pedro maven repository.** Pedro is on **Maven Central**, not
  `maven.pedropathing.com`. That domain returns a 302 to a 404 and
  breaks the build. Do not "fix" `build.dependencies.gradle` to use it.
- **Pose coordinates for mirrored paths.** Pedro's `Pose.mirror()` takes
  an optional field length that defaults to **141.5 inches**, not 144.
  Don't change that unless the user explicitly asks.
- **Kotlin property access on Java getters.** `follower.pose` works
  (maps to `getPose()`) but `follower.startingPose = p` does **not** (no
  `getStartingPose()` exists). Use `follower.setStartingPose(p)`.
- **Subsystem writes in `periodic`.** Don't. That's what commands +
  `writeHardware` are for.

## When the user asks you to add a subsystem

1. Extend `SubsystemBase(name = "…")`.
2. Resolve hardware in `init(hardwareMap)` using `DeviceReaders.motor`
   etc. so a missing device surfaces as `HardwareConfigError` with the
   name baked in.
3. Read in `periodic()`, write targets in `writeHardware()`.
4. Expose a clean API. Commands that touch this subsystem declare
   `requiring(this@MySubsystem)`.
5. Register it on the `Robot` from the op-mode's `configure()` hook —
   not from anywhere else.

## When the user asks you to add a path or auton routine

Prefer the `PathDSL` / `PedroAutoRunner` DSLs in `core/pathing/`. They
already handle the Pedro API correctly and add alliance mirroring and
parallel/race groups on top. Don't drop back to raw `PathBuilder` unless
you have a reason.

## Things not to do unless explicitly asked

- Don't rename hardware-map strings (`frontLeftMotor`, `pinpoint`,
  etc.). Those match the user's actual robot config.
- Don't move files between `java/` and `kotlin/` source roots.
- Don't bump FTC SDK, Pedro, Kotlin, or AGP versions.
- Don't remove or rewrite `pedroPathing/Constants.java` — Pedro reads
  from that exact package path.
- Don't add Kalman filters, PID overhauls, or "cleaner architecture"
  refactors.
- Don't generate example game-specific code (intake, shooter, lift). The
  user will write those when the season starts.

## Workflow

Code is written in this repo (with Claude), then deployed to the robot one
of two ways:

- **Full install** — Android Studio's normal Run / install (`installDebug`).
  A full APK build + install. Use this for the first deploy of a session,
  and after changing anything Sloth can't hot-reload: `@Pinned` classes
  (`DriveConfig`), non-teamcode code, dependencies, or the manifest.
- **Hot reload** — `./gradlew deploySloth` (or a Gradle run configuration
  in Android Studio pointed at it). Pushes only teamcode, ~1s. Use this for
  ordinary iteration on subsystems, op-modes, and command logic.

The Load plugin (`dev.frozenmilk.sinister.sloth.load`, applied in
`TeamCode/build.gradle`) auto-wires `removeSlothRemote` into `installDebug`/
`installRelease`, so a normal Android Studio install always clears any
staged hot-reload jar first — the two paths don't fight each other.

Because hot reload is live, `@Pinned` matters — see the **Sloth hot reload
+ Panels** section above.

Logs live on the robot at `/sdcard/FIRST/logs`. Use `make pull-logs` and
open `.wpilog` files in AdvantageScope.

## Running the project

Run the host unit tests and Android debug assemble with JDK 17:

```
JAVA_HOME="/Users/maximilianreich/Library/Java/JavaVirtualMachines/corretto-17.0.13/Contents/Home" \
  ./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```
