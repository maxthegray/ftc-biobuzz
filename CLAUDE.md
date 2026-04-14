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

### Pedro Follower

```kotlin
follower.update()                        // MUST run every tick (in writeHardware)
follower.pose                            // Pose getter
follower.velocity                        // Pose getter
follower.setPose(p)                      // hard snap — useful for AprilTag correction
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

### Panels telemetry

```kotlin
val tm = PanelsTelemetry.telemetry   // TelemetryManager
tm.addLine("hello")
tm.addData("key", value)
tm.update()                                    // flush to dashboard
tm.wrapper                                     // an FTC Telemetry facade
JoinedTelemetry(telemetry, tm.wrapper)         // wrap both DS + Panels as one Telemetry
```

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

4. **Start in the right follower mode.** TeleOp op-modes must call
   `drive.enableTeleop()` from `onStart()`. Auton op-modes must NOT —
   the default post-init state is passive and ready for `followPath`.

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

Prefer the `PathDSL` / `PedroAutoRunner` DSLs in `starter/pathing/`. They
already handle the Pedro API correctly and add alliance mirroring and
parallel/race groups on top. Don't drop back to raw `PathBuilder` unless
you have a reason.

## Things not to do unless explicitly asked

- Don't rename hardware-map strings (`frontLeftMotor`, `sensor_otos`,
  etc.). Those match the user's actual robot config.
- Don't move files between `java/` and `kotlin/` source roots.
- Don't bump FTC SDK, Pedro, Kotlin, or AGP versions.
- Don't remove or rewrite `pedroPathing/Constants.java` — Pedro reads
  from that exact package path.
- Don't add Kalman filters, PID overhauls, or "cleaner architecture"
  refactors.
- Don't generate example game-specific code (intake, shooter, lift). The
  user will write those when the season starts.

## Running the project

There is no bundled test suite — FTC code runs on the robot and is
effectively tested by deploying to hardware. For type-checking, import
into Android Studio and let Gradle sync. You can also run:

```
./gradlew :TeamCode:assembleDebug
```

which will compile everything (Java + Kotlin) and surface errors. This
is the closest thing to "CI" this codebase has.
