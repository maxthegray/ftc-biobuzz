# Architecture

This doc explains the shape and lifecycle of the starter. It's for humans
making judgment calls — "should I do this in `periodic()` or a command?" —
not for introducing new teammates to FTC programming from scratch.

## The one-tick model

A single op-mode tick always looks like this:

```
┌───────────────────────────────────────────────────────────────────┐
│  OpModeBase.runOpMode() main loop                                 │
│                                                                   │
│   Robot.loop():                                                   │
│     1. BulkReadManager.clearCaches()       ← fresh Lynx data      │
│     2. for s in subsystems: s.periodic()   ← reads + state        │
│     3. driver/operator update              ← edge-detect gamepads │
│     4. onLoop() / control                  ← op-mode decisions    │
│     5. default commands                    ← idle subsystem work  │
│     6. Scheduler.execute()                 ← Ivy ticks commands   │
│     7. for s in subsystems: s.writeHardware() ← motors, servos    │
│     8. telemetry + flight recorder         ← DS, Panels, WPILOG   │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
```

The order matters:

1. **Clear caches first.** Lynx hubs are in `MANUAL` bulk-read mode
   (`BulkReadManager`). The first motor/encoder getter after
   `clearBulkCache()` triggers exactly one bulk read that fills the cache;
   every subsequent getter this tick is free. If you read hardware before
   `clearCaches`, you see stale data from the previous tick.

2. **Subsystems read in `periodic()`**, but do not command hardware.
   `periodic()` is for "update my internal state from the latest bulk read
   + localiser". Leave actuator decisions to commands.

3. **Input precedes control.** `GamepadEx` trigger bindings are sampled
   after fresh subsystem reads, then `onLoop()` runs before the scheduler.
   Commands scheduled here tick in the same loop.
   Do not write raw hardware from here; leave final actuator writes to
   subsystem `writeHardware()`.

4. **Commands run via the Ivy scheduler.** Commands that need hardware
   declare `requiring(subsystem)`. The scheduler resolves conflicts (one
   command per requirement, with priority / override / queue / cancel
   behaviour configurable per-command). This is how default-commands and
   driver-initiated actions coexist without stepping on each other.

   Priority convention:
   - Subsystem defaults: `0`
   - Driver-triggered actions: `10`
   - Autonomous routines: `10`

5. **Subsystems write in `writeHardware()`.** By the time this runs, a
   command has decided what the subsystem should be doing this tick.
   `MecanumDriveSubsystem.writeHardware()` calls `Follower.update()`,
   which is where Pedro actually computes and writes motor powers.

`Robot.initTick()` is the init-loop subset: clear caches and run subsystem
`periodic()` only. Gamepad edges work in init, but trigger bindings are not
polled until start and bindings are locked once the op-mode starts.

Before `configure()`, `OpModeBase` runs `Preflight.check()` against the
op-mode's `requiredDevices`, so missing drive motors or the `pinpoint`
device fail before Pedro tries to construct a follower.

## Threading model

One main thread (the FTC `LinearOpMode.runOpMode()` thread) owns every
piece of hardware except I²C-bound sensors that are unhappy blocking the
main loop. For those we spin up a dedicated bus thread via `I2CBusThread`:

- The thread polls exactly one device at a fixed rate (e.g. Pinpoint at
  200 Hz).
- Published values go through a single volatile slot (`I2CBusThread.Ref`)
  — writer is the bus thread, readers are the main loop. No locks.
- Transient I²C errors are swallowed: the thread skips publishing that
  round so the main loop keeps using the last good value.

We do **not** touch motors or servos from bus threads. Those live on the
main thread only.

## Why the Pedro Follower *is* the drivetrain

`MecanumDriveSubsystem` does not own motors or compute powers. Pedro's
`Follower`, constructed by `pedroPathing.Constants.createFollower`, does
all of that internally — it holds the Localizer (Pinpoint), the
`MecanumDrivetrain`, the PIDs for X / Y / heading, and the path
following state machine. The subsystem exists to:

1. Slot `Follower.update()` into the main loop at the right moment
   (after the scheduler, before telemetry).
2. Translate "driver feel" knobs from `DriveConfig` into `setTeleOpDrive`
   arguments.
3. Give commands a single Object to declare as a requirement so conflicts
   are caught by Ivy's scheduler instead of as crossed motor commands.

The trade-off is that we can't easily swap out Pedro for another follower
without rewriting this subsystem. That's acceptable — Pedro is specifically
why this starter exists.

## Localisation Hooks

The Follower owns the primary localiser (Pinpoint), so wheel-odometry
drift accumulates over long paths. The starter exposes three hooks:

1. `LocalizerSubsystem` is a small subsystem facade over the Follower pose and
   velocity. It records a fixed-size pose history and exposes
   `applyCorrection(measured, timestampNanos)` for latency-compensated
   AprilTag or vision measurements.
2. `PinpointDirect` reaches down to the raw Pinpoint driver for
   init-time IMU recalibration and status reads.
3. `PersistedPose` stores the last live drive pose across op-mode handoff.
   Teleop op-modes opt in with `localizer.restorePersistedPose()`.

Vision pipelines are intentionally not included in this template. If a
season fork needs cameras, add that subsystem in the fork and feed accepted
field-pose corrections through `LocalizerSubsystem.applyCorrection`.

## Telemetry: two audiences, one call

FTC teams need telemetry on two screens during the match:
- The Driver Station phone for the drivers (read-only, terse).
- The Panels dashboard laptop for the coach / pit (dense, live tuning).

`TelemetryBag` owns a structured buffer per tick and fans out to both
when flushed. Op-modes add data once:

```kotlin
telemetryBag.section("Drive") {
    put("pose", drive.pose)
    put("loopHz", robot.loopHz, decimals = 1)
}
```

`OpModeBase` calls `telemetryBag.flush()` at the bottom of every tick.
Don't hand-write to `telemetry` or to `PanelsTelemetry` directly — you'd
just be duplicating lines.

## Flight Recording

`Robot` owns a WPILOG flight recorder enabled by `OpModeBase` for every
op-mode. It records pose, velocity, drive mode, gamepad axes/buttons, loop
phase timings, battery voltage, running command names, and events. Any I/O
failure disables the recorder for that run; it never throws into the loop.

Loop crashes write a final event, close the log, and save
`/sdcard/FIRST/logs/lastcrash.txt`. The next init displays the first line
and deletes the file.

## Hot reload, Sinister, and Sloth

`Sloth` (via the Sinister runtime) is a zero-config hot-reload library
from the Dairy Foundation. Pulling in `dev.frozenmilk.sinister:Sloth` is
enough: Sinister scans the APK at boot and wires up anything annotated
or registered for hot-reloading. We don't use it for any explicit feature
here — it's just present so you can pull it in when you start tuning live.

## What doesn't live in this codebase

- Game-specific logic: scoring, intake state machines, shooter RPM tables.
  Those go in their own subsystems / commands when you fork this for a
  season.
- Pedro path files. Teams usually keep these in a `paths/` subfolder with
  one Kotlin file per routine. Create it when you have a real auton.
- Vision pipelines, camera correction, and pose-fusion filters. The starter
  exposes localisation hooks, but the concrete vision stack belongs in a
  season fork.
- Auto-generated Panels controls. Panels supports `@Configurable` fields;
  wire them up on the specific subsystems you want to tune, don't
  pre-annotate placeholders.
