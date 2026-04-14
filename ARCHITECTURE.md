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
│   driver.update(), operator.update()       ← edge-detect gamepads │
│                                                                   │
│   Robot.loop():                                                   │
│     1. BulkReadManager.clearCaches()       ← fresh Lynx data      │
│     2. for s in subsystems: s.periodic()   ← reads + state        │
│     3. Scheduler.execute()                 ← Ivy ticks commands   │
│     4. for s in subsystems: s.writeHardware() ← motors, servos    │
│                                                                   │
│   onLoop()                                 ← gamepad-driven work  │
│   telemetryBag.flush()                     ← DS + Panels in 1 go  │
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

3. **Commands run via the Ivy scheduler.** Commands that need hardware
   declare `requiring(subsystem)`. The scheduler resolves conflicts (one
   command per requirement, with priority / override / queue / cancel
   behaviour configurable per-command). This is how default-commands and
   driver-initiated actions coexist without stepping on each other.

4. **Subsystems write in `writeHardware()`.** By the time this runs, a
   command has decided what the subsystem should be doing this tick.
   `MecanumDriveSubsystem.writeHardware()` calls `Follower.update()`,
   which is where Pedro actually computes and writes motor powers.

5. **`onLoop()` runs after everything.** Use it for gamepad handling that
   can't be expressed as a default command, and for telemetry gathering.
   Do not command motors from here unless you are absolutely sure no
   command is contending for them.

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

## Localisation fusion

The Follower owns the primary localiser (Pinpoint), so wheel-odometry
drift accumulates at a few percent per path. For routines where that
matters — late-auton, long paths — we layer AprilTag correction on top:

1. `VisionSubsystem` runs the FTC `VisionPortal` + `AprilTagProcessor`.
2. On every tag frame, `AprilTagPipeline.detectionToFieldPose` projects
   the detection through the camera's known mount offset into a field
   pose.
3. That field pose feeds `AprilTagCorrector.submit`, which gates it on
   range and frame-over-frame stability before committing via
   `Localizer.setPose`.

Everything is a hard snap — no Kalman filter. Jitter rejection comes from
requiring two successive agreeing detections of the same tag ID before
committing. If you want something smoother, swap `AprilTagCorrector` for
an implementation that blends instead of snaps.

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
- A Kalman filter for pose fusion. Intentionally left as a hook point —
  the current AprilTag corrector is deliberately simple.
- Auto-generated Panels controls. Panels supports `@Configurable` fields;
  wire them up on the specific subsystems you want to tune, don't
  pre-annotate placeholders.
