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

   Priority ladder (gaps left for season-fork levels):
   - Subsystem defaults: `0`
   - Autonomous routines / teleop auto-assists: `10`
   - Driver-triggered actions: `20` (a driver input always beats an assist)
   - Driver override / panic bindings: `30`

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

**Why measure and act stay fused in `Follower.update()`** (investigated
against the Pedro 2.1.1 sources, considered, rejected): `Follower` does
expose `updatePose()` separately, so commands *could* see a same-tick pose
by measuring in `periodic()` and acting in `writeHardware()`. But
`PoseTracker.update()` calls `localizer.update()`, which for the direct
Pinpoint does the blocking I²C read — splitting would double that read
(~2 ms/tick, the same link contention that killed the background-thread
experiment documented in `pedroPathing/Constants.java`) — and `PoseTracker`
derives velocity from `System.nanoTime()` deltas between updates, so the
second update each tick would compute velocity over a near-zero dt and
destabilise Pedro's drive vector and predictive braking. Conclusion:
commands act on a pose that is one loop period old, which is bounded and
harmless; the pose-history timestamp fix in `LocalizerSubsystem` already
removes the only place that staleness was quantitatively wrong.

## Localisation Hooks

The Follower owns the primary localiser (Pinpoint), so wheel-odometry
drift accumulates over long paths. The starter exposes three hooks:

1. `LocalizerSubsystem` is a small subsystem facade over the Follower pose and
   velocity. It records a fixed-size pose history and exposes
   `applyCorrection(measured, timestampNanos)` for latency-compensated
   AprilTag or vision measurements. Corrections are **gated** (rejected as
   outliers past `LocalizerConfig.maxCorrection*`) and **blended**
   (`LocalizerConfig.correctionBlend` fraction applied per call — 1.0 snaps,
   0.5 behaves like a complementary filter for streaming sources). Every
   accept/reject is recorded as a flight-log event so logs show *why* a
   correction didn't take.

   **Timing contract:** the history is sampled in the localizer's
   `writeHardware()`, immediately after the drive subsystem's
   `writeHardware()` runs `Follower.update()` — so each sample's timestamp
   matches when the pose was actually measured. This is why the drive must
   be registered *before* the localizer; sampling in `periodic()` would
   timestamp the previous tick's pose with this tick's clock and skew every
   correction by one loop period.
2. `PinpointDirect` reaches down to the raw Pinpoint driver for
   init-time IMU recalibration and status reads.
3. `PersistedPose` stores the last live drive pose across op-mode handoff,
   mirrored to `/sdcard/FIRST/persisted-pose.txt` so the auton → teleop
   handoff survives a Robot Controller process restart (DS "Restart Robot",
   a crash, a brownout). Teleop op-modes opt in with
   `localizer.restorePersistedPose()`, which falls back to the disk copy and
   still age-gates whatever it finds.

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

`OpModeBase` also drives a `FieldView`: every real op-mode draws the robot
pose (and the active path while FOLLOWING) on the Panels field view,
throttled to ~10 Hz and self-disabling on repeated failures. Override
`publishFieldView` to false to opt out.

## Flight Recording

`Robot` owns a WPILOG flight recorder enabled by `OpModeBase` for every
op-mode. It records pose, velocity, drive mode, follower error terms while
following/holding (`follow/translationalErrorIn`, `follow/headingErrorRad`
— the difference between "the path was bad" and "the heading PID
oscillated"), gamepad axes/buttons, loop phase timings, battery voltage,
running command names, and events. `PedroAutoRunner` emits a labelled
event per auton step when given `robot::recordEvent`, and
`LocalizerSubsystem` logs every correction accept/reject — so each log
carries a navigable match timeline. Any I/O failure disables the recorder
for that run; it never throws into the loop.

Loop crashes write a final event, close the log, and save
`/sdcard/FIRST/logs/lastcrash.txt` — including the running commands, the
last ~20 recorded events, loop count, and match time, so the file answers
"what was happening", not just "where it died". The next init displays the
first line and deletes the file.

`make analyze` (`tools/analyze_wpilog.py`) prints a post-match one-pager
from the newest log: loop-rate percentiles, per-phase maxima, battery sag,
follower error stats, contained faults, pose-correction accept/reject
counts, and the event timeline. `RUNBOOK.md` maps match-day symptoms to
these channels.

## Headless simulation

`core/sim/` (test sources) runs entire auton routines in JUnit:
`SimFollower` honours the exact `Follower` surface that Ivy's Pedro
commands and this framework call, but replaces Pedro's control law with
kinematic motion — real `PathDSL` geometry and alliance mirroring, real Ivy
scheduling, real subsystem lifecycle, virtual time via `Clock`.
`SimHarness` wires it to a real `Robot`; `SimAutonRoutineTest` shows the
pattern, including the RED/BLUE mirror-symmetry test and the
auton→teleop pose-handoff test. Limitations: it validates routine *logic*,
not Pedro's control quality, and Ivy's `Commands.waitMs` runs on the wall
clock — keep `wait()` steps out of simulated routines.

## Hot reload, Sinister, and Sloth

`Sloth` (via the Sinister runtime) is a zero-config hot-reload library
from the Dairy Foundation. Pulling in `dev.frozenmilk.sinister:Sloth` is
enough: Sinister scans the APK at boot and wires up anything annotated
or registered for hot-reloading. We don't use it for any explicit feature
here — it's just present so you can pull it in when you start tuning live.

## Error philosophy: fail fast, except the teleop command layer

Init failures throw — a missing device surfaces as `HardwareConfigError`
with the name baked in, the op-mode aborts loudly, and you fix the
config. Subsystem `periodic()`/`writeHardware()` and the op-mode's
`onLoop()` also throw: an exception there kills the op-mode mid-match
(`OpModeBase` records the crash to the flight recorder and
`lastcrash.txt` first, then rethrows).

That is deliberate. Swallowing exceptions to "survive the match" means
driving on silently-wrong state — a drivetrain that thinks it's at the
wrong pose is more dangerous than a dead one. If a specific sensor is
known-flaky, handle it explicitly at the read site (stale-data flag,
last-good-value, telemetry warning) rather than via a blanket rescue.

**The one scoped exception is the teleop command layer.** With
`Robot.containCommandFaults` on (which `TeleOpBase` enables), an exception
thrown from trigger polling, default scheduling, or `Scheduler.execute()`
is contained: the scheduler is cleared, every subsystem gets
`onCommandFault()` to safe its actuators (the drive breaks any follow and
zeros; a profiled mechanism freezes in place instead of dropping), and the
loop keeps running — default commands, including teleop drive, resume on
the next tick. The fault count and last exception surface in Health
telemetry and the flight log. Rationale: season mechanism commands are the
code most likely to throw under competition pressure, and a robot that
loses one mechanism is strictly better than a dead one. Auton keeps full
fail-fast — there, driving on after a fault *is* the dangerous outcome.

Three narrow, intentional exceptions remain as before: subsystem `stop()`
is best-effort (exceptions swallowed so every subsystem gets its
shutdown), telemetry flushes are swallowed after logging (a Panels
websocket hiccup must not stop the robot), and `I2CBusThread` swallows
transient I²C errors by design — it keeps publishing the last good value
and exposes failure counters instead.

## Mechanism control toolkit

Season mechanisms (lift, arm, turret) share one season-agnostic core:
`core/control/` holds a `TrapezoidProfile` (re-planned from the current
setpoint every tick, so goal changes mid-motion just work), a
`PIDFController` (kP/kI/kD + kS/kV/kG feedforward, integral clamp), and
`ProfiledController` combining the two — all pure logic, all host-tested.
`ProfiledMotorSubsystem` wraps that around a single motor with the standard
lifecycle (encoder read in `periodic()`, power write in `writeHardware()`),
a `goToCommand(target, tolerance)` factory, an open-loop bring-up mode, and
hold-at-goal behavior after a command ends (no default command needed).
Gains and constraints are plain mutable holders — put them in a season
`@Configurable` object and they are live-tunable from Panels.

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
