# Robot Operations

Physical bring-up, Pedro calibration with AutoTune and diagnosis for
this robot. Host tests prove the code agrees with itself and with the Ivy and
Pedro artifacts; none of it proves the robot drives. The **physical validation
checklist** at the end lists what still has to be checked on hardware.

## Bring-up checklist

Run this after a fresh clone, a hardware rebuild, or a framework change. Do the
steps in order, **robot on blocks until step 4.**

`pedro/Constants.java` carries measured Pinpoint offsets and directions, the
existing motor names and directions, and the **sensorbot's** Foresight tuning
(`FORESIGHT_TUNED = true`). On a new chassis, re-run the Foresight Tuner before
any path; with `FORESIGHT_TUNED = false`, manual driving works and every path,
hold and turn command refuses to start.

## 1. Configuration names

Driver Station config names: `frontLeftMotor`, `frontRightMotor`,
`backLeftMotor`, `backRightMotor`, Pinpoint `pinpoint`. A wrong name fails at
init (Preflight lists what's missing); a *swapped* name shows up in step 2.

## 2. Motor directions (on blocks)

Use AutoTune's **Mecanum Tuner** (below). Positive power must
turn each wheel robot-forward. Fix directions in `pedro/Constants.java`
(`drivetrainConfig`), not by re-wiring.

## 3. Pinpoint axes and heading (on blocks, then by hand)

- Power the robot on with it still: the Pinpoint calibrates its IMU at power-up
  and is not recalibrated at INIT.
- Init **Drive Only**: Health must reach `Localizer: ok` (it shows
  `waiting for start pose …` or `waiting for Pinpoint READY (status …)`
  meanwhile, five-second limit) and the pose must read (0, 0, 0).
- Push the robot by hand and watch the Panels field: forward is +x, left is
  +y, counter-clockwise rotation increases heading. AutoTune → **Tests →
  Localization / Pose** shows the same.
- If signs or distances are wrong, run AutoTune's **Pinpoint Tuner** and copy
  its output into `localizerConfig`.

## 4. Teleop signs and field-centric (on carpet, slow)

In **Drive Only** at low stick: forward/back, strafe left/right, turn
direction. Toggle field-centric (Create+Circle), rotate the robot, confirm
translation stays field-true; reset heading (Create+Triangle) and confirm "away from
the driver" is +x.

**Second-run heading drill.** Every op-mode starts at (0, 0, 0), so teleop's
field-centric "forward" is wherever the robot faces at INIT.
1. Init and start Drive Only with the robot facing away from the driver. Drive
   a metre and rotate it 90°. Stop.
2. Without power-cycling, face the robot away from the driver again and init
   Drive Only. Before START the pose must read (0, 0, 0), not the first run's
   pose, and Health must show `Localizer: ok`.
3. Start: stick forward drives away from the driver with no Create+Triangle.
4. Repeat with an autonomous before teleop: teleop still starts at (0, 0, 0),
   regardless of where the autonomous ended. Face the robot away from the
   driver before INIT, or press Create+Triangle after START.

## 5. AutoTune (clear carpet, full battery)

With the robot connected, open **http://192.168.43.1:10158**. Procedures
(registered in `pedro/Tuning.java`):

1. **Mecanum Tuner** — motor names and directions → `drivetrainConfig`.
2. **Pinpoint Tuner** — pod type, directions, offsets → `localizerConfig`.
3. **Foresight Tuner** — max velocities, natural deceleration, braking and
   heading coefficients, translational/coast/brake gains. Needs room to drive
   forward and left. Paste the **Java** tab into `foresightConfig`, then set
   `FORESIGHT_TUNED = true`.
4. **Tests** — Hold, Line, Curve and Interpolation tests against the tuned
   follower. Line before Curve.

Only values pasted into `pedro/Constants.java` persist. Record the date and
chassis in a comment next to pasted values. Foresight and Tests drive through
`VoltageCompensatedMecanum`, which scales every follower power by
`NOMINAL_VOLTS` / battery volts, so the tuning holds on any battery; keep
`NOMINAL_VOLTS` at 12.0 when tuning and don't change it afterwards.

## 6. First framework path (capped speed)

Re-enable `opmodes/skeletons/LocalizationTestTeleOp.kt` (and add
`@Configurable` back to the class to tune its targets in Panels). From a clear origin:
Triangle follows 24" forward, Cross returns; speed is capped at 30% of max velocity
(`pathSpeedFraction`). Press the button again or move a stick to cancel.
Pressing a button within 0.25 in of its target does nothing; with
`turnToTargetHeading` the robot turns to the target heading instead.
Endpoint drift with low `follow/translationalErrorIn` is localization; high
error is following or battery.

## 7. Fault drills (on blocks)

- **Localizer watchdog:** with a path running, unplug an odometry pod. Health
  must show `Localizer: FAULT`, the log must carry `LOCALIZER FAULT`, the path
  must stop, and sticks must drive robot-centric (`driveMode`
  `ROBOT_CENTRIC_FALLBACK`) with forward, strafe, turn, precision and release
  all correct. Paths and field-centric stay unavailable until a new run. In
  autonomous the pod fault is only logged (`LOCALIZER FAULT (pose still
  live)`) and the routine keeps driving; only a non-finite or frozen pose
  cancels it and leaves the robot parked. An exception
  from the I²C read itself is a `SUBSYSTEM FAULT` on the drive: the motors
  get zero power each tick the read fails, and the path resumes when it
  works again.
- **Command fault:** in a throwaway teleop, bind a button to a `logged()`
  command on the intake whose `setExecute` throws. Health shows `command
  faults`, only the intake stops, driving carries on, and the log carries
  `COMMAND FAULT: <name>: …`.
  Repeat with a command whose `setExecute` calls `TODO()`: the op-mode must
  end with every motor stopped, the Driver Station must show
  `NotImplementedError`, and the log must end with `LOOP CRASHED` and `stop`.

## 8. Vision diagnostics (stationary, no motors)

The Diagnostics OpMode **Vision Test** opens the Limelight and commands no
motors. It shows every tag in view (`id: tx … ty …`) and the goal `GoalAim`
picks for each alliance (see [Development](DEVELOPMENT.md#vision)).

### Hardware and configuration

**Limelight 3A** → USB-C to the Control Hub **USB 3.0** port (Limelight's
instruction). Driver Station → Configure Robot → Scan → the new **Ethernet
Device** → rename `limelight`. Save and activate the configuration.

### Limelight pipeline (web interface)

Configure over a laptop USB connection at `http://limelight.local:5801`
(Limelight 3A quick-start). Use **pipeline 0**:

| Tab | Setting | Value |
|---|---|---|
| Input | Pipeline Type | AprilTags |
| Input | Exposure / Black Level / Sensor Gain | Start low exposure (motion blur), black level 0, gain 15; tune while holding a tag |
| Standard | Family | AprilTag Classic 36h11 |
| Standard | Marker Size | **82.55 mm** (3.25 in black square, manual §9.9) |
| Standard | ID Filter | `30,31,…,45` |
| Advanced | Camera pose in robot space / field map | Leave unset; tags are never used for localization |

Download the pipeline file from the web interface after tuning and commit it.

### Checks on a real field

1. **`ty` sign.** Hold a tag and raise it: its `ty` must grow. The
   "higher CELL wins" rule depends on it; if it shrinks, flip the comparison
   in `GoalAim.goal`.
2. **Lowered tags.** With a HIVE tipped, look at the lowered CELL from around
   the field. If its tags ever appear, check that the goal line still names the
   raised CELL whenever both are in view.
3. **Pivot aim.** From several spots, turn the robot to
   `GoalAim.pivotBearingRad`: the raised CELL's tags should be in view each time.

## Logging

Logging is automatic; see [dutchdocs/LOGGING.md](LOGGING.md) for WPILOG channels,
settings and power samples, MaxScope, pulling logs, diagnosis and recorder
validation (including the hardware recording checklist).

## Physical validation checklist

Everything below is unverified on hardware after the Ivy / Pedro 3 migration.
Record results in `PROGRESS.md`.

**Install and reload**
- [ ] Full APK install succeeds; Driver Station shows Drive Only, Robot Centric
      Drive, Vision Test, Just Path RED (no Pedro Tuning op-mode).
- [ ] AutoTune page loads at `http://192.168.43.1:10158` and lists Mecanum
      Tuner, Pinpoint Tuner, Foresight Tuner and Tests (Sloth 0.2.4 runtime).
- [ ] Panels loads at `:8001`; a `DriveConfig` edit applies live and goes back
      to the code's value when the op-mode restarts.
- [ ] `make hot` after a small TeamCode edit: change takes effect, Panels still
      edits the reloaded values, AutoTune still lists procedures.
- [ ] Run Drive Only, stop, run it again three times, then an autonomous and a
      teleop back to back: no stale command runs at start, no crash.

**AutoTune and Pedro**
- [ ] Mecanum Tuner confirms the four motor directions.
- [ ] Pinpoint Tuner offsets/directions match `localizerConfig`
      (xPodOffset 2.8346 in, yPodOffset 0, X REVERSED, Y FORWARD).
- [ ] Foresight Tuner completes; output pasted; `FORESIGHT_TUNED = true`.
      No path, hold or turn is run on the robot before this.
- [ ] Tests → Hold resists pushing; Line and Curve repeat without drift.
- [ ] A `Paths.line(a, b).linear(a, b)` path (Example Auto's
      return leg) starts at a's heading and ends at b's, on RED and BLUE.

**Driving and turning**
- [ ] Stick signs, precision trigger, squared input curve, diagonal speed.
- [ ] Field-centric stays field-true through a full rotation; Create+Triangle resets heading.
- [ ] Brake mode on stick release matches `DriveConfig.brakeOnTeleop`.
- [ ] `turnToCommand(90°)` reaches heading within 2°; a blocked turn times out
      and the routine goes on to its next step.
- [ ] `holdCommand` settles within 1 in / 2°.

**Cancellation and takeover**
- [ ] Localization Test: moving a stick mid-path stops the path immediately
      and hands control to the sticks; pressing the target button again cancels.
      Pressing a button while already at that target does nothing (`last
      press: … already at …`, no `COMMAND FAULT`); with `turnToTargetHeading`
      and a different heading the robot only turns.
- [ ] A race timeout stops the drive and the routine continues.
- [ ] Stopping the op-mode mid-path stops all four motors at once.

**Faults**
- [ ] Localizer fault drill (step 7) in teleop and in autonomous.
- [ ] Command fault drill (step 7), including the `TODO()` variant: the op-mode
      ends with every motor stopped and the log ends `LOOP CRASHED`, `stop`.
- [ ] Unplugged Pinpoint at init: Health reports it; auto still runs and logs
      `AUTO: starting without a ready localizer`, unless the pose itself is lost.

**Vision** (checks on a real field, above)
- [ ] `ty` grows as a tag rises.
- [ ] Vision Test names the raised CELL whenever our tags are in view.
- [ ] The raised CELL's tags are in view when aimed at the pivot.
