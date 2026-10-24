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

One Diagnostics OpMode, **Vision Test**, brings up the season's cameras. It
commands no motor and applies no pose correction. It opens every camera in the
active configuration, runs them during INIT, and shows one page at a time:
gamepad 1 **dpad left/right** changes page, in INIT too.

| Page | Camera | Purpose |
|---|---|---|
| **Ball camera** | goBILDA 3122-0004-0001 `ballCamera` | Yellow POLLEN color/shape detection, Panels tuning, RC preview |
| **Limelight tags** | Limelight 3A `limelight` | Every HIVE tag: ID, BIOBUZZ meaning, camera-relative measurements, freshness |
| **HIVE survey** | Limelight 3A `limelight` on a tripod | Every tag's height and implied goal, each CELL's fused goal, each HIVE's state (the only page that infers HIVE state) |

### Hardware and configuration

1. **Limelight 3A** → USB-C to the Control Hub **USB 3.0** port (Limelight's
   instruction). Driver Station → Configure Robot → Scan → the new
   **Ethernet Device** → rename `limelight`.
2. **goBILDA Global Shutter USB Camera, SKU 3122-0004-0001** (Arducam OV9782)
   → Control Hub **USB 2.0** port when the Limelight holds the 3.0 port (the
   camera is a USB 2.0 device). Scan → **Webcam 1** → rename `ballCamera`.
3. Save and activate the configuration. **Full APK install** — the lens
   calibration lives in `res/xml/teamwebcamcalibrations.xml`, which a Sloth hot
   reload does not deploy.
4. Verify the calibration matched: Vision Test, ball camera page → *Ball Camera → lens
   calibration* must read `exact 640x480 entry`. `none …` means the camera's
   USB IDs differ from goBILDA's published `VID 0xC45, PID 0x366`; read the
   real ones and fix the `<Camera vid pid>` entry:

   ```sh
   adb shell 'for d in /sys/bus/usb/devices/*; do [ -f $d/idVendor ] && echo "$(cat $d/idVendor):$(cat $d/idProduct) $(cat $d/product 2>/dev/null)"; done'
   ```

The calibration entry is goBILDA's user-guide table (May 14 2026), a
manufacturer starting point rather than a calibration of our unit. goBILDA's
downloadable calibration zip has a `480, 320` typo for the 320×240 entry; the
comment in the XML records why the guide values are used. The ball processor
receives these intrinsics only through `VisionProcessor.init` and uses them
for the *ray angle* telemetry (principal point = 0°, +right, +down). They say
nothing about where the camera sits on the robot.

### Limelight pipeline (web interface)

Configure over a laptop USB connection at `http://limelight.local:5801`
(Limelight 3A quick-start). Reaching the web interface through the Control Hub
has not been verified here. Use **pipeline 0** (`visionDiagnostics.limelightTagPipelineIndex`):

| Tab | Setting | Value |
|---|---|---|
| Input | Pipeline Type | AprilTags |
| Input | Resolution | Record what you choose; Limelight suggests highest for 3D, 640×480 for 2D-only |
| Input | Exposure / Black Level / Sensor Gain | Start low exposure (motion blur), black level 0, gain 15; tune while holding a tag |
| Standard | Family | AprilTag Classic 36h11 |
| Standard | Marker Size | **82.55 mm** (3.25 in black square, manual §9.9) |
| Standard | ID Filter | Blank for the first session, so stray tags show as `NOT A BIOBUZZ TAG`; later `30,31,…,45` |
| Standard | Detector Downscale | Start 1–2 and record it |
| Advanced | Full 3D | **On** — required for camera-space pose |
| Advanced | Camera pose in robot space / field map | Leave unset; the robot code places tags itself from `HiveConfig`, and the HIVES pivot |

Download the pipeline file from the web interface after tuning and commit it.

Units reported: `tx`/`ty` degrees from the crosshair, as the Limelight reports
them (tx positive right; Limelight's JSON spec says ty positive down — confirm
by raising a tag); `ta` as the SDK reports it; camera-space pose in metres and
degrees, Limelight camera space +X right, +Y down, +Z out of the lens.
Timing: *receipt age* is Control Hub wall-clock time since the SDK parsed the
latest poll; *frame age* retains the first receipt age for each Limelight `ts`
and advances on the robot's monotonic clock. Duplicate polls cannot reset it:
tags are cleared when frame age reaches `limelightMaxResultAgeMs`.
*Est. capture age* adds the Limelight's capture and targeting latency to frame
age and still excludes USB transport and the wait before the first poll.
Without a device `ts`, identity falls back to receipt timestamps, so duplicates
across polls cannot be distinguished.

Tags identify a CELL. The tag page does not say which CELL faces up; the
HIVE survey page infers that from measured tag heights. A raised CELL is the
goal, not a promise that it can take a score.

### Panels and the camera preview

- **Panels:** robot powered, laptop on the Control Hub Wi-Fi,
  `http://192.168.43.1:8001`. Configurables `BallCameraConfig` and
  `HiveConfig`; the same telemetry sections as the Driver Station.
  `VisionTestConfig` is deliberately not in Panels: change its defaults
  in code. Panels does not stream camera images.
- **Preview:** on the **Robot Controller** screen while Vision Test has the
  ball camera open (INIT or running): plug an HDMI monitor into the Control
  Hub, or `tools/hub-connect.sh` then `scrcpy`. The SDK's ColorBlobLocatorProcessor
  outlines every blob that passed the filters. On the ball camera page, gamepad 1
  **Circle** toggles rendering (`previewEnabled`); off saves CPU for timing comparisons.

### What applies live

| Live in Panels (next robot loop) | In code, then `make hot` and re-init |
|---|---|
| Area/circularity/aspect/density filters, max observation age, preview | Color space and channel thresholds, ROI, blur/erode/dilate, contour mode, exposure, everything in `VisionTestConfig` |

The SDK builds the color range, region and morphology into the processor at
init, and Panels edits are gone by the next init, so those are changed in
code. The capture resolution is fixed at 640×480 MJPEG.

**Camera mount.** The `ballVision.mount*` fields record where the lens sits:
`mountHeightIn` above the floor, `mountPitchDownDeg` below horizontal,
`mountForwardIn`/`mountLeftIn` from the robot's pose point, `mountYawDeg` from
its front, and `mountMeasured` once the numbers are real. Nothing on the robot
uses them. Re-measure after moving the camera; see [Logging](LOGGING.md)
for recorded mount values and field playback.

**Panels edits are not saved.** They last until the op-mode stops. Copy
values worth keeping into the config object before then.

**Frame age.** The SDK processor publishes no capture timestamp, so *frame
age* is robot-clock time since the latest new frame reached the loop.
Detections clear when it passes `maxObservationAgeMs`.

### Lab procedure

Robot stationary, on blocks. Note the values after each step that settles
something.

1. **Exposure.** One POLLEN ball ~1 m away. With `exposureManual` true, lower
   `exposureMicros` in code until a rolled ball is not smeared; `make hot` and
   re-init after each change. *Ball Camera → exposure* shows what the camera accepted.
2. **Color range.** Adjust `channel*Min/Max` in code, `make hot`, re-init, and check the preview
   until the ball is outlined and tiles, walls, and red/blue NECTAR are not —
   near, far, in shadow, under the brightest light.
3. **Region and blob filters.** Set the ROI to where POLLEN can appear (in
   code, `make hot`). Then, live in Panels: raise `minAreaPercent` just below the farthest useful
   ball's area and tighten `minCircularity` (1.0 is a perfect circle, which a
   traced contour rarely reaches), `maxAspectRatio`, `minDensity` while testing
   two touching balls, a half-hidden ball, and a ball at the frame edge.
4. **Moving and multiple balls.** Roll a ball across the frame: frame age stays
   small and the target never lingers after the ball leaves. With several
   balls, the target is the largest. Cover the lens: the target goes to none.
5. **Both cameras.** Vision Test opens both cameras when both are configured.
   Run it once like that and once with `VisionTestConfig.openLimelight` false
   (in code, `make hot`). Compare *Timing*, library fps, Limelight new-frame
   rate and ages, then compare loop percentiles using the
   [logging workflow](LOGGING.md#review-a-wpilog-in-maxscope).
6. **Limelight tags.** Vision Test, tag page: hold each tag still; confirm ID
   and meaning against its sticker, the signs of tx/ty, and |d| against a tape
   measure; record whether the cluster's four tags appear together.

Then copy the values tuned live in Panels into `BallCameraConfig` and commit.

### HIVE survey (real CELL and field)

Each HIVE is one alliance's. It sits tipped 30° with one CELL raised (the
goal) and the other lowered, and a tip swaps them. Each CELL's four tags are on
its bottom face, below and in front of the opening. FIRST's cluster definition
(SDK 12.0.0) gives every tag's offset to the centre of its CELL's opening, so
each tag implies the same goal point from a different offset. See
`BiobuzzAprilTags`.

Setup:

1. Limelight on a tripod, upright (image not flipped), pipeline 0 as above
   with **Full 3D on**.
2. Measure and set `hive` in Panels (`HiveConfig`): `cameraHeightIn` (lens
   centre above the tiles), `cameraPitchUpDeg` (optical axis above horizontal;
   a phone level on the housing), `cameraYawDeg` 0,
   `cameraForwardIn`/`cameraLeftIn`/`axisForwardIn`/`axisLeftIn` 0 (the tripod
   point is the reference). Set `mountMeasured` true. The turret angle is held
   at 0.
3. Open **Vision Test** and go to the HIVE survey page; *Survey Setup* must read `measured`.

Procedure (**Circle** clears the survey memory):

1. **One CELL head-on.** Aim at your raised CELL from 1.5–2.5 m. *CELL Goals*
   spread under ~1 in with all four tags used. A consistent spread that grows
   as you move sideways means the Limelight rotation convention in
   `GoalGeometry.tagRotation()` is wrong; record the rows and fix that
   function, nothing else.
2. **Distance.** Tape-measure the horizontal distance from the tripod point to
   the centre of the CELL opening; compare *horizontal*. Compare *height* with
   the opening centre's height.
3. **Raised vs lowered.** With both CELLs of a HIVE in view, note the tag heights
   *Tags (latest)* reports for each. Tip the HIVE and repeat. Set
   `hive.raisedMinHeightIn` midway between the raised and lowered heights,
   and keep `classificationMarginIn` well under half the gap. *HIVEs* must
   flip after a tip (event `HIVE TIP: …`) and never flip on a still HIVE.
   *faces … below horizontal* should differ between raised and lowered tags;
   it is a cross-check, not an input.
4. **Match setup.** Stage the field per manual §10.3.1: *HIVEs* must read RED
   audience CELL raised, BLUE far CELL raised (`HiveState.matchSetup`, the
   autonomous prior). If not, fix `matchSetup`.
5. **Every tag.** Walk the tripod round so all 16 tags appear in *Tags
   (latest)*; note any that disagree.

Robot use: `HiveTracker` (registered after the Limelight) with the turret's
measured-angle lookup as `turretAngleAt`, `HiveTracker.MATCH_SETUP` priors in
autonomous (teleop starts unknown), and the localizer's `poseAt` and
pose so a held goal follows the robot's motion. Autonomous can wait for a tip
with `race(waitUntil { hive.tipCount(RED) > before }, waitMs(…))`;
a tip is only seen while the camera can see that HIVE. Shooter speed comes from
`ShotModel.rpmForDistance(goal.horizontalDistanceIn)`, which is not fitted yet.

Aiming reads `hive.aimGoal(alliance)`, not `goal(alliance)`. With no vision
goal (none seen, or lost for over `lostTimeoutMs`) it falls back to
`HiveField`'s approximate field point for the raised CELL, placed with the
current pose (`source = ODOMETRY`). The CELL is the one the tracker believes is
raised, or, with the state unknown, the one on the robot's half of the field.
The points are derived from the manual's drawings, not published coordinates:
on a real field, `Hive/<A>/fieldGoalErrorIn` (vision goal vs `HiveField`, per
frame) should be a few inches. Only shoot on `source = VISION`.

### Not measured yet — needed before powered ball assists

- Camera mounting: height above the tiles, pitch/roll, and lateral/forward
  offset from the robot centre, for both cameras.
- A per-unit lens calibration at the chosen resolution, if the goBILDA table
  disagrees with a checkerboard test.
- Ball image size and ray angle against measured distance, to choose a
  controller measurement and a stopping point. No approach setpoint exists.
- The actual processed frame rate and capture-to-robot latency under match
  load, with drive, Pinpoint, and telemetry running.
- The OV9782 USB IDs.

Follow-up for powered assists: a USB-camera target source with the same
validity gates as `LimelightSubsystem`; controller updates driven by
`BallCameraSubsystem.newFrame`, with the output held between frames; an explicit lost-target timeout; a
latency budget from the measurements above; and on-blocks tests before carpet.

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

**HIVE tags** (HIVE survey, above)
- [ ] Four tags of one CELL agree on its goal within ~1 in, head-on and from the side.
- [ ] Horizontal distance and goal height match a tape measure.
- [ ] Raised and lowered tags separate cleanly; `raisedMinHeightIn` set between them.
- [ ] A tip is reported once; a still HIVE never flips.
- [ ] Staged field reads RED audience / BLUE far raised.
