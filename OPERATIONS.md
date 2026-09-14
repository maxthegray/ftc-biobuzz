# Robot Operations

Physical bring-up, Pedro calibration with AutoTune, logs, and diagnosis for
this robot. Host tests prove the code agrees with itself and with the Ivy and
Pedro artifacts; none of it proves the robot drives. The **physical validation
checklist** at the end lists what still has to be checked on hardware.

## Bring-up checklist

Run this after a fresh fork, a hardware rebuild, or a framework change. Do the
steps in order, **robot on blocks until step 4.**

`pedro/Constants.java` carries measured Pinpoint offsets and directions, the
existing motor names and directions, and **no Foresight tuning**
(`FORESIGHT_TUNED = false`). Until AutoTune's Foresight output is pasted in,
manual driving works and every path, hold and turn command refuses to start.

A **full APK install** is required the first time after this migration: the
dependencies and SDK version changed.

## 0. Chassis-free framework smoke test

**Framework Smoke Test** is disabled in `opmodes/archived/`. Re-enable it for
this check after a framework change or on a new Control Hub. It needs no
configured devices and exercises lifecycle ordering, gamepad input, Ivy
scheduling and preemption, the command-fault policy, telemetry, Panels,
ConfigStore and WPILOG output. During init `periodic ticks` must advance while
`write ticks` stays at zero; after start both advance. **Y** throws from a
command: Health must show `command faults`, the default command must resume,
and the op-mode must keep running. Its loop is paced to ~50 Hz.

## 1. Configuration names

Driver Station config names: `frontLeftMotor`, `frontRightMotor`,
`backLeftMotor`, `backRightMotor`, Pinpoint `pinpoint`. A wrong name fails at
init (Preflight lists what's missing); a *swapped* name shows up in step 2.

## 2. Motor directions (on blocks)

Either AutoTune's **Mecanum Tuner** (below) or the archived **Motor Direction
Test** (dpad selects a motor, triggers spin it at ≤ 20%). Positive power must
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
direction. Toggle field-centric (Back+B), rotate the robot, confirm
translation stays field-true; reset heading (Back+Y) and confirm "away from
the driver" is +x.

**Second-run heading drill.** Every op-mode starts at (0, 0, 0), so teleop's
field-centric "forward" is wherever the robot faces at INIT.
1. Init and start Drive Only with the robot facing away from the driver. Drive
   a metre and rotate it 90°. Stop.
2. Without power-cycling, face the robot away from the driver again and init
   Drive Only. Before START the pose must read (0, 0, 0), not the first run's
   pose, and Health must show `Localizer: ok`.
3. Start: stick forward drives away from the driver with no Back+Y.
4. Repeat with an autonomous before teleop: teleop still starts at (0, 0, 0),
   regardless of where the autonomous ended. Face the robot away from the
   driver before INIT, or press Back+Y after START.

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
chassis in a comment next to pasted values. Pedro 3 has no voltage
compensation; tune on a full battery.

## 6. First framework path (capped speed)

Re-enable `opmodes/skeletons/LocalizationTestTeleOp.kt`. From a clear origin:
Y follows 24" forward, A returns; speed is capped at 30% of max velocity
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
  autonomous the routine must cancel and the robot stay put. An exception
  from the I²C read itself ends the op-mode (hardware stopped first).
- **Command fault:** in a throwaway teleop, bind a button to a command whose
  `setExecute` throws. Health shows `command faults`, every mechanism stops
  for a tick, the drive default resumes, and the log carries `COMMAND FAULT`.
  Repeat with a command whose `setExecute` calls `TODO()`: the op-mode must
  end with every motor stopped, the Driver Station must show
  `NotImplementedError`, and the log must end with `LOOP CRASHED` and `stop`.

## 8. Vision diagnostics (stationary, no motors)

Two Diagnostics OpModes bring up the season's cameras. Neither commands a
motor, applies a pose correction, or decides anything about HIVE state. Both
run their cameras during INIT; gamepad buttons work only after START.

| OpMode | Camera | Purpose |
|---|---|---|
| **Limelight AprilTag Test** | Limelight 3A `limelight` | Every HIVE tag: ID, BIOBUZZ meaning, camera-relative measurements, freshness |
| **Ball Tracking Test** | goBILDA 3122-0004-0001 `ballCamera` | Yellow POLLEN color/shape detection, Panels tuning, RC preview |

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
4. Verify the calibration matched: Ball Tracking Test → *Ball Camera → lens
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
has not been verified here. Keep **pipeline 0** as the archived Ball Follow's yellow color
pipeline; use **pipeline 1** (`visionDiagnostics.limelightTagPipelineIndex`):

| Tab | Setting | Value |
|---|---|---|
| Input | Pipeline Type | Fiducial Markers |
| Input | Resolution | Record what you choose; Limelight suggests highest for 3D, 640×480 for 2D-only |
| Input | Exposure / Black Level / Sensor Gain | Start low exposure (motion blur), black level 0, gain 15; tune while holding a tag |
| Standard | Family | AprilTag Classic 36h11 |
| Standard | Marker Size | **82.55 mm** (3.25 in black square, manual §9.9) |
| Standard | ID Filter | Blank for the first session, so stray tags show as `NOT A BIOBUZZ TAG`; later `30,31,…,45` |
| Standard | Detector Downscale | Start 1–2 and record it |
| Advanced | Full 3D | **On** — required for camera-space pose |
| Advanced | Camera pose in robot space / field map | Leave unset; this diagnostic reports camera-relative values only, and the HIVES pivot |

Download the pipeline file from the web interface after tuning and commit it
next to the lab record.

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

Tags identify a CELL. Seeing one does not show which CELL faces up or whether
it can take a score; every CELL reads `readiness NOT_INFERRED`.

### Panels and the camera preview

- **Panels:** robot powered, laptop on the Control Hub Wi-Fi,
  `http://192.168.43.1:8001`. Configurables `BallVisionConfig` and
  `VisionDiagnosticsConfig`; the same telemetry sections as the Driver Station.
  Panels does not stream camera images.
- **Preview:** it appears on the **Robot Controller** screen while either
  OpMode has the ball camera open (INIT or running). Plug an HDMI monitor into
  the Control Hub, or `make connect` then `scrcpy`.
- **Modes:** `ballVision.previewMode` 0 overlay, 1 original, 2 threshold mask;
  after START, gamepad 1 **X** cycles modes and **B** toggles rendering
  (`previewEnabled`). Overlay: white ROI, cyan accepted blobs, red rejected
  blobs labelled with the failed filter, **thick magenta circle + cross +
  `TARGET`** for the selected blob. Rendering off pauses the view and skips
  mask/overlay work, for timing comparisons.

### What applies live

| Live (next robot loop) | Restart the OpMode |
|---|---|
| Color space, channel thresholds, ROI, blur/erode/dilate, contour mode, area/circularity/aspect/density filters, candidate count, max observation age, preview mode/rendering, exposure, gain, white balance, `resetToDefaults` | `resolutionWidth/Height`, `streamFormat`, everything in `visionDiagnostics` |

A pending restart setting shows a **RESTART REQUIRED** section. Unsupported
stream modes (anything outside goBILDA's table, e.g. 640×480 YUY2) stop INIT
with a message instead of letting EasyOpenCV E-stop the robot.

Panels writes statics on its socket thread. The robot loop copies them once per
tick into an immutable snapshot; only snapshots reach the camera thread and the
camera-control thread, which probes capabilities, clamps to the probed range,
writes only changed controls, and reads back device values about once a
second. Camera telemetry shows *requested → device* for each.

**Tuning in the flight log.** Both OpModes write the `visionDiagnostics`
values as an event at init. The ball camera writes every `ballVision` value
at init, then an event naming only the fields that changed (at most one per
second; slider drags are merged), tagged with the detection settings version
that frames carry in `BallCamera/frame/settingsVersion`. An edit in the last
second before stop can be missing from the log; the tuning file and lab
record still have it.

**Saved vs compiled defaults.** Each OpMode init resets `ballVision` and
`visionDiagnostics` to their compiled defaults, then applies the keys saved in
`/sdcard/FIRST/config/tuning.properties`. Saved keys win; missing or invalid
keys fall back to defaults. Set `ballVision.resetToDefaults` true to reset the
section live (it clears itself and saves), or delete the keys and restart.
STOP saves dirty settings after hardware shutdown, including STOP during INIT;
restart-only edits do not require pressing START to survive reinitialization.

### Timing honesty

Ball Camera telemetry separates the SDK's frame capture timestamp, processing
start/finish, and the robot tick that first saw the frame. *Processed fps* is
detector output from frame numbers; *received fps* is distinct frames the loop
saw; *library fps* is EasyOpenCV's delivery rate; none is the camera's
advertised 120 fps. Observations older than `maxObservationAgeMs` since capture
are cleared (`CLEARED`), and a processing error never yields a target.

### Lab procedure

Robot stationary, on blocks. Save a lab record (gamepad 1 **A** after START)
after each step that settles something.

1. **Camera image.** OVERLAY mode, one POLLEN ball ~1 m away. Set
   `exposureManual` true and lower `exposureMicros` until a rolled ball is not
   smeared; set `whiteBalanceManual` and a fixed `whiteBalanceKelvin` for the
   room. Touch `gain` only if *Camera Controls → gain* shows a range. Confirm
   *requested → device* agree and note any clamp.
2. **Color mask.** MASK mode. Adjust `channel*Min/Max` until the ball is solid
   white and tiles, walls, and red/blue NECTAR stay black — near, far, in
   shadow, under the brightest light. Add a POLLEN ball and a NECTAR ball.
3. **Region and blob filters.** OVERLAY mode. Set the ROI to where POLLEN can
   appear. Raise `minAreaPercent` just below the farthest useful ball's area,
   then tighten `minCircularity`, `maxAspectRatio`, `minDensity` while testing
   two touching balls, a half-hidden ball, and a ball at the frame edge. Read
   rejection labels in the preview and *Ball Candidates*.
4. **Moving and multiple balls.** Roll a ball across the frame: frame status
   stays FRESH, age stays small, the target never lingers after the ball
   leaves. With several balls, the target is the largest accepted blob.
   Cover the lens: the target status must drop to NONE_ACCEPTED at once.
5. **320×240 comparison.** Record the 640×480 numbers (processed fps,
   processing ms, age, farthest detection). Set 320/240, halve the kernel
   sizes, restart, repeat, save a record. Area filters are frame percentages and
   carry over.
6. **Both cameras.** `runBothCameras` true; restart **each** OpMode. Compare
   *Timing Comparison*, processed fps, Limelight new-frame rate and ages against
   the single-camera runs, then `make debug` for loop percentiles.
7. **Save and restart.** Stop, power-cycle, reopen Ball Tracking Test: Panels
   and telemetry show the tuned values and *settings v1*;
   `adb shell grep ballVision /sdcard/FIRST/config/tuning.properties`
   matches the last lab record's `[config]` lines.
8. **Limelight tags.** Limelight AprilTag Test: hold each tag still; confirm ID
   and meaning against its sticker, the signs of tx/ty, and |d| against a tape
   measure; record whether the cluster's four tags appear together.

Then `make pull-lab-records`, fill in the header, and commit the record.
Adopt tuned values by editing the `DEFAULT_*` constants listed under
**[adopt as compiled defaults]** and deleting those keys from the hub's
tuning file.

### Not measured yet — needed before powered ball assists

- Camera mounting: height above the tiles, pitch/roll, and lateral/forward
  offset from the robot centre, for both cameras.
- A per-unit lens calibration at the chosen resolution, if the goBILDA table
  disagrees with a checkerboard test.
- Ball image size and ray angle against measured distance, to choose a
  controller measurement and a stopping point. No approach setpoint exists.
- The actual processed frame rate and capture-to-robot latency under match
  load, with drive, Pinpoint, and telemetry running.
- The OV9782 USB IDs and exposure/gain/white-balance ranges from the probe.

Follow-up for powered assists: a USB-camera target source with the same
validity gates as `LimelightSubsystem`; controller updates driven by
`BallObservation.newFrame` with `dt` taken from successive capture timestamps
and the output held between frames (the archived Ball Follow controllers still
step every loop on loop time — REVIEW B18); an explicit lost-target timeout; a
latency budget from the measurements above; and on-blocks tests before carpet.

## Logs and post-run diagnosis

Every op-mode writes a WPILOG under `/sdcard/FIRST/logs` (newest 30 kept).
Continuous channels are sampled at ≤ 100 Hz; `events` keeps each event's own
timestamp; per-window loop maxima keep spikes between samples.

```sh
make debug       # newest Auto + TeleOp, JSON diagnostic bundle
make pull-logs   # copy all logs for AdvantageScope
make analyze     # pull logs and summarize the newest one
```

For an auton problem inspect `robot-logs/Auto-*.wpilog` explicitly; the
default target is usually the newer TeleOp log.

**There is no command history.** Ivy has no lifecycle hooks, so logs from the
current code have no `commands/running` channel and no per-command
STARTED/FINISHED/INTERRUPTED/FAULTED events (the analyzer prints
`command history: not recorded`). What commands did is visible through
`driveMode`, subsystem channels, and events code records explicitly
(`AUTO: …`, `COMMAND FAULT: …`, `LOCALIZER FAULT: …`, `LOOP CRASHED: …`).
Logs recorded before September 2026 still show their command sets.
`lastcrash.txt` is no longer written; a loop crash's stack trace is in `events`.

### Watching the robot on AdvantageScope's 2D field

Drag **`Field/Robot`** onto the 2D Field tab and pick an FTC field; its default
coordinate system (**Center/Rotated**) is the one the channel is written for.
`Field/Robot` is a WPILib `Pose2d` struct in metres about the field centre;
graph `pose` (raw Pedro inches) when comparing against `Constants.java`.

**Verify the axes once on a real field.** The encoding converts units and
moves the origin from Pedro's corner to the field centre, but does not rotate
anything. Park at a known spot, confirm the drawn robot is there, drive one
tile forward. A quarter-turn error is the axis convention, not the encoding —
fix it in `WpiStruct` and say so there.

### Symptom triage

| Symptom | First evidence to check |
|---|---|
| Op-mode stopped | Driver Station exception, `LOOP CRASHED` event, loop phase maxima, minimum battery |
| All mechanisms twitched off / auto stopped mid-routine | `COMMAND FAULT` event and Health `command faults` |
| Path "finished" short of the target | `follow/translationalErrorIn` at the end; follow ends at the parametric end, add `holdCommand` |
| Robot rotated the wrong way along a line | `.linear(...)` on `Paths.line` (Pedro 3.0.0); use a 3-point curve |
| Auton wrong only when mirrored | Headings not through `Alliance.mirror`, or `PoseFactory.mirrorX` used |
| Paths refuse to start | `FORESIGHT_TUNED` false, or `ROBOT_CENTRIC_FALLBACK` after a localizer fault |
| Auton drifted | `follow/translationalErrorIn`: small error means localization; large means following |
| Sudden pose jump | `pose correction applied` events and correction gates |
| Field-centric wrong | Robot faced elsewhere at INIT (heading starts at 0): Back+Y |
| Driving robot-centric unexpectedly | `LOCALIZER FAULT` event, `Drive/odometryFallback` |
| Loop rate collapsed | Phase maxima: `writeHardware` usually Pinpoint, `telemetry` Panels, `periodic` season I/O |
| Tuned config reverted | Registration, public primitive `@JvmField`, config schema |
| Pinpoint unhealthy | Init Health status (`waiting for start pose` = pose write not landing), I²C cable, robot still at power-up (IMU calibration) |
| Ball target flickers or lingers | `BallCamera/frame/ageMs`, `rate/processedFps`, `target/status`, `candidates/accepted` |
| Tag diagnostic shows nothing | Limelight health (pipeline index/type), `Limelight/fiducial/count`, Full 3D and marker size |

- Repeated `LOOP OVERRUN` events matter; one at init/stop is usually warm-up.
- Swap a battery below 12.0 V resting. Normal operation should not sag below ~10 V.

## Physical validation checklist

Everything below is unverified on hardware after the Ivy / Pedro 3 migration.
Record results in `PROGRESS.md`.

**Install and reload**
- [ ] Full APK install succeeds; Driver Station shows Drive Only, Ball Tracking
      Test, Limelight AprilTag Test (no Pedro Tuning op-mode).
- [ ] AutoTune page loads at `http://192.168.43.1:10158` and lists Mecanum
      Tuner, Pinpoint Tuner, Foresight Tuner and Tests (Sloth 0.2.4 runtime).
- [ ] Panels loads at `:8001`; a `DriveConfig` edit applies live and survives
      an op-mode restart and a power cycle.
- [ ] `make hot` after a small TeamCode edit: change takes effect, Panels still
      edits the reloaded values, AutoTune still lists procedures.
- [ ] Run Drive Only, stop, run it again three times, then an autonomous and a
      teleop back to back: no stale command runs at start, no crash.

**AutoTune and Pedro**
- [ ] Mecanum Tuner confirms the four motor directions.
- [ ] Pinpoint Tuner offsets/directions match `localizerConfig`
      (xPodOffset 2.8346 in, yPodOffset 0, X REVERSED, Y FORWARD).
- [ ] Foresight Tuner completes; output pasted; `FORESIGHT_TUNED = true`.
- [ ] Tests → Hold resists pushing; Line and Curve repeat without drift.
- [ ] A `Paths.line(...).linear(...)` path is avoided; a 3-point curve with
      linear heading rotates the right way.

**Driving and turning**
- [ ] Stick signs, precision trigger, squared input curve, diagonal speed.
- [ ] Field-centric stays field-true through a full rotation; Back+Y resets heading.
- [ ] Brake mode on stick release matches `DriveConfig.brakeOnTeleop`.
- [ ] `turnToCommand(90°)` reaches heading within 2°; a blocked turn times out
      and the routine aborts with `COMMAND FAULT: … turnTo timed out`.
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
- [ ] Command fault drill (step 7).
- [ ] Unplugged Pinpoint at init: Health reports it, auto refuses to start.

**Recording and field view**
- [ ] A 2-minute teleop log opens in AdvantageScope; `Field/Robot` draws the
      robot in the right place and orientation on the FTC field (axis check above).
- [ ] `pose`, `velocity`, `driveMode`, `follow/*`, `battery`, `gamepad1/*`,
      `loop/*`, `Drive/*`, `Localizer/*` and season subsystem channels plot
      with sensible values; events line up with what happened.
- [ ] `make debug` summarises the newest Auto + TeleOp logs.
- [ ] Log size stays bounded (a host test run wrote ~24 kB/s at 50 Hz) and the file is intact
      after stopping normally and after a battery pull.
- [ ] Second-run heading drill (step 4): a second init reads (0, 0, 0), and
      teleop after an autonomous does not inherit the autonomous pose.
- [ ] The last `Field/Robot` sample of a stopped log is where the robot
      actually stopped, not the field origin.
