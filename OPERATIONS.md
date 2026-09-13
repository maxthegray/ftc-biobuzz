# Robot Operations

Physical bring-up, Pedro calibration, and post-run diagnosis for this robot.

## Bring-up checklist

Run this after a fresh fork or a hardware rebuild, before you trust the robot.

The host tests only prove the code agrees with itself — stick mapping, Pinpoint
directions, mirror math. None of that is real until you check it on the actual
robot, and a few of these will throw the robot across the room if they're
backwards. So do them in order, **robot on blocks until step 4.** (Diagnostics
are below; mechanism gains are covered in `DEVELOPMENT.md`.)

The numeric Pedro values in `pedroPathing/Constants.java` are placeholders,
not a calibration for the new chassis. Do not run `Example Auto`
until the localization, dynamics, and control checks below are complete.

## 0. Chassis-free framework smoke test

**Framework Smoke Test** is disabled in `opmodes/archived/`. Re-enable it
for this check after a framework change or when bringing up a new Control Hub.

Run **Framework Smoke Test** before hardware configuration. It needs
no configured devices and verifies the Control Hub runtime, lifecycle ordering,
gamepad input, command scheduling/preemption/containment, telemetry, Panels,
ConfigStore visibility, loop profiling, and WPILOG output. During init,
`periodic ticks` must advance while `write ticks` stays at zero; after start,
both must advance. The op-mode displays its controls and every action is
software-only. Its main loop is intentionally paced to roughly 50 Hz so the
absence of hardware I/O cannot create oversized flight logs.

## 1. Configuration names

Confirm the Driver Station config names: `frontLeftMotor`, `frontRightMotor`,
`backLeftMotor`, `backRightMotor`, and Pinpoint `pinpoint`. A wrong name fails
loudly at init (Preflight lists what's missing); a *swapped* name won't — it'll
show up as step 2 failing instead.

## 2. Per-motor direction (on blocks)

Re-enable `opmodes/archived/MotorDirectionTestTeleOp.kt` for this check.

Run **Motor Direction Test**. Dpad left/right selects a configured
motor; the right and left triggers command that motor forward and reverse at no
more than 20% power. Verify the displayed name matches the wheel that moves and
that positive power turns each wheel in the robot-forward direction. Fix
directions in `pedroPathing/Constants.java` (`*MotorDirection`), not by
re-wiring.

## 3. Pinpoint axes and heading sign (on blocks, then by hand)

- Init a drive op-mode; confirm **Pinpoint status** reads `READY` in the
  Health section first. Initialization refreshes odometry without driving
  motors and allows five seconds for calibration. `Example Auto` refuses to
  start while readiness is pending or faulted.
- Push the robot by hand, watch the Panels field view: +x forward, +y left.
  Rotate CCW by hand: heading must increase. Fix signs via the encoder
  directions in `pedroPathing/Constants.java`, then re-verify.
- Run Pedro `Tuning` for localizer checks and pod-offset verification.

## 4. Teleop signs and field-centric (on carpet, slow)

In `Drive Only` at low stick input, check forward/back, strafe
left/right, and turn sign — these are framework defaults
(`MecanumDriveSubsystem.applyTeleopDrive`), not yet verified on your chassis.
Toggle field-centric (Back+B), rotate the robot, confirm translation stays
field-true; reset heading (Back+Y) and confirm "away from driver" is +x.

## 5. Pedro calibration (clear carpet, full battery)

Use the **Pedro Pathing: Tuning** menu and save every accepted result back into
`pedroPathing/Constants.java`; changes made only in the tuning op-mode are
temporary.

1. **Localization first.** Verify the configured pod model and encoder
   directions. Set both pod offsets to zero before **Offsets Tuner**, then
   enter the measured offsets and verify forward, lateral, and full-turn
   distances by hand.
2. **Drivetrain measurements.** Enter the measured robot mass, then run the
   forward/lateral velocity and zero-power-acceleration tuners with enough
   stopping room.
3. **Control.** Tune translational, heading, and drive control before
   centripetal or predictive-braking behavior.
4. **Validate.** The Line test should work before Triangle, and Triangle
   before Circle. Re-run localization checks if the field pose is wrong even
   when follower error is small.

## 6. First framework path (capped power)

Re-enable `opmodes/skeletons/LocalizationTestTeleOp.kt` for this check.
Run **Localization Test** from a clear origin. Y follows 24" forward
and A returns to the origin; both paths are capped at 30% power by default.
Press the active target button again or move a stick to cancel. Watch the field
view, then drive a slow lap and compare the final pose against the field.
Endpoint drift with low follower error is localization (check pods); high error
is following or battery.

## 7. Fault drills (on blocks)

These check the safety behavior you'd otherwise only find out about mid-match:

- **Watchdog:** with a path running, unplug an odometry pod. The device-status
  check runs about once per second. Health must show `Localizer: FAULT` and
  the log must carry a `LOCALIZER FAULT` event. In teleop, the active drive
  command must end and sticks must work robot-centric; field-centric drive
  and assists stay disabled until a new op-mode run. Repeat while holding a
  ball assist, and verify forward, strafe, turn, precision, and stick release.
  In autonomous, the routine must cancel. If nothing trips, check
  `LocalizerConfig.watchdogEnabled` and the `pinpoint` hardware name. This tests
  a reported sensor fault; an exception from the actual I2C read still follows
  the framework's fail-fast hardware policy.
- **Containment:** in a throwaway teleop, bind a button to a command whose
  `setExecute` throws. Pressing it should print `command faults` in Health
  while the drive keeps responding. If the op-mode dies, fault containment
  regressed — fix before competing.

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

Every op-mode writes a WPILOG under `/sdcard/FIRST/logs`.
Continuous channels are capped at 100 Hz. `events` includes timestamped
`COMMAND STARTED`, `FINISHED`, `INTERRUPTED`, and `FAULTED` records for scheduled
commands, including those that finish within one loop. `commands/running`
remains the sampled running set. The analyzer uses per-window timing maxima
so short spikes are not hidden by the cap.

```sh
make debug       # newest Auto + TeleOp, JSON diagnostic bundle
make pull-logs   # copy all logs for AdvantageScope
make analyze     # pull logs and summarize the newest one
```

For an auton problem, explicitly inspect `robot-logs/Auto-*.wpilog`; the
default analyzer target is normally the newer TeleOp log. `lastcrash.txt` on
the hub contains the previous uncontained exception, running commands, recent
events, loop count, and match time.

### Watching the robot on AdvantageScope's 2D field

Drag **`Field/Robot`** onto the 2D Field tab. It is the same pose as the
`pose` channel, re-encoded as a WPILib `Pose2d` struct — AdvantageScope draws
a robot only from a struct, and the `double[]` format `pose` uses is
deprecated there and goes away in 2027. Pick an FTC field; its default
coordinate system (**Center/Rotated**) is the one the channel is written for.

`pose` stays in raw Pedro inches and is the one to graph — `Field/Robot` is in
metres about the field centre, because that is the struct's unit contract, and
those are the wrong numbers to compare against `Constants.java`.

**Verify the axes once on a real field.** The encoding converts units and
moves the origin from Pedro's corner to the field centre, but it does not
rotate anything: whether Pedro's +X is the FTC frame's +X depends on where the
field origin was set up, which is a team convention. Park the robot at a known
spot, confirm the drawn robot is there, and drive one tile forward. If it comes
out a quarter turn off, that is the axis convention and not the encoding — fix
it in `WpiStruct` and say so there.

### Symptom triage

| Symptom | First evidence to check |
|---|---|
| Op-mode stopped | Driver Station exception, `lastcrash.txt`, loop phase maxima, and minimum battery |
| One mechanism stopped | Health `command faults` and the `COMMAND FAULT` event |
| One binding stopped | `TRIGGER FAULT`; the bad trigger is quarantined while later triggers continue |
| Auton wrong immediately | Starting pose and selector's `WILL RUN` routine/alliance |
| Auton wrong only when mirrored | Bare headings missing `Alliance.mirror(heading)` |
| Auton drifted | `follow/translationalErrorIn`: small error means localization; large means following |
| Sudden pose jump | `pose correction applied` events and vision correction gates |
| Field-centric wrong | Back+Y heading reset; after auton, check `PERSISTED POSE RESTORE` |
| Mechanism hit a stop | Goal vs position channels, homing state, and configured soft limits |
| Loop rate collapsed | Phase maxima: `writeHardware` usually Pinpoint, `telemetry` Panels, `periodic` season I/O |
| Tuned config reverted | Registration, public primitive `@JvmField`, and config schema |
| Pinpoint unhealthy | Init Health status, I²C cable, then stationary IMU recalibration |
| Ball target flickers or lingers | `BallCamera/frame/ageMs`, `rate/processedFps`, `target/status`, `candidates/accepted` |
| Tag diagnostic shows nothing | Limelight health (pipeline index/type), `Limelight/fiducial/count`, Full 3D and marker size in the web UI |

Additional rules:

- A homing timeout is a command fault. It leaves the mechanism FAULTED with
  zero output and never declares a false zero.
- Pedro's `Tuning` op-mode is not persisted by ConfigStore. Copy accepted
  values into `pedroPathing/Constants.java`.
- Repeated `LOOP OVERRUN` events matter; a single init/stop overrun is usually
  runtime warm-up.
- Swap a battery below 12.0 V resting. Normal operation should not sag below
  roughly 10 V.
