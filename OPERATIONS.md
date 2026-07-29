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
not a calibration for the new chassis. Do not run `Starter: Example Auto`
until the localization, dynamics, and control checks below are complete.

## 1. Configuration names

Confirm the Driver Station config names: `frontLeftMotor`, `frontRightMotor`,
`backLeftMotor`, `backRightMotor`, and Pinpoint `pinpoint`. A wrong name fails
loudly at init (Preflight lists what's missing); a *swapped* name won't — it'll
show up as step 2 failing instead.

## 2. Per-motor direction (on blocks)

Run **Starter: Motor Direction Test**. Dpad left/right selects a configured
motor; the right and left triggers command that motor forward and reverse at no
more than 20% power. Verify the displayed name matches the wheel that moves and
that positive power turns each wheel in the robot-forward direction. Fix
directions in `pedroPathing/Constants.java` (`*MotorDirection`), not by
re-wiring.

## 3. Pinpoint axes and heading sign (on blocks, then by hand)

- Init any starter op-mode; confirm **Pinpoint status** reads `READY` in the
  Health section first.
- Push the robot by hand, watch the Panels field view: +x forward, +y left.
  Rotate CCW by hand: heading must increase. Fix signs via the encoder
  directions in `pedroPathing/Constants.java`, then re-verify.
- Run Pedro `Tuning` for localizer checks and pod-offset verification.

## 4. Teleop signs and field-centric (on carpet, slow)

In `Starter: Drive Only` at low stick input, check forward/back, strafe
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

Run **Starter: Localization Test** from a clear origin. Y follows 24" forward
and A returns to the origin; both paths are capped at 30% power by default.
Press the active target button again or move a stick to cancel. Watch the field
view, then drive a slow lap and compare the final pose against the field.
Endpoint drift with low follower error is localization (check pods); high error
is following or battery.

## 7. Fault drills (on blocks)

These check the safety behavior you'd otherwise only find out about mid-match:

- **Watchdog:** with a path running, unplug an odometry pod. Within ~0.5 s the
  Health section must show `Localizer: FAULT`, the path must break (teleop:
  sticks keep working; auton: routine cancels), and the log must carry a
  `LOCALIZER FAULT` event. If nothing trips, check
  `LocalizerConfig.watchdogEnabled` and the `pinpoint` hardware name.
- **Containment:** in a throwaway teleop, bind a button to a command whose
  `setExecute` throws. Pressing it should print `command faults` in Health
  while the drive keeps responding. If the op-mode dies, fault containment
  regressed — fix before competing.

## Logs and post-run diagnosis

Every op-mode writes a WPILOG under `/sdcard/FIRST/logs`.

```sh
make debug       # newest Auto + TeleOp, JSON diagnostic bundle
make pull-logs   # copy all logs for AdvantageScope
make analyze     # pull logs and summarize the newest one
```

For an auton problem, explicitly inspect `robot-logs/Auto-*.wpilog`; the
default analyzer target is normally the newer TeleOp log. `lastcrash.txt` on
the hub contains the previous uncontained exception, running commands, recent
events, loop count, and match time.

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

Additional rules:

- A homing timeout is a command fault. It leaves the mechanism FAULTED with
  zero output and never declares a false zero.
- Pedro's `Tuning` op-mode is not persisted by ConfigStore. Copy accepted
  values into `pedroPathing/Constants.java`.
- Repeated `LOOP OVERRUN` events matter; a single init/stop overrun is usually
  runtime warm-up.
- Swap a battery below 12.0 V resting. Normal operation should not sag below
  roughly 10 V.
