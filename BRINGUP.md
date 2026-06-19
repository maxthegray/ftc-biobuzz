# Bring-Up Checklist

Run this after a fresh fork or a hardware rebuild, before you trust the robot.

The host tests only prove the code agrees with itself — stick mapping, Pinpoint
directions, mirror math. None of that is real until you check it on the actual
robot, and a few of these will throw the robot across the room if they're
backwards. So do them in order, **robot on blocks until step 4.** (Diagnostics
and tuning live elsewhere: loop rate and battery in `RUNBOOK.md`, mechanism
gains in `SEASON-GUIDE.md`.)

## 1. Configuration names

Confirm the Driver Station config names: `frontLeftMotor`, `frontRightMotor`,
`backLeftMotor`, `backRightMotor`, and Pinpoint `pinpoint`. A wrong name fails
loudly at init (Preflight lists what's missing); a *swapped* name won't — it'll
show up as step 2 failing instead.

## 2. Per-motor direction (on blocks)

Drive each wheel individually at low power, verify name + direction.
`MecanumDriveSubsystem.driveRaw(forward, strafe, turn, robotCentric)` is the
sanctioned bring-up API. Fix directions in `pedroPathing/Constants.java`
(`*MotorDirection`), not by re-wiring.

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

## 5. First path (capped power)

Run a single 24" `lineTo` at `followCommand(chain, maxPower = 0.3, holdEnd =
true)` — or `Starter: Localization Test`, which follows on button press. Watch
the field view, then drive a slow lap and compare the final pose against the
field. Endpoint drift with low follower error is localization (check pods);
high error is following/battery — see `RUNBOOK.md`.

## 6. Fault drills (on blocks)

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
