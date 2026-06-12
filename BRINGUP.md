# Bring-Up Checklist

Use this after a fresh fork or hardware rebuild, before trusting autonomous.

The framework's host tests prove the code is consistent with **its own
conventions** — stick mapping, Pinpoint directions, mirror transforms, the
loop budget. None of those are facts until checked against the physical
robot, and several produce violent wrong-direction motion when wrong. Each
step below isolates one convention; do them in order, robot on blocks until
step 4.

## 1. Configuration names

Confirm Driver Station configuration names: `frontLeftMotor`,
`frontRightMotor`, `backLeftMotor`, `backRightMotor`, and Pinpoint
`pinpoint`. A wrong name fails loudly at init (Preflight lists every missing
device); a *swapped* name does not — it shows up as step 2 failing.

## 2. Per-motor direction (on blocks)

Drive each wheel individually at low power and verify name + direction.
`MecanumDriveSubsystem.driveRaw(forward, strafe, turn, robotCentric)` is the
sanctioned bring-up API — a throwaway op-mode binding one wheel-ish motion
per button works, or use the FTC SDK spin-test sample. Fix directions in
`pedroPathing/Constants.java` (`*MotorDirection`), not by re-wiring.

## 3. Pinpoint axes and heading sign (on blocks, then by hand)

- Init any starter op-mode and check **Pinpoint status** in the Health
  telemetry section reads `READY` before anything else.
- Push the robot by hand and watch the Panels field view: +x forward, +y
  left (Pedro convention). Rotate the robot CCW by hand: heading must
  increase. Fix signs via the encoder directions in
  `pedroPathing/Constants.java`, then re-verify.
- Run Pedro `Tuning` for the localizer checks and pod-offset verification.

## 4. Teleop signs and field-centric (on carpet, slow)

In `Starter: Drive Only`, check forward/back, strafe left/right, and turn
sign at low stick input. The sign conventions in
`MecanumDriveSubsystem.applyTeleopDrive` are framework defaults, not yet
verified on every chassis. Then toggle field-centric (Back+B), rotate the
robot, and confirm translation stays field-true; reset heading with Back+Y
and confirm "away from driver" is now +x.

## 5. Loop budget (while driving step 4)

Read the **Loop** telemetry section while driving: total ms / hz and the
per-phase maxima. Record the real numbers (the Pinpoint read inside
`Follower.update()` dominates `writeHardware`). If the loop is slower than
~30 Hz, find the phase that owns it before tuning anything else. The same
numbers land in the flight log (`loop/*Nanos`) for `make analyze`.

## 6. First path (capped power)

Run a single 24" `lineTo` with `followCommand(chain, maxPower = 0.3,
holdEnd = true)` — or `Starter: Localization Test`, which path-follows on
button press. Watch the field view during the follow; afterwards `make
pull-logs` and check `follow/translationalErrorIn` and the
`Drive/motors/*/power` + `Drive/motors/*/currentAmps` channels in
AdvantageScope — error that grows with current draw is friction or battery,
error with low current is tuning.

## 7. Localization drift lap

Run `Starter: Localization Test`: drive a slow lap and compare the final
displayed pose against the field — and against the live robot drawing on
the Panels field view.

## 8. Mirror check

Validate routine logic headless first: copy `SimAutonRoutineTest` for your
routine and confirm sequencing + RED/BLUE mirroring in JUnit before touching
carpet. Then run the same routine on BLUE via the `AutonSelector` and A/B
the mirrored endpoints on carpet — pose mirroring *and* heading
interpolation both have to be right (`RobotConfig.Field.SYMMETRY` must match
the game manual's field drawing).

## 9. Localizer watchdog drill (on blocks)

With the robot on blocks and a path running (or wheels commanded), unplug an
odometry pod cable. Within ~0.5 s the Health section must show
`Localizer: FAULT: …`, the path must break (teleop: sticks keep working;
auton skeleton: routine cancels and stops), and the flight log must carry a
`LOCALIZER FAULT` event. If nothing trips, check
`LocalizerConfig.watchdogEnabled` and that the Pinpoint is in the hardware
map under `pinpoint`. Reconnect, re-init, re-verify `READY`.

## 10. Teleop fault containment

In a throwaway teleop, bind a button to a command whose `setExecute` throws.
Pressing it should print a `command faults` line in Health telemetry while
the drive keeps responding. If the op-mode dies instead, the scheduler's
fault containment regressed — fix before competing.

## 11. Mechanisms

For any `ProfiledMotorSubsystem` mechanism: tune kG first (mechanism holds
against gravity open-loop), then kV along a slow profile, then kP. Verify
the encoder isn't reset between auton and teleop
(`zeroEncoderOnInit = false`), and verify `homeCommand` finds the hard stop.

## 12. Post-session log review

Pull logs and read the summary: `make analyze` (or open the `.wpilog` in
AdvantageScope for the full channel view). Check loop-rate percentiles,
phase maxima, battery sag, motor currents, follower error, and the event
timeline.
