# Bring-Up Checklist

Use this after a fresh fork or hardware rebuild, before trusting autonomous.

1. Confirm Driver Station configuration names: `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`, `backRightMotor`, and Pinpoint `pinpoint`.
2. Run a per-motor spin test from the FTC SDK examples or a short throwaway op-mode. Verify each wheel name and direction before enabling path following.
3. In `Starter: Drive Only`, check forward/back, strafe left/right, and turn sign. The teleop sign conventions in `MecanumDriveSubsystem` are framework defaults, not yet verified on every chassis.
4. Toggle field-centric mode and rotate the robot by hand. Confirm the field-centric rotation sense matches driver expectation.
5. Run Pedro `Tuning` for Pinpoint axis checks. Explicitly verify the encoder directions in `pedroPathing/Constants.java`.
6. Run `Starter: Localization Test`. Drive a slow lap and compare final displayed pose against the field — and against the live robot drawing on the Panels field view.
7. Validate routine logic headless first: copy `SimAutonRoutineTest` for your routine and confirm sequencing + RED/BLUE mirroring in JUnit before touching carpet. Then A/B the mirrored endpoints on carpet before using it in a match routine.
8. Verify teleop fault containment on the robot once: in a throwaway teleop, bind a button to a command whose `setExecute` throws. Pressing it should print a `command faults` line in Health telemetry while the drive keeps responding. If the op-mode dies instead, Ivy's exception path changed — fix before competing.
9. For any `ProfiledMotorSubsystem` mechanism: tune kG first (mechanism holds against gravity open-loop), then kV along a slow profile, then kP. Verify the encoder isn't reset between auton and teleop (`zeroEncoderOnInit = false`).
10. Pull logs and read the summary: `make analyze` (or open the `.wpilog` in AdvantageScope for the full channel view). Check loop-rate percentiles, phase maxima, battery sag, follower error, and the event timeline.
