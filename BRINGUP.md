# Bring-Up Checklist

Use this after a fresh fork or hardware rebuild, before trusting autonomous.

1. Confirm Driver Station configuration names: `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`, `backRightMotor`, and Pinpoint `pinpoint`.
2. Run a per-motor spin test from the FTC SDK examples or a short throwaway op-mode. Verify each wheel name and direction before enabling path following.
3. In `Starter: Drive Only`, check forward/back, strafe left/right, and turn sign. The teleop sign conventions in `MecanumDriveSubsystem` are framework defaults, not yet verified on every chassis.
4. Toggle field-centric mode and rotate the robot by hand. Confirm the field-centric rotation sense matches driver expectation.
5. Run Pedro `Tuning` for Pinpoint axis checks. Explicitly verify the encoder directions in `pedroPathing/Constants.java`.
6. Run `Starter: Localization Test`. Drive a slow lap and compare final displayed pose against the field.
7. Build a tiny mirrored RED/BLUE path pair in a season fork and A/B the endpoints on carpet before using it in a match routine.
8. Pull logs with `make pull-logs`; open the newest `.wpilog` in AdvantageScope and inspect pose, velocity, loop phases, battery, and command state.
