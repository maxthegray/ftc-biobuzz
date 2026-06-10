# Forking For A Season

Keep this starter season-agnostic. A season fork adds game-specific subsystems, op-modes, and paths under `TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode`.

## Where Season Code Goes

- Subsystems: extend `SubsystemBase`, resolve hardware in `init`, read in `periodic`, command outputs from Ivy commands, and write final actuator state in `writeHardware`.
- Teleop: copy `DriveOnlyTeleOp` or `LocalizationTestTeleOp`, register season subsystems in `configure`, and bind controls once with `GamepadEx` triggers.
- Autonomous: build paths with `PathDSL` and compose routines with `PedroAutoRunner`. Use priority `10` for autonomous actions and driver-triggered actions; defaults stay priority `0`.
- Vision: add camera pipelines in the season fork and feed accepted field-pose measurements through `LocalizerSubsystem.applyCorrection`.

## Config And Hot Reload

Pin `@Configurable` classes with `@dev.frozenmilk.sinister.loading.Pinned` only when live-tuned values must survive Sloth reload. Pinned code changes require a full install.

Use a full Android Studio install after dependency, manifest, pinned-class, or non-TeamCode changes. Use `make hot` / `deploySloth` for ordinary subsystem and op-mode iteration.

## Path Rendering Limitation

Pedro 2.1.1 `PathBuilder` has only `Follower`-based constructors, so this starter cannot create a fully hardware-free desktop path renderer without leaning on a fake follower. Keep path symmetry tests focused on pose mirroring in this repo; add richer SVG rendering in a season fork only if Pedro exposes a hardware-free builder or the fork accepts a fake-follower test seam.
