# Forking For A Season

Keep this starter season-agnostic. A season fork adds game-specific subsystems, op-modes, and paths under `TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode`.

If you are evaluating the repo for the first time, start with
[ADOPTING.md](ADOPTING.md). This file is about keeping a fork healthy after
you have decided to use the starter.

## Where Season Code Goes

- Subsystems: extend `SubsystemBase`, resolve hardware in `init`, read in `periodic`, command outputs from scheduler commands, and write final actuator state in `writeHardware`.
- Teleop: extend `TeleOpBase` (copy `DriveOnlyTeleOp`), register season subsystems and bind controls once with `GamepadEx` triggers in `configureTeleop()`. Autons extend `OpModeBase` and use `configure()`.
- Autonomous: build paths with `PathDSL` and compose routines with `PedroAutoRunner`. The priority ladder is defaults `0`, autonomous routines/assists `10`, driver actions `20`, and panic/manual overrides `30`.
- Vision: add camera pipelines in the season fork and feed accepted field-pose measurements through `LocalizerSubsystem.applyCorrection`.

## Config And Hot Reload

Live-tuned values go through `ConfigStore`, not `@Pinned` — register the config
object in `configure()` and tuned values survive reloads, installs, and power
cycles while the code stays hot-reloadable (see SEASON-GUIDE.md "Config
objects"). `PersistedPose` is the only `@Pinned` class.

Change `RobotConfig.CONFIG_SCHEMA` when creating the season fork. ConfigStore
ignores a tuning file written under a different schema, so the new season
starts from its compiled defaults instead of silently inheriting old values.

Use `make hot` / `deploySloth` for ordinary subsystem and op-mode iteration; a
full install after dependency, manifest, `@Pinned`, or non-TeamCode changes.

## Path Rendering Limitation

Pedro 2.1.1 `PathBuilder` has only `Follower`-based constructors, so this starter cannot create a fully hardware-free desktop path renderer without leaning on a fake follower. Keep path symmetry tests focused on pose mirroring in this repo; add richer SVG rendering in a season fork only if Pedro exposes a hardware-free builder or the fork accepts a fake-follower test seam.
