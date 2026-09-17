# ftc-biobuzz

Robot code for BioBuzz's FTC season — a mecanum robot with goBILDA Pinpoint
localization, [Pedro Pathing 3](https://pedropathing.com/docs/pathing) with
AutoTune, the [Ivy](https://pedropathing.com/docs/ivy) command scheduler,
Panels telemetry, WPILOG flight recording for AdvantageScope, and Sloth hot
reload.

Built on [`ftc-starter`](https://github.com/maxthegray/ftc-starter), a
season-agnostic base that gets re-forked every year. That repo stays clean;
this one is where the actual season happens.

## Start here

On your machine (JDK 17):

```sh
make test
make build
```

On the robot:

1. Name every device in the Driver Station "Configure Robot" screen exactly
   as below — Preflight fails at init with the missing name otherwise:

   | Name | Device | Type | Used by |
   |---|---|---|---|
   | `frontLeftMotor` | drive motor | Motor | Drive Only, and every op-mode (drivetrain) |
   | `frontRightMotor` | drive motor | Motor | Drive Only, and every op-mode (drivetrain) |
   | `backLeftMotor` | drive motor | Motor | Drive Only, and every op-mode (drivetrain) |
   | `backRightMotor` | drive motor | Motor | Drive Only, and every op-mode (drivetrain) |
   | `pinpoint` | goBILDA Pinpoint | I2C, `GoBildaPinpointDriver` | Drive Only, and every op-mode (localization) |
   | `limelight` | Limelight 3A | Ethernet device | Limelight AprilTag Test |
   | `ballCamera` | goBILDA Global Shutter USB Camera (3122-0004-0001) | Webcam | Ball Tracking Test |

   The four motor names and `pinpoint` are defined in
   [`RobotConfig.kt`](TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/core/runtime/RobotConfig.kt)
   — change them there, not just on the Driver Station. `limelight` and
   `ballCamera` are defaults on `LimelightSubsystem`/`BallCameraSubsystem`
   instead, since vision hardware names aren't identity in the same sense.
   `srsHub` (`SRSHubSubsystem`'s default name) is wired only in the archived
   SRS Loop Benchmark diagnostic, not on the current sensorbot config.
2. Do a full APK install the first time (`make install`).
3. Work through [OPERATIONS.md](OPERATIONS.md) for bring-up and AutoTune.
   Foresight (Pedro's path follower) is **not tuned yet**: driving works, but
   paths refuse to run until AutoTune's output is in `pedro/Constants.java`.

While the robot is on: Panels at `http://192.168.43.1:8001`, AutoTune at
`http://192.168.43.1:10158`.

Enabled Driver Station op-modes:

| OpMode | Purpose |
|---|---|
| Drive Only | Manual driving and drivetrain checks |
| Ball Tracking Test | USB ball camera tuning and diagnostics |
| Limelight AprilTag Test | AprilTag diagnostics |

`opmodes/archived/` holds disabled bring-up utilities (Framework Smoke Test,
Motor Direction Test, Panels Motor Spin, SRS Loop Benchmark) and the old
Limelight Ball Follow prototype. `opmodes/skeletons/` holds the disabled
Example Auto and Localization Test. Remove `@Disabled` from a specific op-mode
and rebuild when you need it.

## Documentation

- [DEVELOPMENT.md](DEVELOPMENT.md) — add a subsystem, bind a button, write an
  auto, log a value, open a log in AdvantageScope
- [OPERATIONS.md](OPERATIONS.md) — bring-up, AutoTune, logs, symptoms, and the
  physical validation checklist
- [PROGRESS.md](PROGRESS.md) — notes from lab testing, with numbers
- [AI-GUIDE.md](AI-GUIDE.md) — the full framework contract, written for AI assistants

`AGENTS.md` and `CLAUDE.md` just point at the AI guide.

## Main files

| Task | Start here |
|---|---|
| TeleOp | `opmodes/DriveOnlyTeleOp.kt`, `opmodes/TeleOpBase.kt` (`configureTeleop()`) |
| Autonomous | `opmodes/skeletons/ExampleAuto.kt` |
| Buttons | `core/util/GamepadEx.kt`, `Trigger.kt` |
| Drive and drive commands | `core/subsystems/drive/MecanumDriveSubsystem.kt`, `DriveConfig.kt` |
| Localization and vision corrections | `core/subsystems/localization/` |
| Hardware names, field size, config schema | `core/runtime/RobotConfig.kt` |
| Pedro constants and AutoTune | `pedro/Constants.java`, `pedro/Tuning.java` |
| Flight recorder | `core/logging/FlightRecorder.kt` |
| Vision diagnostics | `opmodes/diagnostics/`, `vision/`, then `OPERATIONS.md` §8 |
| Diagnose a run | `make debug`, then `OPERATIONS.md` |

Paths are relative to `TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/`,
except `pedro/`, which lives under the Java source root
(`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`).

## Repository map

```text
TeamCode/src/main/
├── java/org/firstinspires/ftc/teamcode/pedro/
│   ├── Constants.java        Pedro 3 drivetrain, Pinpoint and Foresight config
│   ├── Tuning.java           AutoTune procedure registration
│   └── procedures/           AutoTune procedures (copied from the Pedro Quickstart)
└── kotlin/org/firstinspires/ftc/teamcode/
    ├── core/
    │   ├── control/          PIDF
    │   ├── estimation/       latency-compensated pose correction
    │   ├── hardware/         SRSHub
    │   ├── io/               motor abstraction seam
    │   ├── logging/          WPILOG writer, flight recorder, Panels field view
    │   ├── runtime/          Robot, OpModeBase, SubsystemBase, config
    │   ├── subsystems/       drive, localization, Limelight
    │   └── util/             gamepads, triggers, alliance, telemetry
    ├── vision/               season vision: tag catalog, ball camera, assists
    └── opmodes/              teleop, diagnostics, skeletons, archived
```

`core/` is season- and chassis-agnostic, and it's what gets cherry-picked back
to `ftc-starter`. Season mechanisms go in `subsystems/`, not
`core/subsystems/`.

## How I work in here

Small things go straight to `master`; branch when something would leave the
robot undrivable for a while. Tag at every competition:

```sh
git tag -a quals-2026-11-14 -m "what ran at quals"
```

### The sensorbot

Right now this runs on a **sensorbot**, a temporary chassis. The real robot
replaces it in one commit:

1. Re-run AutoTune and replace the values in `pedro/Constants.java`.
2. Fix the hardware names in `core/runtime/RobotConfig.kt`.
3. Bump `RobotConfig.CONFIG_SCHEMA`, so the sensorbot's tuning file on the
   Control Hub is ignored instead of silently loading onto a heavier robot.

Tag `sensorbot-final` before the swap.

### Sending fixes back to ftc-starter

```sh
git remote add upstream https://github.com/maxthegray/ftc-starter.git
git fetch upstream
git cherry-pick <sha>
```

Only `core/` changes make the trip.

## Daily commands

```sh
make test       # host tests
make build      # debug APK
make install    # full APK install
make hot        # TeamCode-only Sloth reload
make debug      # newest match logs + JSON diagnosis
```

Do a full install after touching dependencies, the manifest, `res/`,
`@Pinned` classes, or anything outside TeamCode — Sloth won't pick those up,
and it fails silently.

Versions are pinned on purpose: FTC SDK 11.2.1, Kotlin 2.0.21, Pedro Pathing
3.0.0 (+ AutoTune 1.0.0), Ivy 1.1.1, Panels 0.2.4+1.0.12, Sloth 0.2.4.
`AI-GUIDE.md` explains the constraints that hold Sloth and the Kotlin stdlib in
place. Check the artifact exists in its real repository before bumping any of
them.
