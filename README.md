# ftc-biobuzz

BioBuzz's robot code for the current FTC season: a mecanum robot with goBILDA
Pinpoint localization, Pedro Pathing, Panels telemetry, WPILOG recording, and
Sloth hot reload.

Forked from [`maxthegray/ftc-starter`](https://github.com/maxthegray/ftc-starter),
our season-agnostic base. See [Repository](#repository) for how the two relate
and how the sensorbot fits in.

## Start here

Workstation:

```sh
make test
make build
```

Robot:

1. Use Control Hub names `frontLeftMotor`, `frontRightMotor`,
   `backLeftMotor`, `backRightMotor`, and `pinpoint`.
2. Do a full APK install for the first deployment.
3. Run `BioBuzz: Framework Smoke Test` to verify the Control Hub, lifecycle,
   gamepad, scheduler, telemetry, and WPILOG pipeline without configured hardware.
4. Follow [OPERATIONS.md](OPERATIONS.md) from the motor-direction test through
   Pedro calibration. The shipped Pedro numbers are placeholders.

The Panels dashboard is available at `http://192.168.43.1:8001` while the
robot is running.

## Documentation

- [DEVELOPMENT.md](DEVELOPMENT.md) — adding subsystems, commands, config,
  autonomous routines, and sensors
- [OPERATIONS.md](OPERATIONS.md) — hardware bring-up, Pedro tuning, logs, and
  symptom diagnosis
- [PROGRESS.md](PROGRESS.md) — running notes from actual lab testing
- [AI-GUIDE.md](AI-GUIDE.md) — complete framework contract for AI assistants

`AGENTS.md` and `CLAUDE.md` are discovery pointers to the same AI guide.

## Main files

| Task | Start here |
|---|---|
| TeleOp | `opmodes/DriveOnlyTeleOp.kt`, then override `configureTeleop()` |
| Autonomous | `opmodes/ExampleAuto.kt` |
| Mechanism | `core/subsystems/ProfiledMotorSubsystem.kt` |
| Buttons and triggers | `core/util/GamepadEx.kt`, `Trigger.kt` |
| Drive feel | `core/subsystems/drive/DriveConfig.kt` |
| Localization and vision corrections | `core/subsystems/localization/` |
| Hardware names, field size, config schema | `core/runtime/RobotConfig.kt` |
| Pedro calibration | `pedroPathing/Constants.java` |
| Diagnose a run | `make debug`, then `OPERATIONS.md` |

Paths above are relative to
`TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/` except
`pedroPathing/Constants.java`, which is under the Java source root.

## Repository map

```text
TeamCode/src/main/
├── java/org/firstinspires/ftc/teamcode/pedroPathing/
│   ├── Constants.java
│   └── Tuning.java
└── kotlin/org/firstinspires/ftc/teamcode/
    ├── core/
    │   ├── command/          scheduler, commands, groups
    │   ├── control/          profiles and PIDF
    │   ├── estimation/       pose correction
    │   ├── geometry/         framework pose/vector types
    │   ├── hardware/         SRSHub and optional I²C thread
    │   ├── io/               real motor abstraction seam
    │   ├── logging/          WPILOG and field view
    │   ├── pathing/          path DSL and auton runner
    │   ├── runtime/          robot lifecycle and config
    │   ├── subsystems/       mechanisms, drive, localization
    │   └── util/             gamepads, triggers, telemetry
    ├── subsystems/           season mechanisms (this year's game)
    └── opmodes/              diagnostics, teleop, auton
```

`core/` is season- and chassis-agnostic and flows back upstream to
`ftc-starter`. Everything outside it is this season's. Season mechanisms go in
`subsystems/`, **not** `core/subsystems/`.

## Repository

### Robots

The code currently runs on the **sensorbot** — a temporary chassis to develop
against while the competition robot is built. The competition robot replaces it
in place; there is no sensorbot branch and no robot-profile switch.

On swap day, in one commit:

1. Re-run Pedro's tuners and replace every number in `pedroPathing/Constants.java`.
2. Update hardware names in `core/runtime/RobotConfig.kt`.
3. Bump `RobotConfig.CONFIG_SCHEMA`. The Control Hub usually moves between
   chassis and carries `/sdcard/FIRST/config/tuning.properties` with it; the
   schema bump is what stops the sensorbot's tuning from silently loading onto
   a much heavier robot.

Tag the last sensorbot commit first — `git tag -a sensorbot-final` — so the
old calibration stays recoverable without a branch to maintain.

### Branches and tags

`main` is protected and always deployable; it is what gets flashed at a meet.
Work happens on short-lived `feat/…` branches merged through PRs, which CI
builds and tests. There is no `develop`/`beta` branch.

Tag at every competition (`git tag -a quals-2026-11-14`). That is what answers
"what exactly was on the robot when it worked."

### Upstream

`ftc-starter` is the season-agnostic base this repo was forked from. It shares
history, so framework fixes cherry-pick cleanly in both directions:

```sh
git remote add upstream https://github.com/maxthegray/ftc-starter.git
git fetch upstream
git cherry-pick <sha>        # send a core/ fix back, or pull one down
```

Only `core/` changes travel upstream. Season code stays here.

## Daily commands

```sh
make test       # host tests
make build      # debug APK
make install    # full APK install
make hot        # TeamCode-only Sloth reload
make debug      # newest match logs + JSON diagnosis
```

Use a full install after dependency, manifest, `@Pinned`, or non-TeamCode
changes. Ordinary TeamCode iteration can use hot reload.

Pinned versions are FTC SDK 11.1.0, Kotlin 2.0.21, Pedro 2.1.1, Panels
1.0.12, and Sloth 0.2.4. Verify artifacts in their real repositories before
changing any version.
