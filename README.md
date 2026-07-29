# ftc-starter

Our Kotlin FTC base for a mecanum robot with goBILDA Pinpoint localization,
Pedro Pathing, Panels telemetry, WPILOG recording, and Sloth hot reload.

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
3. Follow [OPERATIONS.md](OPERATIONS.md) from the motor-direction test through
   Pedro calibration. The shipped Pedro numbers are placeholders.

The Panels dashboard is available at `http://192.168.43.1:8001` while the
robot is running.

## Documentation

- [DEVELOPMENT.md](DEVELOPMENT.md) — adding subsystems, commands, config,
  autonomous routines, and sensors
- [OPERATIONS.md](OPERATIONS.md) — hardware bring-up, Pedro tuning, logs, and
  symptom diagnosis
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
    │   ├── estimation/       pose correction and wall snap
    │   ├── geometry/         framework pose/vector types
    │   ├── hardware/         SRSHub and optional I²C thread
    │   ├── hw/               real/sim motor seam
    │   ├── logging/          WPILOG and field view
    │   ├── pathing/          path DSL and auton runner
    │   ├── runtime/          robot lifecycle and config
    │   ├── subsystems/       mechanisms, drive, localization
    │   └── util/             gamepads, triggers, telemetry
    └── opmodes/              starter diagnostics, teleop, auton
```

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
