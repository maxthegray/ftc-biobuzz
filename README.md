# ftc-starter

Season-agnostic starter for FIRST Tech Challenge. Fork this at the start of
every season, rename the OpModes, and focus on the game, not the plumbing.

Built on:

| Layer              | Library                                      | Version |
|--------------------|----------------------------------------------|---------|
| Robot controller   | `FtcRobotController` (FTC SDK)               | 11.1.0  |
| Path following     | [Pedro Pathing](https://pedropathing.com/)   | 2.1.1   |
| Telemetry adapter  | Pedro Telemetry                              | 1.0.0   |
| Dashboard / tuning | [FTControl Panels](https://panels.bylazar.com/) (`fullpanels`) | 1.0.12  |
| Hot reload         | [Sloth](https://docs.dairy.foundation/sloth/) (Dairy Foundation / Sinister) | 0.2.4 |
| Odometry           | goBILDA Pinpoint (2× dead-wheel + IMU)       | —       |
| Language           | Kotlin                                       | 2.0.21  |

## Quick start

### Without a robot

1. Install Android Studio and a JDK 17 distribution.
2. Open this project in Android Studio, or verify it from a terminal:

   ```
   make test
   make build
   ```

3. Read [ADOPTING.md](ADOPTING.md) before you touch framework code — it covers
   what the starter assumes and the files you'll actually edit first.

### On a robot

1. Make sure the Control Hub's active Configuration has these hardware names:
   - Motors: `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`, `backRightMotor`
   - Pinpoint I²C: `pinpoint`
2. Build and push to the Robot Controller, then run **Starter: Drive Only** to
   check the drivetrain and telemetry work.
3. With the robot running, open the Panels dashboard at
   `http://192.168.43.1:8001` for live pose and tuning knobs.
4. Walk through [BRINGUP.md](BRINGUP.md) before you trust it on the field.

## Repo layout

```
TeamCode/src/main/
├── java/org/firstinspires/ftc/teamcode/pedroPathing/
│   └── Constants.java                 # Pedro's required config path.
│                                      # Physical constants + createFollower().
└── kotlin/org/firstinspires/ftc/teamcode/
    ├── core/
    │   ├── command/        # Scheduler + Command/Commands/Groups (instance-scoped)
    │   ├── control/        # TrapezoidProfile, PIDF, ProfiledController
    │   ├── estimation/     # PoseEstimator, WallSnap (correction math)
    │   ├── geometry/       # Pose2d, Vector2d, angle utils (framework frame)
    │   ├── hardware/       # SRSHub wrapper + optional I2C bus thread
    │   ├── hw/             # MotorIO seam (real + sim, replayable)
    │   ├── logging/        # WPILOG flight recorder + scheduler introspection
    │   ├── pathing/        # PathDSL, chaseTarget, PedroAutoRunner
    │   ├── runtime/        # Robot, OpModeBase, SubsystemBase, selector/config
    │   ├── subsystems/     # ProfiledMotorSubsystem (lift/arm base)
    │   │   ├── drive/      # MecanumDriveSubsystem, DriveConfig
    │   │   └── localization/ # LocalizerSubsystem, LocalizerConfig, PinpointDirect
    │   └── util/           # Alliance, GamepadEx, TelemetryBag, triggers
    └── opmodes/            # Starter teleop + auton examples
```

## Writing season code

Teleops extend `TeleOpBase` (wire subsystems + bindings in `configureTeleop()`);
autons extend `OpModeBase` — copy `ExampleAuto`. Subsystems, commands, paths,
config, and auton are covered end-to-end in [SEASON-GUIDE.md](SEASON-GUIDE.md);
`DriveOnlyTeleOp` and `ExampleAuto` are the copyable skeletons.

## Logs and verification

`make test` / `make build` run the host tests and the JDK-17 debug assemble.
Every op-mode writes a WPILOG under `/sdcard/FIRST/logs`; `make pull-logs` then
open the newest `.wpilog` in AdvantageScope, or `make analyze` for a summary.

## Where to tune what

| I want to change…                               | Edit this                                                   |
|-------------------------------------------------|-------------------------------------------------------------|
| Motor physics (mass, zero-power accel, etc.)    | `pedroPathing/Constants.java`                               |
| Motor / sensor hardware names                   | `core/runtime/RobotConfig.kt`                               |
| Teleop feel (scaling, precision, field-centric) | `core/subsystems/drive/DriveConfig.kt`                      |
| Field length for alliance mirroring             | `core/runtime/RobotConfig.kt`                               |
| Path constraints (max velocity, etc.)           | `pedroPathing/Constants.pathConstraints`                    |

## Further reading

- Architecture decisions and lifecycle timing: [ARCHITECTURE.md](ARCHITECTURE.md)
- Adoption assumptions and first edits: [ADOPTING.md](ADOPTING.md)
- Writing season code on the framework: [SEASON-GUIDE.md](SEASON-GUIDE.md)
- Competition symptom → log channel triage: [RUNBOOK.md](RUNBOOK.md)
- Robot bring-up checklist: [BRINGUP.md](BRINGUP.md)
- Season fork guide: [FORKING.md](FORKING.md)
- Guidance for AI-assisted edits: [CLAUDE.md](CLAUDE.md)
