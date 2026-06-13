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

3. Read [ADOPTING.md](ADOPTING.md) before changing framework code. It lists
   the assumptions this starter makes and the files a new team usually edits.

### On a robot

1. Make sure the Control Hub's active Configuration has these hardware names:
   - Motors: `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`, `backRightMotor`
   - Pinpoint I²C: `pinpoint`
2. Build and push to the Robot Controller. Select the op-mode
   **Starter: Drive Only** to sanity-check the drivetrain and telemetry.
3. Open the Panels dashboard at `http://192.168.43.1:8001` while the robot
   is running to see live pose and tuning knobs.
4. Work through [BRINGUP.md](BRINGUP.md) before trusting autonomous.

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

## Writing a new op-mode

Teleops extend `TeleOpBase` — it registers drive + localizer, installs the
stick-driven teleop default command, restores the persisted auton pose, and
wires the standard chords (Back+Y heading reset, Back+B field-centric):

```kotlin
@TeleOp(name = "Match Teleop", group = "Competition")
class MatchTeleop : TeleOpBase() {
    override fun configureTeleop() {
        // register season subsystems and trigger bindings here...
    }

    override fun onLoop() {
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("loopHz", robot.loopHz, decimals = 1)
        }
    }
}
```

Autons extend `OpModeBase` directly — copy `ExampleAuto` as the skeleton
(alliance/routine/delay selection on dpad in init, paths mirrored from RED
coordinates, sequencing via `autoRoutine`). `OpModeBase` handles the rest:
Lynx bulk-read mode, command scheduler ticking, gamepad edge detection, joined
Driver Station + Panels telemetry flushing.

## Writing a path

The `path { }` DSL sits over Pedro's `PathBuilder`:

```kotlin
val toScore = drive.path(startPose = Pose2d(9.0, 60.0, 0.0), alliance = Alliance.RED) {
    lineTo(Pose2d(30.0, 60.0))
    splineTo(Pose2d(38.0, 58.0), Pose2d(48.0, 40.0))
    constantHeading(Math.toRadians(-45.0))
}
```

And a full auton routine via `PedroAutoRunner` / the `autoRoutine { }` DSL
(pass `robot::recordEvent` to get a labelled step timeline in the flight log):

```kotlin
val runner = autoRoutine(robot, drive, robot::recordEvent) {
    follow(toScore)
    run { /* call a season-specific subsystem */ }
    wait(300)
    parallel {
        follow(toStack)
        run { /* call a season-specific subsystem */ }
    }
    holdPose(Pose2d(36.0, 36.0, 0.0))
}
// in onStart:
runner.schedule()
// in onLoop:
if (runner.isDone) requestOpModeStop()
```

## Logs and verification

Run the host tests and Android debug assemble with JDK 17:

```
make test
make build
```

Every op-mode writes a WPILOG flight-recorder file under
`/sdcard/FIRST/logs`. Pull logs with `make pull-logs`, then open the newest
`.wpilog` in AdvantageScope.

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
