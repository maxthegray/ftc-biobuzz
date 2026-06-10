# ftc-starter

Season-agnostic starter for FIRST Tech Challenge. Fork this at the start of
every season, rename the OpModes, and focus on the game — not the plumbing.

Built on:

| Layer              | Library                                      | Version |
|--------------------|----------------------------------------------|---------|
| Robot controller   | `FtcRobotController` (FTC SDK)               | 11.1.0  |
| Path following     | [Pedro Pathing](https://pedropathing.com/)   | 2.1.1   |
| Command framework  | Pedro **Ivy**                                | 1.0.0   |
| Telemetry adapter  | Pedro Telemetry                              | 1.0.0   |
| Dashboard / tuning | [FTControl Panels](https://panels.bylazar.com/) (`fullpanels`) | 1.0.12  |
| Hot reload         | [Sloth](https://docs.dairy.foundation/sloth/) (Dairy Foundation / Sinister) | 0.2.4 |
| Odometry           | goBILDA Pinpoint (2× dead-wheel + IMU)       | —       |
| Language           | Kotlin                                       | 2.0.21  |

## Quick start

1. Open the project in Android Studio Giraffe+ / Hedgehog+.
2. Make sure the Control Hub's active Configuration has these hardware names:
   - Motors: `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`, `backRightMotor`
   - Pinpoint I²C: `pinpoint`
3. Build & push to the Robot Controller. Select the op-mode
   **Starter: Drive Only** to sanity-check the drivetrain and telemetry.
4. Open the Panels dashboard at `http://192.168.43.1:8001` while the robot
   is running to see live pose and tuning knobs.

## Repo layout

```
TeamCode/src/main/
├── java/org/firstinspires/ftc/teamcode/pedroPathing/
│   └── Constants.java                 # Pedro's required config path.
│                                      # Physical constants + createFollower().
└── kotlin/org/firstinspires/ftc/teamcode/
    ├── core/
    │   ├── control/        # TrapezoidProfile, PIDF, ProfiledController
    │   ├── hardware/       # SRSHub wrapper + optional I2C bus thread
    │   ├── logging/        # WPILOG flight recorder + scheduler introspection
    │   ├── pathing/        # PathDSL, chaseTarget, PedroAutoRunner
    │   ├── runtime/        # Robot, OpModeBase, SubsystemBase, selector/config
    │   ├── subsystems/     # ProfiledMotorSubsystem (lift/arm base)
    │   │   ├── drive/      # MecanumDriveSubsystem, DriveConfig
    │   │   └── localization/ # LocalizerSubsystem, LocalizerConfig, PinpointDirect
    │   └── util/           # Alliance, GamepadEx, TelemetryBag, triggers
    └── opmodes/            # Starter teleop examples
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
Lynx bulk-read mode, Ivy scheduler ticking, gamepad edge detection, joined
Driver Station + Panels telemetry flushing.

## Writing a path

The `path { }` DSL sits over Pedro's `PathBuilder`:

```kotlin
val toScore = drive.path(Pose(9.0, 60.0, 0.0), alliance = Alliance.RED) {
    lineTo(Pose(30.0, 60.0))
    splineTo(Pose(38.0, 58.0), Pose(48.0, 40.0, Math.toRadians(-45.0)))
    constantHeading(Math.toRadians(-45.0))
}
```

And a full auton routine via `PedroAutoRunner` / the `autoRoutine { }` DSL:

```kotlin
val runner = autoRoutine(drive) {
    follow(toScore)
    run { /* call a season-specific subsystem */ }
    wait(300)
    parallel {
        follow(toStack)
        run { /* call a season-specific subsystem */ }
    }
    holdPose(Pose(36.0, 36.0, 0.0))
}
// in onStart:
runner.schedule()
// in onLoop:
if (runner.isDone) requestOpModeStop()
```

## Logs and verification

Run the host tests and Android debug assemble with JDK 17:

```
JAVA_HOME="/Users/maximilianreich/Library/Java/JavaVirtualMachines/corretto-17.0.13/Contents/Home" \
  ./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
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
- Robot bring-up checklist: [BRINGUP.md](BRINGUP.md)
- Season fork guide: [FORKING.md](FORKING.md)
- Guidance for AI-assisted edits: [CLAUDE.md](CLAUDE.md)
