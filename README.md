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
   - Pinpoint I²C: `sensor_otos` (historical name — kept from prior OTOS config)
   - Webcam (optional): `Webcam 1`
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
└── kotlin/org/firstinspires/ftc/teamcode/starter/
    ├── core/           # Robot, OpModeBase, SubsystemBase, Alliance, GamepadEx
    ├── hardware/       # BulkReadManager, I2CBusThread, DeviceReaders
    ├── localization/   # Localizer façade, PinpointLocalizer helper, AprilTagCorrector
    ├── drive/          # MecanumDriveSubsystem, DriveConfig
    ├── pathing/        # PathDSL (Kotlin DSL over PathBuilder), PedroAutoRunner
    ├── vision/         # VisionSubsystem, AprilTagPipeline
    ├── telemetry/      # TelemetryBag (DS + Panels in one call)
    ├── config/         # RobotConfig (hardware name constants)
    └── examples/       # DriveOnlyTeleOp — minimal end-to-end example
```

## Writing a new op-mode

Extend `OpModeBase` and override three hooks:

```kotlin
@TeleOp(name = "Match Teleop", group = "Competition")
class MatchTeleop : OpModeBase() {
    private lateinit var drive: MecanumDriveSubsystem

    override fun configure() {
        val follower = Constants.createFollower(hardwareMap)
        drive = robot.register(MecanumDriveSubsystem(follower))
        // register more subsystems here...
    }

    override fun onStart() {
        drive.enableTeleop()
    }

    override fun onLoop() {
        drive.drive(driver.leftStickY, driver.leftStickX, driver.rightStickX)
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("loopHz", robot.loopHz, decimals = 1)
        }
    }
}
```

`OpModeBase` handles the rest: Lynx bulk-read mode, Ivy scheduler ticking,
gamepad edge detection, joined Driver Station + Panels telemetry flushing.

## Writing a path

The `path { }` DSL sits over Pedro's `PathBuilder`:

```kotlin
val toScore = drive.path(Pose(9.0, 60.0, 0.0)) {
    lineTo(Pose(30.0, 60.0))
    splineTo(Pose(48.0, 40.0, Math.toRadians(-45.0)))
    constantHeading(Math.toRadians(-45.0))
}
```

And a full auton routine via `PedroAutoRunner` / the `autoRoutine { }` DSL:

```kotlin
val runner = autoRoutine(drive) {
    follow(toScore)
    run { intake.score() }
    wait(300)
    parallel {
        follow(toStack)
        run { intake.extend() }
    }
    holdPose(Pose(36.0, 36.0, 0.0))
}
// in onStart:
runner.schedule()
// in onLoop:
if (runner.isDone) requestOpModeStop()
```

## Where to tune what

| I want to change…                               | Edit this                                                   |
|-------------------------------------------------|-------------------------------------------------------------|
| Motor physics (mass, zero-power accel, etc.)    | `pedroPathing/Constants.java`                               |
| Motor / sensor hardware names                   | `pedroPathing/Constants.java` + `starter/config/RobotConfig.kt` |
| Teleop feel (scaling, precision, field-centric) | `starter/drive/DriveConfig.kt`                              |
| AprilTag correction tolerances                  | `starter/localization/AprilTagCorrector.kt` or op-mode      |
| Path constraints (max velocity, etc.)           | `pedroPathing/Constants.pathConstraints`                    |

## Further reading

- Architecture decisions and lifecycle timing: [ARCHITECTURE.md](ARCHITECTURE.md)
- Guidance for AI-assisted edits: [CLAUDE.md](CLAUDE.md)
