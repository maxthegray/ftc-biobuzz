# TeamCode Module

TeamCode is where the robot-specific code lives. In this starter, most code is
under `src/main/kotlin/org/firstinspires/ftc/teamcode`; the Java package is
kept for FTC/Pedro compatibility files such as `pedroPathing/Constants.java`.

Start at the repo root:

- [README.md](../../../../../../../../README.md): quick start and repo layout.
- [ADOPTING.md](../../../../../../../../ADOPTING.md): assumptions and first files
  to edit.
- [SEASON-GUIDE.md](../../../../../../../../SEASON-GUIDE.md): how to add
  subsystems, commands, config, and autonomous routines.
- [BRINGUP.md](../../../../../../../../BRINGUP.md): physical robot checklist.

## Common starting points

| Task | File |
|------|------|
| Drive the base robot | `src/main/kotlin/org/firstinspires/ftc/teamcode/opmodes/DriveOnlyTeleOp.kt` |
| Copy an autonomous skeleton | `src/main/kotlin/org/firstinspires/ftc/teamcode/opmodes/ExampleAuto.kt` |
| Tune drive controls | `src/main/kotlin/org/firstinspires/ftc/teamcode/core/subsystems/drive/DriveConfig.kt` |
| Change hardware names | `src/main/kotlin/org/firstinspires/ftc/teamcode/core/runtime/RobotConfig.kt` |
| Tune Pedro constants | `src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java` |

FTC SDK sample op-modes are still available under the `FtcRobotController`
module, but new season code should usually copy from this starter's op-modes
instead of starting from the SDK samples.
