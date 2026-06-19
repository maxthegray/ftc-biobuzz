# Adopting This Starter

For teams deciding whether to fork this repo, and for new programmers trying to
find the right first file to edit.

## What this starter assumes

- FTC SDK 11.1.0, Android Studio, and JDK 17.
- Kotlin for team code. Java is still present where the FTC SDK or Pedro
  expects it.
- Mecanum drivetrain.
- goBILDA Pinpoint localization with two dead wheels and IMU.
- Pedro Pathing for path following.
- FTControl Panels for dashboard telemetry and tuning.
- Sloth hot reload for fast TeamCode iteration.
- Control Hub hardware names from [README.md](README.md): four drive motors
  and a Pinpoint device named `pinpoint`.

If your robot's different, that's fine — just expect your first work to land in
the drive, localization, and config layers, not only in op-modes.

## First 30 minutes

1. Clone or fork the repo.
2. Install Android Studio and JDK 17.
3. Run:

   ```
   make test
   make build
   ```

4. Open the project in Android Studio.
5. Read the "Usually edit" table below.
6. On hardware, run `Starter: Drive Only` and follow [BRINGUP.md](BRINGUP.md)
   in order.

## Usually edit

| Goal | Start here |
|------|------------|
| Create a teleop | `TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/opmodes/DriveOnlyTeleOp.kt` |
| Create an autonomous | `TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/opmodes/ExampleAuto.kt` |
| Add an intake, lift, arm, or shooter | `SEASON-GUIDE.md`, then a new subsystem under `core/subsystems` or a season package |
| Bind buttons | `configureTeleop()` in your teleop |
| Tune drive feel | `core/subsystems/drive/DriveConfig.kt` |
| Change motor/sensor names | `core/runtime/RobotConfig.kt` |
| Tune Pedro constants | `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java` |
| Diagnose a bad run | `RUNBOOK.md`, `make analyze`, and the newest `.wpilog` |

## Rarely edit at first

| Area | Why |
|------|-----|
| `core/command` | Scheduler behavior is shared by every subsystem and is covered by tests. |
| `core/runtime/Robot.kt` / `OpModeBase.kt` | This owns lifecycle ordering, logging, bulk reads, and fault policy. |
| `core/logging` | Logs are part of the team's debugging contract. Add channels from subsystems before changing the writer. |
| `FtcRobotController` | Treat it as SDK-owned unless you are updating SDK integration. |

## Copyable workflow

For a normal season feature:

1. Add or update a subsystem.
2. Register it in your teleop or autonomous `configure()` method.
3. Give commands clear names with `setName(...)`.
4. Log goals, measurements, and outputs in `logState(...)`.
5. Add a focused host test when the logic can run without the Control Hub.
6. Run `make test`.
7. Use `make hot` for TeamCode-only iteration, and `make install` after
   dependency, manifest, or pinned-class changes.

## Where to learn next

- [SEASON-GUIDE.md](SEASON-GUIDE.md): how to build season code on the framework.
- [BRINGUP.md](BRINGUP.md): physical robot checklist.
- [RUNBOOK.md](RUNBOOK.md): match-day debugging.
- [ARCHITECTURE.md](ARCHITECTURE.md): why the framework is shaped this way.
- [FORKING.md](FORKING.md): keeping a reusable starter separate from season code.
