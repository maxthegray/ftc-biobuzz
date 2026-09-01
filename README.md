# ftc-biobuzz

Robot code for BioBuzz's FTC season — a mecanum robot with goBILDA Pinpoint
localization, Pedro Pathing, Panels telemetry, WPILOG flight recording, and
Sloth hot reload.

Built on [`ftc-starter`](https://github.com/maxthegray/ftc-starter), a
season-agnostic base that gets re-forked every year. That repo stays clean;
this one is where the actual season happens.

If you wandered in from a search: the reusable half is `core/`. There's a
command scheduler with a real requirements system, a trapezoidal-profile +
PIDF mechanism toolkit, WPILOG logging you can open in AdvantageScope, and a
Pedro Pathing wrapper that keeps Pedro's API behind an adapter layer. Most of
it runs headless in JUnit, so you can poke at it without a robot on the desk.

## Start here

On your machine:

```sh
make test
make build
```

On the robot:

1. Name things `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`,
   `backRightMotor`, and `pinpoint` in the Driver Station config.
2. Do a full APK install the first time.
3. Run `BioBuzz: Framework Smoke Test`. It needs no configured hardware and
   checks the Control Hub, lifecycle, gamepad, scheduler, telemetry, and
   WPILOG pipeline in one go.
4. Then work through [OPERATIONS.md](OPERATIONS.md) from the motor-direction
   test to Pedro calibration. The Pedro numbers in here are placeholders until
   you measure your own chassis — don't run auton before that.

Panels is at `http://192.168.43.1:8001` while the robot is on.

## Documentation

- [DEVELOPMENT.md](DEVELOPMENT.md) — subsystems, commands, config, auton, sensors
- [OPERATIONS.md](OPERATIONS.md) — bring-up, Pedro tuning, logs, diagnosing symptoms
- [PROGRESS.md](PROGRESS.md) — notes from actual lab testing, with numbers
- [AI-GUIDE.md](AI-GUIDE.md) — the full framework contract, written for AI assistants

`AGENTS.md` and `CLAUDE.md` just point at the AI guide.

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

Paths are relative to
`TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/`, except
`pedroPathing/Constants.java` which lives under the Java source root.

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

`core/` is season- and chassis-agnostic, and it's what gets cherry-picked back
to `ftc-starter`. Everything outside it belongs to this season. Season
mechanisms go in `subsystems/`, not `core/subsystems/` — that split is the only
thing keeping the upstream merges from becoming manual work.

## How I work in here

It's just me, so there isn't much process. Small things go straight to
`master`. I branch when something's going to take a while and would leave the
robot un-drivable in the meantime.

The one habit worth keeping is tagging at every competition:

```sh
git tag -a quals-2026-11-14 -m "what ran at quals"
```

At 11pm before a meet the only question that matters is "what exactly was on
the robot last time it worked," and a tag answers it for free.

### The sensorbot

Right now this runs on a **sensorbot** — a throwaway test chassis I code
against while the real robot gets built. It's temporary and I'm not going back
to it, so there's no sensorbot branch and no robot-profile switching. The real
robot just replaces it.

When it does, in one commit:

1. Re-run Pedro's tuners and replace every number in `pedroPathing/Constants.java`.
2. Fix the hardware names in `core/runtime/RobotConfig.kt`.
3. Bump `RobotConfig.CONFIG_SCHEMA`.

Step 3 is the easy one to forget and the miserable one to debug. The Control
Hub usually moves to the new robot and brings
`/sdcard/FIRST/config/tuning.properties` along with it, so the sensorbot's
drive tuning would quietly load onto a robot several times heavier — plausible
numbers, no error, just bad. Bumping the schema makes `ConfigStore` ignore the
stale file and fall back to compiled defaults.

I tag `sensorbot-final` before the swap so the old calibration stays
recoverable without keeping a branch alive for it.

### Sending fixes back to ftc-starter

`ftc-starter` shares history with this repo, so commits cherry-pick cleanly in
both directions:

```sh
git remote add upstream https://github.com/maxthegray/ftc-starter.git
git fetch upstream
git cherry-pick <sha>
```

Only `core/` changes make the trip. Season code stays here.

## Daily commands

```sh
make test       # host tests
make build      # debug APK
make install    # full APK install
make hot        # TeamCode-only Sloth reload
make debug      # newest match logs + JSON diagnosis
```

Hot reload is fine for ordinary iteration on subsystems and op-modes. Do a full
install after touching dependencies, the manifest, `@Pinned` classes, or
anything outside TeamCode — Sloth won't pick those up, and it fails silently
rather than telling you.

Versions are pinned on purpose: FTC SDK 11.1.0, Kotlin 2.0.21, Pedro 2.1.1,
Panels 1.0.12, Sloth 0.2.4. Check the artifact actually exists in its real
repository before bumping any of them.
