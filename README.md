# ftc-biobuzz

BioBuzz's FTC robot code, built with Pedro Pathing, Ivy, Panels, and Sloth.

## Quick start

Requires JDK 17.

```sh
make help              # list all commands (also the default for `make`)

make test              # run all host tests (TeamCode, log tools, MaxScope)
make build             # build the debug APK
make clean             # remove Gradle build outputs
make deploy            # deploy, picking full install or hot reload automatically
make install           # force a full APK install
make hot               # force a TeamCode hot reload via Sloth

make connect           # connect to the Control Hub over Wi-Fi
make reset-adb         # stop the ADB server when it gets stuck
make logs              # stream filtered robot logcat

make pull-logs         # download every log the hub keeps (newest 30) to robot-logs/
make pull-lab-records  # download vision lab records to lab-records/
make analyze           # pull the newest match logs (if connected) and summarize
make debug             # same as analyze, as a JSON diagnosis
make viewer            # start MaxScope at http://127.0.0.1:8008
```

Use `make deploy` by default. It does a full install on the first deploy of a
session and after changes to dependencies, the manifest, resources, or files
outside TeamCode, and a hot reload otherwise. Follow the
[operations guide](dutchdocs/OPERATIONS.md) for hardware setup and AutoTune.
Paths require Foresight tuning before they can run.

## Code

- [TeamCode](TeamCode/src/main/) — robot code; `opmodes/`, `subsystems/`, and
  `vision/` under Kotlin, Pedro configuration under Java
- [core](TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/core/) — shared framework

## Docs

- [Development](dutchdocs/DEVELOPMENT.md) — subsystems, controls, autos, logging
- [Operations](dutchdocs/OPERATIONS.md) — hardware, tuning, diagnostics
- [Progress](dutchdocs/PROGRESS.md) — lab notes
- [AI guide](AI-GUIDE.md) — framework contract for coding assistants
