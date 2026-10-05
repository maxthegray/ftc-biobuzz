# ftc-biobuzz

BioBuzz's FTC robot code, built with Pedro Pathing, Ivy, Panels, and Sloth.

## Quick start

Requires JDK 17.

```sh
make test       # host tests
make build      # debug APK
make install    # full APK install
make hot        # TeamCode hot reload
make debug      # pull and analyze recent match logs
```

Use `make install` for the first deploy of a session and after changing
dependencies, the manifest, resources, or files outside TeamCode. Follow the
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
