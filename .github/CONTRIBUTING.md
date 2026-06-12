# Contributing

This repository is an FTC starter, not the official FTC SDK. Contributions
should make the starter easier to adopt, safer on real robots, or easier to
debug during a season.

## Before changing code

- Read [ADOPTING.md](../ADOPTING.md) to understand the starter's assumptions.
- Read [ARCHITECTURE.md](../ARCHITECTURE.md) before changing lifecycle,
  scheduling, logging, localization, or pathing behavior.
- Keep FTC SDK-owned code in `FtcRobotController` as close to upstream as
  practical.
- Prefer season-specific examples and docs over framework changes when the
  behavior only helps one robot.

## Pull request checklist

- Explain the user-facing behavior change.
- List any hardware assumptions or field-safety concerns.
- Add or update docs when the change affects onboarding, bring-up, or tuning.
- Add focused host tests for logic that can run without a Control Hub.
- Run `make test` before asking for review.
- Run `make build` before merging framework or dependency changes.

## Bug reports

Include enough context for another team to reproduce or triage the problem:

- What op-mode was running?
- What robot hardware/configuration was used?
- What did you expect to happen?
- What happened instead?
- Did Driver Station telemetry show a Health warning or exception?
- If available, attach the relevant `.wpilog` or summarize `make analyze`.

## Documentation changes

Docs are part of the adoption surface. Avoid assuming a reader already knows
this team's conventions. Prefer links to the exact file to edit and include
the command a new contributor should run to verify the change.
