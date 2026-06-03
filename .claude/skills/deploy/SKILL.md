---
name: deploy
description: Deploy this FTC robot code to the Control Hub, automatically choosing a full APK install vs a Sloth hot reload based on what changed since the last deploy. Use when the user wants to deploy, install, push code to the robot, or hot reload. Avoids the silent footgun of hot-reloading a @Pinned/dependency/manifest change (which doesn't take effect).
---

# Deploy to the robot

Pick the right deploy path and run it. The two paths must not be confused:

- **Full install** — `./gradlew :TeamCode:installDebug`. Full APK build +
  install. Required after changing anything Sloth can't hot-reload: a
  `@Pinned` class (`DriveConfig`), any dependency/gradle change, the manifest,
  resources, or any source outside the `org.firstinspires.ftc.teamcode`
  package. Also the right call for the first deploy of a session.
- **Hot reload** — `./gradlew deploySloth`. Pushes only teamcode classes,
  ~1s. For ordinary iteration on subsystems, op-modes, and command logic.

The footgun this skill exists to prevent: hot-reloading a `@Pinned`,
dependency, or manifest change **silently does nothing** — the robot keeps
running old code. Always classify before deploying.

## Procedure

1. **Classify the pending changes.** Run:

   ```
   bash .claude/skills/deploy/classify-deploy.sh
   ```

   It prints `RECOMMENDATION: HOT|FULL`, a one-line `REASON`, and the deciding
   files. It decides only — it does not deploy.

2. **Honour an explicit override.** If the user said "full install" / "force
   full" / "hot reload only", do that instead and say you're overriding the
   recommendation. Otherwise use the script's recommendation.

3. **Tell the user the call** in one line: the chosen path and why (quote the
   `REASON`). If FULL was chosen because of specific files (pinned/dep/manifest),
   name them.

4. **Confirm a device is connected.** Run `adb devices`. If none are listed,
   tell the user to connect first — `make connect` (adb connect to
   `192.168.43.1:5555`, the Control Hub default) — and stop; don't run the
   gradle deploy against no device.

5. **Run the deploy:**
   - FULL → `./gradlew :TeamCode:installDebug`
   - HOT  → `./gradlew deploySloth`

   The Load plugin auto-wires `removeSlothRemote` into `installDebug`, so a
   full install correctly clears any staged hot-reload jar — the two paths
   won't fight.

6. **On success, update the deploy marker** so the next run diffs from here:

   ```
   git rev-parse HEAD > .claude/.last-deploy-sha
   ```

   (Skip this if the deploy failed.)

7. **Report** the result concisely: which path ran, success/failure, and for a
   full install after a `@Pinned`/dep change, confirm the previously-hot-reloaded
   state is now baked into the APK.

## Notes

- Run everything from the repo root (the script `cd`s to the git toplevel
  itself, but the gradle commands assume root).
- The marker `.claude/.last-deploy-sha` is local per-machine state and is
  gitignored — don't commit it.
- If `classify-deploy.sh` reports "no changes detected", there's nothing to
  push; mention that rather than deploying needlessly.
- This skill does not tune or run op-modes — it only deploys. Selecting which
  op-mode to run happens on the Driver Station.
